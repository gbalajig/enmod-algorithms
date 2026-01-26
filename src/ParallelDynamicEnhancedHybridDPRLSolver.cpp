#include "enmod/ParallelDynamicEnhancedHybridDPRLSolver.h"
#include "enmod/Logger.h"
#include "enmod/json.hpp"
#include <fstream>
#include <filesystem>
#include <algorithm>

using json = nlohmann::json;

ParallelDynamicEnhancedHybridDPRLSolver::ParallelDynamicEnhancedHybridDPRLSolver(const Grid& grid)
    : Solver(grid, "ParallelDynamicEnhancedHybridDPRL"), ParallelSolverBase(grid), total_cost{0,0,0}, model_loaded(false) {}

void ParallelDynamicEnhancedHybridDPRLSolver::run() {
    Logger::log(LogLevel::INFO, "Running Parallel Dynamic Enhanced Hybrid (CUDA)...");

    // 1. Load Offline RL
    std::string model_file = "data/dqn_model_" + grid.getName() + ".json";
    model_loaded = loadOfflineModel(model_file);

    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    history.clear();

    int max_time = dynamic_grid.getRows() * dynamic_grid.getCols() * 2;
    const auto& events = dynamic_grid.getConfig().value("dynamic_events", json::array());

    for (int t = 0; t < max_time; ++t) {
        // A. Dynamic Update
        for (const auto& event_cfg : events) {
            if (event_cfg.value("time_step", -1) == t) dynamic_grid.addHazard(event_cfg);
        }

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            history.push_back({t, dynamic_grid, current_pos, "SUCCESS", total_cost});
            break;
        }

        // B. GPU Safety Check (BIDP)
        // Upload the *changed* grid to GPU
        uploadGridToGPU(dynamic_grid); 
        runParallelBIDP(); // Calculate gradients on GPU
        downloadCostsFromGPU();
        const auto& gpu_costs = getHostCostMap(); // Linear array [row * cols + col]

        // Check if trapped
        Cost current_node_cost = gpu_costs[current_pos.row * dynamic_grid.getCols() + current_pos.col];
        if (current_node_cost.distance >= 2147483647) {
            history.push_back({t, dynamic_grid, current_pos, "TRAPPED", total_cost});
            break;
        }

        // C. Hybrid Decision
        Position best_next_pos = current_pos;
        std::string source = "DP_GPU";
        
        int rl_action = (model_loaded) ? getRLAction(current_pos) : -1;
        bool rl_safe = false;
        Position rl_target = current_pos;

        if (rl_action != -1) {
             int dr[] = {-1, 1, 0, 0}; 
             int dc[] = {0, 0, -1, 1};
             rl_target = {current_pos.row + dr[rl_action], current_pos.col + dc[rl_action]};
             
             if (dynamic_grid.isWalkable(rl_target.row, rl_target.col)) {
                 // Check safety using GPU result
                 Cost c = gpu_costs[rl_target.row * dynamic_grid.getCols() + rl_target.col];
                 if (c.distance < 2147483647 && c.smoke < 50) rl_safe = true;
             }
        }

        if (rl_safe) {
            best_next_pos = rl_target;
            source = "RL_Offline";
        } else {
            // Fallback: Gradient Descent using GPU Cost Map
            Cost best_c = {2147483647, 0, 0};
            int dr[] = {-1, 1, 0, 0}; 
            int dc[] = {0, 0, -1, 1};
            
            for(int i=0; i<4; ++i) {
                Position n = {current_pos.row + dr[i], current_pos.col + dc[i]};
                if (dynamic_grid.isWalkable(n.row, n.col)) {
                    Cost c = gpu_costs[n.row * dynamic_grid.getCols() + n.col];
                    if (c.distance < best_c.distance) {
                        best_c = c;
                        best_next_pos = n;
                    }
                }
            }
        }

        history.push_back({t, dynamic_grid, current_pos, "MOVE_" + source, total_cost});
        total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
        current_pos = best_next_pos;
    }
}

// ... Implement loadOfflineModel and getRLAction (same as Serial) ...
// ... Implement generateReport ...
// (Helper functions omitted for brevity, identical to serial version)
bool ParallelDynamicEnhancedHybridDPRLSolver::loadOfflineModel(const std::string& filename) {
    if (!std::filesystem::exists(filename)) return false;
    try {
        std::ifstream i(filename);
        json j;
        i >> j;
        q_table.clear();
        for (auto& el : j["q_table"].items()) {
            std::string key = el.key();
            size_t comma = key.find(',');
            int r = std::stoi(key.substr(0, comma));
            int c = std::stoi(key.substr(comma + 1));
            q_table[{r, c}] = el.value().get<std::vector<double>>();
        }
        return true;
    } catch (...) { return false; }
}

int ParallelDynamicEnhancedHybridDPRLSolver::getRLAction(const Position& pos) {
    if (q_table.find(pos) == q_table.end()) return -1;
    const auto& q_vals = q_table[pos];
    return std::distance(q_vals.begin(), std::max_element(q_vals.begin(), q_vals.end()));
}

void ParallelDynamicEnhancedHybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Parallel Dynamic Enhanced Hybrid Report</h2>";
    report_file << "<p><strong>Strategy:</strong> Offline RL + GPU Real-Time Safety Check.</p>";
    for (const auto& step : history) {
        report_file << "<p>Time: " << step.time_step << " | Action: " << step.action << "</p>";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
    }
}