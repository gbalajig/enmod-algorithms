#include "enmod/DynamicEnhancedHybridDPRLSolver.h"
#include "enmod/Logger.h"
#include "enmod/BIDP.h"   // Rely on BIDP for robust Exit-Distance checks
#include "enmod/json.hpp" 
#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <limits>

using json = nlohmann::json;

DynamicEnhancedHybridDPRLSolver::DynamicEnhancedHybridDPRLSolver(const Grid& grid) 
    : Solver(grid, "DynamicEnhancedHybridDPRL"),
      total_cost{0, 0, 0},
      model_loaded(false) {
}

void DynamicEnhancedHybridDPRLSolver::run() {
    // 1. Load Offline Brain
    std::string model_file = "data/dqn_model_" + grid.getName() + ".json";
    model_loaded = loadOfflineModel(model_file);
    
    if (model_loaded) {
        Logger::log(LogLevel::INFO, "DynamicEnhancedHybrid: Loaded Offline DQN Model.");
    } else {
        Logger::log(LogLevel::WARN, "DynamicEnhancedHybrid: No Offline Model found. Will rely on DP fallback.");
    }

    // 2. Simulation Setup
    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    history.clear(); 

    int max_time = dynamic_grid.getRows() * dynamic_grid.getCols() * 2;
    const auto& events = dynamic_grid.getConfig().value("dynamic_events", json::array());

    for (int t = 0; t < max_time; ++t) {
        // A. Evolve Environment
        for (const auto& event_cfg : events) {
            if (event_cfg.value("time_step", -1) == t) dynamic_grid.addHazard(event_cfg);
        }

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            history.push_back({t, dynamic_grid, current_pos, "SUCCESS", total_cost, EvacuationMode::NORMAL});
            break;
        }

        // B. Run BIDP (Real-Time Safety Check)
        // BIDP provides the critical 'Distance to Exit' map.
        BIDP safety_planner(dynamic_grid);
        safety_planner.run();
        const auto& safety_map = safety_planner.getCostMap();

        if (safety_map[current_pos.row][current_pos.col].distance >= 2147483647) {
            history.push_back({t, dynamic_grid, current_pos, "TRAPPED", total_cost, EvacuationMode::PANIC});
            break;
        }

        // C. Hybrid Decision Logic
        Position best_next_pos = current_pos;
        std::string decision_source = "DP_Safety";
        
        // 1. Consult Offline RL
        int rl_action = -1;
        if (model_loaded) {
            rl_action = getRLAction(current_pos);
        }

        // 2. Validate RL Move
        bool rl_is_safe = false;
        Position rl_target = current_pos; 
        
        if (rl_action != -1) {
            int dr[] = {-1, 1, 0, 0}; 
            int dc[] = {0, 0, -1, 1};
            rl_target = {current_pos.row + dr[rl_action], current_pos.col + dc[rl_action]};
            
            if (dynamic_grid.isWalkable(rl_target.row, rl_target.col)) {
                 Cost c = safety_map[rl_target.row][rl_target.col];
                 // Safety Check: Move must lead to a valid path (distance < INF) and low smoke
                 if (c.distance < 2147483647 && c.smoke < 50) { 
                     rl_is_safe = true;
                 }
            }
        }

        // 3. Select Move
        if (rl_is_safe) {
            best_next_pos = rl_target;
            decision_source = "RL_Offline";
        } else {
            // Fallback: Pick neighbor with lowest BIDP cost (Gradient Descent to Exit)
            Cost best_c = {2147483647, 2147483647, 2147483647};
            int dr[] = {-1, 1, 0, 0}; 
            int dc[] = {0, 0, -1, 1};
            
            for(int i=0; i<4; ++i) {
                Position n = {current_pos.row + dr[i], current_pos.col + dc[i]};
                if (dynamic_grid.isWalkable(n.row, n.col)) {
                    Cost c = safety_map[n.row][n.col];
                    if (c < best_c) {
                        best_c = c;
                        best_next_pos = n;
                    }
                }
            }
            decision_source = "DP_Fallback";
        }

        history.push_back({t, dynamic_grid, current_pos, "MOVE_" + decision_source, total_cost, EvacuationMode::NORMAL});
        total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
        current_pos = best_next_pos;
    }
}

Cost DynamicEnhancedHybridDPRLSolver::getEvacuationCost() const { return total_cost; }

void DynamicEnhancedHybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Dynamic Enhanced Hybrid DP-RL)</h2>";
    report_file << "<p><strong>Strategy:</strong> Offline RL Memory + Real-Time BIDP Safety.</p>";
    for (const auto& step : history) {
        report_file << "<div class='step'>";
        report_file << "<strong>Time: " << step.time_step << "</strong> | Action: " << step.action << "<br/>";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
        report_file << "</div>";
    }
}

bool DynamicEnhancedHybridDPRLSolver::loadOfflineModel(const std::string& filename) {
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

int DynamicEnhancedHybridDPRLSolver::getRLAction(const Position& pos) {
    if (q_table.find(pos) == q_table.end()) return -1; 
    const auto& q_vals = q_table[pos];
    return std::distance(q_vals.begin(), std::max_element(q_vals.begin(), q_vals.end()));
}

// [NEW] CPS Controller Integration
Direction DynamicEnhancedHybridDPRLSolver::getNextMove(const Position& current, const Grid& view) {
    // 1. Try Offline RL
    if (model_loaded) {
        int rl_action = getRLAction(current);
        if (rl_action != -1) {
             // Basic validity check on the 'view' (Agent's perception)
             int dr[] = {-1, 1, 0, 0}; 
             int dc[] = {0, 0, -1, 1};
             Position next = {current.row + dr[rl_action], current.col + dc[rl_action]};
             if (view.isWalkable(next.row, next.col)) return static_cast<Direction>(rl_action);
        }
    }

    // 2. Fallback: Run BIDP on the local view to find the path
    BIDP quick_solver(view);
    quick_solver.run(); // Calculate gradient to exit based on current view
    const auto& cost_map = quick_solver.getCostMap();

    // Find neighbor with lowest cost
    Direction best_dir = Direction::STAY;
    Cost min_cost = {2147483647, 2147483647, 2147483647};

    int dr[] = {-1, 1, 0, 0}; 
    int dc[] = {0, 0, -1, 1};
    Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

    for(int i=0; i<4; ++i) {
        Position n = {current.row + dr[i], current.col + dc[i]};
        if (view.isWalkable(n.row, n.col)) {
            Cost c = cost_map[n.row][n.col];
            if (c < min_cost) {
                min_cost = c;
                best_dir = dirs[i];
            }
        }
    }
    return best_dir;
}