#include "enmod/DynamicPolicyBlendingSolver.h"
#include "enmod/Logger.h"
#include "enmod/BIDP.h" // Assuming BIDP is used for global path cost
#include "enmod/Types.h"
#include <algorithm>
#include <cmath>
#include <limits>

DynamicPolicyBlendingSolver::DynamicPolicyBlendingSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "DynamicPolicyBlendingSim") {
}

void DynamicPolicyBlendingSolver::run() {
    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    history.clear();

    const auto& events = dynamic_grid.getConfig().value("dynamic_events", json::array());
    int max_time = 2 * (dynamic_grid.getRows() * dynamic_grid.getCols());

    for (int t = 0; t < max_time; ++t) {
        // 1. Update Environment
        for (const auto& event_cfg : events) {
            if (event_cfg.value("time_step", -1) == t) dynamic_grid.addHazard(event_cfg);
        }

        // 2. Determine Mode
        // (Simple blended logic: Check immediate threat vs global goal)
        // Global Planner (Pull)
        BIDP planner(dynamic_grid);
        planner.run();
        const auto& global_cost_map = planner.getCostMap();

        if (global_cost_map[current_pos.row][current_pos.col].distance == MAX_COST) {
            // Trapped
            history.push_back({t, dynamic_grid, current_pos, "STUCK", total_cost, EvacuationMode::PANIC});
            break;
        }

        // Local Decision (Push) - Check neighbors
        Direction best_dir = Direction::STAY;
        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};
        Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};
        
        Cost best_val; 
        bool first = true;

        for(int i=0; i<4; ++i) {
            Position next = {current_pos.row + dr[i], current_pos.col + dc[i]};
            if(dynamic_grid.isWalkable(next.row, next.col)) {
                // Blending Logic: Avoid heavy smoke even if distance is good
                std::string smoke = dynamic_grid.getSmokeIntensity(next);
                Cost c = global_cost_map[next.row][next.col];
                
                // Penalty for smoke
                if (smoke == "heavy") c.distance += 100;
                else if (smoke == "medium") c.distance += 50;

                if (first || c < best_val) {
                    best_val = c;
                    best_dir = dirs[i];
                    first = false;
                }
            }
        }

        // 3. Move
        std::string action_str = "STAY";
        if (best_dir == Direction::UP) action_str = "UP";
        if (best_dir == Direction::DOWN) action_str = "DOWN";
        if (best_dir == Direction::LEFT) action_str = "LEFT";
        if (best_dir == Direction::RIGHT) action_str = "RIGHT";

        history.push_back({t, dynamic_grid, current_pos, action_str, total_cost, EvacuationMode::NORMAL});

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            history.back().action = "SUCCESS";
            break;
        }

        Position next_pos = dynamic_grid.getNextPosition(current_pos, best_dir);
        if (!dynamic_grid.isWalkable(next_pos.row, next_pos.col)) {
            next_pos = current_pos;
        } else {
            total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
            current_pos = next_pos;
        }
    }
}

Cost DynamicPolicyBlendingSolver::getEvacuationCost() const {
    return total_cost;
}

void DynamicPolicyBlendingSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Dynamic Policy Blending)</h2>\n";
    for (const auto& step : history) {
        report_file << "<h3>Time Step: " << step.time_step << "</h3>\n";
        report_file << "<p>Action: " << step.action << " | Cost: " << step.current_total_cost.distance << "</p>\n";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
    }
}