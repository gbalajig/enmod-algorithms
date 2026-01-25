#include "enmod/DynamicInterlacedSolver.h"
#include "enmod/BIDP.h"
#include "enmod/FIDP.h" // Assuming needed for Interlaced logic
#include <iostream>

DynamicInterlacedSolver::DynamicInterlacedSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "DynamicInterlacedSim") {}

Direction DynamicInterlacedSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    // 1. Backward Wave (Future -> Present)
    BIDP backward_solver(current_grid);
    backward_solver.run();
    const auto& b_cost = backward_solver.getCostMap();

    // 2. Forward Lookahead (Local Gradient Descent on Backward Cost)
    // In a full Interlaced implementation, you might run FIDP here too.
    // For navigation, we greedily follow the Backward Wave's gradient.
    
    if (b_cost[current_pos.row][current_pos.col].distance == MAX_COST) return Direction::STAY;

    Cost best_cost = b_cost[current_pos.row][current_pos.col];
    Direction best_dir = Direction::STAY;

    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};
    Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

    for(int i=0; i<4; ++i) {
        Position next = {current_pos.row + dr[i], current_pos.col + dc[i]};
        if(current_grid.isWalkable(next.row, next.col)) {
            if (b_cost[next.row][next.col] < best_cost) {
                best_cost = b_cost[next.row][next.col];
                best_dir = dirs[i];
            }
        }
    }
    return best_dir;
}

void DynamicInterlacedSolver::run() {
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

        // 2. Dynamic Re-planning
        Direction move_dir = getNextMove(current_pos, dynamic_grid);

        history.push_back({t, dynamic_grid, current_pos, "Interlacing...", total_cost, EvacuationMode::NORMAL});

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            history.back().action = "SUCCESS";
            break;
        }

        // 3. Execute Move
        Position next_pos = dynamic_grid.getNextPosition(current_pos, move_dir);
        if (!dynamic_grid.isWalkable(next_pos.row, next_pos.col)) {
            next_pos = current_pos; // Collision
        }
        
        std::string action_str = "STAY";
        if (move_dir == Direction::UP) action_str = "UP";
        else if (move_dir == Direction::DOWN) action_str = "DOWN";
        else if (move_dir == Direction::LEFT) action_str = "LEFT";
        else if (move_dir == Direction::RIGHT) action_str = "RIGHT";

        history.back().action = action_str;
        total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
        current_pos = next_pos;
    }
    
    if(history.back().action != "SUCCESS") {
         history.push_back({max_time, dynamic_grid, current_pos, "FAILURE", total_cost, EvacuationMode::NORMAL});
    }
}

Cost DynamicInterlacedSolver::getEvacuationCost() const { return total_cost; }

void DynamicInterlacedSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Dynamic Interlaced Solver)</h2>\n";
    for (const auto& step : history) {
        report_file << "<h3>T=" << step.time_step << " | Action: " << step.action << "</h3>\n";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
    }
}