#include "enmod/HybridDPRLSolver.h"
#include "enmod/Logger.h"
#include <algorithm>
#include <cmath>

HybridDPRLSolver::HybridDPRLSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "HybridDPRLSim"), current_mode(EvacuationMode::NORMAL) {
    
    Cost::current_mode = EvacuationMode::NORMAL;
    rl_solver = std::make_unique<QLearningSolver>(grid_ref);
    rl_solver->train(5000); 
}

void HybridDPRLSolver::assessThreatAndSetMode(const Position& current_pos, const Grid& current_grid) {
    current_mode = EvacuationMode::NORMAL; 

    // --- CRITICAL: CHECK CURRENT STANDING POSITION FIRST ---
    if (current_grid.getCellType(current_pos) == CellType::FIRE) {
        current_mode = EvacuationMode::PANIC;
        return; 
    }

    // SCAN GRID FOR NEARBY HAZARDS (Radius 2)
    int radius = 2;
    for (int r = current_pos.row - radius; r <= current_pos.row + radius; ++r) {
        for (int c = current_pos.col - radius; c <= current_pos.col + radius; ++c) {
            if (current_grid.isValid(r, c)) { 
                Position check_pos = {r, c};
                int dist = std::abs(current_pos.row - r) + std::abs(current_pos.col - c);

                if (current_grid.getCellType(check_pos) == CellType::FIRE) {
                    if (dist <= 1) { 
                        current_mode = EvacuationMode::PANIC; 
                        return; 
                    }
                    current_mode = EvacuationMode::ALERT;
                }
                
                if (dist <= 1 && current_grid.getSmokeIntensity(check_pos) == "heavy") {
                    if (current_mode != EvacuationMode::PANIC) current_mode = EvacuationMode::ALERT;
                }
            }
        }
    }
}

Direction HybridDPRLSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    assessThreatAndSetMode(current_pos, current_grid);
    Cost::current_mode = current_mode;

    if (current_mode == EvacuationMode::PANIC) {
        return rl_solver->chooseAction(current_pos);
    } 
    else {
        BIDP step_planner(current_grid);
        step_planner.run();
        const auto& cost_map = step_planner.getCostMap();
        
        Cost best_neighbor_cost = {MAX_COST, MAX_COST, MAX_COST};
        Direction best_direction = Direction::STAY;
        
        if (cost_map[current_pos.row][current_pos.col].distance != MAX_COST) {
            best_neighbor_cost = cost_map[current_pos.row][current_pos.col];
        }

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};
        Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

        for (int i = 0; i < 4; ++i) {
            Position neighbor = {current_pos.row + dr[i], current_pos.col + dc[i]};
            if (current_grid.isValid(neighbor.row, neighbor.col) && current_grid.isWalkable(neighbor.row, neighbor.col)) {
                if (cost_map[neighbor.row][neighbor.col] < best_neighbor_cost) {
                    best_neighbor_cost = cost_map[neighbor.row][neighbor.col];
                    best_direction = dirs[i];
                }
            }
        }
        return best_direction;
    }
}

void HybridDPRLSolver::run() {
    Cost::current_mode = EvacuationMode::NORMAL;
    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    history.clear();

    const auto& events = dynamic_grid.getConfig().value("dynamic_events", json::array());

    for (int t = 0; t < 2 * (dynamic_grid.getRows() * dynamic_grid.getCols()); ++t) {
        for (const auto& event_cfg : events) {
            if (event_cfg.value("time_step", -1) == t) dynamic_grid.addHazard(event_cfg);
        }
        
        Direction move_dir = getNextMove(current_pos, dynamic_grid);
        history.push_back({t, dynamic_grid, current_pos, "Moving", total_cost, current_mode});

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) break;

        Position next_move = dynamic_grid.getNextPosition(current_pos, move_dir);
        if (dynamic_grid.isValid(next_move.row, next_move.col) && dynamic_grid.isWalkable(next_move.row, next_move.col)) {
            total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
            current_pos = next_move;
        }
    }
}

Cost HybridDPRLSolver::getEvacuationCost() const { return total_cost; }

void HybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Hybrid Solver)</h2>\n";
    for (const auto& step : history) {
        report_file << "<h3>Step: " << step.time_step << "</h3>\n";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
    }
}