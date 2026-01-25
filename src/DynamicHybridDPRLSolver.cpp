#include "enmod/DynamicHybridDPRLSolver.h"
#include "enmod/Logger.h"
#include <algorithm>
#include <cmath>

DynamicHybridDPRLSolver::DynamicHybridDPRLSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "DynamicHybridDPRLSim"), current_mode(EvacuationMode::NORMAL) {
    
    Cost::current_mode = EvacuationMode::NORMAL;
    // Pre-train the RL agent for the specific grid environment
    rl_solver = std::make_unique<QLearningSolver>(grid_ref);
    rl_solver->train(5000); 
}

void DynamicHybridDPRLSolver::assessThreatAndSetMode(const Position& current_pos, const Grid& current_grid) {
    const auto& events = current_grid.getConfig().value("dynamic_events", json::array());
    current_mode = EvacuationMode::NORMAL; 

    // 1. Check Fire Proximity
    for (const auto& event : events) {
        if (event.value("type", "") == "fire") {
            Position fire_pos = {event.at("position").at("row"), event.at("position").at("col")};
            if (current_grid.getCellType(fire_pos) == CellType::FIRE) {
                int radius = 1;
                if(event.value("size", "small") == "medium") radius = 2;
                if(event.value("size", "small") == "large") radius = 3;

                int dist = std::abs(current_pos.row - fire_pos.row) + std::abs(current_pos.col - fire_pos.col);
                if (dist <= 1) { current_mode = EvacuationMode::PANIC; return; }
                if (dist <= radius) { current_mode = EvacuationMode::ALERT; }
            }
        }
    }

    // 2. Check Smoke Proximity
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};
    for(int i = 0; i < 4; ++i) {
        Position neighbor = {current_pos.row + dr[i], current_pos.col + dc[i]};
        if(current_grid.getSmokeIntensity(neighbor) == "heavy"){
             if (current_mode != EvacuationMode::PANIC) current_mode = EvacuationMode::ALERT;
        }
    }
}

Direction DynamicHybridDPRLSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    assessThreatAndSetMode(current_pos, current_grid);
    Cost::current_mode = current_mode;

    // THE SWITCH: Panic -> RL (Instinct), Normal -> DP (Math)
    if (current_mode == EvacuationMode::PANIC) {
        return rl_solver->chooseAction(current_pos);
    } 
    else {
        // Re-calculate Optimal Path based on current dynamic grid state
        BIDP step_planner(current_grid);
        step_planner.run();
        const auto& cost_map = step_planner.getCostMap();
        
        Cost best_neighbor_cost;
        Direction best_direction = Direction::STAY;
        
        if (cost_map[current_pos.row][current_pos.col].distance != MAX_COST) {
            best_neighbor_cost = cost_map[current_pos.row][current_pos.col];
        }

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};
        Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

        for (int i = 0; i < 4; ++i) {
            Position neighbor = {current_pos.row + dr[i], current_pos.col + dc[i]};
            if (current_grid.isWalkable(neighbor.row, neighbor.col)) {
                if (cost_map[neighbor.row][neighbor.col] < best_neighbor_cost) {
                    best_neighbor_cost = cost_map[neighbor.row][neighbor.col];
                    best_direction = dirs[i];
                }
            }
        }
        return best_direction;
    }
}

void DynamicHybridDPRLSolver::run() {
    Cost::current_mode = EvacuationMode::NORMAL;
    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    history.clear();

    const auto& events = dynamic_grid.getConfig().value("dynamic_events", json::array());
    int max_time = 2 * (dynamic_grid.getRows() * dynamic_grid.getCols());

    for (int t = 0; t < max_time; ++t) {
        // Apply Dynamic Events
        for (const auto& event_cfg : events) {
            if (event_cfg.value("time_step", -1) == t) {
                dynamic_grid.addHazard(event_cfg);
            }
        }
        
        Direction move_dir = getNextMove(current_pos, dynamic_grid);

        history.push_back({t, dynamic_grid, current_pos, "Planning...", total_cost, current_mode});

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            history.back().action = "SUCCESS: Reached Exit.";
            break;
        }

        // Validate Move
        Position next_move = dynamic_grid.getNextPosition(current_pos, move_dir);
        if (!dynamic_grid.isWalkable(next_move.row, next_move.col)) {
            next_move = current_pos; 
            move_dir = Direction::STAY;
        }

        std::string action = "STAY";
        if (move_dir == Direction::UP) action = "UP";
        else if (move_dir == Direction::DOWN) action = "DOWN";
        else if (move_dir == Direction::LEFT) action = "LEFT";
        else if (move_dir == Direction::RIGHT) action = "RIGHT";
        
        history.back().action = action;
        total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
        current_pos = next_move;
    }

    if(history.empty() || (history.back().action.find("SUCCESS") == std::string::npos)){
         history.push_back({(int)history.size(), dynamic_grid, current_pos, "FAILURE: Timed out.", total_cost, current_mode});
         total_cost = {};
    }
     
    Cost::current_mode = EvacuationMode::NORMAL;
}

Cost DynamicHybridDPRLSolver::getEvacuationCost() const { return total_cost; }

void DynamicHybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Dynamic Hybrid DP-RL Solver)</h2>\n";
    // ... [Same reporting logic as HybridDPRLSolver] ...
    for (const auto& step : history) {
        std::string mode_str = (step.mode == EvacuationMode::PANIC) ? "PANIC" : ((step.mode == EvacuationMode::ALERT) ? "ALERT" : "NORMAL");
        report_file << "<h3>Time Step: " << step.time_step << " (Mode: " << mode_str << ")</h3>\n";
        report_file << "<p>Action: " << step.action << " | Cost: " << step.current_total_cost << "</p>\n";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
    }
}