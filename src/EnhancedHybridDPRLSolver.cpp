#include "enmod/EnhancedHybridDPRLSolver.h"
#include "enmod/Logger.h"
#include "enmod/BIDP.h" 
#include "enmod/DQNSolver.h" 
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue> // Required for Local Forward Wave

EnhancedHybridDPRLSolver::EnhancedHybridDPRLSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "EnhancedHybridDPRLSim"), current_mode(EvacuationMode::NORMAL) {
    
    Cost::current_mode = EvacuationMode::NORMAL;
    
    // 1. Initialize Deep Q-Network
    rl_solver = std::make_unique<DQNSolver>(grid_ref);
    
    // 2. [REQ 1] Run Pre-Deployment Training
    // This ensures the agent has "learned" the map structure before the fire starts.
    rl_solver->train(10000); 
}

void EnhancedHybridDPRLSolver::assessThreatAndSetMode(const Position& current_pos, const Grid& current_grid) {
    const auto& events = current_grid.getConfig().value("dynamic_events", json::array());
    current_mode = EvacuationMode::NORMAL; 

    // Fire & Smoke Checks (Same as before)
    for (const auto& event : events) {
        if (event.value("type", "") == "fire") {
            Position fire_pos = {event.at("position").at("row"), event.at("position").at("col")};
            if (current_grid.getCellType(fire_pos) == CellType::FIRE) {
                int radius = (event.value("size", "small") == "medium") ? 2 : 
                             (event.value("size", "small") == "large") ? 3 : 1;
                int dist = std::abs(current_pos.row - fire_pos.row) + std::abs(current_pos.col - fire_pos.col);
                if (dist <= 1) { current_mode = EvacuationMode::PANIC; return; }
                if (dist <= radius) { current_mode = EvacuationMode::ALERT; }
            }
        }
    }
    // Smoke Check
    int dr[] = {-1, 1, 0, 0}; int dc[] = {0, 0, -1, 1};
    for(int i = 0; i < 4; ++i) {
        Position n = {current_pos.row + dr[i], current_pos.col + dc[i]};
        if(current_grid.isValid(n.row, n.col) && current_grid.getSmokeIntensity(n) == "heavy"){
             if (current_mode != EvacuationMode::PANIC) current_mode = EvacuationMode::ALERT;
        }
    }
}

// [REQ 2] Helper to implement Interlaced Logic (Forward Reachability)
// This effectively replaces FIDP by running a local BFS from the agent.
std::vector<std::vector<double>> computeForwardReachability(const Grid& grid, const Position& start) {
    int rows = grid.getRows();
    int cols = grid.getCols();
    std::vector<std::vector<double>> f_cost(rows, std::vector<double>(cols, MAX_COST));
    
    std::queue<Position> q;
    if(grid.isWalkable(start.row, start.col)) {
        q.push(start);
        f_cost[start.row][start.col] = 0.0;
    }

    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    while(!q.empty()){
        Position curr = q.front(); q.pop();
        double current_c = f_cost[curr.row][curr.col];

        for(int i=0; i<4; ++i){
            Position next = {curr.row + dr[i], curr.col + dc[i]};
            if(grid.isWalkable(next.row, next.col) && f_cost[next.row][next.col] == MAX_COST){
                f_cost[next.row][next.col] = current_c + 1.0; // Simple hop count for reachability
                q.push(next);
            }
        }
    }
    return f_cost;
}

Direction EnhancedHybridDPRLSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    assessThreatAndSetMode(current_pos, current_grid);
    Cost::current_mode = current_mode;

    // --- MODE A: PANIC (DQN REFLEX) ---
    if (current_mode == EvacuationMode::PANIC) {
        return rl_solver->chooseAction(current_pos); 
    } 
    // --- MODE B: NORMAL (INTERLACED BIDP + FIDP) ---
    else {
        // 1. Backward Wave (BIDP): Cost from Exit -> Agent (The "Pull")
        BIDP backward_planner(current_grid);
        backward_planner.run();
        const auto& b_cost = backward_planner.getCostMap();

        // 2. Forward Wave (Local BFS): Validates Reachability from Agent (The "Push")
        // This fulfills your request to "follow the same logic done in interlaced"
        auto f_cost = computeForwardReachability(current_grid, current_pos);
        
        // If exit is unreachable via backward wave, we are trapped
        if (b_cost[current_pos.row][current_pos.col].distance == MAX_COST) {
            return Direction::STAY; 
        }

        Cost best_val;
        Direction best_direction = Direction::STAY;
        bool first = true;

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};
        Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

        for (int i = 0; i < 4; ++i) {
            Position n = {current_pos.row + dr[i], current_pos.col + dc[i]};
            
            if (current_grid.isWalkable(n.row, n.col)) {
                // INTERLACED CHECK: 
                // Node must be reachable from Agent (f_cost != MAX)
                // Node must lead to Exit (b_cost != MAX)
                if (f_cost[n.row][n.col] != MAX_COST && b_cost[n.row][n.col].distance != MAX_COST) {
                    
                    Cost current_c = b_cost[n.row][n.col];
                    if (first || current_c < best_val) {
                        best_val = current_c;
                        best_direction = dirs[i];
                        first = false;
                    }
                }
            }
        }
        return best_direction;
    }
}

void EnhancedHybridDPRLSolver::run() {
    Cost::current_mode = EvacuationMode::NORMAL;
    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    history.clear();

    const auto& events = dynamic_grid.getConfig().value("dynamic_events", json::array());
    int max_time = 2 * (dynamic_grid.getRows() * dynamic_grid.getCols());

    for (int t = 0; t < max_time; ++t) {
        for (const auto& event_cfg : events) {
            if (event_cfg.value("time_step", -1) == t) dynamic_grid.addHazard(event_cfg);
        }
        
        Direction move_dir = getNextMove(current_pos, dynamic_grid);

        history.push_back({t, dynamic_grid, current_pos, "Enhanced Hybrid Plan", total_cost, current_mode});

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            history.back().action = "SUCCESS: Reached Exit.";
            break;
        }

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

Cost EnhancedHybridDPRLSolver::getEvacuationCost() const { return total_cost; }

void EnhancedHybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Enhanced Hybrid DP-RL)</h2>\n";
    for (const auto& step : history) {
        std::string mode_str = (step.mode == EvacuationMode::PANIC) ? "PANIC (DQN)" : "NORMAL (Interlaced)";
        report_file << "<h3>Time Step: " << step.time_step << " (Mode: " << mode_str << ")</h3>\n";
        report_file << "<p>Action: " << step.action << " | Cost: " << step.current_total_cost << "</p>\n";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
    }
}