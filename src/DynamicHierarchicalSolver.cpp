#include "enmod/DynamicHierarchicalSolver.h"
#include "enmod/BIDP.h"

DynamicHierarchicalSolver::DynamicHierarchicalSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "DynamicHierarchicalSim") {}

Direction DynamicHierarchicalSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    // LEVEL 1: STRATEGIC PLANNING (Global Graph Search)
    BIDP global_planner(current_grid);
    global_planner.run();
    const auto& cost_map = global_planner.getCostMap();

    // Check if trapped
    if (cost_map[current_pos.row][current_pos.col].distance == MAX_COST) return Direction::STAY;

    // Determine Desired Move from Global Plan
    Direction desired_dir = Direction::STAY;
    Cost best_cost = cost_map[current_pos.row][current_pos.col];
    
    int dr[] = {-1, 1, 0, 0}; int dc[] = {0, 0, -1, 1};
    Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

    for(int i=0; i<4; ++i){
        Position n = {current_pos.row+dr[i], current_pos.col+dc[i]};
        if(current_grid.isWalkable(n.row, n.col)){
             if(cost_map[n.row][n.col] < best_cost) {
                 best_cost = cost_map[n.row][n.col];
                 desired_dir = dirs[i];
             }
        }
    }

    // LEVEL 2: TACTICAL SAFETY CHECK (Local Override)
    // If desired move leads to a "High Risk" zone (e.g., Smoke Density > Threshold), try alternatives
    Position next = current_grid.getNextPosition(current_pos, desired_dir);
    if (current_grid.getSmokeIntensity(next) == "heavy") {
        // Override! Look for any walkable neighbor with less smoke, even if cost is higher
        for(int i=0; i<4; ++i) {
            Position n = {current_pos.row+dr[i], current_pos.col+dc[i]};
            if(current_grid.isWalkable(n.row, n.col) && current_grid.getSmokeIntensity(n) != "heavy") {
                return dirs[i]; // Evasive maneuver
            }
        }
    }

    return desired_dir;
}

void DynamicHierarchicalSolver::run() {
    // Standard Dynamic Loop
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
        history.push_back({t, dynamic_grid, current_pos, "Hierarchical Plan", total_cost, EvacuationMode::NORMAL});
        
        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) { history.back().action = "SUCCESS"; break; }
        
        Position next_pos = dynamic_grid.getNextPosition(current_pos, move_dir);
        if (!dynamic_grid.isWalkable(next_pos.row, next_pos.col)) next_pos = current_pos;
        
        std::string action_str = "STAY";
        if (move_dir == Direction::UP) action_str = "UP";
        else if (move_dir == Direction::DOWN) action_str = "DOWN";
        else if (move_dir == Direction::LEFT) action_str = "LEFT";
        else if (move_dir == Direction::RIGHT) action_str = "RIGHT";
        
        history.back().action = action_str;
        total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
        current_pos = next_pos;
    }
}

Cost DynamicHierarchicalSolver::getEvacuationCost() const { return total_cost; }
void DynamicHierarchicalSolver::generateReport(std::ofstream& f) const { 
    f << "<h2>Dynamic Hierarchical Solver Report</h2>"; 
    /* Add detail if needed */ 
}