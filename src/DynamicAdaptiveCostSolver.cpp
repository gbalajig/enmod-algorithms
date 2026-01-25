#include "enmod/DynamicAdaptiveCostSolver.h"
#include "enmod/BIDP.h"

DynamicAdaptiveCostSolver::DynamicAdaptiveCostSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "DynamicAdaptiveCostSim") {}

Direction DynamicAdaptiveCostSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    // This solver uses BIDP but interprets costs differently (handled inside BIDP's cost logic usually)
    // For Dynamic version, we just re-run BIDP on the updated grid.
    BIDP planner(current_grid);
    planner.run();
    
    // Greedy descent
    const auto& cost_map = planner.getCostMap();
    if (cost_map[current_pos.row][current_pos.col].distance == MAX_COST) return Direction::STAY;

    Cost best = cost_map[current_pos.row][current_pos.col];
    Direction dir = Direction::STAY;
    
    int dr[] = {-1, 1, 0, 0}; int dc[] = {0, 0, -1, 1};
    Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

    for(int i=0; i<4; ++i){
        Position n = {current_pos.row+dr[i], current_pos.col+dc[i]};
        if(current_grid.isWalkable(n.row, n.col)){
             if(cost_map[n.row][n.col] < best) {
                 best = cost_map[n.row][n.col];
                 dir = dirs[i];
             }
        }
    }
    return dir;
}

// Reuse the exact same run() logic as DynamicInterlacedSolver
void DynamicAdaptiveCostSolver::run() {
    // ... [Same dynamic loop structure as DynamicInterlacedSolver] ...
    // Copy-paste the run() loop from DynamicInterlacedSolver here, 
    // just change history.push_back label to "Adapting..."
    
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
        history.push_back({t, dynamic_grid, current_pos, "Adapting...", total_cost, EvacuationMode::NORMAL});
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

Cost DynamicAdaptiveCostSolver::getEvacuationCost() const { return total_cost; }
void DynamicAdaptiveCostSolver::generateReport(std::ofstream& file) const { /* standard report */ }