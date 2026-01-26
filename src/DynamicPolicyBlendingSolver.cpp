#include "enmod/DynamicPolicyBlendingSolver.h"
#include "enmod/BIDP.h"
#include <limits>

DynamicPolicyBlendingSolver::DynamicPolicyBlendingSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "DynamicPolicyBlendingSim") {}

Direction DynamicPolicyBlendingSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    // POLICY 1: GOAL SEEKING (Global)
    BIDP planner(current_grid);
    planner.run();
    const auto& cost_map = planner.getCostMap();

    // POLICY 2: HAZARD AVOIDANCE (Local)
    // We calculate a "score" for each neighbor:
    // Score = -1 * (Distance_Cost) + -100 * (Smoke_Intensity)

    double best_score = -std::numeric_limits<double>::infinity();
    Direction best_dir = Direction::STAY;

    int dr[] = {-1, 1, 0, 0}; int dc[] = {0, 0, -1, 1};
    Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

    for(int i=0; i<4; ++i){
        Position n = {current_pos.row+dr[i], current_pos.col+dc[i]};
        if(current_grid.isWalkable(n.row, n.col)){
            double global_score = 0;
            if (cost_map[n.row][n.col].distance != MAX_COST) {
                // Invert cost because lower is better
                global_score = -1.0 * cost_map[n.row][n.col].distance; 
            } else {
                global_score = -1000.0; // Dead end
            }

            double local_penalty = 0;
            std::string smoke = current_grid.getSmokeIntensity(n);
            if (smoke == "heavy") local_penalty = -500.0;
            else if (smoke == "medium") local_penalty = -50.0;

            // BLENDING: 0.7 Global + 0.3 Local (Conceptually)
            // Here we just sum the scores
            double total_score = global_score + local_penalty;

            if (total_score > best_score) {
                best_score = total_score;
                best_dir = dirs[i];
            }
        }
    }
    return best_dir;
}

void DynamicPolicyBlendingSolver::run() {
    // Reuse Standard Loop (same as above)
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
        history.push_back({t, dynamic_grid, current_pos, "Blending Policies", total_cost, EvacuationMode::NORMAL});
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

Cost DynamicPolicyBlendingSolver::getEvacuationCost() const { return total_cost; }
void DynamicPolicyBlendingSolver::generateReport(std::ofstream& f) const { f << "<h2>Blending Report</h2>"; }