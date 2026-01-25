#ifndef ENMOD_DYNAMIC_ADAPTIVE_COST_SOLVER_H
#define ENMOD_DYNAMIC_ADAPTIVE_COST_SOLVER_H

#include "DynamicSolver.h"

class DynamicAdaptiveCostSolver : public Solver {
public:
    DynamicAdaptiveCostSolver(const Grid& grid_ref);
    void run() override;
    Direction getNextMove(const Position& current_pos, const Grid& current_grid);
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;
private:
    std::vector<StepReport> history;
    Cost total_cost;
};
#endif