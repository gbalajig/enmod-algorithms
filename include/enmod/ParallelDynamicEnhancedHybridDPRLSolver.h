#ifndef ENMOD_PARALLEL_DYNAMIC_ENHANCED_HYBRID_DPRL_SOLVER_H
#define ENMOD_PARALLEL_DYNAMIC_ENHANCED_HYBRID_DPRL_SOLVER_H

#include "Solver.h" // Inherit Solver, use local history struct like the Serial version
#include "ParallelSolverBase.h"
#include "Grid.h"
#include "Types.h"
#include <vector>
#include <map>

struct ParallelHybridStep {
    int time_step;
    Grid grid_state;
    Position agent_pos;
    std::string action;
    Cost current_total_cost;
};

class ParallelDynamicEnhancedHybridDPRLSolver : public Solver, public ParallelSolverBase {
public:
    ParallelDynamicEnhancedHybridDPRLSolver(const Grid& grid);
    void run() override;
    Cost getEvacuationCost() const override { return total_cost; }
    void generateReport(std::ofstream& report_file) const override;

private:
    std::vector<ParallelHybridStep> history;
    Cost total_cost;
    std::map<Position, std::vector<double>> q_table;
    bool model_loaded;

    bool loadOfflineModel(const std::string& filename);
    int getRLAction(const Position& pos);
};

#endif