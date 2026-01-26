#ifndef ENMOD_PARALLEL_ENHANCED_HYBRID_DPRL_SOLVER_H
#define ENMOD_PARALLEL_ENHANCED_HYBRID_DPRL_SOLVER_H

#include "Solver.h"
#include "ParallelSolverBase.h"
#include "DQNSolver.h" // For Offline RL Logic

class ParallelEnhancedHybridDPRLSolver : public Solver, public ParallelSolverBase {
public:
    ParallelEnhancedHybridDPRLSolver(const Grid& grid);
    void run() override;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

private:
    Cost total_cost;
    std::unique_ptr<DQNSolver> rl_component; // Keeps RL on CPU (efficient for inference)
};

#endif // ENMOD_PARALLEL_ENHANCED_HYBRID_DPRL_SOLVER_H