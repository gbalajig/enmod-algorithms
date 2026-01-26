#ifndef ENMOD_ENHANCED_HYBRID_DPRL_SOLVER_H
#define ENMOD_ENHANCED_HYBRID_DPRL_SOLVER_H

#include "Solver.h"
#include "Grid.h"
#include "InterlacedSolver.h" // The advanced DP component
#include "DQNSolver.h"        // The advanced RL component
#include <vector>
#include <memory>

class EnhancedHybridDPRLSolver : public Solver {
public:
    EnhancedHybridDPRLSolver(const Grid& grid);
    virtual ~EnhancedHybridDPRLSolver() = default;

    void run() override;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

private:
    // We compose the specialized solvers to leverage their logic
    std::unique_ptr<InterlacedSolver> dp_guide;
    std::unique_ptr<DQNSolver> rl_navigator;

    std::vector<Position> path;
    Cost total_cost;

    // Helper to blend the logic
    void executeHybridNavigation();
};

#endif // ENMOD_ENHANCED_HYBRID_DPRL_SOLVER_H