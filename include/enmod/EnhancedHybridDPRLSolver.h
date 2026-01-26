#ifndef ENMOD_ENHANCED_HYBRID_DP_RL_SOLVER_H
#define ENMOD_ENHANCED_HYBRID_DP_RL_SOLVER_H

#include "DynamicSolver.h"
#include "Types.h"
#include "InterlacedSolver.h"
#include "DQNSolver.h" 
#include <memory>

class EnhancedHybridDPRLSolver : public Solver {
public:
    EnhancedHybridDPRLSolver(const Grid& grid_ref);
    void run() override;
    Direction getNextMove(const Position& current_pos, const Grid& current_grid); 
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

private:
    std::vector<StepReport> history;
    Cost total_cost;
    EvacuationMode current_mode;

    // Advanced RL Agent (DQN)
    std::unique_ptr<DQNSolver> rl_solver;

    void assessThreatAndSetMode(const Position& current_pos, const Grid& current_grid);
};

#endif // ENMOD_ENHANCED_HYBRID_DP_RL_SOLVER_H