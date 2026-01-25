#ifndef ENMOD_DYNAMIC_ENHANCED_HYBRID_DPRL_SOLVER_H
#define ENMOD_DYNAMIC_ENHANCED_HYBRID_DPRL_SOLVER_H

#include "Solver.h" 
#include "Grid.h"
#include "Types.h"
#include <vector>
#include <map>
#include <string>

// Local reporting structure
struct HybridStepReport {
    int time_step;
    Grid grid_state;
    Position agent_pos;
    std::string action;
    Cost current_total_cost;
    EvacuationMode mode;
};

class DynamicEnhancedHybridDPRLSolver : public Solver {
public:
    DynamicEnhancedHybridDPRLSolver(const Grid& grid);
    virtual ~DynamicEnhancedHybridDPRLSolver() = default;

    void run() override;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

    // [NEW] Required for Multi-Agent CPS Controller
    // Note: We do not mark as 'override' to avoid errors if the base Solver 
    // class doesn't define it virtual. We just provide the method.
    Direction getNextMove(const Position& current, const Grid& view);

private:
    std::vector<HybridStepReport> history;
    Cost total_cost;
    
    std::map<Position, std::vector<double>> q_table;
    bool model_loaded;

    bool loadOfflineModel(const std::string& filename);
    int getRLAction(const Position& pos);
};

#endif // ENMOD_DYNAMIC_ENHANCED_HYBRID_DPRL_SOLVER_H