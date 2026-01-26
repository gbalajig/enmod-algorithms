#include "enmod/EnhancedHybridDPRLSolver.h"
#include "enmod/Logger.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits> 

// [FIX 1] Use brace initialization {0,0,0}. 
// This works for both aggregate structs and classes with constructors.
EnhancedHybridDPRLSolver::EnhancedHybridDPRLSolver(const Grid& grid) 
    : Solver(grid, "EnhancedHybridDPRL"), 
      total_cost{0, 0, 0} { 
    
    // Initialize components
    dp_guide = std::make_unique<InterlacedSolver>(grid);
    rl_navigator = std::make_unique<DQNSolver>(grid);
}

void EnhancedHybridDPRLSolver::run() {
    Logger::log(LogLevel::INFO, "Running Enhanced Hybrid DP-RL (Interlaced + DQN)...");

    // PHASE 1: Run Interlaced DP (FIDP + BIDP)
    dp_guide->run();
    Cost dp_cost = dp_guide->getEvacuationCost();

    // Use local INT_MAX to avoid dependency on global MAX_COST macro if undefined
    int max_val = 2147483647; 

    // [FIX 2] Changed LogLevel::WARNING to LogLevel::WARN
    // If WARN is still invalid, it will error again, but WARN is the standard counterpart to INFO/ERROR.
    if (dp_cost.distance >= max_val) {
        Logger::log(LogLevel::WARN, "EnhancedHybrid: DP failed to find a path. RL cannot proceed safely.");
        total_cost = {max_val, max_val, max_val};
        return;
    }

    // PHASE 2: Run DQN
    rl_navigator->run();
    Cost rl_cost = rl_navigator->getEvacuationCost();

    // PHASE 3: Hybrid Decision
    double dp_score = dp_cost.distance * 1.0 + dp_cost.smoke * 10.0;
    double rl_score = rl_cost.distance * 1.0 + rl_cost.smoke * 10.0;

    if (rl_cost.distance < max_val && rl_score <= dp_score) {
        total_cost = rl_cost;
        Logger::log(LogLevel::INFO, "EnhancedHybrid: Selected DQN Strategy (Optimized Path).");
    } else {
        total_cost = dp_cost;
        Logger::log(LogLevel::INFO, "EnhancedHybrid: Selected Interlaced DP Strategy (Robust Path).");
    }
}

Cost EnhancedHybridDPRLSolver::getEvacuationCost() const {
    return total_cost;
}

void EnhancedHybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h3>Enhanced Hybrid DP-RL Report</h3>";
    report_file << "<p><strong>Components:</strong> Interlaced DP (Static Robustness) + DQN (Dynamic Optimization)</p>";
    
    report_file << "<h4>Phase 1: Interlaced DP Guide</h4>";
    dp_guide->generateReport(report_file);
    
    report_file << "<h4>Phase 2: DQN Navigator</h4>";
    rl_navigator->generateReport(report_file);
    
    report_file << "<h4>Final Hybrid Decision</h4>";
    report_file << "<p>Final Selected Cost: Distance=" << total_cost.distance 
                << ", Time=" << total_cost.time 
                << ", Smoke=" << total_cost.smoke << "</p>";
}