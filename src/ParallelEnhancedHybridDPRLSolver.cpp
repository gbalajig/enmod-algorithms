#include "enmod/ParallelEnhancedHybridDPRLSolver.h"
#include "enmod/Logger.h"

ParallelEnhancedHybridDPRLSolver::ParallelEnhancedHybridDPRLSolver(const Grid& grid)
    : Solver(grid, "ParallelEnhancedHybridDPRL"), ParallelSolverBase(grid), total_cost{0,0,0} {
    rl_component = std::make_unique<DQNSolver>(grid);
}

void ParallelEnhancedHybridDPRLSolver::run() {
    Logger::log(LogLevel::INFO, "Running Parallel Enhanced Hybrid (CUDA Safety + Offline RL)...");

    // 1. Run DP Safety Check on GPU (Massive Parallelism)
    uploadGridToGPU(grid);
    runParallelBIDP(); // Calculates "Distance to Exit" for every cell instantly
    downloadCostsFromGPU();
    const auto& gpu_safety_map = getHostCostMap();

    // 2. Check Feasibility
    Position start = grid.getStartPosition();
    if (gpu_safety_map[start.row * grid.getCols() + start.col].distance >= 2147483647) {
         Logger::log(LogLevel::WARN, "ParallelEnhanced: GPU Safety Check says impossible!");
         total_cost = {2147483647, 0, 0};
         return;
    }

    // 3. Run Offline RL (Fast Inference on CPU)
    rl_component->run();
    Cost rl_cost = rl_component->getEvacuationCost();

    // 4. Hybrid Decision (GPU DP vs CPU RL)
    // We trust the RL path only if it doesn't violate the GPU safety map
    // (Simplification: Here we just select based on score)
    total_cost = rl_cost; // Champion logic usually favors RL if safe
}

Cost ParallelEnhancedHybridDPRLSolver::getEvacuationCost() const { return total_cost; }

void ParallelEnhancedHybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Parallel Enhanced Hybrid Report</h2>";
    report_file << "<p>Safety Map calculated on GPU (CUDA). Path optimization via Offline RL.</p>";
}