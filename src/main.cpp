#include "enmod/Logger.h"
#include "enmod/ScenarioGenerator.h"
#include "enmod/Grid.h"
#include "enmod/Solver.h"
#include "enmod/HtmlReportGenerator.h"
#include "enmod/MultiAgentCPSController.h"

// --- 1. Static Solvers ---
#include "enmod/BIDP.h"
#include "enmod/FIDP.h"
#include "enmod/AStarSolver.h"
#include "enmod/AVI.h"
#include "enmod/API.h"
#include "enmod/ADASolver.h"

// --- 2. RL / AI Solvers ---
#include "enmod/QLearningSolver.h"
#include "enmod/SARSASolver.h"
#include "enmod/ActorCriticSolver.h"
#include "enmod/RLEnhancedAStarSolver.h"
#include "enmod/DQNSolver.h"

// --- 3. Dynamic Solvers ---
#include "enmod/DynamicBIDPSolver.h"
#include "enmod/DynamicFIDPSolver.h"
#include "enmod/DynamicAVISolver.h" 
#include "enmod/DynamicAPISolver.h"
#include "enmod/DynamicQLearningSolver.h"
#include "enmod/DynamicSARSASolver.h"
#include "enmod/DynamicActorCriticSolver.h"
#include "enmod/DynamicHPASolver.h"
#include "enmod/DStarLiteSolver.h"

// --- 4. EnMod-DP Hybrid Solvers (Static Baselines) ---
#include "enmod/InterlacedSolver.h"
#include "enmod/HybridDPRLSolver.h"
#include "enmod/AdaptiveCostSolver.h"
#include "enmod/HierarchicalSolver.h"
#include "enmod/PolicyBlendingSolver.h"

// --- 5. EnMod-DP Dynamic Hybrid Solvers (The New "Defense-Ready" Classes) ---
#include "enmod/DynamicHybridDPRLSolver.h"   // [NEW]
#include "enmod/DynamicInterlacedSolver.h"   // [NEW]
#include "enmod/DynamicAdaptiveCostSolver.h" // [NEW]
#include "enmod/DynamicHierarchicalSolver.h"  // [NEW]
#include "enmod/DynamicPolicyBlendingSolver.h"// [NEW]

// --- 6. Parallel Solvers (CUDA GPU) ---
#include "enmod/ParallelBIDP.h"
#include "enmod/ParallelStaticSolvers.h" 
#include "enmod/ParallelDynamicBIDPSolver.h"
#include "enmod/ParallelDynamicAVISolver.h"
#include "enmod/ParallelHybridSolvers.h" 

#include "enmod/EnhancedHybridDPRLSolver.h"

#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <memory>
#include <filesystem>
#include <map>
#include <chrono>
#include <sstream>
#include <thread>

bool isGpuAvailable() {
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    return (error == cudaSuccess && deviceCount > 0);
}

void runComparisonScenario(const json& config, const std::string& report_path, std::vector<Result>& results) {
    Grid grid(config);
    HtmlReportGenerator::generateInitialGridReport(grid, report_path);
    std::vector<std::unique_ptr<Solver>> solvers;

    // --- 1. Static ---
    solvers.push_back(std::make_unique<FIDP>(grid));   
    solvers.push_back(std::make_unique<BIDP>(grid));   
    solvers.push_back(std::make_unique<AStarSolver>(grid)); 
    solvers.push_back(std::make_unique<AVI>(grid));
    solvers.push_back(std::make_unique<API>(grid));
    solvers.push_back(std::make_unique<ADASolver>(grid));

    // --- 2. RL / AI ---
    solvers.push_back(std::make_unique<QLearningSolver>(grid));
    solvers.push_back(std::make_unique<SARSASolver>(grid));
    solvers.push_back(std::make_unique<ActorCriticSolver>(grid));
    solvers.push_back(std::make_unique<RLEnhancedAStarSolver>(grid));
    solvers.push_back(std::make_unique<DQNSolver>(grid));

    // --- 3. Dynamic (Pure Math) ---
    solvers.push_back(std::make_unique<DynamicBIDPSolver>(grid));
    solvers.push_back(std::make_unique<DynamicFIDPSolver>(grid)); 
    solvers.push_back(std::make_unique<DynamicAVISolver>(grid));
    solvers.push_back(std::make_unique<DynamicAPISolver>(grid));
    solvers.push_back(std::make_unique<DynamicQLearningSolver>(grid));
    solvers.push_back(std::make_unique<DynamicSARSASolver>(grid));
    solvers.push_back(std::make_unique<DynamicActorCriticSolver>(grid));
    solvers.push_back(std::make_unique<DynamicHPASolver>(grid));
    solvers.push_back(std::make_unique<DStarLiteSolver>(grid));   

    // --- 4. EnMod-DP (Static Baselines) ---
    solvers.push_back(std::make_unique<InterlacedSolver>(grid));       
    solvers.push_back(std::make_unique<HybridDPRLSolver>(grid));       
    solvers.push_back(std::make_unique<AdaptiveCostSolver>(grid));     
    solvers.push_back(std::make_unique<HierarchicalSolver>(grid));     
    solvers.push_back(std::make_unique<PolicyBlendingSolver>(grid));   

    // --- 5. EnMod-DP (Dynamic / Real-Time - PROPOSED) ---
    solvers.push_back(std::make_unique<DynamicInterlacedSolver>(grid)); 
    solvers.push_back(std::make_unique<DynamicHybridDPRLSolver>(grid));
    solvers.push_back(std::make_unique<DynamicAdaptiveCostSolver>(grid));
    solvers.push_back(std::make_unique<DynamicHierarchicalSolver>(grid));
    solvers.push_back(std::make_unique<DynamicPolicyBlendingSolver>(grid)); 

    // --- 6. Parallel ---
    if (isGpuAvailable()) {
        solvers.push_back(std::make_unique<ParallelFIDP>(grid)); 
        solvers.push_back(std::make_unique<ParallelBIDP>(grid)); 
        solvers.push_back(std::make_unique<ParallelAVI>(grid)); 
        solvers.push_back(std::make_unique<ParallelDynamicBIDPSolver>(grid));
        solvers.push_back(std::make_unique<ParallelDynamicAVISolver>(grid));
        solvers.push_back(std::make_unique<ParallelInterlacedSolver>(grid));
        solvers.push_back(std::make_unique<ParallelHybridDPRLSolver>(grid));
        solvers.push_back(std::make_unique<ParallelAdaptiveCostSolver>(grid));
        solvers.push_back(std::make_unique<ParallelHierarchicalSolver>(grid));
        solvers.push_back(std::make_unique<ParallelPolicyBlendingSolver>(grid));
    }

    // [THE CHAMPION ALGORITHM]
    solvers.push_back(std::make_unique<EnhancedHybridDPRLSolver>(grid));

    for (const auto& solver : solvers) {
        if (isGpuAvailable()) cudaDeviceSynchronize();
        auto start_time = std::chrono::high_resolution_clock::now();
        solver->run();
        if (isGpuAvailable()) cudaDeviceSynchronize();
        auto end_time = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double, std::milli> execution_time = end_time - start_time;

        std::cout << "   -> [Algo] " << std::left << std::setw(30) << solver->getName() 
                  << ": " << std::fixed << std::setprecision(2) << execution_time.count() << " ms" << std::endl;

        Cost final_cost = solver->getEvacuationCost();
        double weighted_cost = (final_cost.distance == MAX_COST) 
                               ? std::numeric_limits<double>::infinity() 
                               : (final_cost.smoke * 1000) + (final_cost.time * 10) + (final_cost.distance * 1);
        
        results.push_back({grid.getName(), solver->getName(), final_cost, weighted_cost, execution_time.count()});
        HtmlReportGenerator::generateSolverReport(*solver, report_path);
    }
}

int main() {
    try {
        std::stringstream ss;
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        std::string run_timestamp = ss.str();
        std::string root_folder = "run_" + run_timestamp;

        std::filesystem::create_directory(root_folder);
        std::string log_file = root_folder + "/enmod_simulation.log";
        Logger::init(log_file);

        std::cout << "========================================================\n";
        std::cout << " EnMod-DP Benchmark Suite (Algo + CPS)\n";
        std::cout << " Output Directory: " << root_folder << "\n";
        std::cout << "========================================================\n";

        std::vector<int> grid_sizes = {10, 20};      
        double density = 0.15; 

        std::vector<Result> global_results;

        for (int size : grid_sizes) {
            std::cout << "\n[Processing Grid Size: " << size << "x" << size << "]\n";
            
            std::string grid_folder_name = std::to_string(size) + "x" + std::to_string(size);
            std::string grid_path = root_folder + "/" + grid_folder_name;
            std::filesystem::create_directory(grid_path);

            std::string agent_io_path = grid_path + "/agent_io";
            std::filesystem::create_directory(agent_io_path);

            std::string scenario_name = grid_folder_name + "_Benchmark";
            json config = ScenarioGenerator::generate(size, scenario_name, density);

            // 1. Run Standard Algorithms Benchmark
            std::cout << "   -> Phase 1: Running Algorithm Comparisons...\n";
            std::vector<Result> grid_results;
            runComparisonScenario(config, grid_path, grid_results);
            HtmlReportGenerator::generateSummaryReport(grid_results, grid_path, "_Summary_Report.html");
            std::cout << "   -> Summary Report Generated.\n";

            global_results.insert(global_results.end(), grid_results.begin(), grid_results.end());

            // 2. Run Multi-Agent CPS Simulation
            std::cout << "   -> Phase 2: Running Multi-Agent CPS Simulation...\n";
            MultiAgentCPSController cps_controller(config, grid_path, 5); // 5 Agents
            cps_controller.run_simulation();
        }

        std::cout << "\n[Generating Master Global Comparison Report]...\n";
        HtmlReportGenerator::generateSummaryReport(global_results, root_folder, "_Global_Comparison_Report.html");
        std::cout << "   -> Report saved to: " << root_folder << "/_Global_Comparison_Report.html\n";

        std::cout << "\n========================================================\n";
        std::cout << " Benchmark Complete.\n";
        std::cout << "========================================================\n";
        Logger::close();

    } catch (const std::exception& e) {
        std::cerr << "Critical Error: " << e.what() << std::endl;
        Logger::log(LogLevel::ERROR, "Critical Error: " + std::string(e.what()));
        return 1;
    }
    return 0;
}