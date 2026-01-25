#include "enmod/HtmlReportGenerator.h"
#include <fstream>
#include <map>
#include <algorithm>
#include <vector>
#include <iomanip>
#include <iostream>
#include <set>

// --- Internal Helpers (Hidden from Linker) ---
namespace {

    struct SolverCategory {
        std::string processing;  
        std::string environment; 
        std::string algorithm;   
    };

    // [UPDATED] Detailed descriptions for all algorithms
    std::string getDescription(const std::string& name) {
        // --- Dynamic Enhanced Hybrid ---
        if (name == "DynamicEnhancedHybridDPRL") return "Offline DQN + Real-time BIDP (Robust)";
        if (name == "EnhancedHybridDPRL") return "Offline DQN + Interlaced DP (Optimized)";
        
        // --- Core DP Algorithms ---
        if (name == "FIDP" || name == "DynamicFIDPSim") return "Forward Iterative Dynamic Programming";
        if (name == "BIDP" || name == "DynamicBIDPSim") return "Backward Iterative Dynamic Programming";
        if (name == "Interlaced" || name == "DynamicInterlaced" || name == "InterlacedSim" || name == "DynamicInterlacedSim") 
            return "Real-Time Bi-Directional DP";
        
        // --- Advanced DP ---
        if (name == "ADAStar") return "Anytime Dynamic A* Search";
        if (name == "API" || name == "DynamicAPISim") return "Approximate Policy Iteration";
        if (name == "AVI" || name == "DynamicAVISim") return "Approximate Value Iteration";
        if (name == "DStarLiteSim") return "Incremental Heuristic Search";
        if (name == "DynamicHPAStar") return "Hierarchical Path-Finding A*";
        
        // --- Hybrid Strategies ---
        if (name == "AdaptiveCostSim" || name == "DynamicAdaptiveCostSim") return "DP with Dynamic Hazard Weights";
        if (name == "HierarchicalSim" || name == "DynamicHierarchicalSim") return "Multi-Layered Navigation Logic";
        if (name == "HybridDPRLSim" || name == "DynamicHybridDPRLSim") return "Online Q-Learning + BIDP";
        if (name == "PolicyBlendingSim" || name == "DynamicPolicyBlendingSim") return "Weighted RL-DP Decision Mixing";

        // --- Standard Search ---
        if (name == "AStar") return "Heuristic Search (A*)";
        
        // --- Reinforcement Learning ---
        if (name.find("QLearning") != std::string::npos) return "Online Tabular Q-Learning";
        if (name.find("SARSA") != std::string::npos) return "On-Policy TD Control";
        if (name.find("ActorCritic") != std::string::npos) return "Policy & Value Network Learning";
        if (name.find("DQN") != std::string::npos) return "Deep Q-Network (Offline)";
        if (name.find("RLEnhanced") != std::string::npos) return "A* Guided by RL Heuristics";

        return "Algorithmic Path Planner"; // Fallback
    }

    void writeHtmlHeader(std::ofstream& file, const std::string& title) {
        file << "<!DOCTYPE html>\n<html lang='en'>\n<head>\n";
        file << "<meta charset='UTF-8'>\n";
        file << "<meta name='viewport' content='width=device-width, initial-scale=1.0'>\n";
        file << "<title>" << title << "</title>\n";
        file << "<style>\n";
        file << "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 20px; background-color: #f4f4f9; color: #333; }\n";
        file << "h1, h2 { color: #444; border-bottom: 2px solid #ddd; padding-bottom: 10px; }\n";
        file << "table { border-collapse: collapse; width: 100%; margin-bottom: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }\n";
        file << "th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }\n";
        file << "th { background-color: #007bff; color: white; }\n";
        
        file << ".header-level-1 { background-color: #212529; color: #fff; font-size: 1.2em; font-weight: bold; text-align: left; padding-left: 10px; text-transform: uppercase; letter-spacing: 1px; }\n"; 
        file << ".header-level-2 { background-color: #495057; color: #fff; font-size: 1.0em; font-weight: bold; text-align: left; padding-left: 25px; }\n"; 
        file << ".header-level-3 { background-color: #e9ecef; color: #495057; font-weight: bold; text-align: left; padding-left: 40px; font-style: italic; border-left: 5px solid #007bff; }\n"; 
        file << ".desc-col { text-align: left; font-style: italic; font-size: 0.9em; color: #555; background-color: #fdfdfd; }\n";

        file << ".container { max-width: 1400px; margin: auto; background: white; padding: 20px; border-radius: 8px; }\n";
        file << ".grid-table { border-spacing: 0; border-collapse: separate; }\n";
        file << ".grid-cell { width: 25px; height: 25px; text-align: center; vertical-align: middle; font-size: 12px; border: 1px solid #ccc; }\n";
        file << ".wall { background-color: #343a40; color: white; }\n";
        file << ".start { background-color: #28a745; color: white; }\n";
        file << ".exit { background-color: #dc3545; color: white; }\n";
        file << ".smoke { background-color: #6c757d; color: white; }\n";
        file << ".fire { background-color: #ffc107; color: black; animation: pulse 1s infinite; }\n";
        file << "@keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.6; } 100% { opacity: 1; } }\n";
        file << "</style>\n</head>\n<body>\n<div class='container'>\n";
    }

    void writeHtmlFooter(std::ofstream& file) {
        file << "</div>\n</body>\n</html>";
    }

    // Strict categorization logic
    SolverCategory categorizeSolver(const std::string& name) {
        SolverCategory cat;
        
        // 1. Processing Type
        if (name.find("Parallel") != std::string::npos) cat.processing = "Parallel Processing (GPU)";
        else cat.processing = "Serial Processing (CPU)";

        // 2. Environment Type
        bool is_explicit_dynamic = (name.find("Dynamic") != std::string::npos || 
                                    name.find("DStar") != std::string::npos || 
                                    name.find("EnhancedHybrid") != std::string::npos);
        cat.environment = is_explicit_dynamic ? "Dynamic Environment" : "Static Environment";

        // 3. Algorithm Class
        
        // A. Hybrid DP-RL
        bool is_hybrid_keyword = (name.find("Adaptive") != std::string::npos || 
                                  name.find("Hierarchical") != std::string::npos ||
                                  name.find("Hybrid") != std::string::npos || 
                                  name.find("Interlaced") != std::string::npos || 
                                  name.find("Blending") != std::string::npos || 
                                  name.find("Policy") != std::string::npos ||
                                  name.find("EnhancedHybrid") != std::string::npos);

        if (is_hybrid_keyword) {
            if (name.find("Dynamic") != std::string::npos) {
                cat.algorithm = "Dynamic Hybrid DP-RL"; 
            } else {
                cat.algorithm = "Static Hybrid DP-RL";
            }
            return cat;
        }

        // B. Reinforcement Learning
        bool is_rl = (name.find("QLearning") != std::string::npos || 
                      name.find("SARSA") != std::string::npos || 
                      name.find("ActorCritic") != std::string::npos || 
                      name.find("DQN") != std::string::npos ||
                      name.find("RLEnhanced") != std::string::npos);
        
        if (is_rl) {
            cat.algorithm = "Reinforcement Learning (RL)";
            return cat;
        }

        // C. Default
        cat.algorithm = "Dynamic Programming (DP)";
        return cat;
    }
} // end anonymous namespace

// --- Class Implementations ---

void HtmlReportGenerator::generateInitialGridReport(const Grid& grid, const std::string& path) {
    std::string file_path = path + "/_Initial_Grid.html";
    std::ofstream report_file(file_path);
    if (!report_file) return;
    writeHtmlHeader(report_file, "Initial Grid - " + grid.getName());
    report_file << "<h1>Initial Grid: " << grid.getName() << " (" << grid.getRows() << "x" << grid.getCols() << ")</h1>\n";
    report_file << grid.toHtmlString();
    writeHtmlFooter(report_file);
}

void HtmlReportGenerator::generateSolverReport(const Solver& solver, const std::string& path) {
    std::string file_path = path + "/" + solver.getName() + "_Report.html";
    std::ofstream report_file(file_path);
    if (!report_file) return;
    writeHtmlHeader(report_file, solver.getName() + " Report");
    report_file << "<h1>" << solver.getName() << " Report</h1>\n";
    solver.generateReport(report_file);
    writeHtmlFooter(report_file);
}

void HtmlReportGenerator::generateSummaryReport(const std::vector<Result>& results, const std::string& path, const std::string& filename) {
    std::string file_path = path + "/" + filename;
    std::ofstream report_file(file_path);
    if (!report_file) return;

    writeHtmlHeader(report_file, "Simulation Summary Report");
    report_file << "<h1>Simulation Summary (" << filename << ")</h1>\n";

    struct Stats {
        int count = 0;
        int failures = 0;
        long long smoke = 0;
        long long time = 0;
        long long dist = 0;
        double w_cost = 0;
        double exec_time = 0;
    };

    std::map<std::string, std::map<std::string, std::map<std::string, std::map<std::string, Stats>>>> hierarchy;
    std::vector<std::string> base_scenarios;
    std::map<std::string, std::map<std::string, Stats>> raw_data;

    for (const auto& res : results) {
        std::string base_name = res.scenario_name;
        size_t last_underscore = base_name.find_last_of('_');
        if (last_underscore != std::string::npos && base_name.substr(last_underscore).rfind("_T", 0) == 0) {
            base_name = base_name.substr(0, last_underscore);
        }

        bool found = false;
        for(const auto& s : base_scenarios) if(s == base_name) found = true;
        if(!found) base_scenarios.push_back(base_name);

        Stats& s = raw_data[base_name][res.solver_name];
        s.count++;
        if (res.cost.distance >= 2147483647 || res.cost.distance < 0) {
            s.failures++;
        } else {
            s.smoke += res.cost.smoke;
            s.time += res.cost.time;
            s.dist += res.cost.distance;
            s.w_cost += res.weighted_cost;
            s.exec_time += res.execution_time;
        }

        SolverCategory cat = categorizeSolver(res.solver_name);
        hierarchy[cat.processing][cat.environment][cat.algorithm][res.solver_name] = Stats(); 
    }

    std::sort(base_scenarios.begin(), base_scenarios.end(), [](const std::string& a, const std::string& b){
        try {
            int size_a = std::stoi(a.substr(0, a.find('x')));
            int size_b = std::stoi(b.substr(0, b.find('x')));
            if (size_a != size_b) return size_a < size_b;
        } catch(...) {}
        return a < b;
    });

    std::vector<std::string> proc_order = {"Serial Processing (CPU)", "Parallel Processing (GPU)"};
    std::vector<std::string> env_order = {"Static Environment", "Dynamic Environment"};
    std::vector<std::string> algo_order = {
        "Dynamic Programming (DP)", 
        "Reinforcement Learning (RL)", 
        "Static Hybrid DP-RL", 
        "Dynamic Hybrid DP-RL" 
    };

    int total_colspan = 2 + (base_scenarios.size() * 5);

    report_file << "<table>\n";
    report_file << "<thead><tr><th rowspan='2'>Hierarchy / Algorithm</th><th rowspan='2'>Description</th>";
    
    for(const auto& scn : base_scenarios){
        report_file << "<th colspan='5'>" << scn << "</th>";
    }
    report_file << "</tr>\n<tr>";
    
    for(size_t i = 0; i < base_scenarios.size(); ++i) report_file << "<th>Smoke</th><th>Time</th><th>Dist</th><th>W. Cost</th><th>Exec(ms)</th>";
    report_file << "</tr></thead>\n<tbody>";

    for (const auto& proc : proc_order) {
        if (hierarchy.find(proc) == hierarchy.end()) continue;
        report_file << "<tr><td colspan='" << total_colspan << "' class='header-level-1'>" << proc << "</td></tr>";

        for (const auto& env : env_order) {
            if (hierarchy[proc].find(env) == hierarchy[proc].end()) continue;
            report_file << "<tr><td colspan='" << total_colspan << "' class='header-level-2'>" << env << "</td></tr>";

            for (const auto& algo : algo_order) {
                if (hierarchy[proc][env].find(algo) == hierarchy[proc][env].end()) continue;
                report_file << "<tr><td colspan='" << total_colspan << "' class='header-level-3'>" << algo << "</td></tr>";

                for (auto const& [solver_name, _] : hierarchy[proc][env][algo]) {
                    report_file << "<tr><td>" << solver_name << "</td>";
                    
                    report_file << "<td class='desc-col'>" << getDescription(solver_name) << "</td>";

                    for(const auto& scn : base_scenarios){
                        if (raw_data.find(scn) == raw_data.end() || raw_data[scn].find(solver_name) == raw_data[scn].end()) {
                            report_file << "<td colspan='5' style='color:gray'>Not Run</td>";
                            continue;
                        }
                        
                        const auto& stats = raw_data[scn][solver_name];
                        int successful_runs = stats.count - stats.failures;
                        if (successful_runs > 0) {
                            report_file << "<td>" << (stats.smoke / successful_runs) << "</td>";
                            report_file << "<td>" << (stats.time / successful_runs) << "</td>";
                            report_file << "<td>" << (stats.dist / successful_runs) << "</td>";
                            report_file << "<td>" << static_cast<long long>(stats.w_cost / successful_runs) << "</td>";
                            report_file << "<td>" << std::fixed << std::setprecision(2) << (stats.exec_time / successful_runs) << "</td>";
                        } else {
                            report_file << "<td colspan='5' style='color:red'>ALL FAILED</td>";
                        }
                    }
                    report_file << "</tr>\n";
                }
            }
        }
    }
    report_file << "</tbody></table>\n";
    writeHtmlFooter(report_file);
}