#ifndef ENMOD_HTML_REPORT_GENERATOR_H
#define ENMOD_HTML_REPORT_GENERATOR_H

#include "Grid.h"
#include "Solver.h" 
#include <string>
#include <vector>
#include <fstream> // Required for std::ofstream used in global helpers

// [MOVED HERE] Result struct definition. 
// Since it is missing from Solver.h, we define it here for reporting purposes.
struct Result {
    std::string scenario_name;
    std::string solver_name;
    Cost cost;
    double weighted_cost;
    double execution_time;
};

// [FIX] Forward declarations for global helpers to resolve LNK2019 linker errors.
// These are defined in HtmlReportGenerator.cpp and used by MultiAgentReportGenerator.
void writeHtmlHeader(std::ofstream& file, const std::string& title);
void writeHtmlFooter(std::ofstream& file);

class HtmlReportGenerator {
public:
    static void generateInitialGridReport(const Grid& grid, const std::string& path);
    static void generateSolverReport(const Solver& solver, const std::string& path);
    
    // Now valid because 'Result' is defined above.
    // Default filename parameter matches the usage in main.cpp.
    static void generateSummaryReport(const std::vector<Result>& results, 
                                      const std::string& path, 
                                      const std::string& filename = "_Summary_Report.html");
};

#endif // ENMOD_HTML_REPORT_GENERATOR_H