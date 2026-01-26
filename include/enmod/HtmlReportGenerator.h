#ifndef ENMOD_HTML_REPORT_GENERATOR_H
#define ENMOD_HTML_REPORT_GENERATOR_H

#include "Grid.h"
#include "Solver.h" // [FIX] Include Solver.h to get the 'Result' struct definition
#include <string>
#include <vector>

class HtmlReportGenerator {
public:
    static void generateInitialGridReport(const Grid& grid, const std::string& path);
    static void generateSolverReport(const Solver& solver, const std::string& path);
    static void generateSummaryReport(const std::vector<Result>& results, const std::string& path, const std::string& filename);
};

#endif // ENMOD_HTML_REPORT_GENERATOR_H