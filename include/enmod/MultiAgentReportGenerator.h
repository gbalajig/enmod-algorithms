#ifndef ENMOD_MULTI_AGENT_REPORT_GENERATOR_H
#define ENMOD_MULTI_AGENT_REPORT_GENERATOR_H

#include "Grid.h"
#include <string>
#include <vector>
#include <fstream>

class MultiAgentReportGenerator {
public:
    MultiAgentReportGenerator(const std::string& path);
    ~MultiAgentReportGenerator();

    // [FIXED] Changed signature to accept vector instead of map to match Controller
    void add_timestep(int time_step, const Grid& grid, const std::vector<Position>& agent_positions);
    
    void finalize_report();

private:
    std::ofstream report_file;
};

#endif // ENMOD_MULTI_AGENT_REPORT_GENERATOR_H