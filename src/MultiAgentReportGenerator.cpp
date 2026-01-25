#include "enmod/MultiAgentReportGenerator.h"
#include <iostream>
#include <iomanip>
#include <sstream>

// --- Internal Helper Functions ---
namespace {
    void writeMultiAgentHeader(std::ofstream& file, const std::string& title) {
        file << "<!DOCTYPE html>\n<html lang='en'>\n<head>\n";
        file << "<meta charset='UTF-8'>\n";
        file << "<title>" << title << "</title>\n";
        file << "<style>\n";
        file << "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 20px; background-color: #f4f4f9; color: #333; }\n";
        file << "h1, h2 { color: #444; border-bottom: 2px solid #ddd; padding-bottom: 10px; }\n";
        file << "table { border-collapse: collapse; width: 100%; margin-bottom: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }\n";
        file << "th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }\n";
        file << "th { background-color: #007bff; color: white; }\n";
        
        file << ".grid-table { border-spacing: 0; border-collapse: separate; }\n";
        file << ".grid-cell { width: 25px; height: 25px; text-align: center; vertical-align: middle; font-size: 12px; border: 1px solid #ccc; }\n";
        file << ".wall { background-color: #343a40; color: white; }\n";
        file << ".start { background-color: #28a745; color: white; }\n"; 
        file << ".exit { background-color: #dc3545; color: white; }\n";
        file << ".smoke { background-color: #6c757d; color: white; }\n";
        file << ".fire { background-color: #ffc107; color: black; animation: pulse 1s infinite; }\n";
        file << "</style>\n</head>\n<body>\n<div class='container'>\n";
    }

    void writeMultiAgentFooter(std::ofstream& file) {
        file << "</div>\n</body>\n</html>";
    }
}

// --- Class Implementation ---

MultiAgentReportGenerator::MultiAgentReportGenerator(const std::string& path) {
    std::string file_path = path + "/MultiAgent_Report.html";
    report_file.open(file_path);
    if (report_file.is_open()) {
        writeMultiAgentHeader(report_file, "Multi-Agent Simulation Report");
        report_file << "<h1>Multi-Agent Simulation Report</h1>\n";
    } else {
        std::cerr << "Error: Could not open report file: " << file_path << std::endl;
    }
}

MultiAgentReportGenerator::~MultiAgentReportGenerator() {
    finalize_report();
}

// [FIXED] Now accepts std::vector<Position>
void MultiAgentReportGenerator::add_timestep(int time_step, const Grid& grid, const std::vector<Position>& agent_positions) {
    if (!report_file.is_open()) return;

    report_file << "<h3>Time Step: " << time_step << "</h3>\n";
    
    std::stringstream ss;
    ss << "<table class='grid-table'>\n";
    for (int r = 0; r < grid.getRows(); ++r) {
        ss << "<tr>\n";
        for (int c = 0; c < grid.getCols(); ++c) {
            std::string cell_class = "grid-cell";
            std::string content = "";
            Position current_pos = {r, c};

            // [UPDATED Logic] Check vector for agents
            bool agent_here = false;
            for (size_t i = 0; i < agent_positions.size(); ++i) {
                const auto& pos = agent_positions[i];
                if (pos.row == r && pos.col == c) {
                    cell_class += " start"; 
                    content = "A" + std::to_string(i); // Use index as Agent ID
                    agent_here = true;
                    break;
                }
            }

            if (!agent_here) {
                if (grid.getCellType(current_pos) == CellType::WALL) {
                    cell_class += " wall";
                } else if (grid.getCellType(current_pos) == CellType::EXIT) {
                    cell_class += " exit";
                    content = "E";
                } else if (grid.getCellType(current_pos) == CellType::FIRE) {
                    cell_class += " fire";
                    content = "F";
                } else if (grid.getCellType(current_pos) == CellType::SMOKE) {
                    cell_class += " smoke";
                    content = "S";
                }
            }
            
            ss << "<td class='" << cell_class << "'>" << content << "</td>";
        }
        ss << "</tr>\n";
    }
    ss << "</table>\n";
    
    report_file << ss.str();
    report_file << "<hr>\n";
}

void MultiAgentReportGenerator::finalize_report() {
    if (report_file.is_open()) {
        writeMultiAgentFooter(report_file);
        report_file.close();
    }
}