#include "enmod/MultiAgentReportGenerator.h"
#include "enmod/HtmlReportGenerator.h" // For writeHtmlHeader and writeHtmlFooter
#include <sstream> // For std::stringstream

MultiAgentReportGenerator::MultiAgentReportGenerator(const std::string& filename) {
    report_file.open(filename);
    writeHtmlHeader(report_file, "Multi-Agent Simulation Replay");
// Add CSS for Animated Hazards and Entry points
    report_file << "<style>\n"
                << ".grid-table { border-collapse: collapse; margin-bottom: 20px; }\n"
                << ".grid-cell { width: 30px; height: 30px; border: 1px solid #ccc; text-align: center; font-size: 12px; }\n"
                << ".wall { background-color: #2c3e50; color: white; }\n"
                << ".exit { background-color: #27ae60; color: white; font-weight: bold; }\n"
                << ".entry { background-color: #2980b9; color: white; font-weight: bold; }\n" // Support for "IN" labels
                << ".path { background-color: #3498db; color: white; border-radius: 50%; }\n"
                << ".fire { \n"
                << "    background-color: #e74c3c; \n"
                << "    animation: flicker 0.5s infinite alternate; \n"
                << "}\n"
                << ".smoke { \n"
                << "    background-color: #95a5a6; \n"
                << "    opacity: 0.8; \n"
                << "    animation: drift 2s infinite linear; \n"
                << "}\n"
                << "@keyframes flicker {\n"
                << "    from { background-color: #e74c3c; box-shadow: 0 0 5px #c0392b; }\n"
                << "    to { background-color: #f1c40f; box-shadow: 0 0 15px #f39c12; }\n"
                << "}\n"
                << "@keyframes drift {\n"
                << "    from { transform: scale(1); opacity: 0.8; }\n"
                << "    to { transform: scale(1.1); opacity: 0.6; }\n"
                << "}\n"
                << "</style>\n";
    report_file << "<h1>Multi-Agent Evacuation Replay</h1>\n";
}

void MultiAgentReportGenerator::add_timestep(int timestep, const Grid& grid, const std::vector<Position>& agent_positions) {
    report_file << "<h2>Timestep: " << timestep << "</h2>\n";
    report_file << toHtmlStringWithMultipleAgents(grid, agent_positions);
}

void MultiAgentReportGenerator::finalize_report() {
    writeHtmlFooter(report_file);
    report_file.close();
}

std::string MultiAgentReportGenerator::toHtmlStringWithMultipleAgents(const Grid& grid, const std::vector<Position>& agent_positions) const {
    std::stringstream ss;
    ss << "<table class='grid-table'><tbody>";
    for(int r = 0; r < grid.getRows(); ++r){
        ss << "<tr>";
        for(int c = 0; c < grid.getCols(); ++c){
            std::string content = "";
            bool agent_found = false;
            for(size_t i = 0; i < agent_positions.size(); ++i) {
                if (r == agent_positions[i].row && c == agent_positions[i].col) {
                    content = "A" + std::to_string(i + 1);
                    agent_found = true;
                    break;
                }
            }

            std::string class_name;
            if (!agent_found) {
                CellType cell_type = grid.getCellType({r, c});
                switch (cell_type) {
                    case CellType::WALL: class_name = "wall"; content = "W"; break;
                //    case CellType::START: class_name = "start"; content = "S"; break;
                    case CellType::START: class_name = "entry"; content = "IN"; break;
                    case CellType::EXIT: class_name = "exit"; content = "E"; break;
                    case CellType::SMOKE: class_name = "smoke"; content = "~"; break;
                    case CellType::FIRE: class_name = "fire"; content = "F"; break;
                    default: break;
                }
            } else {
                class_name = "path";
            }
            
            ss << "<td class='grid-cell " << class_name << "'>" << content << "</td>";
        }
        ss << "</tr>";
    }
    ss << "</tbody></table>";
    return ss.str();
}