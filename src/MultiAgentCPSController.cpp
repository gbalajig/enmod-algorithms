#include "enmod/MultiAgentCPSController.h"
#include "enmod/Logger.h"
#include "enmod/SimulatedSensorNetwork.h"
#include "enmod/RealTimeSensorNetwork.h" 
#include <fstream>
#include <iostream>
#include <filesystem>
#include <string>
#include <map>
#include <iomanip>
#include <cstdlib> 

MultiAgentCPSController::MultiAgentCPSController(const json& initial_config, const std::string& report_path, int num_agents)
    : master_grid(initial_config), report_generator(report_path + "/multi_agent_report.html"), report_path(report_path) {
    std::filesystem::create_directory(report_path + "/agent_io");
    Position start_pos = master_grid.getStartPosition();
    for (int i = 0; i < num_agents; ++i) {
        agents.push_back({"agent_" + std::to_string(i), {start_pos.row + i, start_pos.col}});
    }
    solver = std::make_unique<HybridDPRLSolver>(master_grid);
}

void MultiAgentCPSController::write_input_file(int timestep, const Agent& agent) {
    json input_data;
    input_data["agent_id"] = agent.id;
    input_data["timestep"] = timestep;
    input_data["current_position"] = {{"row", agent.position.row}, {"col", agent.position.col}};
    input_data["environment_update"] = master_grid.getConfig();
    std::ofstream o(report_path + "/agent_io/" + agent.id + "_input_t" + std::to_string(timestep) + ".json");
    o << std::setw(4) << input_data << std::endl;
}

Direction MultiAgentCPSController::read_output_file(int timestep, const Agent& agent) {
    std::ifstream i(report_path + "/agent_io/" + agent.id + "_output_t" + std::to_string(timestep) + ".json");
    json output_data;
    if (i.is_open()) {
        try {
            i >> output_data;
            std::string move = output_data.at("next_move");
            if (move == "UP") return Direction::UP;
            if (move == "DOWN") return Direction::DOWN;
            if (move == "LEFT") return Direction::LEFT;
            if (move == "RIGHT") return Direction::RIGHT;
        } catch (...) { return Direction::STAY; }
    }
    return Direction::STAY;
}

void MultiAgentCPSController::run_simulation() {
    std::cout << "\n   [CPS] Starting Simulation...\n";
    bool use_real_data = true; 
    std::string live_data_file = "data/live_sensors.json";
    std::unique_ptr<ISensorNetwork> sensor_network;
    if (use_real_data) sensor_network = std::make_unique<RealTimeSensorNetwork>(live_data_file);
    else sensor_network = std::make_unique<SimulatedSensorNetwork>(master_grid, &agents);

    int max_timesteps = 2 * (master_grid.getRows() * master_grid.getCols());
    for (int t = 0; t < max_timesteps; ++t) {
        for (const auto& event_cfg : master_grid.getConfig().value("dynamic_events", json::array())) {
            if (event_cfg.value("time_step", -1) == t) master_grid.addHazard(event_cfg);
        }
        std::vector<SensorReading> readings = sensor_network->getAllReadings(t);
        Grid digital_twin_grid(master_grid.getConfig()); 
        digital_twin_grid.updateFromSensors(readings);   
        std::vector<Position> current_positions;
        for (const auto& agent : agents) current_positions.push_back(agent.position);
        report_generator.add_timestep(t, master_grid, current_positions); 

        bool all_exited = true;
        for (auto& agent : agents) {
            if (master_grid.isExit(agent.position.row, agent.position.col)) continue;
            all_exited = false;
            write_input_file(t, agent);
            Grid agent_view = digital_twin_grid;
            for (const auto& other : agents) {
                if (agent.id != other.id) agent_view.setCellUnwalkable(other.position);
            }
            Direction next_move = solver->getNextMove(agent.position, agent_view);
            json out_json;
            out_json["agent_id"] = agent.id;
            out_json["next_move"] = (next_move == Direction::UP) ? "UP" : (next_move == Direction::DOWN) ? "DOWN" :
                                   (next_move == Direction::LEFT) ? "LEFT" : (next_move == Direction::RIGHT) ? "RIGHT" : "STAY";
            std::ofstream o(report_path + "/agent_io/" + agent.id + "_output_t" + std::to_string(t) + ".json");
            o << out_json << std::endl;

            // SAFETY VALIDATOR
            Direction received_move = read_output_file(t, agent);
            Position next_pos = master_grid.getNextPosition(agent.position, received_move);
            if (master_grid.getCellType(next_pos) == CellType::FIRE || 
                master_grid.getCellType(agent.position) == CellType::FIRE ||
                !master_grid.isWalkable(next_pos.row, next_pos.col)) {
                Logger::log(LogLevel::WARN, "Safety Block: " + agent.id + " avoided hazard at (" + std::to_string(next_pos.row) + "," + std::to_string(next_pos.col) + ")");
            } else {
                agent.position = next_pos;
            }
        }
        if (all_exited) break;
    }
    report_generator.finalize_report();
    std::string command = "python scripts/animate_report.py " + report_path + "/multi_agent_report.html";
    std::system(command.c_str());
}