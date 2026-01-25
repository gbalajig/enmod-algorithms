#include "enmod/MultiAgentCPSController.h"
#include "enmod/Logger.h"
#include "enmod/SimulatedSensorNetwork.h"
#include "enmod/RealTimeSensorNetwork.h" 
#include <fstream>
#include <iostream>
#include <filesystem>
#include <string>
#include <iomanip> // Required for std::setw

MultiAgentCPSController::MultiAgentCPSController(const json& initial_config, const std::string& report_path, int num_agents)
    : master_grid(initial_config),
      // [FIX] Pass the DIRECTORY only. The generator appends the filename.
      report_generator(report_path), 
      report_path(report_path) {

    // Ensure the output directory exists
    std::filesystem::create_directory(report_path + "/agent_io");

    Position start_pos = master_grid.getStartPosition();
    for (int i = 0; i < num_agents; ++i) {
        Position agent_pos = {start_pos.row + i, start_pos.col};
        
        // Find a valid starting spot nearby if blocked
        while (!master_grid.isWalkable(agent_pos.row, agent_pos.col)) {
            agent_pos.col++; 
            if (!master_grid.isValid(agent_pos.row, agent_pos.col)) {
                agent_pos.col--; 
                agent_pos.row++; 
            }
        }
        
        if (!master_grid.isValid(agent_pos.row, agent_pos.col)) {
            Logger::log(LogLevel::ERROR, "Could not place agent " + std::to_string(i) + " within grid bounds.");
            continue;
        }
        agents.push_back({"agent_" + std::to_string(i), agent_pos});
    }
    
    // Default solver for agents
    solver = std::make_unique<HybridDPRLSolver>(master_grid);
}

void MultiAgentCPSController::write_input_file(int timestep, const Agent& agent) {
    json input_data;
    input_data["agent_id"] = agent.id;
    input_data["timestep"] = timestep;
    input_data["current_position"] = {{"row", agent.position.row}, {"col", agent.position.col}};
    input_data["environment_update"] = master_grid.getConfig();
    
    std::ofstream o(report_path + "/agent_io/" + agent.id + "_input_t" + std::to_string(timestep) + ".json");
    if (o.is_open()) {
        o << std::setw(4) << input_data << std::endl;
    }
}

Direction MultiAgentCPSController::read_output_file(int timestep, const Agent& agent) {
    // Wait/Retry logic could be added here for real systems
    std::ifstream i(report_path + "/agent_io/" + agent.id + "_output_t" + std::to_string(timestep) + ".json");
    if (!i.is_open()) return Direction::STAY; // Safety fallback

    json output_data;
    try {
        i >> output_data;
        std::string move = output_data.at("next_move");
        if (move == "UP") return Direction::UP;
        if (move == "DOWN") return Direction::DOWN;
        if (move == "LEFT") return Direction::LEFT;
        if (move == "RIGHT") return Direction::RIGHT;
    } catch (...) {
        return Direction::STAY;
    }
    return Direction::STAY;
}

void MultiAgentCPSController::run_simulation() {
    std::cout << "\n===== Starting Real-Time Multi-Agent CPS Simulation (Digital Twin Mode) =====\n";

    // [CONFIG] Default to false for standalone benchmarking unless Python script is running
    bool use_real_data = false; 
    std::string live_data_file = "data/live_sensors.json";

    std::unique_ptr<ISensorNetwork> sensor_network;

    if (use_real_data) {
        std::cout << "[INFO] Connecting to Real-Time Sensor Feed: " << live_data_file << std::endl;
        std::ofstream outfile(live_data_file, std::ios::app); 
        outfile.close();
        sensor_network = std::make_unique<RealTimeSensorNetwork>(live_data_file);
    } else {
        std::cout << "[INFO] Using Internal Simulator for Sensors." << std::endl;
        sensor_network = std::make_unique<SimulatedSensorNetwork>(master_grid, &agents);
    }

    int max_time = 2 * (master_grid.getRows() * master_grid.getCols());

    for (int t = 0; t < max_time; ++t) {
        // A. EVOLVE GROUND TRUTH
        for (const auto& event_cfg : master_grid.getConfig().value("dynamic_events", json::array())) {
            if (event_cfg.value("time_step", -1) == t) {
                master_grid.addHazard(event_cfg);
            }
        }

        // B. SENSE & UPDATE DIGITAL TWIN
        std::vector<SensorReading> readings = sensor_network->getAllReadings(t);
        Grid digital_twin_grid(master_grid.getConfig()); 
        digital_twin_grid.updateFromSensors(readings);

        // C. REPORTING (Visualize Ground Truth)
        std::vector<Position> agent_positions;
        for (const auto& agent : agents) agent_positions.push_back(agent.position);
        
        // [CHECK] Ensure this matches the signature in MultiAgentReportGenerator.h
        report_generator.add_timestep(t, master_grid, agent_positions); 

        // D. PLANNING
        bool all_exited = true;
        for (auto& agent : agents) {
            if (master_grid.isExit(agent.position.row, agent.position.col)) continue;
            all_exited = false;

            write_input_file(t, agent);

            // Agents plan based on their view of the Digital Twin + Peers
            Grid agent_view = digital_twin_grid;
            for (const auto& other_agent : agents) {
                if (agent.id != other_agent.id) {
                    agent_view.setCellUnwalkable(other_agent.position);
                }
            }
            
            Direction next_move = solver->getNextMove(agent.position, agent_view);

            // Write decision to output file (Simulating agent response)
            json output_data;
            output_data["agent_id"] = agent.id;
            std::string move_str = "STAY";
            if (next_move == Direction::UP) move_str = "UP";
            else if (next_move == Direction::DOWN) move_str = "DOWN";
            else if (next_move == Direction::LEFT) move_str = "LEFT";
            else if (next_move == Direction::RIGHT) move_str = "RIGHT";
            output_data["next_move"] = move_str;

            std::ofstream o(report_path + "/agent_io/" + agent.id + "_output_t" + std::to_string(t) + ".json");
            o << std::setw(4) << output_data << std::endl;

            // Read back (closing the loop) and Actuate
            Direction received_move = read_output_file(t, agent);
            
            // Validate move against Ground Truth physics
            Position next_pos_check = master_grid.getNextPosition(agent.position, received_move);
            if (master_grid.isWalkable(next_pos_check.row, next_pos_check.col)) {
                agent.position = next_pos_check;
            }
        }

        if (all_exited) {
            std::cout << "SUCCESS: All agents reached the exit." << std::endl;
            Logger::log(LogLevel::INFO, "SUCCESS: All agents reached the exit.");
            break;
        }
    }

    report_generator.finalize_report();
    std::cout << "\nMulti-agent simulation complete. Report generated at " << report_path << "/MultiAgent_Report.html\n";
}