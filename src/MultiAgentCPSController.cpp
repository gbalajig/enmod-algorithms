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

MultiAgentCPSController::MultiAgentCPSController(const json& initial_config, const std::string& report_path, int num_agents)
    : master_grid(initial_config),
      report_generator(report_path + "/multi_agent_report.html"),
      report_path(report_path) {

    std::filesystem::create_directory(report_path + "/agent_io");

    Position start_pos = master_grid.getStartPosition();
    for (int i = 0; i < num_agents; ++i) {
        Position agent_pos = {start_pos.row + i, start_pos.col};
        // Simple collision avoidance for start positions
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
        i >> output_data;
        std::string move = output_data.at("next_move");
        if (move == "UP") return Direction::UP;
        if (move == "DOWN") return Direction::DOWN;
        if (move == "LEFT") return Direction::LEFT;
        if (move == "RIGHT") return Direction::RIGHT;
    }
    return Direction::STAY;
}

void MultiAgentCPSController::run_simulation() {
    std::cout << "\n   [CPS] Starting Real-Time Multi-Agent Simulation...\n";

    // --- CONFIGURATION TOGGLE ---
    // Set to 'false' to use the internal simulator (Standard Benchmark Mode)
    // Set to 'true' if you have the Python sensor feeder running
    bool use_real_data = false; 
    std::string live_data_file = "data/live_sensors.json";
    // ----------------------------

    std::unique_ptr<ISensorNetwork> sensor_network;

    if (use_real_data) {
        std::cout << "   [CPS] Connecting to Live Sensor Feed: " << live_data_file << std::endl;
        std::ofstream outfile(live_data_file, std::ios::app); 
        outfile.close();
        sensor_network = std::make_unique<RealTimeSensorNetwork>(live_data_file);
    } else {
        sensor_network = std::make_unique<SimulatedSensorNetwork>(master_grid, &agents);
    }

    // Performance Tracking
    std::map<std::string, int> exit_times;
    std::map<std::string, double> path_length;

    int max_timesteps = 2 * (master_grid.getRows() * master_grid.getCols());

    for (int t = 0; t < max_timesteps; ++t) {
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

        // C. REPORTING
        std::vector<Position> current_positions;
        for (const auto& agent : agents) current_positions.push_back(agent.position);
        report_generator.add_timestep(t, master_grid, current_positions); 

        // D. PLANNING & EXECUTION
        bool all_exited = true;
        
        for (auto& agent : agents) {
            // Check if agent has already exited
            if (master_grid.isExit(agent.position.row, agent.position.col)) {
                if (exit_times.find(agent.id) == exit_times.end()) {
                    exit_times[agent.id] = t;
                    // std::cout << "   [CPS] " << agent.id << " exited at t=" << t << "\n";
                }
                continue;
            }
            
            all_exited = false;
            write_input_file(t, agent);

            // Create agent view (Digital Twin + Peer Avoidance)
            Grid agent_view = digital_twin_grid;
            for (const auto& other_agent : agents) {
                if (agent.id != other_agent.id) {
                    agent_view.setCellUnwalkable(other_agent.position);
                }
            }
            
            // Solver uses Digital Twin
            Direction next_move = solver->getNextMove(agent.position, agent_view);

            // Write Output (Simulating Network Transmission)
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

            // Execute Move
            Direction received_move = read_output_file(t, agent);
            agent.position = master_grid.getNextPosition(agent.position, received_move);
        }

        if (all_exited) {
            break;
        }
    }

    report_generator.finalize_report();

    // --- CONSOLE SUMMARY ---
    std::cout << "\n   ----------------------------------------\n";
    std::cout << "   | Multi-Agent Performance Summary      |\n";
    std::cout << "   ----------------------------------------\n";
    std::cout << "   | " << std::left << std::setw(15) << "Agent ID" << " | " << std::setw(18) << "Evacuation Time (s)" << " |\n";
    std::cout << "   |-----------------|--------------------|\n";
    
    for (const auto& agent : agents) {
        std::cout << "   | " << std::left << std::setw(15) << agent.id << " | ";
        if (exit_times.find(agent.id) != exit_times.end()) {
            std::cout << std::setw(18) << exit_times[agent.id] << " |";
        } else {
            std::cout << std::setw(18) << "FAILED (TimeOut)" << " |";
        }
        std::cout << "\n";
    }
    std::cout << "   ----------------------------------------\n";
    std::cout << "   -> Visual Report: " << report_path << "/multi_agent_report.html\n\n";
}