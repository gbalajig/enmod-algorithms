#include "enmod/RealTimeSensorNetwork.h"
#include "enmod/json.hpp"
#include "enmod/Logger.h"
#include <fstream>
#include <iostream>
#include <thread> // Added for sleep
#include <chrono> // Added for duration

using json = nlohmann::json;

RealTimeSensorNetwork::RealTimeSensorNetwork(const std::string& data_file_path) 
    : file_path(data_file_path) {}

std::vector<SensorReading> RealTimeSensorNetwork::getAllReadings(double current_time) {
    std::vector<SensorReading> readings;
    
    // [FIX] Retry loop to handle Windows file locking (WinError 5) 
    // This gives the Python feeder time to finish replacing the file.
    for (int retry = 0; retry < 5; ++retry) {
        std::ifstream f(file_path, std::ios::in | std::ios::binary);

        if (!f.is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // [FIX] Check if file is empty to avoid json.exception.parse_error.101 
        // occurring during file initialization.
        f.seekg(0, std::ios::end);
        if (f.tellg() == 0) {
            f.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        f.seekg(0, std::ios::beg);

        try {
            json j;
            f >> j; 
            f.close(); // [CRITICAL] Close handle immediately after reading to free the lock.

            for (const auto& item : j) {
                SensorReading r;
                r.sensor_id = item.value("id", "unknown");
                
                // Sensor Type Mapping
                std::string type_str = item.value("type", "");
                if (type_str == "SMOKE") r.type = SensorType::SMOKE_DETECTOR;
                else if (type_str == "THERMAL") r.type = SensorType::THERMAL_CAMERA;
                else if (type_str == "LIDAR") r.type = SensorType::LIDAR;
                else if (type_str == "AGENT") r.type = SensorType::AGENT_BEACON;
                else continue; // Skip unknown types.

                r.location = {item.value("row", 0), item.value("col", 0)};
                r.value = item.value("value", 0.0);
                r.data = item.value("data", "");
                r.timestamp = item.value("timestamp", 0.0);

                readings.push_back(r);
            }
            return readings; // Success, return the parsed data.
        } catch (const json::exception& e) {
            f.close();
            // Only log on the final retry to reduce log noise.
            if (retry == 4) {
                Logger::log(LogLevel::WARN, "Failed to parse live sensor data: " + std::string(e.what()));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return readings;
}