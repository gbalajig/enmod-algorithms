#include "enmod/DQNSolver.h"
#include "enmod/Logger.h"
#include "enmod/json.hpp" 
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <random>
#include <ctime>

using json = nlohmann::json;

DQNSolver::DQNSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "DQNSolver"),
      epsilon(0.9),      
      epsilon_decay(0.995),
      min_epsilon(0.01),
      alpha(0.1),        
      gamma(0.95),       
      max_episodes(1000),
      total_cost{0,0,0} // Default init
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

// --- CORE LOGIC ---

void DQNSolver::run() {
    std::string model_file = "data/dqn_model_" + grid.getName() + ".json";
    
    // Ensure data directory exists
    if (!std::filesystem::exists("data")) {
        std::filesystem::create_directory("data");
    }

    bool model_loaded = loadModel(model_file);

    if (model_loaded) {
        Logger::log(LogLevel::INFO, "DQN: Model loaded. Skipping training.");
        epsilon = min_epsilon; // Exploitation mode
    } else {
        Logger::log(LogLevel::INFO, "DQN: No model found. Starting training...");
        train();
        saveModel(model_file);
    }

    navigate();
}

void DQNSolver::train() {
    int rows = grid.getRows();
    int cols = grid.getCols();
    Position start = grid.getStartPosition();

    for (int episode = 0; episode < max_episodes; ++episode) {
        Position state = start;
        int step_count = 0;
        int max_steps = rows * cols * 2;
        
        while (step_count < max_steps) {
            if (grid.isExit(state.row, state.col)) break;

            // 1. Action
            int action_idx = chooseAction(state); 

            // 2. Step
            Position next_state = getNextPosition(state, action_idx);
            
            // 3. Reward
            double reward = -1.0; 
            if (grid.isExit(next_state.row, next_state.col)) reward = 100.0;
            else if (grid.getCellType(next_state) == CellType::WALL) reward = -100.0;
            else if (grid.getCellType(next_state) == CellType::FIRE) reward = -50.0;
            else if (grid.getSmokeIntensity(next_state) == "heavy") reward = -20.0;

            // 4. Update
            double old_q = getQValue(state, action_idx);
            double max_next_q = getMaxQ(next_state);
            double new_q = old_q + alpha * (reward + gamma * max_next_q - old_q);
            setQValue(state, action_idx, new_q);

            // 5. Transition
            if (grid.isWalkable(next_state.row, next_state.col)) {
                state = next_state;
            }

            step_count++;
        }

        if (epsilon > min_epsilon) epsilon *= epsilon_decay;
    }
}

void DQNSolver::navigate() {
    path.clear();
    total_cost = {0, 0, 0}; 

    Position current = grid.getStartPosition();
    path.push_back(current);
    
    int steps = 0;
    int max_steps = grid.getRows() * grid.getCols() * 2;

    while (steps < max_steps) {
        if (grid.isExit(current.row, current.col)) break;

        // Greedy choice
        int best_action = -1;
        double best_val = -1e9;
        
        for (int a = 0; a < 4; ++a) {
            Position next = getNextPosition(current, a);
            if (grid.isWalkable(next.row, next.col)) {
                double q = getQValue(current, a);
                if (q > best_val) {
                    best_val = q;
                    best_action = a;
                }
            }
        }

        if (best_action == -1) break; // Trapped

        Position next_pos = getNextPosition(current, best_action);
        
        // Sum up cost
        Cost move_cost = grid.getMoveCost(current);
        total_cost = total_cost + move_cost;

        current = next_pos;
        path.push_back(current);
        steps++;
    }
}

// --- PERSISTENCE ---

void DQNSolver::saveModel(const std::string& filename) const {
    try {
        json j;
        for (const auto& entry : q_table) {
            std::string key = std::to_string(entry.first.row) + "," + std::to_string(entry.first.col);
            j["q_table"][key] = entry.second;
        }
        std::ofstream o(filename);
        o << std::setw(4) << j << std::endl;
        Logger::log(LogLevel::INFO, "DQN: Saved model to " + filename);
    } catch (const std::exception& e) {
        Logger::log(LogLevel::ERROR, "DQN: Save failed - " + std::string(e.what()));
    }
}

bool DQNSolver::loadModel(const std::string& filename) {
    if (!std::filesystem::exists(filename)) return false;
    try {
        std::ifstream i(filename);
        json j;
        i >> j;
        q_table.clear();
        for (auto& el : j["q_table"].items()) {
            std::string key = el.key();
            size_t comma = key.find(',');
            int r = std::stoi(key.substr(0, comma));
            int c = std::stoi(key.substr(comma + 1));
            q_table[{r, c}] = el.value().get<std::vector<double>>();
        }
        return true;
    } catch (...) {
        return false;
    }
}

// --- HELPERS ---

int DQNSolver::chooseAction(const Position& state) {
    if ((double)std::rand() / RAND_MAX < epsilon) return std::rand() % 4;
    
    int best_a = 0;
    double max_q = -1e9;
    for (int a = 0; a < 4; ++a) {
        double q = getQValue(state, a);
        if (q > max_q) {
            max_q = q;
            best_a = a;
        }
    }
    return best_a;
}

double DQNSolver::getQValue(const Position& state, int action) {
    if (q_table.find(state) == q_table.end()) q_table[state] = std::vector<double>(4, 0.0);
    return q_table[state][action];
}

void DQNSolver::setQValue(const Position& state, int action, double val) {
    if (q_table.find(state) == q_table.end()) q_table[state] = std::vector<double>(4, 0.0);
    q_table[state][action] = val;
}

double DQNSolver::getMaxQ(const Position& state) {
    if (q_table.find(state) == q_table.end()) return 0.0;
    const auto& q_vals = q_table[state];
    return *std::max_element(q_vals.begin(), q_vals.end());
}

Position DQNSolver::getNextPosition(const Position& p, int action) {
    int r = p.row; 
    int c = p.col;
    if (action == 0) r--; // UP
    else if (action == 1) r++; // DOWN
    else if (action == 2) c--; // LEFT
    else if (action == 3) c++; // RIGHT
    return {r, c};
}

Cost DQNSolver::getEvacuationCost() const {
    return total_cost;
}

void DQNSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h3>DQN Solver Report</h3>";
    report_file << "<p><strong>Training Episodes:</strong> " << max_episodes << "</p>";
    report_file << "<p><strong>Path Length:</strong> " << path.size() << "</p>";
    report_file << grid.toHtmlStringWithPath(path);
}