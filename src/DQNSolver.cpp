#include "enmod/DQNSolver.h"
#include "enmod/Logger.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// --- Neural Net Implementation ---

double sigmoid(double x) { return 1.0 / (1.0 + exp(-x)); }
double sigmoid_derivative(double x) { double s = sigmoid(x); return s * (1.0 - s); }
double random_weight() { return ((double)rand() / RAND_MAX) * 2.0 - 1.0; }

SimpleNeuralNet::SimpleNeuralNet(int in, int hidden, int out) 
    : input_size(in), hidden_size(hidden), output_size(out) {
    
    W1.resize(in, std::vector<double>(hidden));
    b1.resize(hidden);
    W2.resize(hidden, std::vector<double>(out));
    b2.resize(out);

    for(int i=0; i<in; ++i) for(int j=0; j<hidden; ++j) W1[i][j] = random_weight();
    for(int j=0; j<hidden; ++j) b1[j] = random_weight();
    for(int j=0; j<hidden; ++j) for(int k=0; k<out; ++k) W2[j][k] = random_weight();
    for(int k=0; k<out; ++k) b2[k] = random_weight();
}

std::vector<double> SimpleNeuralNet::forward(const std::vector<double>& inputs) {
    std::vector<double> hidden(hidden_size);
    std::vector<double> output(output_size);

    for(int j=0; j<hidden_size; ++j) {
        double sum = b1[j];
        for(int i=0; i<input_size; ++i) sum += inputs[i] * W1[i][j];
        hidden[j] = sigmoid(sum);
    }

    for(int k=0; k<output_size; ++k) {
        double sum = b2[k];
        for(int j=0; j<hidden_size; ++j) sum += hidden[j] * W2[j][k];
        output[k] = sum; 
    }
    return output;
}

void SimpleNeuralNet::backprop(const std::vector<double>& inputs, const std::vector<double>& targets, double lr) {
    std::vector<double> hidden(hidden_size);
    std::vector<double> hidden_raw(hidden_size);
    
    for(int j=0; j<hidden_size; ++j) {
        double sum = b1[j];
        for(int i=0; i<input_size; ++i) sum += inputs[i] * W1[i][j];
        hidden_raw[j] = sum;
        hidden[j] = sigmoid(sum);
    }

    std::vector<double> output(output_size);
    for(int k=0; k<output_size; ++k) {
        double sum = b2[k];
        for(int j=0; j<hidden_size; ++j) sum += hidden[j] * W2[j][k];
        output[k] = sum;
    }

    std::vector<double> output_deltas(output_size);
    for(int k=0; k<output_size; ++k) output_deltas[k] = output[k] - targets[k]; 

    std::vector<double> hidden_deltas(hidden_size);
    for(int j=0; j<hidden_size; ++j) {
        double error = 0.0;
        for(int k=0; k<output_size; ++k) error += output_deltas[k] * W2[j][k];
        hidden_deltas[j] = error * sigmoid_derivative(hidden_raw[j]);
    }

    for(int k=0; k<output_size; ++k) {
        b2[k] -= lr * output_deltas[k];
        for(int j=0; j<hidden_size; ++j) W2[j][k] -= lr * output_deltas[k] * hidden[j];
    }

    for(int j=0; j<hidden_size; ++j) {
        b1[j] -= lr * hidden_deltas[j];
        for(int i=0; i<input_size; ++i) W1[i][j] -= lr * hidden_deltas[j] * inputs[i];
    }
}

// --- DQNSolver Implementation ---

DQNSolver::DQNSolver(const Grid& grid_ref) : Solver(grid_ref, "DQNSim") {
    q_network = std::make_unique<SimpleNeuralNet>(2, 16, 4);
    std::random_device rd;
    rng.seed(rd());
}

std::vector<double> DQNSolver::getStateVector(const Position& pos) const {
    return { 
        (double)pos.row / std::max(1, grid.getRows()), 
        (double)pos.col / std::max(1, grid.getCols()) 
    };
}

int DQNSolver::getActionIndex(Direction dir) const {
    if (dir == Direction::UP) return 0;
    if (dir == Direction::DOWN) return 1;
    if (dir == Direction::LEFT) return 2;
    return 3;
}

Direction DQNSolver::getDirectionFromIndex(int idx) const {
    if (idx == 0) return Direction::UP;
    if (idx == 1) return Direction::DOWN;
    if (idx == 2) return Direction::LEFT;
    return Direction::RIGHT;
}

Position DQNSolver::simulateMove(Position pos, Direction dir) {
    Position next = grid.getNextPosition(pos, dir);
    if (!grid.isWalkable(next.row, next.col)) return pos;
    return next;
}

void DQNSolver::train(int episodes) {
    for (int e = 0; e < episodes; ++e) {
        Position state = grid.getStartPosition();
        int steps = 0;
        int max_steps = grid.getRows() * grid.getCols();

        while (!grid.isExit(state.row, state.col) && steps < max_steps) {
            std::vector<double> state_vec = getStateVector(state);
            std::vector<double> q_values = q_network->forward(state_vec);

            int action_idx = 0;
            if (((double)rand() / RAND_MAX) < epsilon) {
                action_idx = rand() % 4;
            } else {
                action_idx = (int)std::distance(q_values.begin(), std::max_element(q_values.begin(), q_values.end()));
            }

            Direction action = getDirectionFromIndex(action_idx);
            Position next_state = simulateMove(state, action);
            
            double reward = -0.1; 
            if (grid.isExit(next_state.row, next_state.col)) reward = 100.0;
            else if (next_state == state) reward = -1.0; 

            std::vector<double> next_state_vec = getStateVector(next_state);
            std::vector<double> next_q = q_network->forward(next_state_vec);
            double max_next_q = *std::max_element(next_q.begin(), next_q.end());
            
            double target = reward + gamma * max_next_q;
            std::vector<double> targets = q_values;
            targets[action_idx] = target; 

            q_network->backprop(state_vec, targets, learning_rate);

            state = next_state;
            steps++;
        }
        if (epsilon > epsilon_min) epsilon *= epsilon_decay;
    }
}

Direction DQNSolver::chooseAction(const Position& current_pos) {
    std::vector<double> state_vec = getStateVector(current_pos);
    std::vector<double> q_values = q_network->forward(state_vec);
    int action_idx = (int)std::distance(q_values.begin(), std::max_element(q_values.begin(), q_values.end()));
    return getDirectionFromIndex(action_idx);
}

void DQNSolver::run() {
    Position current = grid.getStartPosition();
    total_cost = {0,0,0};
    
    // [FIX] clear() works now that vector<StepReport> is valid
    history.clear(); 
    
    int steps = 0;
    EvacuationMode mode = EvacuationMode::NORMAL;

    while(!grid.isExit(current.row, current.col) && steps < 200) {
        Direction dir = chooseAction(current);
        Position next = simulateMove(current, dir);
        
        std::string act = "STAY";
        if(dir == Direction::UP) act = "UP";
        if(dir == Direction::DOWN) act = "DOWN";
        if(dir == Direction::LEFT) act = "LEFT";
        if(dir == Direction::RIGHT) act = "RIGHT";

        // [FIX] Correct construction of StepReport
        history.push_back({
            steps, 
            grid, 
            current, 
            act, 
            total_cost, 
            mode
        });
        
        if(current == next) break; // Stuck
        current = next;
        total_cost.distance++;
        total_cost.time++;
        steps++;
    }
}

Cost DQNSolver::getEvacuationCost() const { return total_cost; }
void DQNSolver::generateReport(std::ofstream& f) const { f << "<h2>DQN Report</h2>"; }