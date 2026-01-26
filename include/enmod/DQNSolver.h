#ifndef ENMOD_DQN_SOLVER_H
#define ENMOD_DQN_SOLVER_H

#include "Solver.h"
#include "enmod/Types.h"  // [CRITICAL FIX] Full path to Types.h
#include <vector>
#include <memory>
#include <random>

// --- Simple Neural Network for Q-Approximation ---
struct SimpleNeuralNet {
    int input_size;
    int hidden_size;
    int output_size;
    
    std::vector<std::vector<double>> W1;
    std::vector<double> b1;
    std::vector<std::vector<double>> W2;
    std::vector<double> b2;

    SimpleNeuralNet(int in, int hidden, int out);
    std::vector<double> forward(const std::vector<double>& inputs);
    void backprop(const std::vector<double>& inputs, const std::vector<double>& targets, double learning_rate);
};

class DQNSolver : public Solver {
public:
    DQNSolver(const Grid& grid_ref);
    
    // API Required by Enhanced Hybrid
    void train(int episodes);
    Direction chooseAction(const Position& current_pos);

    void run() override;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

private:
    std::unique_ptr<SimpleNeuralNet> q_network;
    
    // [FIX] Now Types.h is included, this vector compiles
    std::vector<StepReport> history; 
    
    Cost total_cost;
    double gamma = 0.95;
    double epsilon = 1.0;
    double epsilon_min = 0.01;
    double epsilon_decay = 0.9995;
    double learning_rate = 0.01;
    
    std::mt19937 rng;
    
    std::vector<double> getStateVector(const Position& pos) const;
    int getActionIndex(Direction dir) const;
    Direction getDirectionFromIndex(int idx) const;
    Position simulateMove(Position pos, Direction dir);
};

#endif // ENMOD_DQN_SOLVER_H