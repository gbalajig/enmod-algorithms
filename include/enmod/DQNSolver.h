#ifndef ENMOD_DQN_SOLVER_H
#define ENMOD_DQN_SOLVER_H

#include "Solver.h"
#include "Types.h"
#include <vector>
#include <map>
#include <string>

class DQNSolver : public Solver {
public:
    DQNSolver(const Grid& grid_ref);
    virtual ~DQNSolver() = default;

    void run() override;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

    // Offline Training / Persistence
    void saveModel(const std::string& filename) const;
    bool loadModel(const std::string& filename);

private:
    // Core Logic
    void train();
    void navigate();
    
    // Helpers
    int chooseAction(const Position& state);
    double getQValue(const Position& state, int action);
    void setQValue(const Position& state, int action, double val);
    double getMaxQ(const Position& state);
    Position getNextPosition(const Position& p, int action);

    // Hyperparameters
    double epsilon;
    double epsilon_decay;
    double min_epsilon;
    double alpha;
    double gamma;
    int max_episodes;

    // State
    Cost total_cost;
    std::vector<Position> path;

    // Q-Table: Map<Position, vector<double> (4 actions)>
    // Using Position as key requires it to be comparable (operator< exists in Types.h)
    std::map<Position, std::vector<double>> q_table;
};

#endif // ENMOD_DQN_SOLVER_H