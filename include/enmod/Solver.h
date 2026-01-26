#ifndef ENMOD_SOLVER_H
#define ENMOD_SOLVER_H

#include "Grid.h"   // [CRITICAL] Grid includes Types.h, so we get everything
#include <string>
#include <vector>
#include <fstream>

// [FIXED] StepReport moved here to resolve circular dependency with Types.h
struct StepReport {
    int time_step;
    Grid grid_state;
    Position agent_pos;
    std::string action;
    Cost current_total_cost;
    EvacuationMode mode;
};

// Return type for simulation
struct Result {
    std::string scenario_name;
    std::string solver_name;
    Cost cost;
    double weighted_cost;
    double execution_time;
};

class Solver {
public:
    Solver(const Grid& grid_ref, const std::string& name) : grid(grid_ref), solver_name(name) {}
    virtual ~Solver() = default;

    virtual void run() = 0;
    virtual Cost getEvacuationCost() const = 0;
    virtual void generateReport(std::ofstream& report_file) const = 0;

    std::string getName() const { return solver_name; }

protected:
    Grid grid;
    std::string solver_name;
};

#endif // ENMOD_SOLVER_H