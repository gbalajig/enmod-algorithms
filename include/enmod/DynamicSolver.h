#ifndef ENMOD_DYNAMIC_SOLVER_H
#define ENMOD_DYNAMIC_SOLVER_H

#include "Solver.h" // Inherits StepReport definition from here
#include "Types.h"

// Do NOT redefine StepReport here.

class DynamicSolver : public Solver {
public:
    DynamicSolver(const Grid& grid_ref, const std::string& name) 
        : Solver(grid_ref, name) {}
    
    virtual ~DynamicSolver() = default;
    
    // Common interface for dynamic solvers could go here
};

#endif // ENMOD_DYNAMIC_SOLVER_H