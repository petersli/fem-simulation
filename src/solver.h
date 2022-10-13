#ifndef SOLVER_H
#define SOLVER_H

#include "system.h"

class Solver
{
public:
    Solver();
    void rungeKuttaStep(std::unique_ptr<System> &sys, double timestep);
    void midpointStep(std::unique_ptr<System> &sys, double timestep);
    void eulerStep(std::unique_ptr<System> &sys, double timestep);
};

#endif // SOLVER_H
