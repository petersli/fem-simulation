#include "solver.h"
#include <iostream>

using namespace Eigen;

Solver::Solver()
{

}

/** Perform a time step with RK4 method */
void Solver::rungeKuttaStep(std::unique_ptr<System> &sys, double timestep) {
    double time = sys->getTime();
    VectorXd x0 = sys->getState();
    // k1: slope from euler's method
    VectorXd k1 = sys->evalTimeDerivative();
    // k2: slope from midpoint method using k1
    sys->setState(x0 + timestep * k1 / 2.0, time + timestep / 2.0);
    VectorXd k2 = sys->evalTimeDerivative();
    // k3: slope from midpoint method using k2
    sys->setState(x0 + timestep * k2 / 2.0, time + timestep / 2.0);
    VectorXd k3 = sys->evalTimeDerivative();
    // k4: slope at end of interval using k3
    sys->setState(x0 + timestep * k3, time);
    VectorXd k4 = sys->evalTimeDerivative();
    // Take weighted average
    VectorXd slope = (1.0 / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
    sys->setState(x0 + timestep * slope, time + timestep);
}

/** Perform a time step with the midpoint method */
void Solver::midpointStep(std::unique_ptr<System> &sys, double timestep) {
    double time = sys->getTime();
    VectorXd x0 = sys->getState();
    VectorXd eulerSlope = sys->evalTimeDerivative();
    sys->setState(x0 + timestep * eulerSlope / 2.0, time + timestep / 2.0);
    VectorXd midpointSlope = sys->evalTimeDerivative();
    sys->setState(x0 + timestep * midpointSlope, time + timestep);
}

/** Perform a time step with the Euler method. */
void Solver::eulerStep(std::unique_ptr<System> &sys, double timestep) {
    double time = sys->getTime();
    VectorXd x0 = sys->getState();
    VectorXd slope = sys->evalTimeDerivative();
    sys->setState(x0 + timestep * slope, time + timestep);
}
