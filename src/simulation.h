#ifndef SIMULATION_H
#define SIMULATION_H

#include <QSettings>
#include "graphics/shape.h"
#include "solver.h"
#include <map>
#include <unordered_set>

const double _timestep = 1.0 / 60.0;
const int _maxStepsPerFrame = 3;

class Shader;

class Simulation
{
public:
    Simulation();

    void init();

    void update(float seconds);

    void draw(Shader *shader);

    void toggleWire();

    void togglePause();
private:
    bool _paused;
    std::unique_ptr<System> _system;
    std::unique_ptr<Solver> _solver;
    Shape _shape;
    Shape _ground;
    Shape _collider;
    void initGround();
    void initCollider();
    std::vector<Eigen::Vector3i> extractSurface(std::vector<Eigen::Vector4i> tets);
    std::vector<int> getKeyForFace(Eigen::Vector3i face);
};

#endif // SIMULATION_H
