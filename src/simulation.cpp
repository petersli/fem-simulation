#include "simulation.h"

#include <iostream>

#include "graphics/MeshLoader.h"

using namespace Eigen;

Simulation::Simulation() :
    _paused(false)
{
}

void Simulation::init()
{
    QSettings settings("config.ini", QSettings::IniFormat);
    std::vector<Vector3d> vertices;
    std::vector<Vector4i> tets;
    std::string meshPath = settings.value("path").toString().toStdString();
    if(MeshLoader::loadTetMesh(meshPath, vertices, tets)) {
        _system = std::make_unique<System>(vertices, tets);
        _solver = std::make_unique<Solver>();
        std::vector<Vector3i> faces = extractSurface(tets);
        _shape.init(vertices, faces, tets);
        std::cout << "Loaded tet mesh from " << meshPath << std::endl;
    } else {
        std::cerr << "Failed to load tet mesh" << std::endl;
        return;
    }
    Affine3f shapeModel = Affine3f(Translation3f(0, 4, 0));
    _shape.setModelMatrix(shapeModel);
    _system->setShapeModelMatrix(shapeModel.matrix().cast<double>());

    initGround();
    initCollider();
}

/** Extracts the faces of tetrahedra that are on the exterior surface for rendering */
std::vector<Vector3i> Simulation::extractSurface(std::vector<Vector4i> tets) {
    // Count how many times each face occurs (ignoring winding order)
    std::map<std::vector<int>, int> faceCountMap;
    // Store in proper winding order
    std::vector<Vector3i> faces;
    for (auto const &tet : tets) {
        // The 4 faces of a tetrahedron
        Vector3i f0(tet[0], tet[2], tet[1]);
        Vector3i f1(tet[0], tet[1], tet[3]);
        Vector3i f2(tet[1], tet[2], tet[3]);
        Vector3i f3(tet[0], tet[3], tet[2]);
        Vector3i tetFaces [4] = {f0, f1, f2, f3};
        for (int i = 0; i < 4; i++) {
            Vector3i tetFace = tetFaces[i];
            faces.push_back(tetFace);
            faceCountMap[getKeyForFace(tetFace)]++;
        }
    }
    // Surface faces are faces that belong to exactly 1 tetrahedron
    std::vector<Vector3i> surfaceFaces;
    for (auto const &face : faces) {
        int count = faceCountMap[getKeyForFace(face)];
        if (count == 1) {
            surfaceFaces.push_back(face);
        }
    }
    return surfaceFaces;
}

/** Sort vertex indices to get key that does not depend on winding order */
std::vector<int> Simulation::getKeyForFace(Eigen::Vector3i face) {
    std::vector<int> key{face[0], face[1], face[2]};
    std::sort(key.begin(), key.end());
    return key;
}

void Simulation::update(float seconds)
{
    if (_paused) return;
    _system->resolveCollisions();
    double timestep = std::min(_timestep, static_cast<double>(seconds));
    int numSteps = seconds / timestep;
    for (int i = 0; i < numSteps && i < _maxStepsPerFrame; i++) {
        _solver->rungeKuttaStep(_system, timestep);
        _shape.setVertices(_system->getVertices());
    }
}

void Simulation::draw(Shader *shader)
{
    _shape.draw(shader);
    _ground.draw(shader);
    _collider.draw(shader);
}

void Simulation::toggleWire()
{
    _shape.toggleWireframe();
}

/** Create sphere collider centered at origin */
void Simulation::initCollider() {
    std::vector<Vector3d> vertices;
    std::vector<Vector4i> tets;
    if(MeshLoader::loadTetMesh("/Users/peter/course/cs2240/simulation-petersli/example-meshes/sphere.mesh", vertices, tets)) {
        std::vector<Vector3i> faces = extractSurface(tets);
        _collider.init(vertices, faces, tets);
    } else {
        std::cerr << "Failed to load collider tet mesh" << std::endl;
    }
}

void Simulation::initGround()
{
    std::vector<Vector3d> groundVerts;
    std::vector<Vector3i> groundFaces;
    groundVerts.emplace_back(-5, groundHeight, -5);
    groundVerts.emplace_back(-5, groundHeight, 5);
    groundVerts.emplace_back(5, groundHeight, 5);
    groundVerts.emplace_back(5, groundHeight, -5);
    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);
    _ground.init(groundVerts, groundFaces);
}

/** Pause the simulation */
void Simulation::togglePause() {
    _paused = !_paused;
}
