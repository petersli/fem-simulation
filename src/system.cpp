#include "system.h"
#include <iostream>

using namespace Eigen;

System::System()
{

}

/** Initialize system */
System::System(std::vector<Vector3d> vertices, std::vector<Vector4i>tets) {
    _time = 0;
    for (const Vector3d &vertex : vertices) {
        Node node(vertex, Vector3d(0, 0, 0), Vector3d(0, 0, 0), 0);
        _nodes.push_back(node);
    }
    for (const Vector4i &tet: tets) {
        Matrix3d beta = computeBetaForTet(tet, vertices);
        std::vector<ProcessedFace> faces = processFaces(tet, vertices);
        Element element(tet, beta, faces);
        _elements.push_back(element);
    }
    assignNodesMass(tets);
}

/** Return the updated vertex positions */
std::vector<Vector3d> System::getVertices() {
    std::vector<Vector3d> vertices;
    for (const Node &node : _nodes) {
        vertices.push_back(node.pos);
    }
    return vertices;
}

/** Set state and time of system */
void System::setState(VectorXd state, double time) {
    _time = time;
    int stateSize = state.size();
    for (uint i = 0; i < _nodes.size(); i++) {
        int stateIndex = i * 6;
        assert(stateIndex+5 < stateSize);
        _nodes[i].pos[0] = state[stateIndex];
        _nodes[i].pos[1] = state[stateIndex+1];
        _nodes[i].pos[2] = state[stateIndex+2];
        _nodes[i].vel[0] = state[stateIndex+3];
        _nodes[i].vel[1] = state[stateIndex+4];
        _nodes[i].vel[2] = state[stateIndex+5];
    }
}

/** Package state (pos, vel) of the system into generic format for solver */
VectorXd System::getState() {
    std::vector<double> state;
    for (const Node &node : _nodes) {
        Vector3d pos = node.pos;
        Vector3d vel = node.vel;
        state.push_back(pos[0]);
        state.push_back(pos[1]);
        state.push_back(pos[2]);
        state.push_back(vel[0]);
        state.push_back(vel[1]);
        state.push_back(vel[2]);
    }
    Eigen::Map<VectorXd> result(state.data(), state.size());
    return result;
}


/** Package time derivative (vel, F/m) of the system into generic format for solver */
VectorXd System::getTimeDerivative() {
    std::vector<double> timeDeriv;
    for (const Node &node : _nodes) {
        Vector3d vel = node.vel;
        Vector3d acc = (node.force / node.mass) + gravity;
        timeDeriv.push_back(vel[0]);
        timeDeriv.push_back(vel[1]);
        timeDeriv.push_back(vel[2]);
        timeDeriv.push_back(acc[0]);
        timeDeriv.push_back(acc[1]);
        timeDeriv.push_back(acc[2]);
    }
    Eigen::Map<VectorXd> result(timeDeriv.data(), timeDeriv.size());
    return result;
}

/**
 * Eval time derivative of the phase position (pos, vel) and pack into generic format
 * for solver
 */
VectorXd System::evalTimeDerivative() {
    computeForces();
    return getTimeDerivative();
}

/** Compute and store forces for nodes */
void System::computeForces() {
    computeDeformationGradients();
    computeStrainAndStress();
    computeNodeForces();
}

/** Project vertex out of collider and set velocity normal to collider to zero */
void System::resolveCollisions() {
    #pragma omp parallel for
    for (Node &node : _nodes) {
        Vector4d objectPos = Vector4d(node.pos[0], node.pos[1], node.pos[2], 1);
        Vector4d worldSpacePos = _shapeModelMatrix * objectPos;
        // Ground collisions
        if (worldSpacePos[1] < groundHeight) {
            worldSpacePos[1] = groundHeight + 0.001;
            Vector4d newObjectPos = _shapeInverseModelMatrix * worldSpacePos;
            node.pos = newObjectPos(seq(0, 2));
            node.vel[1] = 0;
            node.vel = node.vel * friction;
        }
        // Collisions with sphere collider
        double worldSpaceNorm = worldSpacePos(seq(0, 2)).norm();
        if (worldSpaceNorm < sphereColliderRadius) {
            worldSpacePos = worldSpacePos * (sphereColliderRadius / worldSpaceNorm);
            worldSpacePos = worldSpacePos * 1.001;
            worldSpacePos[3] = 1; // homogenous coordinates
            Vector4d newObjectPos = _shapeInverseModelMatrix * worldSpacePos;
            node.pos = newObjectPos(seq(0, 2));
            Vector3d normal = worldSpacePos(seq(0, 2)).normalized();
            Vector3d tangentVel = node.vel - vectorProjection(node.vel, normal);
            node.vel = tangentVel * friction;
        }
    }
}

/** Compute face and node faces from elements */
void System::computeNodeForces() {
    // Zero out force accumulators
    for (Node &node : _nodes) {
        node.force = Vector3d(0, 0, 0);
    }
    // Each node accumulates force from opposite faces in tets
    #pragma omp parallel for
    for (uint i = 0; i < _elements.size(); i++) {
        Vector4i tet = _elements[i].tet;
        Matrix3d deformGrad = _elements[i].deformGrad;
        Matrix3d stress = _elementToStress[i];
        for (const ProcessedFace& processedFace : _elements[i].faces) {
            int vertexIndex = processedFace.vertex;
            double area = processedFace.area;
            Vector3d normal = processedFace.normal;
            Vector3d faceForce = stress * area * normal;
            Vector3d worldSpaceForce = deformGrad * faceForce;
            _nodes[vertexIndex].force += worldSpaceForce;
        }
    }
}

/** Compute deformation (dx/du) and velocity (dv/du) gradients */
void System::computeDeformationGradients() {
    #pragma omp parallel for
    for (Element &element : _elements) {
        Matrix3d beta = element.beta;
        Vector4i tet = element.tet;
        // compute P
        Vector3d p1 = _nodes[tet[0]].pos;
        Vector3d p2 = _nodes[tet[1]].pos;
        Vector3d p3 = _nodes[tet[2]].pos;
        Vector3d p4 = _nodes[tet[3]].pos;
        Matrix3d P;
        P << (p1 - p4), (p2 - p4), (p3 - p4);
        // compute V
        Vector3d v1 = _nodes[tet[0]].vel;
        Vector3d v2 = _nodes[tet[1]].vel;
        Vector3d v3 = _nodes[tet[2]].vel;
        Vector3d v4 = _nodes[tet[3]].vel;
        Matrix3d V;
        V << (v1 - v4), (v2 - v4), (v3 - v4);
        // Store updated gradients
        element.deformGrad = P * beta;
        element.velGrad = V * beta;
    }
}

/** Compute strain and stress per element */
void System::computeStrainAndStress() {
    #pragma omp parallel for
    for (uint i = 0; i < _elements.size(); i++) {
        Element element = _elements[i];
        Matrix3d I = Matrix3d::Identity();
        Matrix3d F = element.deformGrad;
        Matrix3d velGrad = element.velGrad;
        Matrix3d strain = F.transpose() * F - I;
        Matrix3d strainRate = F.transpose() * velGrad + velGrad.transpose() * F;
        Matrix3d elasticStress = lambda * I * strain.trace() + 2 * mu * strain;
        Matrix3d viscousStress = phi * I * strainRate.trace() + 2 * psi * strainRate;
        Matrix3d totalStress = elasticStress + viscousStress;
        _elementToStress[i] = totalStress;
    }
}


/** Compute beta (for barycentric coords) for given tet */
Matrix3d System::computeBetaForTet(Vector4i tet, std::vector<Vector3d> vertices) {
    Vector3d m1 = vertices[tet[0]];
    Vector3d m2 = vertices[tet[1]];
    Vector3d m3 = vertices[tet[2]];
    Vector3d m4 = vertices[tet[3]];
    Vector3d col1 = m1 - m4;
    Vector3d col2 = m2 - m4;
    Vector3d col3 = m3 - m4;
    Matrix3d betaInverse;
    betaInverse << col1, col2, col3;
    return betaInverse.inverse();
}

/** Compute mass of all nodes based on tets */
void System::assignNodesMass(std::vector<Vector4i> tets) {
    for (const Vector4i &tet : tets) {
        double volume = tetVolume(tet);
        double massContribution = density * volume / 4.0;
        _nodes[tet[0]].mass += massContribution;
        _nodes[tet[1]].mass += massContribution;
        _nodes[tet[2]].mass += massContribution;
        _nodes[tet[3]].mass += massContribution;
    }
}

/** Connect faces to opposite vertices, compute areas and normals */
std::vector<ProcessedFace> System::processFaces(Vector4i tet, std::vector<Vector3d> vertices) {
    ProcessedFace f0;
    f0.face = Vector3i(tet[0], tet[2], tet[1]);
    f0.vertex = tet[3];
    ProcessedFace f1;
    f1.face = Vector3i(tet[0], tet[1], tet[3]);
    f1.vertex = tet[2];
    ProcessedFace f2;
    f2.face = Vector3i(tet[1], tet[2], tet[3]);
    f2.vertex = tet[0];
    ProcessedFace f3;
    f3.face = Vector3i(tet[0], tet[3], tet[2]);
    f3.vertex = tet[1];
    std::vector<ProcessedFace> result {f0, f1, f2, f3};
    for (uint i = 0; i < result.size(); i++) {
        result[i].area = getFaceArea(result[i].face, vertices);
        result[i].normal = getFaceNormal(result[i].face, vertices);
    }
    return result;
}

/** Compute volume of a tetrahedron using scalar triple product */
double System::tetVolume(Vector4i tet) {
    Vector3d v0 = _nodes[tet[0]].pos;
    Vector3d v1 = _nodes[tet[1]].pos;
    Vector3d v2 = _nodes[tet[2]].pos;
    Vector3d v3 = _nodes[tet[3]].pos;
    Vector3d a = v1 - v0;
    Vector3d b = v2 - v0;
    Vector3d c = v3 - v0;
    return (1.0 / 6.0) * (a.cross(b)).dot(c);
}

/** Compute vector projection of v onto w */
Vector3d System::vectorProjection(Vector3d v, Vector3d w) {
    return (v.dot(w) / w.dot(w)) * w;
}

/** Return the area of a face */
double System::getFaceArea(Vector3i face, std::vector<Vector3d> vertices) {
    Vector3d v0 = vertices[face[0]];
    Vector3d v1 = vertices[face[1]];
    Vector3d v2 = vertices[face[2]];
    Vector3d a = v1 - v0;
    Vector3d b = v2 - v0;
    return (a.cross(b).norm()) / 2.0;
}

/** Return the normal of a face */
Vector3d System::getFaceNormal(Vector3i face, std::vector<Vector3d> vertices) {
    Vector3d v0 = vertices[face[0]];
    Vector3d v1 = vertices[face[1]];
    Vector3d v2 = vertices[face[2]];
    Vector3d a = v1 - v0;
    Vector3d b = v2 - v0;
    return a.cross(b).normalized();
}

/** Set shape model matrix */
void System::setShapeModelMatrix(Matrix4d model) {
    _shapeModelMatrix = model;
    _shapeInverseModelMatrix = model.inverse();
}

/** Return system time */
double System::getTime() {
    return _time;
}
