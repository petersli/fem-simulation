#ifndef SYSTEM_H
#define SYSTEM_H

#include <vector>
#include <map>
#include <Eigen/Dense>

const Eigen::Vector3d gravity = Eigen::Vector3d(0.0, -0.1, 0.0); // gravity
// Material params
const double lambda = 4e3; // incompressibility for the whole material
const double mu = 4e3; // rigidity for the whole material
const double phi = 100; // coefficient of viscosity
const double psi = 100; // coefficient of viscosity
const double density = 1200.0; // density
// Collision params
const double friction = 0.5;
const double groundHeight = 0;
const double sphereColliderRadius = 1.0; // constant based on sphere.mesh

// Types
struct ProcessedFace {
    Eigen::Vector3i face;
    int vertex;
    double area;
    Eigen::Vector3d normal;
    ProcessedFace() :
        face(Eigen::Vector3i(0, 0, 0)),
        vertex(0),
        area(0),
        normal(Eigen::Vector3d(0, 0, 0))
    {}
    ProcessedFace(Eigen::Vector3i face, int vertex,
                  double area, Eigen::Vector3d normal) :
        face(face),
        vertex(vertex),
        area(area),
        normal(normal)
    {}
};

struct Element {
    Eigen::Vector4i tet;
    Eigen::Matrix3d deformGrad; // deformation gradient
    Eigen::Matrix3d velGrad; // velocity gradient
    Eigen::Matrix3d beta; // for barycentric coords
    std::vector<ProcessedFace> faces; // faces
    Element(Eigen::Vector4i tet, Eigen::Matrix3d deformGrad,
        Eigen::Matrix3d velGrad, Eigen::Matrix3d beta,
        std::vector<ProcessedFace> faces) :
        tet(tet),
        deformGrad(deformGrad),
        velGrad(velGrad),
        beta(beta),
        faces(faces)
    {}
    Element(Eigen::Vector4i tet, Eigen::Matrix3d beta, std::vector<ProcessedFace> faces) :
        tet(tet),
        deformGrad(Eigen::Matrix3d::Zero()),
        velGrad(Eigen::Matrix3d::Zero()),
        beta(beta),
        faces(faces)
    {}
};
struct Node {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d force;
    double mass;
    Node(Eigen::Vector3d pos, Eigen::Vector3d vel,
             Eigen::Vector3d force, double mass) :
        pos(pos),
        vel(vel),
        force(force),
        mass(mass)
    {}
};

class System
{
public:
    System();
    System(std::vector<Eigen::Vector3d> vertices, std::vector<Eigen::Vector4i> tets);
    std::vector<Eigen::Vector3d> getVertices();
    double getTime();
    Eigen::VectorXd getState();
    void setState(Eigen::VectorXd state, double time);
    void setShapeModelMatrix(Eigen::Matrix4d model);
    Eigen::VectorXd evalTimeDerivative();
    void resolveCollisions();

private:
    Eigen::VectorXd getTimeDerivative();
    void computeForces();
    void computeDeformationGradients();
    void computeStrainAndStress();
    void computeNodeForces();
    void assignNodesMass(std::vector<Eigen::Vector4i> tets);
    Eigen::Vector3d vectorProjection(Eigen::Vector3d v, Eigen::Vector3d w);
    double tetVolume(Eigen::Vector4i tet);
    Eigen::Matrix3d computeBetaForTet(Eigen::Vector4i tet, std::vector<Eigen::Vector3d> vertices);
    std::vector<ProcessedFace> processFaces(Eigen::Vector4i tet, std::vector<Eigen::Vector3d> vertices);
    Eigen::Vector3d getFaceNormal(Eigen::Vector3i face, std::vector<Eigen::Vector3d> vertices);
    double getFaceArea(Eigen::Vector3i face, std::vector<Eigen::Vector3d> vertices);

    Eigen::Matrix4d _shapeModelMatrix;
    Eigen::Matrix4d _shapeInverseModelMatrix;
    std::map<int, Eigen::Matrix3d> _elementToStress;
    std::vector<Node> _nodes;
    std::vector<Element> _elements;
    double _time;
};

#endif // SYSTEM_H
