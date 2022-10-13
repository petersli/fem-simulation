# FEM simulation
<img width="1340" alt="Screen Shot 2022-10-12 at 8 23 01 PM" src="https://user-images.githubusercontent.com/25734828/195471280-a773c91f-e6d6-4122-b344-16d680c6bdea.png">

This program uses the finite element method (FEM) to simulate the continuum mechanics of deformable solids. It computes stress and strain (Green's strain) to get internal forces for each tetrahedron. Collision detection is implemented for the ground plane and spherical colliders. 

## Extra features

The code is parallelized, and the 4th-order Runge–Kutta (RK4) integrator is used to move the simulation forward in time. Eulerian (1st order) and midpoint (2nd order) integrators are also implemented, and can be used if desired.

## Running

To build and run the program, open `simulation.pro` in QT Creator and click "Run". Make sure to set the working directory to the root of the repository. 

To specify the path of the tetrahedral mesh to simulate, you can edit the `path` variable in `config.ini`. Material and collision properties can be found in `system.h`.

## Controls
- WASD to move camera horizontally, QE to move camera up/down
- P to pause/unpause simulation
- R to restart simulation (and read in new config values)

## Videos

These videos demonstrate the simulation running with the RK4 integrator and these material params:

```
const Vector3d gravity = Vector3d(0.0, -0.1, 0.0);
const double lambda = 4e3; // incompressibility for the whole material
const double mu = 4e3; // rigidity for the whole material
const double phi = 100; // coefficient of viscosity
const double psi = 100; // coefficient of viscosity
const double density = 1200.0;
```
Ellipsoid

https://user-images.githubusercontent.com/25734828/159385269-5b47537d-e708-4e7f-9554-cde3c126e362.mp4

Cube

https://user-images.githubusercontent.com/25734828/159385511-6f31fe43-5658-4e9b-b943-499761cf4772.mov

Sphere

https://user-images.githubusercontent.com/25734828/159389095-c9e09646-6071-4839-9e8d-577b83c1e28a.mp4






