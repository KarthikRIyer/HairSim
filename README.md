# Hair Simulation with PBD

## Prequisites

- OpenGL 4.x
- GLM
- GLEW
- Eigen

Set the following environment variables:
```console
GLM_INCLUDE_DIR=/Users/karthik/lib/glm-0.9.9.8;GLFW_DIR=/Users/karthik/lib/glfw-3.3.8;GLEW_DIR=/Users/karthik/lib/glew-2.1.0
```

## How to build and run

Clone the project and run the following commands in the root directory of the project:

```console
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

To run the simulator:

```console
./HairSim ../resources ../data
```