# Hair Simulation with PBD

![image](https://raw.githubusercontent.com/KarthikRIyer/HairSim/main/img/hair-sim.png?token=GHSAT0AAAAAACJ2SG5MNQTENA4NZWQY6KE4ZLX6BKA)

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