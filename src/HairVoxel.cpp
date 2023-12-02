//
// Created by Karthik Iyer on 14/11/23.
//

#include "HairVoxel.h"
#include "Particle.h"
#include "GLSL.h"
#include <iostream>

HairVoxel::HairVoxel(double xMin, double yMin, double zMin, double voxelSize, int voxelCount) : xMin(xMin), yMin(yMin),
zMin(zMin), voxelSize(voxelSize), voxelCount(voxelCount) {

    xMax = xMin + voxelCount * voxelSize;
    yMax = yMin + voxelCount * voxelSize;
    zMax = zMin + voxelCount * voxelSize;

//    densityVoxel = std::vector<std::vector<std::vector<double>>>(voxelCount,
//                                                                 std::vector<std::vector<double>>(voxelCount,
//                                                                         std::vector<double>(voxelCount, 0)));
//
//    velocityVoxel = std::vector<std::vector<std::vector<Eigen::Vector3d>>>(voxelCount,
//                                                                 std::vector<std::vector<Eigen::Vector3d>>(voxelCount,
//                                                                                                  std::vector<Eigen::Vector3d>(voxelCount, Eigen::Vector3d(0, 0, 0))));

    densityVoxel.clear();
    velocityVoxel.clear();
}

void HairVoxel::reset() {
//    densityVoxel = std::vector<std::vector<std::vector<double>>>(voxelCount,
//                                                                 std::vector<std::vector<double>>(voxelCount,
//                                                                                                  std::vector<double>(voxelCount, 0)));

//    for (int i = 0; i < densityVoxel.size(); i++) {
//        for (int j = 0; j < densityVoxel[0].size(); j++) {
//            for (int k = 0; k < densityVoxel[0][0].size(); k++) {
//                densityVoxel[i][j][k] = 0;
//            }
//        }
//    }
//    for (int i = 0; i < velocityVoxel.size(); i++) {
//        for (int j = 0; j < velocityVoxel[0].size(); j++) {
//            for (int k = 0; k < velocityVoxel[0][0].size(); k++) {
//                velocityVoxel[i][j][k] = Eigen::Vector3d(0, 0, 0);
//            }
//        }
//    }

    densityVoxel.clear();
    velocityVoxel.clear();

}

bool checkIfAbsent(std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> &densityVoxel, int x, int y, int z) {
    return densityVoxel.find(x) == densityVoxel.end() ||
            densityVoxel.find(x)->second.find(y) == densityVoxel.find(x)->second.end() ||
            densityVoxel.find(x)->second.find(y)->second.find(z) == densityVoxel.find(x)->second.find(y)->second.end();
}

bool checkIfAbsent(std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, Eigen::Vector3d>>> &velocityVoxel, int x, int y, int z) {
    return velocityVoxel.find(x) == velocityVoxel.end() ||
            velocityVoxel.find(x)->second.find(y) == velocityVoxel.find(x)->second.end() ||
            velocityVoxel.find(x)->second.find(y)->second.find(z) == velocityVoxel.find(x)->second.find(y)->second.end();
}

void HairVoxel::addParticleDensity(const std::shared_ptr<Particle>& particle) {
    Eigen::Vector3d p = particle->x;
    assert(p.x() >= xMin);
    assert(p.y() >= yMin);
    assert(p.z() >= zMin);

    int x0Index = floor((p.x() - xMin)/voxelSize);
    int x1Index = x0Index+1;
    double x0 = xMin + x0Index*voxelSize;
    double x1 = xMin + x1Index*voxelSize;

    int y0Index = floor((p.y() - yMin)/voxelSize);
    int y1Index = y0Index+1;
    double y0 = yMin + y0Index*voxelSize;
    double y1 = yMin + y1Index*voxelSize;

    int z0Index = floor((p.z() - zMin)/voxelSize);
    int z1Index = z0Index+1;
    double z0 = zMin + z0Index*voxelSize;
    double z1 = zMin + z1Index*voxelSize;

    if (checkIfAbsent(densityVoxel, x0Index, y0Index, z0Index)) densityVoxel[x0Index][y0Index][z0Index] = 0;
    if (checkIfAbsent(densityVoxel, x0Index, y0Index, z1Index)) densityVoxel[x0Index][y0Index][z1Index] = 0;
    if (checkIfAbsent(densityVoxel, x0Index, y1Index, z0Index)) densityVoxel[x0Index][y1Index][z0Index] = 0;
    if (checkIfAbsent(densityVoxel, x1Index, y0Index, z0Index)) densityVoxel[x1Index][y0Index][z0Index] = 0;
    if (checkIfAbsent(densityVoxel, x1Index, y1Index, z0Index)) densityVoxel[x1Index][y1Index][z0Index] = 0;
    if (checkIfAbsent(densityVoxel, x0Index, y1Index, z1Index)) densityVoxel[x0Index][y1Index][z1Index] = 0;
    if (checkIfAbsent(densityVoxel, x1Index, y0Index, z1Index)) densityVoxel[x1Index][y0Index][z1Index] = 0;
    if (checkIfAbsent(densityVoxel, x1Index, y1Index, z1Index)) densityVoxel[x1Index][y1Index][z1Index] = 0;

    densityVoxel[x0Index][y0Index][z0Index] += ((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z0)));
    densityVoxel[x0Index][y0Index][z1Index] += ((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z1)));
    densityVoxel[x0Index][y1Index][z0Index] += ((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z0)));
    densityVoxel[x1Index][y0Index][z0Index] += ((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z0)));
    densityVoxel[x1Index][y1Index][z0Index] += ((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z0)));
    densityVoxel[x0Index][y1Index][z1Index] += ((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z1)));
    densityVoxel[x1Index][y0Index][z1Index] += ((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z1)));
    densityVoxel[x1Index][y1Index][z1Index] += ((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z1)));

}

void HairVoxel::buildDensity() {
    double x = 10;
    for (int i = 0; i < densityVoxel.size(); i++) {
        for (int j = 0; j < densityVoxel[0].size(); j++) {
            for (int k = 0; k < densityVoxel[0][0].size(); k++) {
                double x0 = xMin + i*voxelSize;
                double y0 = yMin + j*voxelSize;
                double z0 = zMin + k*voxelSize;
                densityVoxel[i][j][k] = sqrt(x0*x0 + y0*y0 + z0*z0);
            }
        }
    }
}

void HairVoxel::addParticleVelocity(const std::shared_ptr<Particle>& particle) {
    Eigen::Vector3d p = particle->x;
    assert(p.x() >= xMin);
    assert(p.y() >= yMin);
    assert(p.z() >= zMin);

    int x0Index = floor((p.x() - xMin)/voxelSize);
    int x1Index = x0Index+1;
    double x0 = xMin + x0Index*voxelSize;
    double x1 = xMin + x1Index*voxelSize;

    int y0Index = floor((p.y() - yMin)/voxelSize);
    int y1Index = y0Index+1;
    double y0 = yMin + y0Index*voxelSize;
    double y1 = yMin + y1Index*voxelSize;

    int z0Index = floor((p.z() - zMin)/voxelSize);
    int z1Index = z0Index+1;
    double z0 = zMin + z0Index*voxelSize;
    double z1 = zMin + z1Index*voxelSize;

    if (checkIfAbsent(velocityVoxel, x0Index, y0Index, z0Index)) velocityVoxel[x0Index][y0Index][z0Index] = Eigen::Vector3d(0,0,0);
    if (checkIfAbsent(velocityVoxel, x0Index, y0Index, z1Index)) velocityVoxel[x0Index][y0Index][z1Index] = Eigen::Vector3d(0,0,0);
    if (checkIfAbsent(velocityVoxel, x0Index, y1Index, z0Index)) velocityVoxel[x0Index][y1Index][z0Index] = Eigen::Vector3d(0,0,0);
    if (checkIfAbsent(velocityVoxel, x1Index, y0Index, z0Index)) velocityVoxel[x1Index][y0Index][z0Index] = Eigen::Vector3d(0,0,0);
    if (checkIfAbsent(velocityVoxel, x1Index, y1Index, z0Index)) velocityVoxel[x1Index][y1Index][z0Index] = Eigen::Vector3d(0,0,0);
    if (checkIfAbsent(velocityVoxel, x0Index, y1Index, z1Index)) velocityVoxel[x0Index][y1Index][z1Index] = Eigen::Vector3d(0,0,0);
    if (checkIfAbsent(velocityVoxel, x1Index, y0Index, z1Index)) velocityVoxel[x1Index][y0Index][z1Index] = Eigen::Vector3d(0,0,0);
    if (checkIfAbsent(velocityVoxel, x1Index, y1Index, z1Index)) velocityVoxel[x1Index][y1Index][z1Index] = Eigen::Vector3d(0,0,0);

    velocityVoxel[x0Index][y0Index][z0Index] += (((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z0))) * particle->v)/densityVoxel[x0Index][y0Index][z0Index];
    velocityVoxel[x0Index][y0Index][z1Index] += (((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z1))) * particle->v)/densityVoxel[x0Index][y0Index][z1Index];
    velocityVoxel[x0Index][y1Index][z0Index] += (((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z0))) * particle->v)/densityVoxel[x0Index][y1Index][z0Index];
    velocityVoxel[x1Index][y0Index][z0Index] += (((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z0))) * particle->v)/densityVoxel[x1Index][y0Index][z0Index];
    velocityVoxel[x1Index][y1Index][z0Index] += (((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z0))) * particle->v)/densityVoxel[x1Index][y1Index][z0Index];
    velocityVoxel[x0Index][y1Index][z1Index] += (((1.0 - abs(p.x() - x0))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z1))) * particle->v)/densityVoxel[x0Index][y1Index][z1Index];
    velocityVoxel[x1Index][y0Index][z1Index] += (((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y0))*(1.0 - abs(p.z() - z1))) * particle->v)/densityVoxel[x1Index][y0Index][z1Index];
    velocityVoxel[x1Index][y1Index][z1Index] += (((1.0 - abs(p.x() - x1))*(1.0 - abs(p.y() - y1))*(1.0 - abs(p.z() - z1))) * particle->v)/densityVoxel[x1Index][y1Index][z1Index];
}

Eigen::Vector3d HairVoxel::getGridVelocity(const Eigen::Vector3d& pos) {
    int x0Index = floor((pos.x() - xMin)/voxelSize);
    int x1Index = x0Index+1;
    double x0 = xMin + x0Index*voxelSize;
//    double x1 = xMin + x1Index*voxelSize;

    int y0Index = floor((pos.y() - yMin)/voxelSize);
    int y1Index = y0Index+1;
    double y0 = yMin + y0Index*voxelSize;
//    double y1 = yMin + y1Index*voxelSize;

    int z0Index = floor((pos.z() - zMin)/voxelSize);
    int z1Index = z0Index+1;
    double z0 = zMin + z0Index*voxelSize;
//    double z1 = zMin + z1Index*voxelSize;

    double xu = (pos.x() - x0)/(voxelSize);
    double yu = (pos.y() - y0)/(voxelSize);
    double zu = (pos.z() - z0)/(voxelSize);

    Eigen::Vector3d velocityX0Y0Z0 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x0Index, y0Index, z0Index)) velocityX0Y0Z0 = velocityVoxel[x0Index][y0Index][z0Index];
    Eigen::Vector3d velocityX1Y0Z0 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x1Index, y0Index, z0Index)) velocityX1Y0Z0 = velocityVoxel[x1Index][y0Index][z0Index];
    Eigen::Vector3d velocityX0Y1Z0 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x0Index, y1Index, z0Index)) velocityX0Y1Z0 = velocityVoxel[x0Index][y1Index][z0Index];
    Eigen::Vector3d velocityX1Y1Z0 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x1Index, y1Index, z0Index)) velocityX1Y1Z0 = velocityVoxel[x1Index][y1Index][z0Index];

    Eigen::Vector3d blx0 = (1-xu)*velocityX0Y0Z0 + (xu)*velocityX1Y0Z0;
    Eigen::Vector3d blx1 = (1-xu)*velocityX0Y1Z0 + (xu)*velocityX1Y1Z0;
    Eigen::Vector3d blz0 = (1-yu)*blx0 + (yu)*blx1;

    Eigen::Vector3d velocityX0Y0Z1 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x0Index, y0Index, z1Index)) velocityX0Y0Z1 = velocityVoxel[x0Index][y0Index][z1Index];
    Eigen::Vector3d velocityX1Y0Z1 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x1Index, y0Index, z1Index)) velocityX1Y0Z1 = velocityVoxel[x1Index][y0Index][z1Index];
    Eigen::Vector3d velocityX0Y1Z1 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x0Index, y1Index, z1Index)) velocityX0Y1Z1 = velocityVoxel[x0Index][y1Index][z1Index];
    Eigen::Vector3d velocityX1Y1Z1 = Eigen::Vector3d(0,0,0);
    if (!checkIfAbsent(velocityVoxel, x1Index, y1Index, z1Index)) velocityX1Y1Z1 = velocityVoxel[x1Index][y1Index][z1Index];

    Eigen::Vector3d blx2 = (1-xu)*velocityX0Y0Z1 + (xu)*velocityX1Y0Z1;
    Eigen::Vector3d blx3 = (1-xu)*velocityX0Y1Z1 + (xu)*velocityX1Y1Z1;
    Eigen::Vector3d blz1 = (1-yu)*blx2 + (yu)*blx3;

    Eigen::Vector3d gridVel = (1-zu)*blz0 + (zu)*blz1;
    return gridVel;
}

Eigen::Vector3d HairVoxel::getGradient(const Eigen::Vector3d& pos) {
    int x0Index = floor((pos.x() - xMin)/voxelSize);
    int x1Index = x0Index+1;
    double x0 = xMin + x0Index*voxelSize;

    int y0Index = floor((pos.y() - yMin)/voxelSize);
    int y1Index = y0Index+1;
    double y0 = yMin + y0Index*voxelSize;

    int z0Index = floor((pos.z() - zMin)/voxelSize);
    int z1Index = z0Index+1;
    double z0 = zMin + z0Index*voxelSize;

    double xu = (pos.x() - x0)/(voxelSize);
    double yu = (pos.y() - y0)/(voxelSize);
    double zu = (pos.z() - z0)/(voxelSize);

    // x grad
    double densityX0Y0Z0 = 0;
    if (!checkIfAbsent(densityVoxel, x0Index, y0Index, z0Index)) densityX0Y0Z0 = densityVoxel[x0Index][y0Index][z0Index];
    double densityX0Y0Z1 = 0;
    if (!checkIfAbsent(densityVoxel, x0Index, y0Index, z1Index)) densityX0Y0Z1 = densityVoxel[x0Index][y0Index][z1Index];
    double densityX0Y1Z0 = 0;
    if (!checkIfAbsent(densityVoxel, x0Index, y1Index, z0Index)) densityX0Y1Z0 = densityVoxel[x0Index][y1Index][z0Index];
    double densityX0Y1Z1 = 0;
    if (!checkIfAbsent(densityVoxel, x0Index, y1Index, z1Index)) densityX0Y1Z1 = densityVoxel[x0Index][y1Index][z1Index];

    double densityX1Y0Z0 = 0;
    if (!checkIfAbsent(densityVoxel, x1Index, y0Index, z0Index)) densityX1Y0Z0 = densityVoxel[x1Index][y0Index][z0Index];
    double densityX1Y0Z1 = 0;
    if (!checkIfAbsent(densityVoxel, x1Index, y0Index, z1Index)) densityX1Y0Z1 = densityVoxel[x1Index][y0Index][z1Index];
    double densityX1Y1Z0 = 0;
    if (!checkIfAbsent(densityVoxel, x1Index, y1Index, z0Index)) densityX1Y1Z0 = densityVoxel[x1Index][y1Index][z0Index];
    double densityX1Y1Z1 = 0;
    if (!checkIfAbsent(densityVoxel, x1Index, y1Index, z1Index)) densityX1Y1Z1 = densityVoxel[x1Index][y1Index][z1Index];

    double blz0 = (1-zu)*densityX0Y0Z0 + zu*densityX0Y0Z1;
    double blz1 = (1-zu)*densityX0Y1Z0 + zu*densityX0Y1Z1;
    double bly0 = (1-yu)*blz0 + yu*blz1;
    double blz2 = (1-zu)*densityX1Y0Z0 + zu*densityX1Y0Z1;
    double blz3 = (1-zu)*densityX1Y1Z0 + zu*densityX1Y1Z1;
    double bly1 = (1-yu)*blz2 + yu*blz3;
    double xGrad = (bly1 - bly0)/voxelSize;

    // y grad
    blz0 = (1-zu)*densityX0Y0Z0 + zu*densityX0Y0Z1;
    blz1 = (1-zu)*densityX1Y0Z0 + zu*densityX1Y0Z1;
    double blx0 = (1-xu)*blz0 + xu*blz1;
    blz2 = (1-zu)*densityX0Y1Z0 + zu*densityX0Y1Z1;
    blz3 = (1-zu)*densityX1Y1Z0 + zu*densityX1Y1Z1;
    double blx1 = (1-xu)*blz2 + xu*blz3;
    double yGrad = (blx1 - blx0)/voxelSize;

    // z grad
    bly0 = (1-yu)*densityX0Y0Z0 + zu*densityX0Y1Z0;
    bly1 = (1-yu)*densityX1Y0Z0 + zu*densityX1Y1Z1;
    blx0 = (1-xu)*bly0 + xu*bly1;
    double bly2 = (1-zu)*densityX0Y0Z1 + zu*densityX0Y1Z1;
    double bly3 = (1-zu)*densityX1Y0Z1 + zu*densityX1Y1Z1;
    blx1 = (1-xu)*bly2 + xu*bly3;
    double zGrad = (blx1 - blx0)/voxelSize;

    Eigen::Vector3d grad(xGrad, yGrad, zGrad);
    grad.normalize();
    return grad;
}

void HairVoxel::draw() {
    glColor3f(0.8, 0.8,0.8);
    glPointSize(10);
    glBegin(GL_POINTS);
    glVertex3d(0.0, 0.0, 1.0); // debug drawing to show positive z direction
    glEnd();
    for (int i = 0; i < densityVoxel.size(); i++) {
        for (int j = 0; j < densityVoxel[0].size(); j++) {
            for (int k = 0; k < densityVoxel[0][0].size(); k++) {
                double x = xMin + i*voxelSize;
                double y = yMin + j*voxelSize;
                double z = zMin + k*voxelSize;
                double density = densityVoxel[i][j][k];
//                glColor3f(density, density, density);
                if (density == 0) continue;
                glPointSize(density*2);
                glBegin(GL_POINTS);
                glVertex3d(x, y, z);
//                std::cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<"\n";
                glEnd();
                GLSL::checkError(GET_FILE_LINE);
            }
        }
    }
}