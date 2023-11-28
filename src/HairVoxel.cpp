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

    densityVoxel = std::vector<std::vector<std::vector<double>>>(voxelCount,
                                                                 std::vector<std::vector<double>>(voxelCount,
                                                                         std::vector<double>(voxelCount, 0)));

    velocityVoxel = std::vector<std::vector<std::vector<Eigen::Vector3d>>>(voxelCount,
                                                                 std::vector<std::vector<Eigen::Vector3d>>(voxelCount,
                                                                                                  std::vector<Eigen::Vector3d>(voxelCount, Eigen::Vector3d(0, 0, 0))));
}

void HairVoxel::reset() {
//    densityVoxel = std::vector<std::vector<std::vector<double>>>(voxelCount,
//                                                                 std::vector<std::vector<double>>(voxelCount,
//                                                                                                  std::vector<double>(voxelCount, 0)));

    for (int i = 0; i < densityVoxel.size(); i++) {
        for (int j = 0; j < densityVoxel[0].size(); j++) {
            for (int k = 0; k < densityVoxel[0][0].size(); k++) {
                densityVoxel[i][j][k] = 0;
            }
        }
    }
    for (int i = 0; i < velocityVoxel.size(); i++) {
        for (int j = 0; j < velocityVoxel[0].size(); j++) {
            for (int k = 0; k < velocityVoxel[0][0].size(); k++) {
                velocityVoxel[i][j][k] = Eigen::Vector3d(0, 0, 0);
            }
        }
    }

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

    Eigen::Vector3d blx0 = (1-xu)*velocityVoxel[x0Index][y0Index][z0Index] + (xu)*velocityVoxel[x1Index][y0Index][z0Index];
    Eigen::Vector3d blx1 = (1-xu)*velocityVoxel[x0Index][y1Index][z0Index] + (xu)*velocityVoxel[x1Index][y1Index][z0Index];
    Eigen::Vector3d blz0 = (1-yu)*blx0 + (yu)*blx1;

    Eigen::Vector3d blx2 = (1-xu)*velocityVoxel[x0Index][y0Index][z1Index] + (xu)*velocityVoxel[x1Index][y0Index][z1Index];
    Eigen::Vector3d blx3 = (1-xu)*velocityVoxel[x0Index][y1Index][z1Index] + (xu)*velocityVoxel[x1Index][y1Index][z1Index];
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
    double blz0 = (1-zu)*densityVoxel[x0Index][y0Index][z0Index] + zu*densityVoxel[x0Index][y0Index][z1Index];
    double blz1 = (1-zu)*densityVoxel[x0Index][y1Index][z0Index] + zu*densityVoxel[x0Index][y1Index][z1Index];
    double bly0 = (1-yu)*blz0 + yu*blz1;
    double blz2 = (1-zu)*densityVoxel[x1Index][y0Index][z0Index] + zu*densityVoxel[x1Index][y0Index][z1Index];
    double blz3 = (1-zu)*densityVoxel[x1Index][y1Index][z0Index] + zu*densityVoxel[x1Index][y1Index][z1Index];
    double bly1 = (1-yu)*blz2 + yu*blz3;
    double xGrad = (bly1 - bly0)/voxelSize;

    // y grad
    blz0 = (1-zu)*densityVoxel[x0Index][y0Index][z0Index] + zu*densityVoxel[x0Index][y0Index][z1Index];
    blz1 = (1-zu)*densityVoxel[x1Index][y0Index][z0Index] + zu*densityVoxel[x1Index][y0Index][z1Index];
    double blx0 = (1-xu)*blz0 + xu*blz1;
    blz2 = (1-zu)*densityVoxel[x0Index][y1Index][z0Index] + zu*densityVoxel[x0Index][y1Index][z1Index];
    blz3 = (1-zu)*densityVoxel[x1Index][y1Index][z0Index] + zu*densityVoxel[x1Index][y1Index][z1Index];
    double blx1 = (1-xu)*blz2 + xu*blz3;
    double yGrad = (blx1 - blx0)/voxelSize;

    // z grad
    bly0 = (1-yu)*densityVoxel[x0Index][y0Index][z0Index] + zu*densityVoxel[x0Index][y1Index][z0Index];
    bly1 = (1-yu)*densityVoxel[x1Index][y0Index][z0Index] + zu*densityVoxel[x1Index][y1Index][z1Index];
    blx0 = (1-xu)*bly0 + xu*bly1;
    double bly2 = (1-zu)*densityVoxel[x0Index][y0Index][z1Index] + zu*densityVoxel[x0Index][y1Index][z1Index];
    double bly3 = (1-zu)*densityVoxel[x1Index][y0Index][z1Index] + zu*densityVoxel[x1Index][y1Index][z1Index];
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