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

void HairVoxel::draw() {
    glColor3f(0.8, 0.8,0.8);
    for (int i = 0; i < densityVoxel.size(); i++) {
        for (int j = 0; j < densityVoxel[0].size(); j++) {
            for (int k = 0; k < densityVoxel[0][0].size(); k++) {
                double x = xMin + i*voxelSize;
                double y = yMin + j*voxelSize;
                double z = zMin + k*voxelSize;
                double density = densityVoxel[i][j][k];
                if (density == 0) continue;
                glPointSize(density);
                glBegin(GL_POINTS);
                glVertex3d(x, y, z);
//                std::cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<"\n";
                glEnd();
                GLSL::checkError(GET_FILE_LINE);
            }
        }
    }
}