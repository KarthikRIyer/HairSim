//
// Created by Karthik Iyer on 14/11/23.
//

#include "HairVoxel.h"
#include "Particle.h"

HairVoxel::HairVoxel(double xMin, double yMin, double zMin, double voxelSize, int voxelCount) : xMin(xMin), yMin(yMin),
zMin(zMin), voxelSize(voxelSize), voxelCount(voxelCount) {

    xMax = xMin + voxelCount * voxelSize;
    yMax = yMin + voxelCount * voxelSize;
    zMax = zMin + voxelCount * voxelSize;

    densityVoxel = std::vector<std::vector<std::vector<double>>>(voxelCount,
                                                                 std::vector<std::vector<double>>(voxelCount,
                                                                         std::vector<double>(voxelCount, 0)));
}

void HairVoxel::addParticleDensity(const std::shared_ptr<Particle>& particle) {
    Eigen::Vector3d p = particle->x;
    assert(p.x >= xMin);
    assert(p.y >= yMin);
    assert(p.z >= zMin);

    long x0Index = floor((p.x() - xMin)/voxelSize);
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