//
// Created by Karthik Iyer on 14/11/23.
//

#ifndef HAIRSIM_HAIRVOXEL_H
#define HAIRSIM_HAIRVOXEL_H

#include <vector>

class Particle;

class HairVoxel {
public:
    HairVoxel(double xMin, double yMin, double zMin, double voxelSize, int voxelCount);
    void addParticleDensity(const std::shared_ptr<Particle>& particle);
    void reset();
private:
    double xMin;
    double yMin;
    double zMin;
    double xMax;
    double yMax;
    double zMax;
    double voxelSize;
    int voxelCount;
    std::vector<std::vector<std::vector<double>>> densityVoxel;
};


#endif //HAIRSIM_HAIRVOXEL_H
