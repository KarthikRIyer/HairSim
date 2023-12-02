//
// Created by Karthik Iyer on 14/11/23.
//

#ifndef HAIRSIM_HAIRVOXEL_H
#define HAIRSIM_HAIRVOXEL_H

#include <vector>
#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Particle;

class HairVoxel {
public:
    HairVoxel(double xMin, double yMin, double zMin, double voxelSize, int voxelCount);
    void addParticleDensity(const std::shared_ptr<Particle>& particle);
    void buildDensity();
    void addParticleVelocity(const std::shared_ptr<Particle>& particle);
    Eigen::Vector3d getGridVelocity(const Eigen::Vector3d& pos);
    Eigen::Vector3d getGradient(const Eigen::Vector3d& pos);
    void reset();
    void draw();
private:
    double xMin;
    double yMin;
    double zMin;
    double xMax;
    double yMax;
    double zMax;
    double voxelSize;
    int voxelCount;
//    std::vector<std::vector<std::vector<double>>> densityVoxel;
//    std::vector<std::vector<std::vector<Eigen::Vector3d>>> velocityVoxel;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> densityVoxel;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, Eigen::Vector3d>>> velocityVoxel;
};


#endif //HAIRSIM_HAIRVOXEL_H
