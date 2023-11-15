//
// Created by Karthik Iyer on 13/11/23.
//

#ifndef HAIRSIM_STRAND_H
#define HAIRSIM_STRAND_H

#include <vector>
#include <glm/vec3.hpp>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Particle;

class Strand {
public:
    Strand(double mass, double segmentLength, int particleCount, const Eigen::Vector3d& rootLoc);
    virtual ~Strand();
    void draw();
    void reset();
    std::vector<std::shared_ptr<Particle>>& getParticles();
private:
    std::vector<std::shared_ptr<Particle>> particles;
    double segmentLength;
};


#endif //HAIRSIM_STRAND_H
