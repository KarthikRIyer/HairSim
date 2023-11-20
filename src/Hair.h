//
// Created by Karthik Iyer on 13/11/23.
//

#ifndef HAIRSIM_HAIR_H
#define HAIRSIM_HAIR_H

#include <vector>
#include "Strand.h"
#include "HairVoxel.h"

class Hair {
public:
    Hair(int particleCount, int strandCount, double mass, double hairLength);
    virtual ~Hair();

    void step(double h, const Eigen::Vector3d &grav, const std::vector< std::shared_ptr<Particle> > spheres);
    void init();
    void draw();
    void reset();

private:
    int particleCount;
    int strandCount;
    double segmentLength;
    std::vector<std::shared_ptr<Strand>> strands;
    std::shared_ptr<HairVoxel> hairVoxel;
    bool handleCollision(std::shared_ptr<Particle> object, std::shared_ptr<Particle> dynamicParticle, double kc);
};


#endif //HAIRSIM_HAIR_H
