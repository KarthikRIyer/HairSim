//
// Created by Karthik Iyer on 13/11/23.
//

#ifndef HAIRSIM_HAIR_H
#define HAIRSIM_HAIR_H

#include <vector>
#include "Strand.h"
#include "HairVoxel.h"
#include "Program.h"

class IForceField;

class Hair {
public:
    Hair(int particleCount, int strandCount, double mass, double hairLength, std::string hairGenMesh);
    virtual ~Hair();

    void step(double h, std::vector<std::shared_ptr<IForceField>> &forceFields, const std::vector< std::shared_ptr<Particle> > spheres);
    void init();
    void draw(const std::shared_ptr<Program> prog);
    void reset();
    void setDampingConst(double sDamping);
    void setFrictionConst(double sFriction);
    void setRepulsionConst(double sRepulsion);

private:
    int particleCount;
    int strandCount;
    double segmentLength;
    double sDamping = 0.9;
    double sFriction = 0.1;
    double sRepulsion = 0.00005;
    std::vector<unsigned int> eleBuf;
    std::vector<float> posBuf;
    unsigned VAO;
    unsigned eleBufID;
    unsigned posBufID;
    std::vector<std::shared_ptr<Strand>> strands;
    std::shared_ptr<HairVoxel> hairVoxel;
    bool handleCollision(std::shared_ptr<Particle> object, std::shared_ptr<Particle> dynamicParticle, double kc);
};


#endif //HAIRSIM_HAIR_H
