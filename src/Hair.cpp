//
// Created by Karthik Iyer on 13/11/23.
//

#include "Hair.h"
#include "Particle.h"

Hair::Hair(int particleCount, int strandCount, double mass, double hairLength): particleCount(particleCount), strandCount(strandCount) {
    strands.clear();
    strands.resize(strandCount);
    segmentLength = hairLength / (particleCount - 1);

    hairVoxel = std::make_shared<HairVoxel>(-hairLength - 0.1, -hairLength - 0.1, -hairLength - 0.1,
                                            segmentLength/2, particleCount*5);

    std::vector<Eigen::Vector3d> dirs;
    dirs.resize(strandCount);
    double angleInc = (2*M_PI)/strandCount;
    for (int i = 0; i < strandCount; i++) {
        double angle = i*angleInc;
        Eigen::Vector3d dir(sin(angle),0,cos(angle));
        dirs[i] = dir;
    }

    for (int i = 0; i < strandCount; i++) {
        std::shared_ptr<Strand> strand = std::make_shared<Strand>(mass, segmentLength, particleCount, Eigen::Vector3d(0, 0, 0), dirs[i]);
        strands[i] = strand;
    }
}

Hair::~Hair() {}

void Hair::step(double h, const Eigen::Vector3d &grav, const std::vector< std::shared_ptr<Particle> > spheres) {
    double sDamping = 0.9;
    double sFriction = 0.1;
//    double sRepulsion = 0.00005;
    double sRepulsion = 0.00005;
    hairVoxel->reset();
    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        // accumulate forces
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            particle->f += particle->m * grav;
        }
        // update velocity and temp pos
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            particle->v = particle->v + h * (particle->f / particle->m);
            particle->xTemp = particle->x + particle->v * h + particle->f * h * h;
            particle->f = Eigen::Vector3d(0, 0, 0);
        }

        // solve constraint
        for (int j = 1; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle0 = particles[j-1];
            std::shared_ptr<Particle> particle1 = particles[j];
            Eigen::Vector3d particle1Pos = particle1->xTemp;
            Eigen::Vector3d dir = particle1->xTemp - particle0->xTemp;
            dir.normalize();
            particle1->xTemp = particle0->xTemp + dir * segmentLength; // maintain inextensibility
            particle1->d = particle1Pos - particle1->xTemp; // correction vector
        }
        for (int j = 1; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle0 = particles[j-1];
            std::shared_ptr<Particle> particle1 = particles[j];
            if (particle0->fixed) continue;

            particle0->v = ((particle0->xTemp - particle0->x)/h) + sDamping * (particle1->d/h);
            particle0->x = particle0->xTemp;
        }
        std::shared_ptr<Particle> lastParticle = particles[particles.size()-1]; // no damping for last particle
        lastParticle->v = (lastParticle->xTemp - lastParticle->x)/h;
        lastParticle->x = lastParticle->xTemp;

        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            for (int k = 0; k < spheres.size(); k++) {
                handleCollision(spheres[k], particle, 50.0);
            }
        }
    }

    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            hairVoxel->addParticleDensity(particle);
        }
    }
//    hairVoxel->buildDensity();
    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            hairVoxel->addParticleVelocity(particle);
        }
    }
    // handle friction
    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            Eigen::Vector3d gridVel = hairVoxel->getGridVelocity(particle->x);
            particle->v = (1.0 - sFriction) * particle->v + sFriction * gridVel;
            Eigen::Vector3d grad = hairVoxel->getGradient(particle->x);
            particle->v += (sRepulsion*-grad)/h;
        }
    }

}

void Hair::handleCollision(std::shared_ptr<Particle> object, std::shared_ptr<Particle> dynamicParticle, double kc) {
    Eigen::Vector3d dist = dynamicParticle->x - object->x;
    double distNorm = dist.norm();
    if (distNorm <= object->r + dynamicParticle->r) {
        Eigen::Vector3d tVec = (dist/distNorm) * (object->r + dynamicParticle->r - distNorm);
        dynamicParticle->f += kc * tVec;
    }
}

void Hair::init() {

}

void Hair::reset() {
    for (int i = 0; i < strands.size(); i++) {
        strands[i]->reset();
    }
}

void Hair::draw() {
    for (int i = 0; i<strands.size();i++) {
        strands[i]->draw();
    }
//    hairVoxel->draw();
}