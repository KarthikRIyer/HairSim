//
// Created by Karthik Iyer on 13/11/23.
//

#include "Hair.h"
#include "Particle.h"

Hair::Hair(int particleCount, int strandCount, double mass, double hairLength): particleCount(particleCount), strandCount(strandCount) {
    strands.clear();
    strands.resize(strandCount);
    segmentLength = hairLength / (particleCount - 1);
    for (int i = 0; i < strandCount; i++) {
        std::shared_ptr<Strand> strand = std::make_shared<Strand>(mass, segmentLength, particleCount, Eigen::Vector3d(0, 0, 0));
        strands[i] = strand;
    }
}

Hair::~Hair() {}

void Hair::step(double h, const Eigen::Vector3d &grav) {
    double sDamping = 0.8;

    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        // accumulate forces
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            particle->f = particle->m * grav;
        }
        // update velocity and temp pos
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            particle->v = particle->v + h * (particle->f / particle->m);
            particle->xTemp = particle->x + particle->v * h + particle->f * h * h;
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
}