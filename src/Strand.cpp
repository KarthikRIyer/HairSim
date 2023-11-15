//
// Created by Karthik Iyer on 13/11/23.
//

#include "Strand.h"
#include "Particle.h"
#include "GLSL.h"

Strand::Strand(double mass, double segmentLength, int particleCount, const Eigen::Vector3d& rootLoc) : segmentLength(segmentLength) {
    particles.clear();
    particles.resize(particleCount);

    Eigen::Vector3d dir(1, 0, 0);
    double particleMass = mass/particleCount;
    for (int i = 0; i < particleCount; i++) {
        std::shared_ptr<Particle> particle = std::make_shared<Particle>();
        particle->x0 = rootLoc + i * segmentLength * dir;
        particle->x = particle->x0;
        particle->xTemp = particle->x0;
        particle->m = particleMass;
        particle->r = 0.1;
        particle->v = Eigen::Vector3d(0, 0, 0);
        if (i == 0) {
            particle->fixed = true;
        } else {
            particle->fixed = false;
        }
        particles[i] = particle;
    }
}

Strand::~Strand() {}

std::vector<std::shared_ptr<Particle>>& Strand::getParticles() {
    return particles;
}

void Strand::reset() {
    for (int i = 0; i < particles.size(); i++) {
        auto particle = particles[i];
        particle->x = particle->x0;
        particle->v = particle->v0;
    }
}

void Strand::draw() {
    glColor3f(0, 0, 1.0);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (int i = 0; i < particles.size(); i++) {
        glVertex3f(particles[i]->x.x(), particles[i]->x.y(), particles[i]->x.z());
    }
    glEnd();
    glColor3f(0.8, 0.8,0.8);
    glLineWidth(2.0);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < particles.size(); i++) {
        glVertex3f(particles[i]->x.x(), particles[i]->x.y(), particles[i]->x.z());
    }
    glEnd();
}