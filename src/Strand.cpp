//
// Created by Karthik Iyer on 13/11/23.
//

#include "Strand.h"
#include "Particle.h"
#include "GLSL.h"

Strand::Strand(double mass, double hairLength, int particleCount, const Eigen::Vector3d& rootLoc) {
    particles.clear();
    particles.resize(particleCount);

    segmentLength = hairLength / (particleCount - 1);

    Eigen::Vector3d dir(1, 0, 0);

    for (int i = 0; i < particleCount; i++) {
        std::shared_ptr<Particle> particle = std::make_shared<Particle>();
        particle->x0 = rootLoc + i * segmentLength * dir;
        particle->x = particle->x0;
        particle->m = mass;
        particle->r = 0.1;
        particle->v = Eigen::Vector3d(0, 0, 0);
        if (i == 0) {
            particle->fixed = true;
        }
        particles[i] = particle;
    }
}

Strand::~Strand() {}

void Strand::draw() {
    glColor3f(0, 0, 1.0);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (int i = 0; i < particles.size(); i++) {
        glVertex3f(particles[i]->x.x(), particles[i]->x.y(), particles[i]->x.z());
    }
    glEnd();
}