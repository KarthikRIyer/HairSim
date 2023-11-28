//
// Created by Karthik Iyer on 13/11/23.
//

#include "Hair.h"
#include "Particle.h"
#include "GLSL.h"
#include <tbb/tbb.h>

Hair::Hair(int particleCount, int strandCount, double mass, double hairLength): particleCount(particleCount), strandCount(strandCount) {
    strands.clear();
    strands.resize(strandCount);
    segmentLength = hairLength / (particleCount - 1);
    double voxelSize = 0.5 * segmentLength;
    double coordMin = -(2 * voxelSize * (particleCount - 1) + 0.5 * voxelSize);
    hairVoxel = std::make_shared<HairVoxel>(coordMin, coordMin, coordMin,
                                            voxelSize, particleCount*5);

    std::vector<Eigen::Vector3d> dirs;
    dirs.resize(strandCount);
    double angleInc = (2*M_PI)/strandCount;
    for (int i = 0; i < strandCount; i++) {
        double angle = i*angleInc + (M_PI*0.5);
        Eigen::Vector3d dir(sin(angle),0,cos(angle));
        dirs[i] = dir;
    }

    posBuf.clear();
    eleBuf.clear();
    posBuf.resize(strandCount * particleCount * 3);
    eleBuf.resize(strandCount * particleCount * 2);

    for (int i = 0; i < strandCount; i++) {
        std::shared_ptr<Strand> strand = std::make_shared<Strand>(mass, segmentLength, particleCount, Eigen::Vector3d(0, 0, 0), dirs[i]);
        strands[i] = strand;
    }
    int posBufIndex = 0;
    int eleBufIndex = 0;
    int particleIndex = -1;
    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            posBuf[posBufIndex++] = particle->x.x();
            posBuf[posBufIndex++] = particle->x.y();
            posBuf[posBufIndex++] = particle->x.z();
            if (j != 0) {
                eleBuf[eleBufIndex++] = particleIndex;
                eleBuf[eleBufIndex++] = particleIndex+1;
            }
            particleIndex++;
        }
    }
}

Hair::~Hair() {}

void Hair::step(double h, const Eigen::Vector3d &grav, const std::vector< std::shared_ptr<Particle> > spheres) {
    double sDamping = 0.9;
    double sFriction = 0.1;
//    double sRepulsion = 0.00000;
    double sRepulsion = 0.00005;
    hairVoxel->reset();

    tbb::parallel_for((size_t)0, strands.size(), [=](size_t i){
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
//        for (int iter = 0; iter < 2; iter++) {
//
//        }
        for (int j = 1; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle0 = particles[j-1];
            std::shared_ptr<Particle> particle1 = particles[j];
            Eigen::Vector3d particle1Pos = particle1->xTemp;
            Eigen::Vector3d dir = particle1->xTemp - particle0->xTemp;
            dir.normalize();
            particle1->xTemp = particle0->xTemp + dir * segmentLength; // maintain inextensibility
            particle1->d = particle1Pos - particle1->xTemp; // correction vector

//            for (int k = 0; k < spheres.size(); k++) {
//                bool collision = handleCollision(spheres[k], particle1, 50.0);
//                if (collision) {
//                    Eigen::Vector3d collisionDir = particle1->xTemp - particle0->xTemp;
//                    collisionDir.normalize();
//                    particle1->xTemp = particle0->xTemp + collisionDir * segmentLength;
//                    particle1->d = particle1Pos - particle1->xTemp; // correction vector
//                }
//            }

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

        for (int j = 1; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle1 = particles[j];
            std::shared_ptr<Particle> particle0 = particles[j-1];
            if (particle1->fixed) continue;
            for (int k = 0; k < spheres.size(); k++) {
                bool collision = handleCollision(spheres[k], particle1, 50.0);
                if (collision) {
//                    Eigen::Vector3d dir = particle1->x - particle0->x;
//                    dir.normalize();
//                    particle1->x = particle0->x + dir * segmentLength;
                }
            }
        }
    });

//    for (int i = 0; i < strands.size(); i++) {
//        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
//        // accumulate forces
//        for (int j = 0; j < particles.size(); j++) {
//            std::shared_ptr<Particle> particle = particles[j];
//            if (particle->fixed) continue;
//            particle->f += particle->m * grav;
//        }
//        // update velocity and temp pos
//        for (int j = 0; j < particles.size(); j++) {
//            std::shared_ptr<Particle> particle = particles[j];
//            if (particle->fixed) continue;
//            particle->v = particle->v + h * (particle->f / particle->m);
//            particle->xTemp = particle->x + particle->v * h + particle->f * h * h;
//            particle->f = Eigen::Vector3d(0, 0, 0);
//        }
//
//        // solve constraint
//        for (int j = 1; j < particles.size(); j++) {
//            std::shared_ptr<Particle> particle0 = particles[j-1];
//            std::shared_ptr<Particle> particle1 = particles[j];
//            Eigen::Vector3d particle1Pos = particle1->xTemp;
//            Eigen::Vector3d dir = particle1->xTemp - particle0->xTemp;
//            dir.normalize();
//            particle1->xTemp = particle0->xTemp + dir * segmentLength; // maintain inextensibility
//            particle1->d = particle1Pos - particle1->xTemp; // correction vector
//        }
//        for (int j = 1; j < particles.size(); j++) {
//            std::shared_ptr<Particle> particle0 = particles[j-1];
//            std::shared_ptr<Particle> particle1 = particles[j];
//            if (particle0->fixed) continue;
//
//            particle0->v = ((particle0->xTemp - particle0->x)/h) + sDamping * (particle1->d/h);
//            particle0->x = particle0->xTemp;
//        }
//        std::shared_ptr<Particle> lastParticle = particles[particles.size()-1]; // no damping for last particle
//        lastParticle->v = (lastParticle->xTemp - lastParticle->x)/h;
//        lastParticle->x = lastParticle->xTemp;
//
//        for (int j = 1; j < particles.size(); j++) {
//            std::shared_ptr<Particle> particle1 = particles[j];
//            std::shared_ptr<Particle> particle0 = particles[j-1];
//            if (particle1->fixed) continue;
//            for (int k = 0; k < spheres.size(); k++) {
//                bool collision = handleCollision(spheres[k], particle1, 50.0);
//                if (collision) {
////                    Eigen::Vector3d dir = particle1->x - particle0->x;
////                    dir.normalize();
////                    particle1->x = particle0->x + dir * segmentLength;
//                }
//            }
//        }
//    }

    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
//            if (particle->fixed) continue;
            hairVoxel->addParticleDensity(particle);
        }
    }
//    hairVoxel->buildDensity();
    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
//            if (particle->fixed) continue;
            hairVoxel->addParticleVelocity(particle);
        }
    }
    // handle friction
    tbb::parallel_for((size_t)0, strands.size(), [=](int i){
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            if (particle->fixed) continue;
            Eigen::Vector3d gridVel = hairVoxel->getGridVelocity(particle->x);
            particle->v = (1.0 - sFriction) * particle->v + sFriction * gridVel;
            Eigen::Vector3d grad = hairVoxel->getGradient(particle->x);
            particle->v += (sRepulsion*-grad)/h;
        }
    });
//    for (int i = 0; i < strands.size(); i++) {
//        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
//        for (int j = 0; j < particles.size(); j++) {
//            std::shared_ptr<Particle> particle = particles[j];
//            if (particle->fixed) continue;
//            Eigen::Vector3d gridVel = hairVoxel->getGridVelocity(particle->x);
//            particle->v = (1.0 - sFriction) * particle->v + sFriction * gridVel;
//            Eigen::Vector3d grad = hairVoxel->getGradient(particle->x);
//            particle->v += (sRepulsion*-grad)/h;
//        }
//    }
    int posBufIndex = 0;
    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            posBuf[posBufIndex++] = particle->x.x();
            posBuf[posBufIndex++] = particle->x.y();
            posBuf[posBufIndex++] = particle->x.z();
        }
    }

}

bool Hair::handleCollision(std::shared_ptr<Particle> object, std::shared_ptr<Particle> dynamicParticle, double kc) {
    Eigen::Vector3d dist = dynamicParticle->xTemp - object->x;
    double distNorm = dist.norm();
    if (distNorm < object->r + dynamicParticle->r) {
        Eigen::Vector3d tVec = (dist/distNorm) * (object->r + dynamicParticle->r - distNorm);
//        dynamicParticle->xTemp += tVec;
                dynamicParticle->f += kc * tVec;
        return true;
    }
    return false;
}

void Hair::init() {
// Send the position array to the GPU
    glGenBuffers(1, &posBufID);
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_STATIC_DRAW);

    glGenBuffers(1, &eleBufID);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
    // Unbind the arrays
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    GLSL::checkError(GET_FILE_LINE);
}

void Hair::reset() {
    for (int i = 0; i < strands.size(); i++) {
        strands[i]->reset();
    }
    int posBufIndex = 0;
    for (int i = 0; i < strands.size(); i++) {
        std::vector<std::shared_ptr<Particle>> particles = strands[i]->getParticles();
        for (int j = 0; j < particles.size(); j++) {
            std::shared_ptr<Particle> particle = particles[j];
            posBuf[posBufIndex++] = particle->x.x();
            posBuf[posBufIndex++] = particle->x.y();
            posBuf[posBufIndex++] = particle->x.z();
        }
    }
}

void Hair::draw(const std::shared_ptr<Program> prog) {
    GLSL::checkError(GET_FILE_LINE);
    // Bind position buffer
    int h_pos = prog->getAttribute("aPos");
    glEnableVertexAttribArray(h_pos);
    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);

    glColor3f(0.0f, 0.0f, 1.0f);
    glPointSize(5.0);
    int count = (int)posBuf.size()/3; // number of indices to be rendered
    glDrawArrays(GL_POINTS , 0, count);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);

    // Draw
    glColor3f(0.8f, 0.8f, 0.8f);
    glLineWidth(2.0);
//    int count = (int)posBuf.size()/3; // number of indices to be rendered
//    glDrawArrays(GL_LINE_STRIP , 0, count);
    glDrawElements(GL_LINES, eleBuf.size(), GL_UNSIGNED_INT, (const void *)0);

    glDisableVertexAttribArray(h_pos);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    GLSL::checkError(GET_FILE_LINE);
//    for (int i = 0; i<strands.size();i++) {
//        strands[i]->draw();
//    }
//    hairVoxel->draw();
}