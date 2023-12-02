//
// Created by Karthik Iyer on 13/11/23.
//

#include "Hair.h"
#include "Particle.h"
#include "GLSL.h"
//#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <tbb/tbb.h>
#include <iostream>
#include <random>
#include <limits>

Hair::Hair(int particleCount, int strandCount, double mass, double hairLength, std::string hairGenMesh): particleCount(particleCount), strandCount(strandCount) {
    strands.clear();
    segmentLength = hairLength / (particleCount - 1);

    const float range_from  = 0;
    const float range_to    = 1;
    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_real_distribution<float>  distr(range_from, range_to);

    std::vector<Eigen::Vector3d> roots;
    double xmin = std::numeric_limits<double>::max(), xmax = std::numeric_limits<double>::min();
    double ymin = std::numeric_limits<double>::max(), ymax = std::numeric_limits<double>::min();
    double zmin = std::numeric_limits<double>::max(), zmax = std::numeric_limits<double>::min();

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, hairGenMesh.c_str());
    if(!warn.empty()) {
        //std::cout << warn << std::endl;
    }
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }
    if(!ret) {
        return;
    }
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            float ax, ay, az;
            float bx, by, bz;
            float cx, cy, cz;

            tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + 0];
            ax = attrib.vertices[3*idx.vertex_index+0];
            ay = attrib.vertices[3*idx.vertex_index+1];
            az = attrib.vertices[3*idx.vertex_index+2];
            Eigen::Vector3d a(ax, ay, az);

            idx = shapes[s].mesh.indices[index_offset + 1];
            bx = attrib.vertices[3*idx.vertex_index+0];
            by = attrib.vertices[3*idx.vertex_index+1];
            bz = attrib.vertices[3*idx.vertex_index+2];
            Eigen::Vector3d b(bx, by, bz);

            idx = shapes[s].mesh.indices[index_offset + 2];
            cx = attrib.vertices[3*idx.vertex_index+0];
            cy = attrib.vertices[3*idx.vertex_index+1];
            cz = attrib.vertices[3*idx.vertex_index+2];
            Eigen::Vector3d c(cx, cy, cz);

            for (int i = 0; i < 1; i++) {
                float alpha = distr(generator);
                float beta = distr(generator);
                if (alpha > beta) {
                    float t = alpha;
                    alpha = beta;
                    beta = t;
                }
                float p = alpha;
                float q = beta - alpha;
                float r = 1.0f - beta;
                Eigen::Vector3d pt = p*a + q*b + r*c;
                xmin = std::min(xmin, pt.x());
                ymin = std::min(ymin, pt.y());
                zmin = std::min(zmin, pt.z());
                xmax = std::max(xmax, pt.x());
                ymax = std::max(ymax, pt.y());
                zmax = std::max(zmax, pt.z());
                roots.push_back(pt);
            }

            index_offset += fv;
        }
    }

//    std::vector<Eigen::Vector3d> dirs;
//    dirs.resize(strandCount);
//    double angleInc = (2*M_PI)/strandCount;
//    for (int i = 0; i < strandCount; i++) {
//        double angle = i*angleInc + (M_PI*0.5);
//        Eigen::Vector3d dir(sin(angle),0,cos(angle));
//        dirs[i] = dir;
//    }

    strandCount = roots.size();
//    strandCount = 10;
    strands.resize(strandCount);
    posBuf.clear();
    eleBuf.clear();
    posBuf.resize(strandCount * particleCount * 3);
    eleBuf.resize(strandCount * particleCount * 2);

    for (int i = 0; i < strandCount; i++) {
//        std::shared_ptr<Strand> strand = std::make_shared<Strand>(mass, segmentLength, particleCount, Eigen::Vector3d(0, 0.51, -0.02), dirs[i]);
        std::shared_ptr<Strand> strand = std::make_shared<Strand>(mass, segmentLength, particleCount, roots[i], Eigen::Vector3d(0.1,1,0));
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

    double voxelSize = 0.5 * segmentLength;

    double maxX = std::max(std::abs(xmin), std::abs(xmax));
    double maxY = std::max(std::abs(ymin), std::abs(ymax));
    double maxZ = std::max(std::abs(zmin), std::abs(zmax));

    double minX = std::min(-std::abs(xmin), -std::abs(xmax));
    double minY = std::min(-std::abs(ymin), -std::abs(ymax));
    double minZ = std::min(-std::abs(zmin), -std::abs(zmax));

    double maxCoord = std::max(maxX, std::max(maxY, maxZ));
    double minCoord = std::min(minX, std::min(minY, minZ));
//    double coordMin = -(2 * voxelSize * (particleCount - 1) + 0.5 * voxelSize);
    double coordMin = minCoord -(2 * voxelSize * (particleCount - 1) + 0.5 * voxelSize);
    double coordMax = maxCoord +(2 * voxelSize * (particleCount - 1) + 0.5 * voxelSize);
    int voxelCount = ((coordMax - coordMin)/voxelSize) + 1;

//    hairVoxel = std::make_shared<HairVoxel>(coordMin, coordMin, coordMin,
//                                            voxelSize, particleCount*5);
    hairVoxel = std::make_shared<HairVoxel>(coordMin, coordMin, coordMin,
                                            voxelSize, voxelCount*2);
}

Hair::~Hair() {}

void Hair::step(double h, const Eigen::Vector3d &grav, const std::vector< std::shared_ptr<Particle> > spheres) {
    double sDamping = 0.9;
    double sFriction = 0.1;
//    double sRepulsion = 0.000000;
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

            for (int k = 0; k < spheres.size(); k++) {
                bool collision = handleCollision(spheres[k], particle1, 50.0);
//                if (collision) {
//                    Eigen::Vector3d collisionDir = particle1->xTemp - particle0->xTemp;
//                    collisionDir.normalize();
//                    particle1->xTemp = particle0->xTemp + collisionDir * segmentLength;
//                    particle1->d = particle1Pos - particle1->xTemp; // correction vector
//                }
            }

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