//
// Created by Karthik Iyer on 13/11/23.
//

#include "Hair.h"

Hair::Hair(int particleCount, int strandCount, double mass): particleCount(particleCount), strandCount(strandCount) {
    strands.clear();
    strands.resize(strandCount);

    for (int i = 0; i < strandCount; i++) {
        std::shared_ptr<Strand> strand = std::make_shared<Strand>(mass, 0.4, particleCount, Eigen::Vector3d(0, 0, 0));
        strands[i] = strand;
    }
}

Hair::~Hair() {}

void Hair::step(double h) {}

void Hair::init() {

}

void Hair::draw() {
    for (int i = 0; i<strands.size();i++) {
        strands[i]->draw();
    }
}