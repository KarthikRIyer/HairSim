//
// Created by Karthik Iyer on 13/11/23.
//

#ifndef HAIRSIM_HAIR_H
#define HAIRSIM_HAIR_H

#include <vector>
#include "Strand.h"

class Hair {
public:
    Hair(int particleCount, int strandCount, double mass);
    virtual ~Hair();

    void step(double h);
    void init();
    void draw();

private:
    int particleCount;
    int strandCount;
    std::vector<std::shared_ptr<Strand>> strands;
};


#endif //HAIRSIM_HAIR_H
