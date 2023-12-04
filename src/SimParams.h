//
// Created by Karthik Iyer on 04/12/23.
//

#ifndef HAIRSIM_SIMPARAMS_H
#define HAIRSIM_SIMPARAMS_H

struct SimParams {
    float windStrength = 0.0;
    float windOscilationSpeed = 1.0;
    double sDamping = 0.9;
    double sFriction = 0.1;
    double sRepulsion = 0.00005;
    double kc = 50.0;

    double minSDamping = 0.0;
    double maxSDamping = 1.0;
    double minSFriction = 0.0;
    double maxSFriction = 1.0;
    double minSRepulsion = 0.0;
    double maxSRepulsion = 0.0001;
    double minKc = 0.0;
    double maxKc = 100.0;
};

#endif //HAIRSIM_SIMPARAMS_H
