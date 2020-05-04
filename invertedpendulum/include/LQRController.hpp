#pragma once

#include <math.h>
#include "StateUpdater.hpp"

class LQRController
{
private:
    double (&gainVector)[4];
    double lengthOfGainVector;
    double pendulumBound;

public:
    LQRController(double (&arr)[4], double bound);
    double computeControlInput(const StateVector &state);
};