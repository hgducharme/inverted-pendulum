#include "RotaryEncoder/RotaryEncoder.h"

#ifndef INVERTEDPENDULUM_H
#define INVERTEDPENDULUM_H

class StateVector
{
public:
    double pendulumAngle;
    double cartPosition;
    double pendulumAngularVelocity;
    double cartVelocity;
};

class InvertedPendulum
{
    RotaryEncoder& cartEncoder;
    RotaryEncoder& pendulumEncoder;
    StateVector state;

public:
    InvertedPendulum(RotaryEncoder& encoder1, RotaryEncoder& encoder2);
    void updateStateVector();
};

#endif