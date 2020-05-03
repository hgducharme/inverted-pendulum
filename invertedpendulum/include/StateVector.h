#pragma once

class StateVector
{
    public:
        double pendulumAngle = 0;
        double cartPosition = 0;
        double pendulumAngularVelocity = 0;
        double cartVelocity = 0;

        StateVector();
        StateVector(double angle, double position, double angularVelocity, double velocity);
};