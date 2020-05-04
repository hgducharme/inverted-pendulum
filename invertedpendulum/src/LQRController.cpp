#include "LQRController.hpp"

LQRController::LQRController(double (&arr)[4], double bound) : gainVector(arr)
{
    lengthOfGainVector = sizeof(gainVector) / sizeof(gainVector[0]);
    pendulumBound = bound;
}

double LQRController::computeControlInput(const StateVector &state)
{
    double controlInput = 0.0;

    if (abs(state.pendulumAngle) <= pendulumBound) {
        double controlInput = (gainVector[0] * state.pendulumAngle) + (gainVector[1] * state.cartPosition) + (gainVector[2] * state.pendulumAngularVelocity) + (gainVector[3] * state.cartVelocity);
    }

    return controlInput;
}