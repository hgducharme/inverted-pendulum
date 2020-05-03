#include "StateVector.h"

StateVector::StateVector() {};

StateVector::StateVector(double _pendulumAngle = 0, double _cartPosition = 0, double _pendulumAngularVelocity = 0, double _cartVelocity = 0) {
    pendulumAngle = _pendulumAngle;
    cartPosition = _cartPosition;
    pendulumAngularVelocity = _pendulumAngularVelocity;
    cartVelocity = _cartVelocity;
}