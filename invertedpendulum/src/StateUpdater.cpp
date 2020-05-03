#include "StateUpdater.hpp"

StateUpdater::StateUpdater(EncoderWrapper & c, EncoderWrapper & p, double pulleyRadius, double loopRate) : 
    cartEncoder(c), pendulumEncoder(p), IDLER_PULLEY_RADIUS(pulleyRadius), SYSTEM_LOOP_RATE_SECONDS(loopRate / 1000) {};

void StateUpdater::update(StateVector & state, StateVector & previousState) {
    state.pendulumAngle = calculatePendulumAngle();
    state.cartPosition = calculateCartPosition();
    state.pendulumAngularVelocity = calculatePendulumAngularVelocity(state.pendulumAngle, previousState.pendulumAngle);
    state.cartVelocity = calculateCartVelocity(state.cartPosition, previousState.cartPosition);
}

double StateUpdater::calculatePendulumAngle() {
    double pendulumAngle = PI + pendulumEncoder.readRadians();
    return normalizeAngle(pendulumAngle);
}

double StateUpdater::normalizeAngle(double angle)
{
    // Constrain an angle between [-pi, pi). Output is in radians
    // see: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code

    angle = fmod(angle + PI, 2 * PI);
    if (angle < 0)
            angle += 2 * PI;
    return angle - PI;
}

double StateUpdater::calculateCartPosition()
{
    return IDLER_PULLEY_RADIUS * cartEncoder.readRadians();
}

double StateUpdater::calculatePendulumAngularVelocity(double currentPendulumAngle, double previousPendulumAngle) {
    return (currentPendulumAngle - previousPendulumAngle) / SYSTEM_LOOP_RATE_SECONDS;
}

double StateUpdater::calculateCartVelocity(double currentCartPosition, double previousCartPosition) {
    return (currentCartPosition - previousCartPosition) / SYSTEM_LOOP_RATE_SECONDS;
}