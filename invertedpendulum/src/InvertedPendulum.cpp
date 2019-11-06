#include "invertedpendulum.h"

InvertedPendulum::InvertedPendulum(RotaryEncoder& encoder1, RotaryEncoder& encoder2) 
: cartEncoder(encoder1), pendulumEncoder(encoder2)
{};

void InvertedPendulum::updateStateVector()
{
    /*
        state.pendulumAngle = encoderCountToPendulumAngleRadians(pendulumEncoderCount, ENCODER_PPR);          // radians
        state.cartPosition = encoderCountToCartPosition(cartEncoderCount, ENCODER_PPR, IDLER_PULLEY_RADIUS);  // meters
        state.pendulumAngularVelocity = (state.pendulumAngle - previousPendulumAngle) / (TIMEFRAME / 1000.0); // radians/s
        state.cartVelocity = (state.cartPosition - previousCartPosition) / (TIMEFRAME / 1000.0);              // meters/s
        */

    state.pendulumAngle = pendulumEncoder.readInRadians();
}
