#ifndef UTILS_LQR
#define UTILS_LQR

struct stateVector
{
    double pendulumAngle;
    double cartPosition;
    double pendulumAngularVelocity = 6.0;
    double cartVelocity = 5.0;
};

float normalizeAngle(float angle);
float encoderCountToAngleRadians(long encoderCount, double encoderPPR);
float encoderCountToPendulumAngleRadians(long encoderCount, double encoderPPR);
float encoderCountToCartPosition(long cartEncoderCount, const double encoderPPR, const double idlerPulleyRadius);
double computeControlInput(stateVector state);

float normalizeAngle(float angle)
{
    // Constrain an angle between [-pi, pi). Output is in radians
    // see: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code

    angle = fmod(angle + PI, 2 * PI);
    if (angle < 0)
        angle += 2 * PI;
    return angle - PI;
}

float encoderCountToAngleRadians(long encoderCount, double encoderPPR)
{
    return (encoderCount / encoderPPR) * (2.0 * PI);
}

float encoderCountToPendulumAngleRadians(long encoderCount, double encoderPPR)
{
    // Since we use the <Encoder.h> library which initializes the encoder to zero at the start,
    // we need to correct the value because we start at a pendulum angle of +180 degrees.

    float pendulumAngle = PI + encoderCountToAngleRadians(encoderCount, encoderPPR);
    return normalizeAngle(pendulumAngle);
}

float encoderCountToCartPosition(long cartEncoderCount, const double encoderPPR, const double idlerPulleyRadius)
{
    float cartAngle = encoderCountToAngleRadians(cartEncoderCount, encoderPPR);
    return idlerPulleyRadius * cartAngle;
}

double computeControlInput(stateVector state, double bound)
{
    double gainVector[4] = {-2000.0, 400.0,  -100.0, 800.0};

    if (abs(state.pendulumAngle) <= bound) {
        double controlInput = (gainVector[0] * state.pendulumAngle) + (gainVector[1] * state.cartPosition) + (gainVector[2] * state.pendulumAngularVelocity) + (gainVector[3] * state.cartVelocity);
        return controlInput;
    }
    else {
        return 0.0;
    }
}

#endif