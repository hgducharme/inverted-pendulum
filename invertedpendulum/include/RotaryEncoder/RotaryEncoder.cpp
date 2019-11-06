#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(int phaseA, int phaseB, double encoderPPR) 
: encoder(phaseA, phaseB), encoderPPR(encoderPPR)
{}

inline long RotaryEncoder::getCount()
{
    return encoder.read();
}

inline double RotaryEncoder::getRadians(double angleOffset = 0)
{
    long encoderCount = getCount();
    double radians = angleOffset + convertEncoderCountToRadians(encoderCount);
    radians = normalizeAngle(radians);

    return radians;
}

inline double RotaryEncoder::convertEncoderCountToRadians(long encoderCount)
{
    return (encoderCount / (encoderPPR)) * (2.0 * PI);
}

inline double RotaryEncoder::normalizeAngle(double angle)
{
    // Constrain an angle between [-pi, pi). Output is in radians
    // see: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code

    angle = fmod(angle + PI, 2 * PI);
    
    if (angle < 0)
        angle += 2 * PI;

    return angle - PI;
}

inline void RotaryEncoder::setAngleLowerAndUpperBounds(double lowerBound, double upperBound)
{
    angleLowerBound = lowerBound;
    angleUpperBound = upperBound;
}