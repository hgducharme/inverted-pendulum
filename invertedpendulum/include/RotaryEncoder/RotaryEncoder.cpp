#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(Encoder &encoderReference, double encoderPPR) : encoder(encoderReference)
{
    encoderPPR = encoderPPR;
}

double RotaryEncoder::getPreviousValue()
{
    return previousValue;
}

void RotaryEncoder::setPreviousValue(double value)
{
    previousValue = value;
}

double RotaryEncoder::read()
{
    return encoder.read();
}

double RotaryEncoder::readInRadians()
{
    double encoderCount = read();
    double radians = convertEncoderCountToRadians(encoderCount);
}

double RotaryEncoder::convertEncoderCountToRadians(double encoderCount)
{
    return (encoderCount / (encoderPPR)) * (2.0 * PI);
}