#include <Encoder.h>

#ifndef RotaryEncoder_H
#define RotaryEncoder_H

class RotaryEncoder
{
private:
    Encoder encoder;
    double encoderPPR;
    double previousValue;

public:
    RotaryEncoder(Encoder &encoderReference, double encoderPPR);
    
    double getPreviousValue();
    void setPreviousValue(double value);
    double read();
    double readInRadians();
    double convertEncoderCountToRadians(double encoderCount);
};

#endif