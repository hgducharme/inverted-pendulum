#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H

class RotaryEncoder
{
private:
    Encoder encoder;
    long encoderCount;
    double encoderPPR;
    double angleLowerBound;
    double angleUpperBound;

    double convertEncoderCountToRadians(long encoderCount);
    double normalizeAngle(double angle);

public:
    RotaryEncoder(int phaseA, int phaseB, double encoderPPR);
    long getCount();
    double getRadians(double angleOffset);
    void setAngleLowerAndUpperBounds(double lowerBound, double upperBound);
};

#endif /* ROTARYENCODER_H */