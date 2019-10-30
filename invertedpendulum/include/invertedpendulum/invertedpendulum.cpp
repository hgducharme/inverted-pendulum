#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

class InvertedPendulum
{
    RotaryEncoder cartEncoder;
    RotaryEncoder pendulumEncoder;
    StateVector state;

public:
    InvertedPendulum(RotaryEncoder cart, RotaryEncoder pendulum) : cartEncoder(cart), pendulumEncoder(pendulum){};

    void updateStateVector()
    {
        /*
            state.pendulumAngle = encoderCountToPendulumAngleRadians(pendulumEncoderCount, ENCODER_PPR);          // radians
            state.cartPosition = encoderCountToCartPosition(cartEncoderCount, ENCODER_PPR, IDLER_PULLEY_RADIUS);  // meters
            state.pendulumAngularVelocity = (state.pendulumAngle - previousPendulumAngle) / (TIMEFRAME / 1000.0); // radians/s
            state.cartVelocity = (state.cartPosition - previousCartPosition) / (TIMEFRAME / 1000.0);              // meters/s
            */

        state.pendulumAngle = pendulumEncoder.readInRadians();
    };
};

class RotaryEncoder
{
private:
    Encoder encoder;
    double encoderPPR;
    double previousValue;

public:
    RotaryEncoder(Encoder &encoderReference, double encoderPPR) : encoder(encoderReference)
    {
        this->encoderPPR = encoderPPR;
    }

    double getPreviousValue()
    {
        return this->previousValue;
    }

    void setPreviousValue(double value)
    {
        this->previousValue = value;
    }

    double read()
    {
        return this->encoder.read();
    }

    double readInRadians()
    {
        double encoderCount = read();
        double radians = convertEncoderCountToRadians(encoderCount);
    }

    double convertEncoderCountToRadians(double encoderCount)
    {
        return (encoderCount / (this -> encoderPPR) )* (2.0 * PI);
    }
};


class StateVector
{
    public:
        double pendulumAngle;
        double cartPosition;
        double pendulumAngularVelocity;
        double cartVelocity;
};