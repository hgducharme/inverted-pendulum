#ifndef PYTHON_UTILS
#define PYTHON_UTILS

#include <Arduino.h>
#include <math.h>

char DATA_START_MARKER = '<';
char DATA_END_MARKER = '>';

struct stateVector
{
    double pendulumAngle;
    double cartPosition;
    double pendulumAngularVelocity = 6.0;
    double cartVelocity = 5.0;
};

float encoderCountToAngleDegrees(long encoderCount);
float encoderCountToCartPositionMeters(long cartEncoderCount, double encoderPPR);
void sendStateVectorToPython(stateVector state);
double readControlInputFromPython();

float encoderCountToAngleRadians(long encoderCount, double encoderPPR)
{
    return (encoderCount / encoderPPR) * (2.0 * PI);
}

float encoderCountToCartPositionMeters(long cartEncoderCount, double encoderPPR)
{
    float idlerPulleyRadius = 0.0048006;                                        // meters
    float cartAngle = encoderCountToAngleRadians(cartEncoderCount, encoderPPR); // radians
    return idlerPulleyRadius * cartAngle;
}

float normalizeAngle(float angle)
{
    // Constrain an angle between [-180, 180) in radians
    // see: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code

    angle = fmod(angle + PI, 2 * PI);
    if (angle < 0)
        angle += 2 * PI;
    return angle - PI;
}

float encoderCountToPendulumAngleRadians(long encoderCount, double encoderPPR)
{
    // Since we use the <Encoder.h> library which initializes the encoder to zero at the start,
    // we need to correct the value because we start at a pendulum angle of +180 degrees.

    float pendulumAngle = PI + encoderCountToAngleRadians(encoderCount, encoderPPR);
    return normalizeAngle(pendulumAngle);
}

void sendStateVectorToPython(stateVector state)
{
    Serial.print(DATA_START_MARKER);
    Serial.print(state.pendulumAngle);
    Serial.print(",");
    Serial.print(state.cartPosition);
    Serial.print(",");
    Serial.print(state.pendulumAngularVelocity);
    Serial.print(",");
    Serial.print(state.cartVelocity);
    Serial.print(DATA_END_MARKER);
    Serial.println();
}

double readControlInputFromPython()
{
    bool recievedFullLineOfData = false;
    bool recievingDataInProgress = false;
    uint8_t bufferIndex = 0;
    char buffer[32];
    double controlInput = 0;

    while ((Serial.available() > 0) && (recievedFullLineOfData == false))
    {
        char character = Serial.read();

        if (recievingDataInProgress == true)
        {

            // If we're at the end of the data string then process the data
            if (character == DATA_END_MARKER)
            {
                recievedFullLineOfData = true;
                recievingDataInProgress = false;
                buffer[bufferIndex] = '\0';
                controlInput = atof(buffer);
                bufferIndex = 0;
            }

            // Else just append the data to the buffer
            else
            {
                buffer[bufferIndex] = character;
                bufferIndex++;
            }
        }

        else if (character == DATA_START_MARKER)
        {
            recievingDataInProgress = true;
        }

        else {}
    }

    return controlInput;
}

#endif