#ifndef PYTHON_UTILS
#define PYTHON_UTILS

#include <Arduino.h>
#include <math.h>

char DATA_START_MARKER = '<';
char DATA_END_MARKER = '>';

void sendStateVectorToPython(stateVector state);
double readControlInputFromPython();

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