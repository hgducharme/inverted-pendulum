#include "LQRController.h"

LQRController::LQRController(double (&array)[4]) : gainVector(array)
{
    lengthOfGainVector = sizeof(array) / sizeof(array[0]);
}

double LQRController::computeControlInput(double (&stateVector)[2])
{

    // validateStateVectorLength(stateVector);

    double controlInput;

    for (int i = 0; i < lengthOfGainVector; i++) 
    {
        controlInput += (gainVector[i] * stateVector[i]);
    }
    
    return controlInput;
}