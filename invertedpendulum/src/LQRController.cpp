#include "LQRController.hpp"

LQRController::LQRController(double (&array)[4]) : gainVector(array)
{
    lengthOfGainVector = sizeof(array) / sizeof(array[0]);
}

double LQRController::computeInput(double (&stateVector)[4])
{

    // validateStateVectorLength(stateVector);

    double controlInput;

    for (int i = 0; i < lengthOfGainVector; i++) 
    {
        controlInput += (gainVector[i] * stateVector[i]);
    }
    
    return controlInput;
}