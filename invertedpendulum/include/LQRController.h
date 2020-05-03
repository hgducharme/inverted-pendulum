#pragma once

class LQRController
{
private:
    double (&gainVector)[4];
    int lengthOfGainVector;

public:
    LQRController(double (&gainVector)[4]);
    double computeInput(double (&stateVector)[4]);
};