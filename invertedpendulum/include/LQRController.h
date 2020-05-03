#pragma once

class LQRController
{
private:
    double (&gainVector)[4];
    int lengthOfGainVector;

public:
    LQRController(double (&gainVector)[4]);
    double computeControlInput(double (&stateVector)[2]);
};