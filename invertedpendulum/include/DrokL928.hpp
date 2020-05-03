#pragma once

#include <Arduino.h>
#include "IMotorController.hpp"

class DrokL928 : public IMotorController {
    private:
        const int motorIN1;
        const int motorIN2;
        const int motorENA;

    public:
        DrokL928(int _IN1, int _IN2, int _ENA);
        virtual void registerPinsWithArduino();
        virtual void rotateMotorClockwise(double dutyCyclePWM);
        virtual void rotateMotorCounterClockwise(double dutyCyclePWM);
        virtual void stopMotor();
        virtual void testMotor();
};