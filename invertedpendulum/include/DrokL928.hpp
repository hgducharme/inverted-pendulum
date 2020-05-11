#pragma once

#include <IMotorController.hpp>

class DrokL928 : public IMotorController {
    private:
        const int motorIN1;
        const int motorIN2;
        const int motorENA;

    public:
        DrokL928(int _IN1, int _IN2, int _ENA);
        void registerPinsWithArduino();
        void rotateMotorClockwise(double dutyCyclePWM);
        void rotateMotorCounterClockwise(double dutyCyclePWM);
        void stopMotor();
        void testMotor();
};