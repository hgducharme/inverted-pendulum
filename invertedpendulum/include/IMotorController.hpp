#pragma once

class IMotorController {
    public:
        virtual ~IMotorController() {};
        virtual void registerPinsWithArduino() = 0;
        virtual void rotateMotorClockwise(double dutyCyclePWM) = 0;
        virtual void rotateMotorCounterClockwise(double dutyCyclePWM) = 0;
        virtual void stopMotor() = 0;
};