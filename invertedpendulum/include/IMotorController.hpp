#pragma once

class IMotorController {
    public:
        IMotorController() {};
        virtual ~IMotorController() = 0;
        virtual void rotateMotorClockwise(double dutyCyclePWM) = 0;
        virtual void rotateMotorCounterClockwise(double dutyCyclePWM) = 0;
        virtual void stopMotor() = 0;
};