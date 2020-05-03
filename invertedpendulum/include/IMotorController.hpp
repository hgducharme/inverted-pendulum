#pragma once

class IMotorController {
    public:
        IMotorController() {};
        virtual ~IMotorController() {};
        virtual void rotateMotorClockwise(double voltage) = 0;
        virtual void rotateMotorCounterClockwise(double voltage) = 0;
        virtual void stopMotor() = 0;
};