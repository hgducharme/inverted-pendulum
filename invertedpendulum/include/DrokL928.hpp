#pragma once

#include <Arduino.h>
#include "IMotorController.hpp"

class DrokL928 : public IMotorController {
    private:
        const int motorChannelIN1;
        const int motorChannelIN2;
        const int motorChannelENA;

    public:
        DrokL928(const int _IN1, const int _IN2, const int _ENA);
        void registerPinsWithArduino();
        virtual void rotateMotorClockwise(double voltage);
        virtual void rotateMotorCounterClockwise(double voltage);
        virtual void stopMotor();
        void moveCartRight(double voltage);
        void moveCartLeft(double voltage);
        void brake();
        void testMotor();
};