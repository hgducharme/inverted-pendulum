#pragma once

#include "IMotorController.hpp"
#include <Arduino.h>

class Cart {
    IMotorController & motorController;

    public:
        Cart(IMotorController & motorController);
        void moveLeft(double dutyCyclePWM);
        void moveRight(double dutyCyclePWM);
        void brake();
        void dispatch(double controlInput);
};