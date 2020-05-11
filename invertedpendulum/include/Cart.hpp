#pragma once

#include "IMotorController.hpp"

class Cart {
    IMotorController &motorController;

    public:
        Cart(IMotorController &motorController);
        void moveLeft(double dutyCyclePWM);
        void moveRight(double dutyCyclePWM);
        void brake();
        void dispatch(double controlInput);
        
    private:
        long map(long x, long in_min, long in_max, long out_min, long out_max);
};