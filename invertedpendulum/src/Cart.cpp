#include "Cart.hpp"

Cart::Cart(IMotorController * m) : motorController(m) {};

void Cart::moveLeft(double dutyCyclePWM) {
    motorController->rotateMotorClockwise(dutyCyclePWM);
}

void Cart::moveRight(double dutyCyclePWM) {
    motorController->rotateMotorCounterClockwise(dutyCyclePWM);
}

void Cart::brake() {
    motorController->stopMotor();
}

void Cart::dispatch(double controlInput) {
    // NOTE: A PWM value of 35 is essentially the lowest value to start moving the cart due to friction
    double mappedInput = map(abs(controlInput), 0, 1000, 20, 255);

    if (mappedInput > 255)
    {
        mappedInput = 255;
    }

    if (controlInput < 0)
    {
        moveLeft(mappedInput);
    }
    else if (controlInput > 0)
    {
        moveRight(mappedInput);
    }
    else
    {
        brake();
    }
}