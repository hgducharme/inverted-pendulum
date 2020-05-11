#include <Cart.hpp>
#include <math.h>

Cart::Cart(IMotorController &m) : motorController(m) {};

void Cart::moveLeft(double dutyCyclePWM) {
    motorController.rotateMotorClockwise(dutyCyclePWM);
}

void Cart::moveRight(double dutyCyclePWM) {
    motorController.rotateMotorCounterClockwise(dutyCyclePWM);
}

void Cart::brake() {
    motorController.stopMotor();
}

void Cart::dispatch(double controlInput) {
    // NOTE: A PWM value of 35 is essentially the lowest value to start moving the cart due to friction
    double mappedInput = map(fabs(controlInput), 0, 1000, 20, 255);

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

long Cart::map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}