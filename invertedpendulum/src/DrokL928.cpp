#include "DrokL928.hpp"

DrokL928::DrokL928(int _IN1, int _IN2, int _ENA) : 
    motorIN1(_IN1), motorIN2(_IN2), motorENA(_ENA) {

};

void DrokL928::registerPinsWithArduino() {
    pinMode(motorIN1, OUTPUT);
    pinMode(motorIN2, OUTPUT);
    pinMode(motorENA, OUTPUT);
}

void DrokL928::rotateMotorClockwise(double dutyCyclePWM) {
    digitalWrite(motorIN1, HIGH); // Negative terminal
    digitalWrite(motorIN2, LOW);  // Positive terminal
    analogWrite(motorENA, dutyCyclePWM);
}

void DrokL928::rotateMotorCounterClockwise(double dutyCyclePWM)
{
    digitalWrite(motorIN1, LOW);  // Negative terminal
    digitalWrite(motorIN2, HIGH); // Positive terminal
    analogWrite(motorENA, dutyCyclePWM);
}

void DrokL928::stopMotor()
{
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, LOW);
    analogWrite(motorENA, 0);
    delay(130);
}

void DrokL928::testMotor() {
    rotateMotorClockwise(35);
    delay(100);
    stopMotor();
    rotateMotorCounterClockwise(35);
    delay(100);
    stopMotor();
}