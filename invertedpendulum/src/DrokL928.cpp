#include "DrokL928.hpp"

DrokL928::DrokL928(const int _IN1, const int _IN2, const int _ENA) : 
    motorChannelIN1(_IN1), motorChannelIN2(_IN2), motorChannelENA(_ENA) {

};

void DrokL928::registerPinsWithArduino() {
    pinMode(motorChannelIN1, OUTPUT);
    pinMode(motorChannelIN2, OUTPUT);
    pinMode(motorChannelENA, OUTPUT);
}

void DrokL928::rotateMotorClockwise(double voltage) {
    digitalWrite(motorChannelIN1, HIGH); // Negative terminal
    digitalWrite(motorChannelIN2, LOW);  // Positive terminal
    analogWrite(motorChannelENA, voltage);
}

void DrokL928::rotateMotorCounterClockwise(double voltage)
{
    digitalWrite(motorChannelIN1, LOW);  // Negative terminal
    digitalWrite(motorChannelIN2, HIGH); // Positive terminal
    analogWrite(motorChannelENA, voltage);
}

void DrokL928::stopMotor()
{
    digitalWrite(motorChannelIN1, LOW);
    digitalWrite(motorChannelIN2, LOW);
    analogWrite(motorChannelENA, 0);
    delay(130);
}

void DrokL928::moveCartRight(double voltage) {
    rotateMotorCounterClockwise(voltage);
}

void DrokL928::moveCartLeft(double voltage) {
    rotateMotorClockwise(voltage);
}

void DrokL928::brake() {
    stopMotor();
}

void DrokL928::testMotor() {
    rotateMotorClockwise(35);
    delay(100);
    stopMotor();
    rotateMotorCounterClockwise(35);
    delay(100);
    stopMotor();
}