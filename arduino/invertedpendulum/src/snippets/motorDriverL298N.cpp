// L298N motor driver

#include <Arduino.h>

#define IN1 7
#define IN2 8
#define ENA 9

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

}

void loop() {
  // moveCart(speed, direction);

}

void moveCart(int speed, int direction) {

  // The joystick is really sensitive to movement
  // so add a buffer that way the cart doesn't move when the joystick is being moved
  int buffer = 50;

  // Move the cart to the right
  if (direction > (508 + buffer)) {
    analogWrite(ENA, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // Move the cart to the left
  else if (direction < (508 - buffer) ) {
    analogWrite(ENA, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  // Else, turn the motor off
  else {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

}
