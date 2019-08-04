#include <NewPing.h>

// L298N motor driver
#define IN1 7
#define IN2 8
#define ENA 9

// LPD3806 rotary encoder
#define rotaryEncoderPhaseA 2
#define rotaryEncoderPhaseB 3
volatile long rotaryEncoderAngle;

// HC-SR04 ultrasonic Sensor
#define ultrasonicTrigger 11
#define ultrasonicEcho 12
NewPing sonar(ultrasonicTrigger, ultrasonicEcho);

#define joystick A0
#define pot A1
#define button 4

// Initialize potentiometer speed, joystick direction, and button state to zero
int direction = 0;
int speed = 0;
int buttonState = 0;

void setup() {

  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(ultrasonicEcho, INPUT);
  pinMode(ultrasonicTrigger, OUTPUT);

  pinMode(pot, INPUT);
  pinMode(joystick, INPUT);
  pinMode(rotaryEncoderPhaseA, INPUT_PULLUP);
  pinMode(rotaryEncoderPhaseB, INPUT_PULLUP);
  pinMode(button, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(rotaryEncoderPhaseA), readRotaryEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(rotaryEncoderPhaseB), readRotaryEncoder, CHANGE);

}

void loop() {

  // Read in values
  direction = readJoystick();
  speed = readPotentiometer();
  readButton();

  double distance = sonar.ping_cm();
  double distanceIn = sonar.ping_in();

  Serial.print(distance);
  Serial.print(" cm, ");
  Serial.print(distanceIn);
  Serial.print(" in");
  Serial.println();

  double duration = sonar.ping_median(3)*1.0/2.0;
  
  
  
//  double speedOfSoundEnglish = 1133.0; // ft/s
//  double speedOfSoundMetric = 343.0; // m/s
//
//  double distanceIn = speedOfSoundEnglish*duration
//  Serial.print(" in");
//  Serial.println();

  moveCart(speed, direction);
  if (buttonState == 1) {
    controlCartBasedOnAngle(rotaryEncoderAngle);
  }

}

int readJoystick() {

  return analogRead(joystick);

}

int readPotentiometer() {

  speed = analogRead(pot);
  speed = map(speed, 0, 1023, 255, 0);

  return speed;

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

void readRotaryEncoder() {

  // If they're the same, then we're moving clockwise
  if (digitalRead(rotaryEncoderPhaseA) == digitalRead(rotaryEncoderPhaseB)) {
    rotaryEncoderAngle--;
  }

  // If they're not the same, then we're moving counter clockwise
  else {
    rotaryEncoderAngle++;
  }

  Serial.println(rotaryEncoderAngle);

}

void readButton() {

  int buttonPressed = digitalRead(button);

  // If the button is pressed then toggle the button state
  if (buttonPressed == 0) {

    // If the state is on, turn it off
    if (buttonState == 1) {
      Serial.println("State: OFF");
      buttonState = 0;
    }

    // Else, turn it on
    else {
      Serial.println("State: ON");
      buttonState = 1;
    }

    delay(200);
  }

}

void controlCartBasedOnAngle(int angle) {

  // If the angle is positive, slowly move the cart to the right
  if (angle > 50) {
    moveCart(80, 600);
  }

  // If the angle is negative, slowly move the cart to the left
  else if (angle < -50) {
    moveCart(80, 400);
  }


  else {}
}
