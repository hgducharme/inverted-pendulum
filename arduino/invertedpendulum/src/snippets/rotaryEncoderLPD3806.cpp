// LPD3806 rotary encoder

#include <Arduino.h>

#define rotaryEncoderPhaseA 2
#define rotaryEncoderPhaseB 3
volatile float rotaryEncoderCount;
volatile float angle;
int previousAngle = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(rotaryEncoderPhaseA, INPUT_PULLUP);
  pinMode(rotaryEncoderPhaseB, INPUT_PULLUP);

  // Using just one interrupt gives us 1200 PPR, where as two gives 2400 PPR
  attachInterrupt(digitalPinToInterrupt(rotaryEncoderPhaseA), readPhaseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryEncoderPhaseB), readPhaseB, CHANGE);

}

void loop() {
  Serial.println(angle);
}

void readPhaseA() {
  
  // If they're the same, then we're moving clockwise
  if (digitalRead(rotaryEncoderPhaseA) == digitalRead(rotaryEncoderPhaseB)) {
    rotaryEncoderCount--;
  }

  // If they're not the same, then we're moving counter clockwise
  else {
    rotaryEncoderCount++;
  }

  angle = (rotaryEncoderCount/1200.0)*360.0;

}

void readPhaseB() {
  
  // If they're the same, then we're moving clockwise
  if (digitalRead(rotaryEncoderPhaseA) == digitalRead(rotaryEncoderPhaseB)) {
    rotaryEncoderCount++;
  }

  // If they're not the same, then we're moving counter clockwise
  else {
    rotaryEncoderCount--;
  }

  angle = (rotaryEncoderCount/1200.0)*360.0;

}
