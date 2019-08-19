// LPD3806 rotary encoder

#include <Arduino.h>
#include "rotaryEncoderLPD3806.h"
#include "motorControllerDrokL298.h"

void setup()
{

  Serial.begin(9600);

  // Motor controller
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Rotary encoder
  // Using just one interrupt gives us 1200 PPR, where as two gives 2400 PPR
  // However, 2400 PPR makes errors get multiplied so decreases accuracy.
  pinMode(rotaryEncoderPhaseA, INPUT_PULLUP);
  pinMode(rotaryEncoderPhaseB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rotaryEncoderPhaseA), readPhaseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryEncoderPhaseB), readPhaseB, CHANGE);
}

void loop()
{
  float angleDegrees;
  float angleRadians;
  float distance;
  float idlerRadius = 0.20;

  if (rotaryEncoderCount != previousCount)
  {
    angleRadians = (rotaryEncoderCount / 2400.0) * (2 * PI);
    distance = idlerRadius * angleRadians;
    Serial.print("Distance travelled: ");
    Serial.print(distance);
    Serial.print(" in");
    Serial.println(" ");
  }

  // Serial.println("Moving right");
  // moveCart('R', 0);
  // delay(1000);
  // Serial.println("Moving left");
  // moveCart('L', 20);
}