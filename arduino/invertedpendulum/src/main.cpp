#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>
#include <Encoder.h>
#include "motorControllerDrokL298.h"

// Initialize encoders
#define cartEncoderPhaseA 3
#define cartEncoderPhaseB 4
#define pendulumEncoderPhaseA 2
#define pendulumEncoderPhaseB 5

Encoder cartEncoder(cartEncoderPhaseA, cartEncoderPhaseB);
Encoder pendulumEncoder(pendulumEncoderPhaseA, pendulumEncoderPhaseB);

unsigned long timeframe = 5000;
unsigned long previousMilliseconds = 0;

void sendEncoderValuesToPython(long cartEncoderCount, long pendulumEncoderCount) {

  float cartAngle;           // radians
  float pendulumAngle;       // radians
  float cartDistance;        // depends on units of idlerRadius
  float idlerRadius = 0.189; // inches

  // Calculate linear distance travelled by cart x = r * theta
  cartAngle = (cartEncoderCount / 2400.0) * (360.0);
  cartDistance = idlerRadius * cartAngle;

  // Calculate pendulum angle
  pendulumAngle = (pendulumEncoderCount / 2400.0) * (360.0);

  Serial.print(cartAngle);
  Serial.print(".");
  Serial.print(pendulumAngle);
  Serial.println();
}

void setup()
{
  Serial.begin(9400);

  // Motor controller
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void loop()
{
  long cartEncoderCount = cartEncoder.read();
  long pendulumEncoderCount = pendulumEncoder.read();
  unsigned long currentMilliseconds = millis();

  if ( (currentMilliseconds - previousMilliseconds) > timeframe ) {
    sendEncoderValuesToPython(cartEncoderCount, pendulumEncoderCount);
    previousMilliseconds = millis();
  }

  // moveCart('R', 20);
  // delay(100);

  // brake();

  // moveCart('L', 20);
  // delay(100);

  // brake();
}