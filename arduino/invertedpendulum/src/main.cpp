#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>
#include <Encoder.h>
#include "motorControllerDrokL298.h"
#include "utils2.h"

// Initialize encoders
#define cartEncoderPhaseA 3
#define cartEncoderPhaseB 4
#define pendulumEncoderPhaseA 2
#define pendulumEncoderPhaseB 5

Encoder cartEncoder(cartEncoderPhaseA, cartEncoderPhaseB);
Encoder pendulumEncoder(pendulumEncoderPhaseA, pendulumEncoderPhaseB);

// Initialize named constants
const unsigned long TIMEFRAME = 100;

// Initialize variables
unsigned long previousMilliseconds = 0;

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

  // Send values to python every n milliseconds
  if ( (currentMilliseconds - previousMilliseconds) > TIMEFRAME ) {
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