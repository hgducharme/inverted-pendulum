#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <Encoder.h>
#include "motorControllerDrokL298.h"
#include "pythonUtils.h"

// Initialize encoders
#define cartEncoderPhaseA 3
#define cartEncoderPhaseB 4
#define pendulumEncoderPhaseA 2
#define pendulumEncoderPhaseB 5

Encoder cartEncoder(cartEncoderPhaseA, cartEncoderPhaseB);
Encoder pendulumEncoder(pendulumEncoderPhaseA, pendulumEncoderPhaseB);

// Initialize named constants
const unsigned long TIMEFRAME = 100;
const double ENCODER_PPR = 2400.0;

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
  unsigned long currentMilliseconds = millis();
  long cartEncoderCount = cartEncoder.read();
  long pendulumEncoderCount = pendulumEncoder.read();

  // Send values to python every n milliseconds
  if ( (currentMilliseconds - previousMilliseconds) > TIMEFRAME ) {

    stateVector state;
    state.pendulumAngle = encoderCountToAngleDegrees(pendulumEncoderCount, ENCODER_PPR);
    state.cartPosition = encoderCountToCartPositionInches(cartEncoderCount, ENCODER_PPR);

    sendStateVectorToPython(state);

    previousMilliseconds = millis();

  }

  // moveCart('R', 20);
  // delay(100);

  // brake();

  // moveCart('L', 20);
  // delay(100);

  // brake();
}