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
const unsigned long TIMEFRAME = 100; // milliseconds
const double ENCODER_PPR = 2400.0;

// Initialize variables
unsigned long previousMilliseconds = 0;
float previousCartPosition = 0;
float previousPendulumAngle = 0;

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

    // Compute the state
    stateVector state;
    state.pendulumAngle = encoderCountToAngleRadians(pendulumEncoderCount, ENCODER_PPR);              // radians
    state.cartPosition = encoderCountToCartPositionInches(cartEncoderCount, ENCODER_PPR);             // in
    state.pendulumAngularVelocity = (state.pendulumAngle - previousPendulumAngle)/(TIMEFRAME/1000.0); // radians/s
    state.cartVelocity = (state.cartPosition - previousCartPosition)/(TIMEFRAME/1000.0);              // in/s

    sendStateVectorToPython(state);

    // Store the current data for computation in the next loop
    previousMilliseconds = millis();
    previousPendulumAngle = state.pendulumAngle;
    previousCartPosition = state.cartPosition;

  }

  // moveCart('R', 20);
  // delay(50);

  // brake();

  // moveCart('L', 20);
  // delay(50);

  // brake();
}