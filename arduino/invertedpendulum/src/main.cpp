#include <Arduino.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
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
const unsigned long TIMEFRAME = 50; // milliseconds
const double ENCODER_PPR = 2400.0;

// Initialize variables
unsigned long previousMilliseconds = 0;
float previousCartPosition = 0.0; // meters
float previousPendulumAngle = PI; // radians

void setup()
{
  Serial.begin(19200);

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
  double controlInput; // voltage

  // Send values to python every n milliseconds
  if ( (currentMilliseconds - previousMilliseconds) > TIMEFRAME ) {

    // Compute the state
    stateVector state;
    state.pendulumAngle = encoderCountToPendulumAngleRadians(pendulumEncoderCount, ENCODER_PPR);      // radians
    state.cartPosition = encoderCountToCartPositionMeters(cartEncoderCount, ENCODER_PPR);             // in
    state.pendulumAngularVelocity = (state.pendulumAngle - previousPendulumAngle)/(TIMEFRAME/1000.0); // radians/s
    state.cartVelocity = (state.cartPosition - previousCartPosition)/(TIMEFRAME/1000.0);              // in/s

    sendStateVectorToPython(state);

    // Store the current data for computation in the next loop
    previousMilliseconds = TIMEFRAME;
    previousPendulumAngle = state.pendulumAngle;
    previousCartPosition = state.cartPosition;

  }

  // NOTE: A PWM value of 35 is essentially the lowest value to start moving the cart due to friction
  controlInput = readControlInputFromPython();
  double mappedInput = map(abs(controlInput), 0, 60, 20, 255);

  if (controlInput < 0) {
    moveCart('L', mappedInput);
  }
  else if (controlInput > 0) {
    moveCart('R', mappedInput);
  }
  else {
    brake();
  }

}