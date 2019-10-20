#include <Arduino.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "motorControllerDrokL298.h"
#include "utilsLQR.h"
#include <math.h>

// Initialize encoders
#define cartEncoderPhaseA 3
#define cartEncoderPhaseB 4
#define pendulumEncoderPhaseA 2
#define pendulumEncoderPhaseB 5

Encoder cartEncoder(cartEncoderPhaseA, cartEncoderPhaseB);
Encoder pendulumEncoder(pendulumEncoderPhaseA, pendulumEncoderPhaseB);

// Initialize variables and named constants
const double ENCODER_PPR = 2400.0;
const double IDLER_PULLEY_RADIUS = 0.0048006;   // meters
const unsigned long TIMEFRAME = 3;              // milliseconds
const double ANGLE_BOUND = 30.0 * (PI / 180.0); // radians
double previousCartPosition = 0.0;              // meters
double previousPendulumAngle = PI;              // radians
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
  double controlInput; // voltage

  // Send values to python every n milliseconds
  if ((currentMilliseconds - previousMilliseconds) >= TIMEFRAME)
  {

    // Compute the state
    stateVector state;
    state.pendulumAngle = encoderCountToPendulumAngleRadians(pendulumEncoderCount, ENCODER_PPR);          // radians
    state.cartPosition = encoderCountToCartPosition(cartEncoderCount, ENCODER_PPR, IDLER_PULLEY_RADIUS);  // meters
    state.pendulumAngularVelocity = (state.pendulumAngle - previousPendulumAngle) / (TIMEFRAME / 1000.0); // radians/s
    state.cartVelocity = (state.cartPosition - previousCartPosition) / (TIMEFRAME / 1000.0);              // meters/s

    // Compute control input using LQR and handle saturation
    // NOTE: A PWM value of 35 is essentially the lowest value to start moving the cart due to friction
    controlInput = computeControlInput(state, ANGLE_BOUND);
    double mappedInput = map(abs(controlInput), 0, 1000, 20, 255);

    if (mappedInput > 255)
    {
      mappedInput = 255;
    }

    // Move the cart based on the computed input
    if (controlInput < 0)
    {
      moveCart('L', mappedInput);
    }
    else if (controlInput > 0)
    {
      moveCart('R', mappedInput);
    }
    else
    {
      brake();
    }

    // Store the current data for computation in the next loop
    previousMilliseconds = currentMilliseconds;
    previousPendulumAngle = state.pendulumAngle;
    previousCartPosition = state.cartPosition;
  }
}