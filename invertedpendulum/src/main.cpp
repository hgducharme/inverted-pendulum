#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>
#include <Encoder.h>
#include <math.h>
#include "LQRController.h"
#include "utils/utilsLQR.h"
#include "StateVector.h"
#include "DrokL928.hpp"
#include "Cart.hpp"

// Initialize encoders
#define cartEncoderPhaseA 3
#define cartEncoderPhaseB 4
#define pendulumEncoderPhaseA 2
#define pendulumEncoderPhaseB 5

const int motorChannelIN1 = 7;
const int motorChannelIN2 = 8;
const int motorChannelENA = 9;
double gainVector[4] = {-2000.0, 900.0, -100.0, 300.0};

DrokL928 motorController(motorChannelIN1, motorChannelIN2, motorChannelENA);
Encoder cartEncoder(cartEncoderPhaseA, cartEncoderPhaseB);
Encoder pendulumEncoder(pendulumEncoderPhaseA, pendulumEncoderPhaseB);
LQRController LQR(gainVector);
Cart cart(&motorController);

// Initialize variables and named constants
const double ENCODER_PPR = 2400.0;
const double IDLER_PULLEY_RADIUS = 0.0048006;   // meters
const unsigned long TIMEFRAME = 3;              // milliseconds
const double ANGLE_BOUND = 30.0 * (PI / 180.0); // radians
double previousCartPosition = 0.0;              // meters
double previousPendulumAngle = PI;              // radians
unsigned long previousMilliseconds = 0;
StateVector state(0, 0, 5, 6);

void setup()
{
  Serial.begin(9400);

  motorController.registerPinsWithArduino();
}

void loop()
{

  double controlInput; // voltage
  unsigned long currentMilliseconds = millis();
  long cartEncoderCount = cartEncoder.read();
  long pendulumEncoderCount = pendulumEncoder.read();

  // Send values to python every n milliseconds
  if ((currentMilliseconds - previousMilliseconds) >= TIMEFRAME)
  {

    // Compute the state
    state.pendulumAngle = encoderCountToPendulumAngleRadians(pendulumEncoderCount, ENCODER_PPR);          // radians
    state.cartPosition = encoderCountToCartPosition(cartEncoderCount, ENCODER_PPR, IDLER_PULLEY_RADIUS);  // meters
    state.pendulumAngularVelocity = (state.pendulumAngle - previousPendulumAngle) / (TIMEFRAME / 1000.0); // radians/s
    state.cartVelocity = (state.cartPosition - previousCartPosition) / (TIMEFRAME / 1000.0);              // meters/s

    // Compute control input using LQR and handle saturation
    // NOTE: A PWM value of 35 is essentially the lowest value to start moving the cart due to friction
    controlInput = computeControlInput(state, ANGLE_BOUND);
    cart.dispatch(controlInput);

    // Store the current data for computation in the next loop
    previousMilliseconds = currentMilliseconds;
    previousPendulumAngle = state.pendulumAngle;
    previousCartPosition = state.cartPosition;
  }
}