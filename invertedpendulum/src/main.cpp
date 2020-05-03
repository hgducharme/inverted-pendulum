#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>
#include <Encoder.h>
#include <math.h>
#include "LQRController.h"
#include "utils/utilsLQR.h"
#include "StateVector.h"
#include "DrokL928.hpp"
#include "Cart.hpp"
#include "EncoderWrapper.hpp"
#include "StateUpdater.hpp"

// Define pins
#define cartEncoderPhaseA 3
#define cartEncoderPhaseB 4
#define pendulumEncoderPhaseA 2
#define pendulumEncoderPhaseB 5
const int motorChannelIN1 = 7;
const int motorChannelIN2 = 8;
const int motorChannelENA = 9;

// Initialize variables and named constants
const double ENCODER_PPR = 2400.0;
const double IDLER_PULLEY_RADIUS = 0.0048006;   // meters
const unsigned long LOOP_RATE = 3;              // milliseconds
const double ANGLE_BOUND = 30.0 * (PI / 180.0); // radians
unsigned long previousMilliseconds = 0;
double gainVector[4] = {-2000.0, 900.0, -100.0, 300.0};

// Initialize hardware layer objects
Encoder c(cartEncoderPhaseA, cartEncoderPhaseB);
Encoder p(pendulumEncoderPhaseA, pendulumEncoderPhaseB);
EncoderWrapper cartEncoder(c, ENCODER_PPR);
EncoderWrapper pendulumEncoder(p, ENCODER_PPR);
DrokL928 motorController(motorChannelIN1, motorChannelIN2, motorChannelENA);

// Initialize application layer objects
Cart cart(&motorController);
LQRController LQR(gainVector);
StateVector state(0, 0, 5, 6);
StateVector previousState(0, 0, 0, PI);
StateUpdater stateCalculator(cartEncoder, pendulumEncoder, IDLER_PULLEY_RADIUS, LOOP_RATE);
// TODO:
// Scheduler?

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
  if ((currentMilliseconds - previousMilliseconds) >= LOOP_RATE)
  {

    stateCalculator.update(state, previousState);
    controlInput = computeControlInput(state, ANGLE_BOUND);
    cart.dispatch(controlInput);

    // Store the current data for computation in the next loop
    previousMilliseconds = currentMilliseconds;

    // TODO: previousState.store(currentState) ?
    previousState.pendulumAngle = state.pendulumAngle;
    previousState.cartPosition = state.cartPosition;
  }
}