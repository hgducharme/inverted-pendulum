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

namespace pins {
	static const int cartEncoderPhaseA = 3;
	static const int cartEncoderPhaseB = 4;
	static const int pendulumEncoderPhaseA = 2;
	static const int pendulumEncoderPhaseB = 5;
	static const int motorIN1 = 7;
	static const int motorIN2 = 8;
	static const int motorENA = 9;
};

namespace constants {
	static const double ENCODER_PPR = 2400.0;
	static const double IDLER_PULLEY_RADIUS = 0.0048006;   // meters
	static const unsigned long LOOP_RATE = 3;              // milliseconds
	static const double ANGLE_BOUND = 30.0 * (PI / 180.0); // radians
}

unsigned long previousMilliseconds = 0;

// Initialize hardware layer objects
Encoder c(pins::cartEncoderPhaseA, pins::cartEncoderPhaseB);
Encoder p(pins::pendulumEncoderPhaseA, pins::pendulumEncoderPhaseB);
EncoderWrapper cartEncoder(c, constants::ENCODER_PPR);
EncoderWrapper pendulumEncoder(p, constants::ENCODER_PPR);
DrokL928 motorController(pins::motorIN1, pins::motorIN2, pins::motorENA);

// Initialize application layer objects
Cart cart(&motorController);
double gainVector[4] = {-2000.0, 900.0, -100.0, 300.0};
LQRController LQR(gainVector);
StateVector state(0, 0, 5, 6);
StateVector previousState(0, 0, 0, PI);
StateUpdater stateCalculator(cartEncoder, pendulumEncoder, constants::IDLER_PULLEY_RADIUS, constants::LOOP_RATE);

void setup()
{
  Serial.begin(9400);

  motorController.registerPinsWithArduino();
}

void loop()
{

  double controlInput; // voltage
  unsigned long currentMilliseconds = millis();

  // Send values to python every n milliseconds
  if ((currentMilliseconds - previousMilliseconds) >= constants::LOOP_RATE)
  {

    stateCalculator.update(state, previousState);
    controlInput = computeControlInput(state, constants::ANGLE_BOUND);
    cart.dispatch(controlInput);

    // Store the current data for computation in the next loop
    previousMilliseconds = currentMilliseconds;

    // TODO: previousState.store(currentState) ?
    previousState.pendulumAngle = state.pendulumAngle;
    previousState.cartPosition = state.cartPosition;
  }
}