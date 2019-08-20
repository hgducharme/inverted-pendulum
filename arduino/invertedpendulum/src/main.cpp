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

long previousCartEncoderCount = -999.0;
long previousPendulumEncoderCount = -999.0;

void sendEncoderValuesToPython(long cartEncoderCount, long pendulumEncoderCount) {

  /*
  NOTE: We can probably get rid of the attach interrupts and instead just read the encoder values every 0.01 or 0.1 seconds.
  We would do this because attach interrupts are expensive (I think), but mostly because we would be sending the values
  to python whenever we DETECT a change, so python doesn't know when it will get a change back, and when it does get a change
  it doesn't know how many bytes, because we can have several quick changes. I think it would be better just to operate on a discrete
  known time scale (eg. every 0.1 seconds). 
  */

    // If one of the encoders changes, update and print values to serial
  if (cartEncoderCount != previousCartEncoderCount || pendulumEncoderCount != previousPendulumEncoderCount)
  {

    float cartAngle;           // radians
    float pendulumAngle;       // radians
    float cartDistance;        // depends on units of idlerRadius
    float idlerRadius = 0.189; // inches

    previousCartEncoderCount = cartEncoderCount;
    previousPendulumEncoderCount = pendulumEncoderCount;

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
}

unsigned long timeframe = 50;
unsigned long previousMilliseconds = 0;
void setup()
{

  Serial.begin(115200);

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
  }

  // moveCart('R', 20);
  // delay(3000);

  // brake();

  // moveCart('L', 20);
  // delay(3000);

  // brake();
}