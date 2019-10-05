#define rotaryEncoderPhaseA 2
#define rotaryEncoderPhaseB 3

volatile float rotaryEncoderCount;
volatile float previousCount = 0;

void readPhaseA();
void readPhaseB();

void readPhaseA()
{

  // If they're the same, then we're moving clockwise
  if (digitalRead(rotaryEncoderPhaseA) == digitalRead(rotaryEncoderPhaseB))
  {
    rotaryEncoderCount--;
  }

  // If they're not the same, then we're moving counter clockwise
  else
  {
    rotaryEncoderCount++;
  }

  previousCount = rotaryEncoderCount;
}

void readPhaseB()
{

  // If they're the same, then we're moving clockwise
  if (digitalRead(rotaryEncoderPhaseA) == digitalRead(rotaryEncoderPhaseB))
  {
    rotaryEncoderCount++;
  }

  // If they're not the same, then we're moving counter clockwise
  else
  {
    rotaryEncoderCount--;
  }

  previousCount = rotaryEncoderCount;
}