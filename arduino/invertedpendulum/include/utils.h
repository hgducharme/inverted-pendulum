int readJoystick()
{

  return analogRead(joystick);
}

int readPotentiometer()
{

  speed = analogRead(pot);
  speed = map(speed, 0, 1023, 255, 0);

  return speed;
}

void readRotaryEncoder()
{

  // If they're the same, then we're moving clockwise
  if (digitalRead(rotaryEncoderPhaseA) == digitalRead(rotaryEncoderPhaseB))
  {
    rotaryEncoderAngle--;
  }

  // If they're not the same, then we're moving counter clockwise
  else
  {
    rotaryEncoderAngle++;
  }

  Serial.println(rotaryEncoderAngle);
}

void readButton()
{

  int buttonPressed = digitalRead(button);

  // If the button is pressed then toggle the button state
  if (buttonPressed == 0)
  {

    // If the state is on, turn it off
    if (buttonState == 1)
    {
      Serial.println("State: OFF");
      buttonState = 0;
    }

    // Else, turn it on
    else
    {
      Serial.println("State: ON");
      buttonState = 1;
    }

    delay(200);
  }
}

void controlCartBasedOnAngle(int angle)
{

  // If the angle is positive, slowly move the cart to the right
  if (angle > 50)
  {
    moveCart(80, 600);
  }

  // If the angle is negative, slowly move the cart to the left
  else if (angle < -50)
  {
    moveCart(80, 400);
  }

  else
  {
  }
}