// HC-SR04 ultrasonic Sensor
#define ultrasonicTrigger 11
#define ultrasonicEcho 12
NewPing sonar(ultrasonicTrigger, ultrasonicEcho);

#define joystick A0
#define pot A1
#define button 4

// Initialize potentiometer speed, joystick direction, and button state to zero
int direction = 0;
int speed = 0;
int buttonState = 0;


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