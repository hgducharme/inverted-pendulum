#define IN1 7
#define IN2 8
#define ENA 9

void moveCart(char direction, int speed);
void brake();
void testMotor();

void moveCart(char direction, int speed)
{

    if (speed == 0)
    {
        brake();
    }

    else
    {
        // Rotate motor counter-clockwise
        if (direction == 'R')
        {
            digitalWrite(IN1, LOW);  // Negative terminal
            digitalWrite(IN2, HIGH); // Positive terminal
        }

        // Rotate motor clockwise
        else if (direction == 'L')
        {
            digitalWrite(IN1, HIGH); // Negative terminal
            digitalWrite(IN2, LOW);  // Positive terminal
        }

        else
        {
            brake();
        }

        analogWrite(ENA, speed);

    }
}

void brake() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    delay(130);
}

void testMotor() {
    moveCart('L', 255);
    delay(100);
    brake();
    moveCart('R', 255);
    delay(100);
    brake();
}