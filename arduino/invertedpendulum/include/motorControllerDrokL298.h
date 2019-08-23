#define IN1 7
#define IN2 8
#define ENA 9

void moveCart(char direction, int speed);
void brake();

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
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }

        // Rotate motor clockwise
        else if (direction == 'L')
        {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
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