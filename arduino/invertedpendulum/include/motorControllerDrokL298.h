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
        analogWrite(ENA, speed);

        // Rotate motor clockwise
        if (direction == 'L')
        {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }

        // Rotate motor counter-clockwise
        else if (direction == 'R')
        {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        }

        else
        {
            brake();
        }
    }
}

void brake() {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    delay(100);
}