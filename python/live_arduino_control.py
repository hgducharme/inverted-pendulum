import serial

"""
This script needs work. We need to find a way to read lines every 0.1 seconds.
We also need to find a way to differentiate empty lines from lines with data we want.
see: https://stackoverflow.com/questions/1093598/pyserial-how-to-read-the-last-line-sent-from-a-serial-device

I'm calling it a night
"""

# Define the serial port and baud rate.
_BAUD_RATE = 9400
arduino = serial.Serial("/dev/cu.usbmodem14101", _BAUD_RATE, timeout=0)

def has_numbers(string):
    return any(char.isdigit() for char in str(string))

if __name__ == "__main__":

    last_line_recieved = ""

    while True:
        serial_bytes = arduino.readline()
        decoded_line = str(serial_bytes.decode('utf8'))

        # Read from buffer until we capture the whole line of data
        if (len(serial_bytes) != 0) and (b'\n' in serial_bytes):

            # Check if the first character is a digit that overflowed from the previous line
            if decoded_line[0].isdigit():
                overflowed_digits = [ char for char in decoded_line.split() if char.isdigit() ]
                string_digits = ("").join(overflowed_digits)
                last_line_recieved += string_digits

            # Store the data now that we have the entire line
            data = last_line_recieved.split(",")
            state_vector = {
                'pendulum_angle': data[0],
                'cart_position': data[1],
                'pendulum_angular_velocity': data[2],

                # Sometimes the carriage return (\r) stays appended
                'cart_velocity': (data[3] if '\r' not in data[3] else data[3].split('\r')[0])
            }

            print(state_vector)

            # Reset the variable
            last_line_recieved = ""

        elif len(serial_bytes) != 0:
            last_line_recieved += decoded_line

        else:
            pass
