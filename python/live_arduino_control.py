import serial

"""
This script needs work. We need to find a way to read lines every 0.1 seconds.
We also need to find a way to differentiate empty lines from lines with data we want.
see: https://stackoverflow.com/questions/1093598/pyserial-how-to-read-the-last-line-sent-from-a-serial-device

I'm calling it a night
"""

# Define the serial port and baud rate.
_BAUD_RATE = 115200

arduino = serial.Serial("/dev/cu.usbmodem14101", 9400, timeout=0)

def has_numbers(string):
    return any(char.isdigit() for char in str(string))

if __name__ == "__main__":

    state_data = b""

    while True:
        line = arduino.readline()

        # Read from buffer until we capture the whole line of data
        if (len(line) != 0) and (b'\n' in line):

            """
            Maybe a more efficient way to do this is:
            1) Check first character like below
            2) Get all the overflowed digits (maybe store as byte right off the bat?)
            3) somehow use python array module, and stuff from this page: https://docs.python.org/3.3/library/stdtypes.html#binary-sequence-types-bytes-bytearray-memoryview
            4) Instead of converting list to bytes, convert with encoding or something? idk

            I just realized I did all this work and didn't read the SO post at the top. Do that tomorrow.
            I'm calling it a night
            """

            decoded_line = str(line.decode('utf8'))

            # Check if the first character is a digit that overflowed from the previous line
            if decoded_line[0].isdigit():
                overflowed_digits = [ int(char) for char in decoded_line.split() if char.isdigit() ]
                state_data += bytes(overflowed_digits)
            
            print(state_data.decode())


            # print(line)
            # print(line)
            # print(state_data)
            # pendulum_angle = state_data.split(",")[0]
            # cart_position = state_data.split(",")[1]
            # pendulum_angular_velocity = state_data.split(",")[2]
            # cart_velocity = state_data.split(",")[3]

            # print(pendulum_angle, " -- ", cart_position, " -- ", pendulum_angular_velocity, " -- ", cart_velocity)

            state_data = b""

        elif len(line) != 0:
            state_data += line

        else:
            pass
