import serial

"""
This script needs work. We need to find a way to read lines every 0.1 seconds.
We also need to find a way to differentiate empty lines from lines with data we want.
see: https://stackoverflow.com/questions/1093598/pyserial-how-to-read-the-last-line-sent-from-a-serial-device

I'm calling it a night
"""

# Define the serial port and baud rate.
_BAUD_RATE = 115200

arduino = serial.Serial('/dev/cu.usbmodem14101', 9400, timeout = 0)

if __name__ == "__main__":
    while True:
        line = arduino.readlines()
        print(line[0]) if line else None