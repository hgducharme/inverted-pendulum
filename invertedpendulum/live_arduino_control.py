import serial

# Define the serial port and baud rate.
_BAUD_RATE = 115200

arduino = serial.Serial('/dev/cu.usbmodem14101', _BAUD_RATE, timeout = 0.1)

if __name__ == "__main__":
    while True:
        line = arduino.readlines()
        print(line)