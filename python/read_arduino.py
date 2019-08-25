import serial
import numpy as np
from math import pi


def _has_digits(string):
    return any(char.isdigit() for char in str(string))


def _check_for_overflowed_digits(string):
    if _has_digits(string):
        overflowed_digits = [char for char in string.split() if char.isdigit()]
        digits_to_string = ("").join(overflowed_digits)
        return digits_to_string
    else:
        return None


def run_arduino_control(action, port, baud_rate=0, timeout=0):
    arduino = serial.Serial(port, baud_rate, timeout=timeout)
    last_line_recieved = ""

    try:
        while True:
            serial_bytes = arduino.readline()
            decoded_line = str(serial_bytes.decode("utf8"))

            # Read from serial until we capture the whole line of data
            if (len(serial_bytes) != 0) and (b"\n" in serial_bytes):

                # Sometimes digits overflow from one line onto the next
                overflowed_digits = _check_for_overflowed_digits(decoded_line)
                if overflowed_digits is not None:
                    last_line_recieved += overflowed_digits

                action(arduino, last_line_recieved)

                # Reset the variable
                last_line_recieved = ""

            elif len(serial_bytes) != 0:
                last_line_recieved += decoded_line

            else:
                pass

    except KeyboardInterrupt:
        arduino.close()
        pass


if __name__ == "__main__":

    def compute_input(state):
        state_vector = np.array(
            [
                [state["pendulum_angle"]],
                [state["cart_position"]],
                [state["pendulum_angular_velocity"]],
                [state["cart_velocity"]],
            ]
        )
        gain_matrix = np.array(
            [[-98.93469078, -23.53755766, -15.49560488, -26.21828687]]
        )

        control_input = np.dot(gain_matrix, state_vector)[0][0]
        return control_input

    def parse_state_vector(data_string):
        data_array = data_string.split(",")

        # Sometimes the carriage return "\r" remains in the string, so get rid of it
        cart_velocity = (
            data_array[3] if "\r" not in data_array[3] else data_array[3].split("\r")[0]
        )

        state_vector = {
            "pendulum_angle": float(data_array[0]),
            "cart_position": float(data_array[1]),
            "pendulum_angular_velocity": float(data_array[2]),
            "cart_velocity": float(cart_velocity),
        }

        return state_vector

    def write_to_arduino(arduino_serial_port, string):
        pass

    def interpret_data(arduino_serial_port, data_string):

        # Parse data into state_vector and send control input using LQR
        state_vector = parse_state_vector(data_string)

        # If the pendulum angle is outside of the bounds (45 degrees), don't supply any control input
        if abs(state_vector["pendulum_angle"]) > (pi / 4.0):
            control_input = 0.0
        else:
            control_input = compute_input(state_vector)

        write_to_arduino(arduino_serial_port, str(control_input).encode())

    run_arduino_control(interpret_data, "/dev/cu.usbmodem14101", baud_rate=9400, timeout=0)