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


def run_arduino_control(action, port, baud_rate=9400, timeout=0, start_character = None, end_character = None):

    """
    TODO: This needs to be refactored. Instead of the if else statement I have going on for the start and end characters,
    the code inside the if else statement should be outsourced to another function or something. This is not very DRY right now.
    """

    arduino = serial.Serial(port, baud_rate, timeout=timeout)
    last_line_recieved = ""
    recieved_full_line_of_data = False
    recieving_data_in_progress = False

    try:
        # If there is a start character and end character, parse the data with that
        if (start_character is not None) and (end_character is not None):
            while True:
                serial_bytes = arduino.readline()
                decoded_line = str(serial_bytes.decode("utf8"))
                
                if recieving_data_in_progress:
                    last_line_recieved += decoded_line

                    # If we reached the end of this data packet, process the data
                    if decoded_line.find(end_character) != -1:
                        recieved_full_line_of_data = True
                        recieving_data_in_progress = False
                        data_string = last_line_recieved.split(start_character)[1].split(end_character)[0]
                        action(arduino, data_string)
                        last_line_recieved = ""

                # If we see the start of a data packet, start reading in the data
                elif decoded_line.find(start_character) != -1:
                    recieving_data_in_progress = True
                    last_line_recieved += decoded_line

                # Else, we haven't recieved anything meaningful so do nothing
                else:
                    pass

                
        # Else, parse the data by new line "\n"
        else:
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
            [[-13.73292129, -0.67947076,  -2.04676789, -5.30433475]]#[[-16.35427012, -2.14867521,  -2.43567308, -6.13142817]]
        )

        # Perform the matrix multiplication and round the voltage to two decimal places
        control_input = round(np.dot(gain_matrix, state_vector)[0][0], 2)

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
        arduino_serial_port.write(string)

    def interpret_data(arduino_serial_port, data_string):

        """
        Parse data into state_vector and send control input using LQR
        """

        previous_control_input = 0
        previous_state_vector = {}

        # If I try to print to serial from Arduino it messes up code in parse_state_vector
        # So catch the IndexError that way I can print things from Arduino for debugging purposes
        try:
            state_vector = parse_state_vector(data_string)

            # If the pendulum angle is outside of the bounds, don't supply any control input
            if abs(state_vector["pendulum_angle"]) > np.deg2rad(35):
                control_input = 0.0
            else:
                control_input = compute_input(state_vector)

            encoded_control_input_string = ("<" + str(control_input) + ">").encode()
            write_to_arduino(arduino_serial_port, encoded_control_input_string)

            if (control_input != previous_control_input) or (state_vector != previous_state_vector):
                print(f"{control_input} -- {state_vector}")
                previous_control_input = control_input
                previous_state_vector = state_vector

        except IndexError:
            print(data_string)

    run_arduino_control(interpret_data, "/dev/cu.usbmodem14101", baud_rate=19200, timeout=0, start_character="<", end_character=">")