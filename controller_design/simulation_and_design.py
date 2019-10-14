import numpy as np
from control import *
from control.matlab import ctrb, obsv, ss2tf, step
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches
from matplotlib import animation

from invertedpendulum import InvertedPendulum


def pid_transfer_function(kp, ki, kd, filter_coeff):
    proportional_tf = tf( [kp, 0], [1])
    integral_tf = tf( [ki], [1, 0])
    derivative_tf = tf( [kd * filter_coeff, 0], [1, filter_coeff] )
    controller_tf = parallel(proportional_tf, integral_tf, derivative_tf)

    return controller_tf

def PID(kp = None, ki = None, kd = None, filter_coefficient = None, error = None, previous_error = None, dt = None):
    proportional = kp * error
    integral = ki * error * dt
    derivative = kd * (error - previous_error) / dt

    return proportional + integral + derivative

def plot_dynamics(time, theta, cart_position, title = ""):
    fig, ax = plt.subplots(2, 1, sharex = True, sharey = False)
    ax[0].plot(time, np.rad2deg(theta))
    ax[1].plot(time, cart_position * (100.0 / 2.54))
    ax[0].set(ylabel="theta [deg]")
    ax[1].set(xlabel="time [s]", ylabel="cart position [in]")
    ax[1].tick_params()
    ax[0].set(title=title)
    plt.show()

def get_transfer_functions(statespace_model):
    transfer_functions = ss2tf(statespace_model)

    # Loop through the coefficients and set ones close to zero equal to zero.
    # This will modify the actual transfer_functions arrays
    all_coeffs = [ transfer_functions.num[0][0], transfer_functions.den[0][0], transfer_functions.num[1][0], transfer_functions.den[1][0]]
    for i in range(len(all_coeffs)):
        for j in range(len(all_coeffs[i])):
            if abs(all_coeffs[i][j]) < (10**(-10)):
                all_coeffs[i][j] = 0

    theta_tf = tf(all_coeffs[0], all_coeffs[1])
    cart_tf = tf(all_coeffs[2], all_coeffs[3])

    return theta_tf, cart_tf

def is_controllable_and_observable(A, B, C):
    controllabilityMatrix = ctrb(A, B)
    observabilityMatrix = obsv(A, C)
    eigenvalues, eigenvectors = np.linalg.eig(A)

    if np.linalg.matrix_rank(controllabilityMatrix) == np.linalg.matrix_rank(A):
        controllable = True
    else:
        controllable = False

    if np.linalg.matrix_rank(observabilityMatrix) == np.linalg.matrix_rank(A):
        observable = True
    else:
        observable = False

    return controllable, observable



if __name__ == "__main__":

    # Shim measurements
    # mass: 0.016 kg
    # length: 12 in


    #####################################
    #          Natural response         #
    #####################################


    # Create an inverted pendulum system
    system = InvertedPendulum(massCart=0.27, massPendulum=0.016, lengthCM=0.1524, totalLength=0.3048, frictionCoeff=2.5, cartWidth=0.0635, cartHeight=0.053975, railsLength = .45)
    system.controlLaw = 0

    # Solve the non-linear dynamics for no control input
    initialConditions = [0, 0, 1, 0]
    timeSpan = [0, 9]
    stepSize = 0.01
    solution = solve_ivp(system.integrate_nonlinear_dynamics, timeSpan, initialConditions, max_step = stepSize)

    # Plot and animate the result
    time = solution.t
    theta_response = solution.y[0]
    cart_response = solution.y[1]
    plot_dynamics(time, theta_response, cart_response, "Natural Response")
    system.make_animation(time, theta_response, cart_response)
    
    # Create the state space model to start analyzing control
    system.stateSpace = (1)
    model = system.stateSpace
    is_controllable, is_observable = is_controllable_and_observable(model.A, model.B, model.C)


    #####################################
    #            LQR Design             #
    #####################################


    # Define the values that build the Q and R matricies
    # This has the form [desired settling time, max desired value]
    desired_settling_times = {
        'theta': 0.2,     # sec
        'cart': 1,        # sec
        'theta_dot': 0.1, # sec
        'cart_dot': 1,    # sec
    }
    max_desired_values = {
        'theta': np.deg2rad(6),      # rad/s
        'cart': 0.1,                 # m/s
        'theta_dot': np.deg2rad(90), # rad/s
        'cart_dot': None,            # m/s
        'control_input': 12          # volts
    }

    # Build the Q and R matricies
    q1 = ( desired_settling_times['theta'] * (max_desired_values['theta']**2) )**(-1)
    q2 = ( desired_settling_times['cart'] * (max_desired_values['cart']**2) )**(-1)
    q3 = ( desired_settling_times['theta_dot'] * (max_desired_values['theta_dot']**2) )**(-1)
    q4 = 10
    r1 = ( max_desired_values['control_input']**(-2) )

    # Custom values
    # q1 = 100
    # q2 = 10
    # q3 = 50
    # q4 = 1

    Q = [
        [q1, 0, 0, 0],
        [0, q2, 0, 0],
        [0, 0, q3, 0],
        [0, 0, 0, q4],
    ]
    
    R = [0.01]

    # Solve the Riccati equation and compute K
    system.stateSpace = (1)
    pendulum_up_model = system.stateSpace
    feedback_gain, P, closedloop_eigenvalues = lqr(pendulum_up_model, Q, R)
    print(f"gain: {feedback_gain}")

    # Compute the response to a step input 
    closedloop_sys = ss(model.A - (model.B)*feedback_gain, model.B, model.C, model.D)
    time_span = np.linspace(0, 10, 10/0.01)
    IC = [np.deg2rad(-5), 0, -2, 0]
    time, response = step_response(closedloop_sys, time_span, IC)

    # Plot and animate the response
    theta_response = response[0]
    cart_response = response[1]
    plot_dynamics(time, theta_response, cart_response, "LQR Control")
    system.make_animation(time, theta_response, cart_response)

    # Get each output as a transfer function so we can analyze its step response
    # tf() is very picky about the inputs
    theta_tf, cart_tf = get_transfer_functions(model)
    print(f"eigenvalues: {closedloop_eigenvalues}")


    #####################################
    #       Kalman Filter Design        #
    #####################################

    