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

def plot_dynamics(time, theta, cart_position):
    fig, ax = plt.subplots(2, 1, sharex = True, sharey = False)
    ax[0].plot(time, np.rad2deg(theta))
    ax[1].plot(time, cart_position * (100.0 / 2.54))
    ax[0].set(ylabel="theta [deg]")
    ax[1].set(xlabel="time [s]", ylabel="cart position [in]")
    ax[1].tick_params()
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


if __name__ == "__main__":

    # Create an inverted pendulum system
    sys = InvertedPendulum(massCart=0.3, massPendulum=0.125, lengthCM=0.15, totalLength=0.30, frictionCoeff=2.5, cartWidth=0.0635, cartHeight=0.053975, railsLength = .45)
    sys.controlLaw = 0

    # Solve and plot the non-linear dynamics for no control input
    initialConditions = [0, 0, 0.1, 0]
    timeSpan = [0, 9]
    stepSize = 0.01
    solution = solve_ivp(sys.modelIntegrator, [0, 9], [0, 0, 0.1, 0], max_step = stepSize)
    plot_dynamics(solution.t, solution.y[0], solution.y[1])
    sys.make_animation(solution.t, solution.y[0], solution.y[1])

    # Create the state space model to start analyzing control
    sys.stateSpace = (1)
    model = sys.stateSpace
    controllabilityMatrix = ctrb(model.A, model.B)
    observabilityMatrix = obsv(model.A, model.C)
    eigenvalues, eigenvectors = np.linalg.eig(model.A)

    if np.linalg.matrix_rank(controllabilityMatrix) == 4:
        print('The system is controllable.')
    else:
        print('The system is not controllable.')

    if np.linalg.matrix_rank(observabilityMatrix) == 4:
        print('The system is observable.')
    else:
        print('The system is not observable.')

    # Define the values that build the Q and R matricies
    # This has the form [desired settling time, max desired value]
    desired_settling_times = {
        'theta': 0.3,
        'cart': 5,
        'theta_dot': None,
        'cart_dot': None
    }
    max_desired_values = {
        'theta': np.deg2rad(30),
        'cart': 0.19,
        'theta_dot': None,
        'cart_dot': None,
        'control_input': 18
    }

    # Build the Q and R matricies
    q1 = ( desired_settling_times['theta'] * (max_desired_values['theta']**2) )**(-1)
    q2 = ( desired_settling_times['cart'] * (max_desired_values['cart']**2) )**(-1)
    q3 = 1
    q4 = 1
    r1 = ( max_desired_values['control_input']**(-2) )

    Q = [
        [q1, 0, 0, 0],
        [0, q2, 0, 0],
        [0, 0, q3, 0],
        [0, 0, 0, q4],
    ]
    
    R = [
        [r1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]

    # Solve the Riccati equation and compute K
    sys.stateSpace = (1)
    pendulum_up_model = sys.stateSpace
    feedback_gain, P, closedloop_eigenvalues = lqr(pendulum_up_model, Q, r1)

    # Simulate the closed loop system to an inital condition
    closedloop_sys = ss(model.A - (model.B)*feedback_gain, model.B, model.C, model.D)
    time_span = np.linspace(0, 5, 5/0.01)
    IC = [0, 0, 2, 0]
    time, response = step_response(closedloop_sys, time_span, IC)
    theta_response = response[0]
    cart_response = response[1]
    plot_dynamics(time, theta_response, cart_response)
    sys.make_animation(time, theta_response, cart_response)

    # Get each output as a transfer function so we can analyze its step response
    # tf() is very picky about the inputs
    theta_tf, cart_tf = get_transfer_functions(model)

    # Build a PID transfer function and analyze the feedback step response
    # PID transfer function has the form: C(s) = Kp*s + Ki/s + Kd * (s * N)/(s + N)
    # kp = 0
    # ki = 0
    # kd = 0
    # filter_coeff = 100
    # pid_tf = pid_transfer_function(kp, ki, kd, filter_coeff)
    
    # # Analyze the closed loop step response
    # closedloop_tf = series(pid_tf, theta_tf)
    
    # time_step = 0.1
    # time_span = np.linspace(0, 15, 15/time_step)
    # response = feedback(closedloop_tf, 1)
    
    # t, theta_response = step_response(closedloop_tf, time_span)
    # plt.plot(t, np.rad2deg(theta_response))
    # plt.legend(['Theta'],loc = 'best')
    # plt.show()

    # Since theta_tf is unstabe without feedback, this plot just shows the exponential growth
    # Since there is an eigenvalue in the right half plane one of the exponential terms will grow due
    # to the positive real part of said eigenvalue
    # t, theta_response = step_response(theta_tf, time_span)
    # plt.plot(t, np.rad2deg(theta_response))
    # plt.legend(['Theta'],loc = 'best')
    # plt.show()

    # solution = solve_ivp(sys.modelIntegrator, [0, 20], [0, 0, 0.1, 0], max_step = 0.01)
    # plot_dynamics(solution.t, solution.y[0], solution.y[1])
    # make_animation(solution)