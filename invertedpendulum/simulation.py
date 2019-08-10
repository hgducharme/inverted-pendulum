import numpy as np
from control.matlab import *
from scipy.integrate import solve_ivp
from scipy import signal

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches
from matplotlib import animation

from invertedpendulum import InvertedPendulum


def PID(kp, ki, kd, filter_coefficient, reference, value, previous_value, dt):
    error = value - reference
    proportional = kp * error
    integral = ki * error * dt
    derivative = kd * (error - previous_error)

def plot_results(time, theta, cart_position):
    fig, ax = plt.subplots(2, 1, sharex = True, sharey = False)
    ax[0].plot(time, np.rad2deg(theta))
    ax[1].plot(time, cart_position * (100.0 / 2.54))
    ax[0].set(ylabel="theta [deg]")
    ax[1].set(xlabel="time [s]", ylabel="cart position [in]")
    ax[1].tick_params()
    plt.show()


def make_animation(solution):

    def init_animation():
        ax.add_patch(cart)
        ax.add_line(pendulum)

        return (cart, pendulum)

    def animate(iter):
        # Define and caclulate cart parameters
        cart_width = 2.50
        cart_height = 2.125
        cart_bottom_left_x_position = solution.y[1][iter] - (cart_width / 2.0)
        cart_center = [cart_bottom_left_x_position + (0.5 * cart_width), 0.5 * cart_height]

        # Define and calculate pendulum paramters
        pendulum_length = 12
        angle = solution.y[0][iter]
        pendulum_end_point = [
            cart_center[0] - pendulum_length * np.sin(angle),
            pendulum_length * np.cos(angle),
        ]

        # Update positions
        cart.set_width(cart_width)
        cart.set_height(cart_height)
        cart.set_xy([cart_bottom_left_x_position, 0])

        pendulum.set_xdata([cart_center[0], pendulum_end_point[0]])
        pendulum.set_ydata([cart_center[1], pendulum_end_point[1]])

        return (cart, pendulum)

    # Initialize the simulation figure
    linear_rails_length = 17.60

    fig, ax = plt.subplots(figsize=(10, 6))
    plt.axhline(0, xmin=-9, xmax=9, linewidth="0.75", color="k")
    left_constraint = plt.vlines(
        -linear_rails_length / 2.0, ymin=0, ymax=1, linestyle="-", color="r"
    )
    right_constraint = plt.vlines(
        linear_rails_length / 2.0, ymin=0, ymax=1, linestyle="-", color="r"
    )

    ax.set(xlim=(-15, 15), ylim=(-15, 15))
    plt.gca().set_aspect("equal")
    ax.set(xlabel="x-position [in]", ylabel="y-position [in]")

    # Create the cart and pendulum shapes
    cart = patches.Rectangle((0, 0), 0, 0, fc="k", fill=False)
    pendulum = lines.Line2D([], [], linewidth=4, color="c")

    # Define the animation parameters
    frames_per_second = 60
    interval = 1 / frames_per_second

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init_animation,
        frames=len(solution.t),
        interval=1000 * interval,
        blit=True,
    )

    plt.show()


if __name__ == "__main__":


    # Create an inverted pendulum system and define the control law
    sys = InvertedPendulum(massCart=0.3, massPendulum=0.125, lengthCM=0.15, totalLength=0.30, frictionCoeff=2.5)
    sys.controlLaw = 0
    initialConditions = [0, 0, 0.1, 0]
    timeSpan = [0, 9]
    stepSize = 0.01

    # Solve and plot the non linear dynamics
    solution = solve_ivp(sys.modelIntegrator, [0, 9], [0, 0, 0.1, 0], max_step = stepSize)
    plot_results(solution.t, solution.y[0], solution.y[1])
    make_animation(solution)

    # Create the state space model to start controlling the dynamics
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
    
    # This needs work.
    timeSpan = np.linspace(0, 20, 100)
    t, y = step(model, timeSpan)
    plt.plot(y, t)
    plt.legend(['y1','y2'],loc='best')
    plt.show()
