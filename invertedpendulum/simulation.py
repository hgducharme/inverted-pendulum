import numpy as np
from control.matlab import *
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches
from matplotlib import animation

from invertedpendulum import InvertedPendulum


def modelWrapper(t, y):
    """

    scipy.integrate.solve_ivp() requires that a function only take inputs t and y.
    A work around for supplying additional arguments is to use a wrapper function like this.
    see: https://stackoverflow.com/questions/48245765/pass-args-for-solve-ivp-new-scipy-ode-api

    """

    sys = InvertedPendulum(
        massCart=0.25,
        massPendulum=0.125,
        lengthCM=0.15,
        totalLength=0.30,
        frictionCoeff=2.5,
    )
    control_law = 0
    sys.nonlinearModel = (y, control_law)

    return sys.nonlinearModel


# Solve and plot the non linear dynamics
solution = solve_ivp(modelWrapper, [0, 30], [0, 0, 0.1, 0])

fig, ax = plt.subplots(1, 1)
ax.plot(solution.t, np.rad2deg(solution.y[0]))
ax.set(xlabel="time [s]", ylabel="theta [deg]")

fig, ax = plt.subplots(1, 1)
ax.plot(solution.t, solution.y[1] * (100.0 / 2.54))
ax.set(xlabel="time [s]", ylabel="Cart position [in]")

# Initialize the simulation figure
linear_rails_length = 17.60
fig, ax = plt.subplots(figsize=(10, 7))
plt.axhline(0, xmin=-9, xmax=9, linewidth="0.75", color='k')
left_constraint = plt.vlines(-linear_rails_length/2.0, ymin=-1, ymax=1, linestyle="dashdot", color='r')
right_constraint = plt.vlines(linear_rails_length/2.0, ymin=-1, ymax=1, linestyle="dashdot", color='r')

ax.set(xlim=(-20, 20), ylim=(-20, 20))

# Create the cart and pendulum shapes
cart = patches.Rectangle((0, 0), 0, 0, fc="k", fill=False)
pendulum = lines.Line2D([], [], linewidth = 3, color = 'c')
pendulum = lines.Line2D([], [], linewidth = 3, color = 'c')
pendulum = lines.Line2D([], [], linewidth = 3, color = 'c')


def init():
    ax.add_patch(cart)
    ax.add_line(pendulum)

    return (cart, pendulum, )


def animate(iter):
    # Define and caclulate cart parameters
    cart_width = 2.50
    cart_height = 2.125
    cart_bottom_left_x_position = solution.y[1][iter] - (cart_width / 2.0)
    cart_center = [cart_bottom_left_x_position + (0.5*cart_width), 0.5*cart_height]

    # Define and calculate pendulum paramters
    pendulum_length = 12
    angle = solution.y[0][iter]
    pendulum_end_point = [ cart_center[0] - pendulum_length * np.sin(angle), pendulum_length * np.cos(angle)]

    # Update positions
    cart.set_width(cart_width)
    cart.set_height(cart_height)
    cart.set_xy([cart_bottom_left_x_position, 0])

    pendulum.set_xdata( [cart_center[0], pendulum_end_point[0]] )
    pendulum.set_ydata( [cart_center[1], pendulum_end_point[1]] )

    return (cart, pendulum, )


anim = animation.FuncAnimation(
    fig, animate, init_func=init, frames=len(solution.t), interval=75, blit=True
)

plt.show()
