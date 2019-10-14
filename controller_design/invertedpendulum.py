from control.matlab import *
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches
import matplotlib.text
from matplotlib import animation


_g = 9.807


class InvertedPendulum:

    def __init__(
        self,
        massCart=None,
        massPendulum=None,
        lengthCM=None,
        totalLength=None,
        frictionCoeff=None,
        cartWidth=1,
        cartHeight=1,
        railsLength = 10,
    ):
        self.massCart = massCart
        self.massPendulum = massPendulum
        self.lengthCM = lengthCM
        self.totalLength = totalLength
        self.frictionCoeff = frictionCoeff
        self.__previous_theta_error__ = 0.0
        self.__previous_time__ = 0.0
        self.controlLaw = 0
        self.cartWidth = cartWidth
        self.cartHeight = cartHeight
        self.railsLength = railsLength

    def __str__(self):
        
        dictionary = {
            'massCart': self.massCart,
            'massPendulum': self.massPendulum,
            'lengthCM': self.lengthCM,
            'totalLength': self.totalLength,
            'frictionCoeff': self.frictionCoeff,
            'controlLaw': self.controlLaw,
            'cartWidth': self.cartWidth,
            'cartHeight': self.cartHeight,
            'railsLength': self.railsLength,
        }

        return f"{dictionary}"

    def __repr__(self):
        return self

    @property
    def massSystem(self):
        return self.massCart + self.massPendulum

    @property
    def MOI(self):
        return (1.0 / 12.0) * self.massPendulum * (self.totalLength ** 2.0)

    @property
    def _beta(self):
        return (
            self.massPendulum
            * self.lengthCM
            / (self.massPendulum * (self.lengthCM ** 2.0) + self.MOI)
        )

    @property
    def controlLaw(self):
        return self._controlLaw

    @controlLaw.setter
    def controlLaw(self, value):
        self._controlLaw = value

    @property
    def nonLinearModel(self):
        return self._nonLinearModel

    @nonLinearModel.setter
    def nonLinearModel(self, stateVector):

        # Define short hand for variables
        x1 = stateVector[0]  # theta (rad)
        x2 = stateVector[1]  # x
        x3 = stateVector[2]  # theta dot (rad)
        x4 = stateVector[3]  # x dot
        u = self.controlLaw

        # System of nonlinear first order differential equations
        nonlinearDenominator = self.massSystem - (
            self._beta * self.massPendulum *
            self.lengthCM * (np.cos(x1) ** 2)
        )

        x1Dot = x3
        x2Dot = x4
        x3Dot = (self._beta / nonlinearDenominator) * (
            (_g * self.massSystem * np.sin(x1))
            + (self.frictionCoeff * x4 - u) * np.cos(x1)
            - (
                self.massPendulum
                * self.lengthCM
                * (x3 ** 2)
                * np.sin(x1)
                * np.cos(x1)
            )
        )
        x4Dot = (1 / nonlinearDenominator) * (
            u
            - (self.frictionCoeff * x4)
            + self.massPendulum
            * self.lengthCM
            * np.sin(x1)
            * ((x3 ** 2) - (_g * self._beta * np.cos(x1)))
        )

        self._nonLinearModel = [x1Dot, x2Dot, x3Dot, x4Dot]

    @property
    def stateSpace(self):
        return self._stateSpace

    @stateSpace.setter
    def stateSpace(self, equilibriumPoint=None):
        denominator = self.massSystem - (self._beta * self.massPendulum * self.lengthCM)

        A31 = _g * self._beta * self.massSystem * equilibriumPoint / denominator
        A34 = -self.frictionCoeff * self._beta * equilibriumPoint / denominator
        A41 = self.massPendulum * _g * self.lengthCM * self._beta / denominator
        A44 = -self.frictionCoeff / denominator

        B31 = self._beta * equilibriumPoint / denominator
        B41 = 1 / denominator

        A = [[0, 0, 1, 0], [0, 0, 0, 1], [A31, 0, 0, A34], [A41, 0, 0, A44]]
        B = [[0], [0], [B31], [B41]]
        C = [[1, 0, 0, 0], [0, 1, 0, 0]]
        D = [[0], [0]]

        self._stateSpace = ss(A, B, C, D)

    def integrate_nonlinear_dynamics(self, time, state):
        """

        scipy.integrate.solve_ivp() requires that a function only take inputs t and y.
        A work around for supplying additional arguments is to use a wrapper function like this.
        see: https://stackoverflow.com/questions/48245765/pass-args-for-solve-ivp-new-scipy-ode-api

        """


        # print(time)
        # dt = time - self.__previous_time__

        # # Calculate error
        # theta_error = 0 - state[0]

        # # Run it through the PID controller
        # kp = 70
        # ki = 150
        # kd = 100

        # proportion = kp * theta_error
        # integration = ki * theta_error * dt

        # if dt == 0:
        #     derivative = 0
        # else:
        #     derivative = kd  * (theta_error - self.__previous_theta_error__) / dt

        # self.controlLaw = proportion + integration + derivative

        # # Store the previous error
        # self.__previous_theta_error__ = theta_error
        # self.__previous_time__ = time

        self.nonLinearModel = (state)
        return self.nonLinearModel

    def make_animation(self, time, theta, cart_position, slow_motion_factor = 1):

        def init_animation():
            ax.add_patch(cart)
            ax.add_line(pendulum)

            return (cart, pendulum)

        def animate(iter):

            # Define and caclulate cart parameters
            cart_bottom_left_x_position = cart_position[iter] - (self.cartWidth / 2.0)
            cart_center = [cart_position[iter], 0.5 * self.cartHeight]

            # Define and calculate pendulum paramters
            angle = theta[iter]
            pendulum_end_point = [
                cart_center[0] - self.totalLength * np.sin(angle),
                self.totalLength * np.cos(angle),
            ]

            # Update positions
            cart.set_width(self.cartWidth)
            cart.set_height(self.cartHeight)
            cart.set_xy([cart_bottom_left_x_position, 0])

            pendulum.set_xdata([cart_center[0], pendulum_end_point[0]])
            pendulum.set_ydata([cart_center[1], pendulum_end_point[1]])

            return (cart, pendulum)

        # Make sure everything fits on the screen
        if self.railsLength/2.0 > self.totalLength:
            x_axis = self.railsLength/2.0
            y_axis = self.totalLength
        else:
            x_axis = self.totalLength*1.15
            y_axis = self.totalLength*1.15

        # Initialize the simulation figure
        fig, ax = plt.subplots(figsize=(10, 6))
        plt.axhline(0, xmin=-1, xmax=1, linewidth="0.75", color="k")
        left_constraint = plt.vlines(
            -self.railsLength / 2.0, ymin=0, ymax=(0.05)*(y_axis), linestyle="-", color="r"
        )
        right_constraint = plt.vlines(
            self.railsLength / 2.0, ymin=0, ymax=(0.05)*(y_axis), linestyle="-", color="r"
        )

        ax.set(xlim=(-x_axis, x_axis), ylim=(-y_axis, y_axis))
        plt.gca().set_aspect("equal")
        ax.set(xlabel="x-position", ylabel="y-position")

        # Define the artists
        cart = patches.Rectangle((0, 0), 0, 0, fc="k", fill=False)
        pendulum = lines.Line2D([], [], linewidth=4, color="c")
        # angle_text = Annotation(f"Angle: ", )

        # Define the animation parameters
        frames_per_second = 60
        interval = 1 / frames_per_second

        anim = animation.FuncAnimation(
            fig,
            animate,
            init_func=init_animation,
            frames=len(time),
            interval=1000 * interval * slow_motion_factor,
            blit=True,
        )

        plt.show()
