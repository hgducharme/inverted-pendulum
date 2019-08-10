from control.matlab import *
import numpy as np

_g = 9.807


class InvertedPendulum:
    def __init__(
        self,
        massCart=None,
        massPendulum=None,
        lengthCM=None,
        totalLength=None,
        frictionCoeff=None,
    ):
        self.massCart = massCart
        self.massPendulum = massPendulum
        self.lengthCM = lengthCM
        self.totalLength = totalLength
        self.frictionCoeff = frictionCoeff

    def __str__(self):
        return "System"

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
    def nonLinearModel(self, value):

        stateVector, controlInput = value

        # Define short hand for variables
        x1 = stateVector[0]  # theta (rad)
        x2 = stateVector[1]  # x
        x3 = stateVector[2]  # theta dot (rad)
        x4 = stateVector[3]  # x dot
        u = controlInput

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
        A34 = self.frictionCoeff * self._beta * equilibriumPoint / denominator
        A41 = -self.massPendulum * _g * self.lengthCM * self._beta / denominator
        A44 = -self.frictionCoeff / denominator

        B31 = -self._beta * equilibriumPoint / denominator
        B41 = 1 / denominator

        A = [[0, 0, 1, 0], [0, 0, 0, 1], [A31, 0, 0, A34], [A41, 0, 0, A44]]
        B = [[0], [0], [B31], [B41]]
        C = [[1, 0, 0, 0], [0, 1, 0, 0]]
        D = [[0], [0]]

        self._stateSpace = ss(A, B, C, D)

    def modelIntegrator(self, time, state):
        """

        scipy.integrate.solve_ivp() requires that a function only take inputs t and y.
        A work around for supplying additional arguments is to use a wrapper function like this.
        see: https://stackoverflow.com/questions/48245765/pass-args-for-solve-ivp-new-scipy-ode-api

        """

        self.nonLinearModel = (state, self.controlLaw)
        return self.nonLinearModel
