from control.matlab import *
import numpy as np

_g = 9.807

class InvertedPendulum():

    def __init__(self, massCart = None, massPendulum = None, lengthCM = None, totalLength = None, frictionCoeff = None):
        self.massCart = massCart
        self.massPendulum = massPendulum
        self.lengthCM = lengthCM
        self.totalLength = totalLength
        self.frictionCoeff = frictionCoeff

    def __str__(self):
        return "System"

    @property
    def massCart(self):
        return self._massCart
    
    @property
    def massPendulum(self):
        return self._massPendulum

    @property
    def massSystem(self):
        return self._massCart + self._massPendulum

    @property
    def lengthCM(self):
        return self._lengthCM

    @property
    def totalLength(self):
        return self._totalLength

    @property
    def frictionCoeff(self):
        return self._frictionCoeff

    @property
    def MOI(self):
        return (1.0/12.0) * self._massPendulum * (self._totalLength**2.0)

    @property
    def _beta(self):
        return self._massPendulum * self._lengthCM/(self._massPendulum * (self._lengthCM**2.0) + self.MOI)

    @property
    def _denominator(self):
        return self.massSystem - (self._beta * self._massPendulum * self._lengthCM)

    @massCart.setter
    def massCart(self, value):
        self._massCart = value

    @massPendulum.setter
    def massPendulum(self, value):
        self._massPendulum = value

    @lengthCM.setter
    def lengthCM(self, value):
        self._lengthCM = value

    @totalLength.setter
    def totalLength(self, value):
        self._totalLength = value

    @frictionCoeff.setter
    def frictionCoeff(self, value):
        self._frictionCoeff = value

    @property
    def nonlinearModel(self):
        return self._nonlinearModel

    @nonlinearModel.setter
    def nonlinearModel(self, value):

        stateVector, controlInput = value

        # Define short hand for variables
        x1 = stateVector[0] # theta (rad)
        x2 = stateVector[1] # x
        x3 = stateVector[2] # theta dot (rad)
        x4 = stateVector[3] # x dot
        u = controlInput

        # System of nonlinear first order differential equations
        nonlinearDenominator = self.massSystem - ( self._beta * self._massPendulum * self._lengthCM * (np.cos(x1)**2) )

        x1Dot = x3
        x2Dot = x4
        x3Dot = (self._beta/nonlinearDenominator) * ( (_g * self.massSystem * np.sin(x1)) + (self._frictionCoeff * x4 - u) * np.cos(x1) - (self._massPendulum * self._lengthCM * (x3**2) * np.sin(x1) * np.cos(x1)) )
        x4Dot = (1/nonlinearDenominator) * ( u - (self._frictionCoeff * x4) + self._massPendulum * self._lengthCM * np.sin(x1)*( (x3**2) - (_g * self._beta * np.cos(x1)) ))

        vectorField = [x1Dot, x2Dot, x3Dot, x4Dot]

        self._nonlinearModel = vectorField

    @property
    def stateSpace(self):
        return self._stateSpace
    
    @stateSpace.setter
    def stateSpace(self, equilibriumPoint):
        A31 = _g * self._beta * self.massSystem * equilibriumPoint/self._denominator
        A34 = self._frictionCoeff * self._beta * equilibriumPoint/self._denominator
        A41 = -self._massPendulum * _g * self._lengthCM * self._beta/self._denominator
        A44 = -self._frictionCoeff/self._denominator

        B31 = -self._beta * equilibriumPoint/self._denominator
        B41 = 1/self._denominator

        A = [ [0, 0, 1, 0], [0, 0, 0, 1], [A31, 0, 0, A34], [A41, 0, 0, A44] ]
        B = [ [0], [0], [B31], [B41] ]  
        C = [ [1, 0, 0, 0], [0, 1, 0, 0] ]
        D = [ [0], [0] ]

        model = ss(A, B, C, D)

        self._stateSpace = model
