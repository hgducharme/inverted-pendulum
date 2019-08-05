import numpy as np

# Define system parameters
g = 9.807
massPendulum = 0.25
massCart = 0.320
massSystem = massPendulum + massCart
lengthCM = 0.15
totalLength = lengthCM * 2
frictionCoeff = 0.5

# Define derived quantities
MOI = (1/3) * massPendulum * totalLength**2
beta = massPendulum * lengthCM/(massPendulum * lengthCM**2 + MOI)
denominator = massCart + massPendulum * (1 - lengthCM * beta)

def nonlinearModel(stateVector, controlInput, massCart, massPendulum, massSystem, frictionCoeff, beta, lengthCM):

    # Define short hand for variables
    x1 = stateVector[0] # theta
    x2 = stateVector[1] # x
    x3 = stateVector[2] # theta dot
    x4 = stateVector[3] # x dot
    u = controlInput

    # System of nonlinear first order differential equations
    nonlinearDenominator = massCart + massPendulum * (1 - lengthCM * beta * np.cos(x1)**2)

    x1Dot = x3
    x2Dot = x4
    x3Dot = (beta/nonlinearDenominator) * ( (g * massSystem * np.sin(x1)) + (u - frictionCoeff * x4) * np.cos(x1) - (massPendulum * lengthCM * (x3**2) * np.sin(x1) * np.cos(x1)) )
    x4Dot = (1/nonlinearDenominator) * (u - (frictionCoeff * x4) + (massPendulum * lengthCM * g * beta * np.sin(x1) * np.cos(x1)) - (massPendulum * lengthCM * (x3**2) * np.sin(x1)) )

    vectorField = [x1Dot, x2Dot, x3Dot, x4Dot]

    return vectorField

def modelWrapper(t, y):
    '''

    scipy.integrate.solve_ivp() requires that a function only take inputs t and y.
    A work around for supplying additional arguments is to use a wrapper function like this.
    see: https://stackoverflow.com/questions/48245765/pass-args-for-solve-ivp-new-scipy-ode-api

    '''

    return nonlinearModel(y, 0, massCart, massPendulum, massSystem, frictionCoeff, beta, lengthCM)