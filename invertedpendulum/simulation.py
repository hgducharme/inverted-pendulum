from control.matlab import *
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from nonlinearModel import modelWrapper

# # Define system parameters
# g = 9.807
# massPendulum = 0.25
# massCart = 0.320
# massSystem = massPendulum + massCart
# lengthCM = 0.15
# totalLength = lengthCM * 2
# frictionCoeff = 0.5

# # Define derived quantities
# MOI = (1/3) * massPendulum * totalLength**2
# beta = massPendulum * lengthCM/(massPendulum * lengthCM**2 + MOI)
# denominator = massCart + massPendulum * (1 - lengthCM * beta)
# equilibriumPoint = 1

# # Define state space model
# A31 = g * beta * massSystem * equilibriumPoint/denominator
# A34 = -frictionCoeff * beta * equilibriumPoint/denominator
# A41 = massPendulum * g * totalLength * beta/denominator
# A44 = -frictionCoeff/denominator

# B31 = beta * equilibriumPoint/denominator
# B41 = 1/denominator

# A = [ [0, 0, 1, 0], [0, 0, 0, 1], [A31, 0, 0, A34], [A41, 0, 0, A44] ]
# B = [ [0], [0], [B31], [B41] ]  
# C = [ [1, 0, 0, 0], [0, 1, 0, 0] ]
# D = [ [0], [0] ]

# model = ss(A, B, C, D)
#model.StateName = {'theta', 'x', 'thetaDot', 'xDot'};
#model.OutputName = {'theta', 'x'};


### All the above can now be replaced with
from invertedpendulum import InvertedPendulum

sys = InvertedPendulum(0.25, 0.320, 0.15, 0.30, 0.50)
sys.nonlinearModel = 1
model = sys.nonlinearModel

# if __name__ == "__main__":

#     solution = solve_ivp(modelWrapper, [0, 20], [0, 0, 0, 0])
    
#     fig, ax = plt.subplots(1, 1)
#     ax.plot(solution.t, solution.y[0])
#     ax.set(xlabel='t',ylabel='y1')

#     fig, ax = plt.subplots(1, 1)
#     ax.plot(solution.t, solution.y[1])
#     ax.set(xlabel='t',ylabel='y2')
    
#     plt.show()