
import math
import numpy as np
from Plottings import plot_cartpole
import matplotlib.pyplot as plt

# Load data
data = np.load('./Results/BaS_Cartpole_Testing_lim_1.npy', allow_pickle=True).item()
solved_u = data['solved_controls'].flatten()
dt = data['params']['dt']
horizon = data['params']['horizon']
cart_lim = data['params']['cart_lim']

# Parameters
mc, mp, l, g = 0.5, 0.5, 1, 9.81

# Initialization
X0 = [0, 0, 0, 0, 1/cart_lim**2]
traj = [X0]

# System propagation
for i in range(len(solved_u)):
    U = solved_u[i]
    X = traj[i]

    # Augmented Continuous Dynamics
    dx = X[2]
    dq = X[3]
    ddx = (U + mp * np.sin(X[1]) * (l * X[3] * X[3] + g * np.cos(X[1]))) / (
                mc + mp * np.sin(X[1]) * np.sin(X[1]))                                # acceleration of x
    ddq = (-U * np.cos(X[1]) - mp * l * X[3] * X[3] * np.sin(X[1]) * np.cos(X[1]) - (mc + mp) * g * np.sin(X[1])) / (
            l * mc + l * mp * np.sin(X[1]) * np.sin(X[1]))                            # acceleration of theta
    dz = (- (1 / (cart_lim ** 2 - X[0] ** 2)) ** 2) * (-2 * X[0]) * X[2]              # BaS Derivative

    # Euler integration
    x = X[0] + dt * dx
    q = X[1] + dt * dq
    dx = X[2] + dt * ddx
    dq = X[3] + dt * ddq
    z = X[4] + dt * dz

    traj += [[x, q, dx, dq, z]]

plot_cartpole.plotcartpole([0, math.pi, 0, 0, 1], np.array(traj), [], [], 1)
plt.show()