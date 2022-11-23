import math

from Plottings import plot_cartpole
from casadi import *
import scipy.io as sio
import matplotlib.pyplot as plt

load_data = sio.loadmat('./Results/BaS_Cartpole_firstTest.mat')
traj = load_data['results']['solved_trajectory'][0, 0]
ctrl = load_data['results']['solved_controls'][0, 0]

dt = 0.12
horizon = 25
times = np.linspace(0, dt*horizon-dt, horizon+1)
plot_cartpole.plotcartpole([0, 0, 0, 0, 0], [0, math.pi, 0, 0, 0], times, traj.T, ctrl.T, 1)
plt.show()

# plt.plot(traj[:, 0], label='x')
# plt.show()
# plt.plot(traj[:, 1], label='theta')
# plt.show()
# plt.plot(traj[:, 2], label='vel')
# plt.show()
# plt.plot(traj[:, 3], label='ang_vel')
# plt.show()
# plt.plot(traj[:, 4], label='BaS')
# plt.show()

print('o teu cu eh mau')
print('engoliu meu pau')