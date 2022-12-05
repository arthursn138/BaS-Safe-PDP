import math

from Plottings import plot_cartpole
from casadi import *
import scipy.io as sio
import matplotlib.pyplot as plt

load_data = sio.loadmat('./Results/SPlan_Cartpole_Arthur_lim_0.4.mat')
traj = load_data['results']['solved_trajectory'][0, 0]

# ctrl = load_data['results']['solved_controls'][0, 0]
# h = load_data['results']['barrier_function'][0, 0]
# dt = load_data['results']['dt'][0, 0]
# horizon = load_data['results']['horizon'][0, 0]
#
# dt = dt.item()
# horizon = horizon.item()

# FOR SPLAN ONLY, NEEDS TO COMMENT ABOVE
h = load_data['results']['inverse_BaS'][0, 0]
dt = 0.12
horizon = 30
ctrl = np.linspace(0, dt*horizon-dt, horizon+1).T

times = np.linspace(0, dt*horizon-dt, horizon+1)
plot_cartpole.plotcartpole([0, 0, 0, 0, 0], [0, math.pi, 0, 0, 0], times, traj.T, ctrl.T, h.T, 1)
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