import numpy as np

from SafePDP import SafePDP
from SafePDP import PDP
from NascimEnv import NascimEnv
from BaS import BaS
from Plottings import plot_cartpole
from casadi import *
import scipy.io as sio
import matplotlib.pyplot as plt
import time
import random

# --------------------------- load environment ----------------------------------------
env = NascimEnv.BaS_CartPole()
mc, mp, l = 0.5, 0.5, 1
max_x = 1        # CONSTRAINT / OBSTACLE FOR BAS ----- PROBLEMATIC AROUND 0.9!!! -> 0.91 = BREACH OF SAFETY!!!!!
max_u = 10e40       # CONTROL LIMIT [original: 4]
env.initDyn(mc=mc, mp=mp, l=l, cart_limit=max_x, gamma=0)
# wx, wq, wdx, wdq, wz, wu = 0.3, 1.5, 0.1, 0.1, 0.1, 0.1     # Worked beautifully before
wx, wq, wdx, wdq, wz, wu = 0.3, 1.5, 0.1, 0.1, 100, 0.001
# wx, wq, wdx, wdq, wz, wu = 0.1, 0.1, 0.1, 0.1, 0.1, 0.1
env.initCost(wx=wx, wq=wq, wdx=wdx, wdq=wdq, wz=wz, cart_limit=max_x, wu=wu)
# Not used, since gamma = 0
env.initConstraints(max_u=max_u, max_x=max_x)
dt = 0.02            # Hassan: make 0.02 // orig 0.12
horizon = 150        # Hassan: make 150 // orig 25(?), eu mudei pra 30
init_state = [0, 0, 0, 0, 1/max_x**2]
dyn = env.X + dt * env.f
# # # OVERWRITING THE BARRIER STATES NOW THAT THE SYSTEM GOT DISCRETIZED, FOR DEBUGGING:
# # # INDICATION THAT THE BAS AUGMENTATION CAN BE DONE OUTSIDE NASCIMENV! ---- Not so much.... doesn't work quite well here
# env.dz = (-(max_x ** 2 - env.x ** 2) ** (-2)) * (-2 * env.x * env.dx)        # NOTE: last term multiplies xdot instead of fminus[1]
# dyn[4] = env.z * dt + env.dz

# --------------------------- create Safe PDP SPlan object ----------------------------------------
planner = SafePDP.CSysOPT()
planner.setStateVariable(env.X)
planner.setControlVariable(env.U)
planner.setDyn(dyn)
planner.setPathCost(env.path_cost)
planner.setFinalCost(env.final_cost)
planner.setPathInequCstr(env.path_inequ)
gamma = 0 ###### THIS IS VERY IMPORTANT: gamma = 0  means that we will cancell the cost penalty (need gamma=0 for BaS)
# # # NOTE: The above gamma is for the PENALTY method; not for the BaS equation --- if !=0 then penalty will be active
planner.convert2BarrierOC(gamma=gamma)

# --------------------------- create COC object only for result comparison ----------------------------------------
coc = SafePDP.COCsys()
coc.setStateVariable(planner.state)
coc.setControlVariable(planner.control)
coc.setDyn(planner.dyn)
coc.setPathCost(planner.path_cost)
coc.setFinalCost(planner.final_cost)
coc.setPathInequCstr(planner.path_inequ_cstr)
coc_sol = coc.ocSolver(init_state=init_state, horizon=horizon)
print('constrained cost', coc_sol['cost'])

h_opt = []
for j in range(len(coc_sol['state_traj_opt'])):
    h_opt += [1/(max_x ** 2 - (coc_sol['state_traj_opt'][j, 0]) ** 2)]

h_opt = np.array(h_opt)
plt.plot(coc_sol['state_traj_opt'][:, 0], label='X')
plt.plot(coc_sol['state_traj_opt'][:, 1], label=r'$\theta$')
plt.plot(coc_sol['state_traj_opt'][:, 4], label='BaS')
plt.plot(h_opt, label=r'$\frac{1}{h(x)}$', linestyle='dashed')
plt.plot(np.zeros(151), 'k', linestyle='dotted')
plt.plot(np.ones(151) * max_x, 'k', linestyle='dashdot')
plt.plot(np.ones(151) * -max_x, 'k', linestyle='dashdot')
plt.plot(np.ones(151) * np.pi, 'k', linestyle='dotted')
plt.legend()
plt.show()

# env.play_animation(pole_len=2, dt=dt, state_traj=coc_sol['state_traj_opt'], save_option=0, title='NLP Solver on BaS-'
#                                                                                                  'augmented system')
