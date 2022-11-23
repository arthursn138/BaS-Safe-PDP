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
max_x = 0.8        # CONSTRAINT / OBSTACLE FOR BAS
max_u = 4          # CONTROL LIMIT
env.initDyn(mc=mc, mp=mp, l=l, cart_limit=max_x)
wx, wq, wdx, wdq, wz, wu = 0.3, 1, 0.1, 0.1, 0.1, 0.1
env.initCost(wx=wx, wq=wq, wdx=wdx, wdq=wdq, wz=wz, cart_limit=max_x, wu=wu)
# Not used, since gamma = 0
env.initConstraints(max_u=max_u, max_x=max_x)
dt = 0.12
horizon = 30
init_state = [0, 0, 0, 0, 0]
dyn = env.X + dt * env.f
# --------------------------- create Safe PDP SPlan object ----------------------------------------
planner = SafePDP.CSysOPT()
planner.setStateVariable(env.X)
planner.setControlVariable(env.U)
planner.setDyn(dyn)
planner.setPathCost(env.path_cost)
planner.setFinalCost(env.final_cost)
planner.setPathInequCstr(env.path_inequ)
gamma = 0 ###### THIS IS VERY IMPORTANT: gamma = 0  means that we do not add any barrier at all! (to the cost function)
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
env.play_animation(pole_len=2, dt=dt, state_traj=coc_sol['state_traj_opt'], save_option=0, title='NLP Solver')
# plt.plot(coc_sol['control_traj_opt'], label='ct_control')
# plt.plot(coc_sol['state_traj_opt'][:, 0], label='ct_cart_pos')
# plt.fill_between(np.arange(0, horizon), 1, -1, color='red', alpha=0.2)
# plt.fill_between(np.arange(0, horizon), max_u, -max_u, color='green', alpha=0.2)
# plt.legend()
# plt.show()

# TODO: SOLVE WITH DDP AS BASELINE (MAYBE SUBSTITUTE ALTRO?)

# # --------------------------- Barrier States Augmentation ----------------------------------------
# TODO: MAKE IT MODULAR!!!!
# cart_limit = max_x
# bas = env
# # Create obstacles and barrier functions
# h = cart_limit ** 2 - bas.f[0] ** 2
# h_fn = casadi.Function('h_fn', [bas.X], [h])
# B = 1/h
# B_fn = casadi.Function('B_fn', [bas.X], [B])
# # Augment the dynamics / State-space
# z = SX.sym('z')
# bas_states = vertcat(bas.X, z) # NA VDD TEM QUE STACKAR O X0 E XF
# # AÍ COMO É QUE STACKA O B EMBAIXO DAS OUTRAS EQ DE DYN SEM SER NA MÃO????
#
# # bas_dyn = vertcat(bas.f, B_fn)
# # bas_dyn_disc = bas.X + dt * bas_dyn
#
# # FODAC, FAZ NA MÃO MEMO SÓ PRA TER ALGUMA COISA, DEPOIS PENSA EM COMO FAZER AUTOMATICAMENTE
#
# # SE ENSEBAR MTO (40MIN MAXIMO) METE DIRETÃO NO NASCIMENV UM CTRL C+V DO CARTPOLE PRA CARTPOLE_BAS, E SÓ ENFIA A FUNCAO LÁ NA MAO MEMO
#
#
# TODO: MAKE IT MODULAR!!!!
# --------------------------- Safe Motion SPlan ----------------------------------------
# set the policy as polynomial
n_poly = 10
planner.setPolyTraj(horizon=horizon, n_poly=n_poly)
# set the initial condition
nn_seed = None
init_parameter = np.zeros(planner.n_control_auxvar)  # all zeros initial condition
# nn_seed = 200 # e.g. 200, 300, 400, 500
# init_parameter = 0.1*np.random.randn(planner.n_control_auxvar)  # random initial condition

# planning parameter setting
max_iter = 3000
loss_barrier_trace, loss_trace = [], []
parameter_trace = np.empty((max_iter, init_parameter.size))
control_traj, state_traj = 0, 0
lr = 1e-1

# start safe motion planning
current_parameter = init_parameter
for k in range(int(max_iter)):
    # one iteration of PDP
    loss_barrier, loss, dp, state_traj, control_traj, = planner.step(init_state=init_state, horizon=horizon,
                                                                     control_auxvar_value=current_parameter)
    # storage
    loss_barrier_trace += [loss_barrier]
    loss_trace += [loss]
    parameter_trace[k, :] = current_parameter

    # update
    current_parameter -= lr * dp

    # print
    if k % 100 == 0:
        print('Iter #:', k, 'Loss_barrier:', loss_barrier, 'Loss:', loss)

# # save the results
# if True:
#     save_data = {'parameter_trace': parameter_trace,
#                  'loss_trace': loss_trace,
#                  'loss_barrier_trace': loss_barrier_trace,
#                  'gamma': gamma,
#                  'solved_trajectory': state_traj,
#                  'solved_controls': control_traj,
#                  'coc_sol': coc_sol,
#                  'lr': lr,
#                  'init_parameter': init_parameter,
#                  'n_poly': n_poly,
#                  'dt': dt,
#                  'horizon': horizon
#                  }
#     np.save('./Results/BaS_Cartpole_firstTest.npy', save_data)                      # .npy
#     sio.savemat('./Results/BaS_Cartpole_firstTest.mat', {'results': save_data})     # .mat

# plt.plot(control_traj, label='SPDP_control')
# plt.plot(coc_sol['control_traj_opt'], label='ct_control')
# plt.plot(state_traj[:, 0], label='SPDP_cart_pos')
# plt.plot(coc_sol['state_traj_opt'][:, 0], label='ct_cart_pos')
# plt.fill_between(np.arange(0, horizon), max_x, -max_x, color='red', alpha=0.2)
# plt.fill_between(np.arange(0, horizon), max_u, -max_u, color='green', alpha=0.2)
# plt.legend()
# plt.show()
times = np.linspace(0, dt*horizon-dt, horizon+1)
plot_cartpole.plotcartpole(init_state, env.xf, times, state_traj.T, control_traj.T, max_x)
plt.show()
env.play_animation(pole_len=2, dt=dt, state_traj=state_traj, save_option=0, title='BaS-Learned Motion (barrier at 0.8)')

# TODO: Add cart limits in the animation