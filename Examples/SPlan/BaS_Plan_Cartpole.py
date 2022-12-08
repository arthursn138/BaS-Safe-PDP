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
max_u = 4       # CONTROL LIMIT [original: 4]
env.initDyn(mc=mc, mp=mp, l=l, cart_limit=max_x, gamma=0)
# wx, wq, wdx, wdq, wz, wu = 0.3, 1.5, 0.1, 0.1, 0.1, 0.1     # Worked beautifully before
wx, wq, wdx, wdq, wz, wu = 2.5, 5, 0.1, 0.1, 50, 0.10
# wx, wq, wdx, wdq, wz, wu = 0.1, 0.1, 0.1, 0.1, 0.1, 0.1
env.initCost(wx=wx, wq=wq, wdx=wdx, wdq=wdq, wz=wz, cart_limit=max_x, wu=wu)
# Not used, since gamma = 0:
env.initConstraints(max_u=max_u, max_x=max_x)
dt = 0.02           # Hassan: make 0.02 // orig 0.12
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

# h_opt = []
# for j in range(len(coc_sol['state_traj_opt'])):
#     h_opt += [1/(max_x ** 2 - (coc_sol['state_traj_opt'][j, 0]) ** 2)]
#
# h_opt = np.array(h_opt)
# plt.plot(coc_sol['state_traj_opt'][:, 0], label='X')
# plt.plot(coc_sol['state_traj_opt'][:, 1], label=r'$\theta$')
# plt.plot(coc_sol['state_traj_opt'][:, 4], label='BaS')
# plt.plot(h_opt, label=r'$\frac{1}{h(x)}$', linestyle='dashed')
# plt.plot(np.zeros(151), 'k', linestyle='dotted')
# plt.plot(np.ones(151) * max_x, 'k', linestyle='dashdot')
# plt.plot(np.ones(151) * -max_x, 'k', linestyle='dashdot')
# plt.plot(np.ones(151) * np.pi, 'k', linestyle='dotted')
# plt.legend()
# plt.show()

# env.play_animation(pole_len=2, dt=dt, state_traj=coc_sol['state_traj_opt'], save_option=0, title='NLP Solver on BaS-'
#                                                                                                  'augmented system')

# TODO: Replace NLP with something else, like DDP?

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
max_iter = 10000             # original 3000
loss_barrier_trace, loss_trace = [], []
parameter_trace = np.empty((max_iter, init_parameter.size))
control_traj, state_traj = 0, 0
lr = 1e-1

# start safe motion planning
current_parameter = init_parameter
h = []
safety = []
safe_aux = []
J = []
for k in range(int(max_iter)):
    # one iteration of PDP
    loss_barrier, loss, dp, state_traj, control_traj, h_propagated, = planner.step(init_state=init_state, horizon=horizon,
                                                                        cart_limit=max_x,
                                                                        control_auxvar_value=current_parameter)
    # storage
    loss_barrier_trace += [loss_barrier]
    loss_trace += [loss]
    parameter_trace[k, :] = current_parameter

    # update
    current_parameter -= lr * dp

    # print
    if k % 100 == 0:
        print('Iter #:', k, 'Loss:', loss)

    # Check safety violations while learning    # TODO Implement the same for success
    for i in range(len(state_traj)):            # TODO [IMPORTANT] NOT h, x[4]!!!!
        h += [1 / (max_x ** 2 - state_traj[i, 0] ** 2)]
        if 1/h[i] < 1e-15:
            safe_aux += [1]
        else:
            safe_aux += [0]
    h = np.array(h)

    if sum(safe_aux) != 0:
        safety += [1]
    else:
        safety += [0]

    # Converge break
    if k != 0:
        converge = 10e-10
        delta_J = loss_trace[k-1] - loss_trace[k]
        if 0 < delta_J < converge:
            print(' ')
            print('Previous loss:', loss_trace[k-1], '; Current loss: ', loss_trace[k], '; Loss difference: ', delta_J)
            print('Total # of iterations: ', k)
            break
        else:
            if k == max_iter-1:
                print(' ')
                print('Convergence difference was greater than specified: ', converge)
                print('Code ended due to maximum # of iterations allowed: ', k+1, ' iterations')

print(' ')
print('There were ', sum(safety), ' iterations in which safety was violated')
print(' ')
print('BARRIER AT:', max_x, '; WEIGHTS: diag(Q) = [', wx, ',', wq, ',', wdx, ',', wdq, ',', wz, ']; R = ', wu, '; diag(S) = 20*Q' )


# # save the results
if True:
    params = {'cart_lim': max_x,
              'states_weights': [wx, wq, wdx, wdq, wz],
              'control_weight': wu,
              'dt': dt,
              'horizon': horizon,
              'convergence_limit': converge,
              'total_iterations': k,
              'learning_rate': lr,
              'init_parameter': init_parameter,
              'n_poly': n_poly,
              }

    save_data = {'loss_trace': loss_trace,
                 'solved_trajectory': state_traj,
                 'solved_controls': control_traj,
                 'barrier_function': h_propagated,
                 'safety': safety,
                 'coc_sol': coc_sol,
                 'params': params,
                 }
    np.save('./Results/BaS_Cartpole_Testing_lim_' + str(max_x) + '.npy', save_data)                      # .npy
    sio.savemat('./Results/BaS_Cartpole_Testing_lim_' + str(max_x) + '.mat', {'results': save_data})     # .mat

# Plot states over time
# times = np.linspace(0, dt*horizon-dt, horizon+1)
plot_cartpole.plotcartpole(env.xf, state_traj, control_traj, h, max_x)
plt.show()

# # Plot animation
# env.play_animation(pole_len=2, dt=dt, state_traj=state_traj, save_option=1,
#                    title='BaS-Learned Motion (barrier at ' + str(max_x) + ')')

# TODO: Add cart limits in the animation