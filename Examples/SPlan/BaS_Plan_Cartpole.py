import numpy as np
from SafePDP import SafePDP
from NascimEnv import NascimEnv
from Plottings import plot_cartpole
from casadi import *
import scipy.io as sio
import matplotlib.pyplot as plt

# --------------------------- Load environment ----------------------------------------
env = NascimEnv.BaS_CartPole()
mc, mp, l = 0.5, 0.5, 1
max_x = 1       # CONSTRAINT / OBSTACLE FOR BAS
max_u = 10000000       # CONTROL LIMIT [original: 4]
env.initDyn(mc=mc, mp=mp, l=l, cart_limit=max_x, gamma=0)
# wx, wq, wdx, wdq, wz, wu = 0.3, 1.5, 0.1, 0.1, 0.1, 0.1     # Worked beautifully before
wx, wq, wdx, wdq, wz, wu = 5, 40, 0.1, 0.1, 70, 0.015
# wx, wq, wdx, wdq, wz, wu = 0.1, 0.1, 0.1, 0.1, 0.1, 0.1     # Original values
env.initCost(wx=wx, wq=wq, wdx=wdx, wdq=wdq, wz=wz, cart_limit=max_x, wu=wu)
env.initConstraints(max_u=max_u, max_x=max_x)                 # Not used, since gamma = 0
dt = 0.05           # orig 0.12
horizon = 60       # orig 25
init_state = [0, 0, 0, 0, 1/max_x**2]
dyn = env.X + dt * env.f

# --------------------------- Create Safe PDP SPlan object ----------------------------------------
planner = SafePDP.CSysOPT()
planner.setStateVariable(env.X)
planner.setControlVariable(env.U)
planner.setDyn(dyn)
planner.setPathCost(env.path_cost)
planner.setFinalCost(env.final_cost)
planner.setPathInequCstr(env.path_inequ)
gamma = 0           # THIS IS VERY IMPORTANT: gamma=0 means that the cost penalty is cancelled (need gamma=0 for BaS)
# # # NOTE: The above gamma is for the PENALTY method; not for the BaS equation --- if !=0, then penalty will be active
planner.convert2BarrierOC(gamma=gamma)

# ------------------ Create COC object and propagate (for result comparison and debug) -------------------------------
# Create object
coc = SafePDP.COCsys()
coc.setStateVariable(planner.state)
coc.setControlVariable(planner.control)
coc.setDyn(planner.dyn)
coc.setPathCost(planner.path_cost)
coc.setFinalCost(planner.final_cost)
coc.setPathInequCstr(planner.path_inequ_cstr)
# Solve Classic Optimal Control problem
coc_sol = coc.ocSolver(init_state=init_state, horizon=horizon)
print('constrained cost', coc_sol['cost'])

# # Check propagated and expected BaS offset
# h_opt = []
# for j in range(len(coc_sol['state_traj_opt'])):
#     h_opt += [1/(max_x ** 2 - (coc_sol['state_traj_opt'][j, 0]) ** 2)]
#
# h_opt = np.array(h_opt)

# # Plot Classical optimal control solution
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

# # Play animation
# env.play_animation(pole_len=2, dt=dt, state_traj=coc_sol['state_traj_opt'], save_option=0, title='NLP Solver on BaS-'
#                                                                                                  'augmented system')

# TODO: Replace NLP with something else, like DDP?

# --------------------------- Solve motion planning through learning with BaS ----------------------------------------
# Set the policy as polynomial
n_poly = 10
planner.setPolyTraj(horizon=horizon, n_poly=n_poly)

# Set the initial condition
nn_seed = None
init_parameter = np.zeros(planner.n_control_auxvar)  # all zeros initial condition
# nn_seed = 200 # e.g. 200, 300, 400, 500
# init_parameter = 0.1*np.random.randn(planner.n_control_auxvar)  # random initial condition

# Planning parameter setting
max_iter = 5000             # original 3000
loss_barrier_trace, loss_trace = [], []
parameter_trace = np.empty((max_iter, init_parameter.size))
control_traj, state_traj = 0, 0
lr = 1e-1

# Initialize auxiliary vectors
intermediate_traj = []
intermediate_control = []
h = []
safety = []
safe_aux = []
J = []

# Start safe motion planning
current_parameter = init_parameter

for k in range(int(max_iter)):
    # One iteration of PDP
    loss_barrier, loss, dp, state_traj, control_traj, h_propagated, = planner.step(init_state=init_state, horizon=horizon,
                                                                        cart_limit=max_x,
                                                                        control_auxvar_value=current_parameter)
    # Storage
    loss_barrier_trace += [loss_barrier]
    loss_trace += [loss]
    parameter_trace[k, :] = current_parameter

    # Update (gradient descent)
    current_parameter -= lr * dp

    # Print and save intermediate solutions (for training comparisons)
    if k % 100 == 0:
        intermediate_traj += [state_traj]
        intermediate_control += [control_traj]
        print('Iter #:', k, 'Loss:', loss)

    # Check safety violations while learning    # TODO Implement the same for success
    for i in range(len(state_traj)):            # TODO [IMPORTANT] NOT h, x[4]!!!!
        h += [1 / (max_x ** 2 - state_traj[i, 0] ** 2)]
        if 1/h[i] < 1e-10:
            safe_aux += [1]
        else:
            safe_aux += [0]
    h = np.array(h)

    if sum(safe_aux) != 0:
        safety += [1]
    else:
        safety += [0]

    # Convergence break
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

# Print report
print(' ')
print('There were ', sum(safety), ' iterations in which safety was violated')
print(' ')
print('BARRIER AT:', max_x, '; WEIGHTS: diag(Q) = [', wx, ',', wq, ',', wdx, ',', wdq, ',', wz, ']; R = ', wu, '; diag(S) = 20*Q' )


# # Save the results
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
                 'intermediate_trajectory': intermediate_traj,
                 'intermediate_controls': intermediate_control,
                 'barrier_function': h_propagated,
                 'safety': safety,
                 'coc_sol': coc_sol,
                 'params': params,
                 }
    # np.save('./Results/BaS_Cartpole_dt_' + str(dt) + '.npy', save_data)                      # .npy
    # sio.savemat('./Results/BaS_Cartpole_dt_' + str(dt) + '.mat', {'results': save_data})     # .mat
    np.save('./Results/BaS_tuning_dt_' + str(dt) + 'lim_' + str(max_x) + '.npy', save_data)  # .npy
    sio.savemat('./Results/BaS_tuning_dt_' + str(dt) + 'lim_' + str(max_x) + '.mat', {'results': save_data})  # .mat

# Plot states over time
plot_cartpole.plotcartpole(env.xf, state_traj, control_traj, h, max_x)
plt.show()

# # Plot animation
# env.play_animation(pole_len=2, dt=dt, state_traj=state_traj, save_option=1,
#                    title='BaS-Learned Motion (barrier at ' + str(max_x) + ')')

# TODO: Add cart limits in the animation