import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches

def plot2dvehicle(x0,xf, T, X, U, class_constraint, ax=None):
    if ax is None:
        fig1, ax = plt.subplots(1, subplot_kw={'aspect': 'equal'})
    ax.plot(X[0, :], X[1, :])
    ax.plot(x0[0], x0[1], 'ro')
    # ax.plot(X[0, 0], X[1, 0], 'bo')
    # ax.plot(X[0, -1], X[1,-1], 'ro')
    ax.plot(xf[0], xf[1], 'go')
    # ax.set_xlim([-5, 5])
    # ax.set_ylim([-5, 5])

    for i in range(len(class_constraint)):
        obstacles_info = class_constraint[i].obstacles()
        ox = obstacles_info[:, 0]
        oy = obstacles_info[:, 1]
        r = obstacles_info[:, 2]
        for ii in range(obstacles_info.shape[0]):
            ax.add_patch(plt.Circle((ox[ii], oy[ii]), r[ii], color='k', alpha=0.75))

    # rect1 = matplotlib.patches.Rectangle((-1, 2), 2, 4, color ='k')
    # rect2 = matplotlib.patches.Rectangle((-1, -6), 2, 4, color ='k')
    # rect1 = mpatches.FancyBboxPatch((-1, 2.2), 2, 4, boxstyle=mpatches.BoxStyle("Round", pad=0.02))
    # rect2 = mpatches.FancyBboxPatch((-1, -6), 2, 3.8, boxstyle=mpatches.BoxStyle("Round", pad=0.02))
    # ax.patches.append(fancybox)
    # ax.add_patch(rect1)
    # ax.add_patch(rect2)

    # fig2, au = plt.subplots(2, sharex=True)
    # au[0].plot(T[0:-1], U[0, :])
    # au[1].plot(T[0:-1], U[1, :])
    # au[0].set_xlim([0, T[-1]])
    # au[1].set_xlim([0, T[-1]])
    # au[0].set_ylim([-10, 10])
    # au[1].set_ylim([-6, 6])
    return

def plot2dvehicle_tracking(x0,xf, T, X, U, class_constraint, X_track):
    fig1, ax = plt.subplots(1, subplot_kw={'aspect': 'equal'})
    ax.plot(X_track[0, :], X_track[1, :], color='red', linestyle='dashed', label='Unconstrained Planned Path')
    ax.plot(X[0, :], X[1, :], label='DBaS-DDP')
    ax.plot(x0[0], x0[1], 'ro')
    # ax.plot(X[0, 0], X[1, 0], 'bo')
    # ax.plot(X[0, -1], X[1,-1], 'ro')
    ax.plot(xf[0], xf[1], 'go')
    # ax.set_xlim([-5, 5])
    # ax.set_ylim([-5, 5])
    plt.legend()
    for i in range(len(class_constraint)):
        obstacles_info = class_constraint[i].obstacles()
        ox = obstacles_info[:, 0]
        oy = obstacles_info[:, 1]
        r = obstacles_info[:, 2]
        for ii in range(obstacles_info.shape[0]):
            ax.add_patch(plt.Circle((ox[ii], oy[ii]), r[ii], color='k', alpha=0.75))

    # fig2, au = plt.subplots(2, sharex=True)
    # au[0].plot(T[0:-1], U[0, :])
    # au[1].plot(T[0:-1], U[1, :])
    # au[0].set_xlim([0, T[-1]])
    # au[1].set_xlim([0, T[-1]])
    # au[0].set_ylim([-10, 10])
    # au[1].set_ylim([-6, 6])
    return fig1, fig2


# def plot2dvehicle(n_agents, n1, delta, agent_radius, x0, xf, T, X, U, obstacles_info):
#
#     if (n_agents > 1):
#         fig, ax = plt.subplots(2, subplot_kw={'aspect': 'equal'})
#         for ii in range(n_agents):
#             ax[0].plot(X[ii * n1, :], X[ii * n1 + 1, :])
#             ax[0].plot(x0[ii * n1], x0[ii * n1 + 1], 'bo')
#             ax[0].plot(X[ii * n1, 0], X[ii * n1 + 1, 0], 'bo')
#             ax[0].plot(X[ii * n1, -1], X[ii * n1 + 1, -1], 'ro')
#             ax[0].plot(xf[ii * n1], xf[ii * n1 + 1], 'go')
#         ax[0].set_xlim([-agent_radius-1, agent_radius+1])
#         ax[0].set_ylim([-agent_radius-1, agent_radius+1])
#         dists = []
#         for ii in range(n_agents):
#             for jj in range(ii + 1, n_agents):
#                 dists_ij = np.sqrt(np.sum((X[ii * n1:ii * n1 + 2, :]-X[jj * n1:jj * n1 + 2, :]) ** 2, 0))
#                 ax[1].plot(T, dists_ij)
#                 print("Collided?:", (np.array(dists_ij) < delta).any())
#         ax[1].plot(T, np.ones(len(T)) * delta, linestyle='dashed')
#
#     else:
#         fig, ax = plt.subplots(1, subplot_kw={'aspect': 'equal'})
#         for ii in range(n_agents):
#             ax.plot(X[ii * n1, :], X[ii * n1 + 1, :])
#             ax.plot(x0[ii * n1], x0[ii * n1 + 1], 'bo')
#             ax.plot(X[ii * n1, 0], X[ii * n1 + 1, 0], 'bo')
#             ax.plot(X[ii * n1, -1], X[ii * n1 + 1, -1], 'ro')
#             ax.plot(xf[ii * n1], xf[ii * n1 + 1], 'go')
#         ax.set_xlim([-5, 5])
#         ax.set_ylim([-5, 5])
#
#     if obstacles_info != None:
#         ox = obstacles_info[:, 0]
#         oy = obstacles_info[:, 1]
#         r = obstacles_info[:, 2]
#
#     # for ii in range(obstacles_info.shape[0]):
#     # ax.add_patch(plt.Circle((ox[ii], oy[ii]), r[ii], color='r', alpha=0.5))
