import matplotlib.pyplot as plt
import numpy as np


def plotcartpole(x0, xf, T, X, U, cart_limit):
    T = T.T
    # fig, ax = plt.subplots(2, 2, sharey=True)
    fig, ax = plt.subplots(2, 2)
    ax[0, 0].plot(T, X[0, :])
    ax[0, 0].plot(T, np.ones((T.shape[0], 1))*cart_limit)
    ax[0, 0].plot(T, np.ones((T.shape[0], 1))*-cart_limit)
    ax[0, 0].title.set_text(r'$x$')

    ax[0, 1].plot(T, X[1, :])
    ax[0, 1].title.set_text(r'$\theta$')
    ax[0, 1].plot(T, np.ones((T.shape[0], 1))*xf[1])

    ax[1, 0].plot(T, X[2, :])
    ax[1, 0].title.set_text(r'$\dot{x}$')

    ax[1, 1].plot(T, X[3, :])
    ax[1, 1].title.set_text(r'$\dot{\theta}$')

    fig_u, ax_u = plt.subplots()
    ax_u.plot(T, X[4, :])
    ax_u.set_title('BaS')

    # fig_u, ax_u = plt.subplots()
    # ax_u.plot(T[0:-1], U[0, :])
    # ax_u.set_title('Control effort over time')

    return fig, fig_u
