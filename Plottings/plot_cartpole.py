import matplotlib.pyplot as plt
import numpy as np


def plotcartpole(x0, xf, T, X, U, h, cart_limit):
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
    # ax[1, 1].title.set_text(r'$\dot{\theta}$')

    inv_h = []
    y = []
    for i in range(len(X[1, :])):
        inv_h += [1 / (cart_limit ** 2 - X[0, i] ** 2)]
        y += [inv_h[i] - X[4, i]]

    fig_bas, ax_bas = plt.subplots()
    ax_bas.plot(T, X[4, :], label='BaS (X(4))')
    ax_bas.plot(T, inv_h, 'k--', label=r'$\frac{1}{h(x)}$')
    ax_bas.plot(T, np.zeros(T.shape[0]))
    ax_bas.set_title('BaS vs reference')
    ax_bas.plot(T, y, label=r'BaS error ($\frac{1}{h(x)} - $X(4))')
    ax_bas.legend()

    # fig_check, ax_check = plt.subplots()
    # ax_check.plot(T, X[0, :], label=r'$x$')
    # ax_check.plot(T, X[1, :], label=r'$\theta$')
    # ax_check.plot(T, X[2, :], label=r'$\dot{x}$')
    # ax_check.plot(T, X[3, :], label=r'$\dot{\theta}$')
    # ax_check.plot(T, X[4, :], label='BaS')
    # ax_check.plot(T, np.zeros(T.shape[0]), 'k--')
    # # ax_check.plot(T, np.ones(T.shape[0]) * cart_limit, 'k--')
    # # ax_check.plot(T, np.ones(T.shape[0]) * -cart_limit, 'k--')
    # # ax_check.plot(T, np.ones(T.shape[0]) * np.pi, 'k--')
    # ax_check.set_title('ALL THE STATES')
    # ax_check.legend()

    # fig_u, ax_u = plt.subplots()
    # ax_u.plot(T[0:-1], U[0, :])
    # ax_u.set_title('Control effort over time')

    return fig, fig_bas
