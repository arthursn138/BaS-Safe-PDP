import matplotlib.pyplot as plt


def plot3dvehicle(x0,xf, T, X, U):
    ax = plt.axes(projection='3d')
    ax.plot3D(X[9, :], X[10, :], X[11, :], 'gray')
    # draw sphere
    # u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    # x = r*np.cos(u)*np.sin(v) + o1
    # y = r*np.sin(u)*np.sin(v) + o2
    # z = r*np.cos(v) + o3
    # ax.plot_wireframe(x, y, z, color="r")
    ax.scatter(xf[9], xf[10], xf[11], color="g", s=100)
    ax.scatter(x0[9], x0[10], x0[11], color="r", s=100)

# plt.figure(2)
# plt.plot(t, X[:, 9])
# plt.figure(3)
# plt.plot(t, X[:, 10])
# plt.figure(4)
# plt.plot(t, X[:, 11])
# plt.xlabel('Time (s)')
# plt.ylabel('States')
# plt.title("States vs Time", fontsize='large')

# plt.figure(5)
# plt.plot(t, X[:, -1:])
# plt.xlabel('Time (s)')
# plt.ylabel('Barrier State (z)')
# plt.title("BaS vs Time", fontsize='large')