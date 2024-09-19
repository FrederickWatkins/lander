import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

G = 6.6743 * 10**-11

def euler(t_array: list, dt: float, m: float, x, v) -> tuple[list, list]:
    x_list = []
    v_list = []
    for t in t_array:

        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -(G * m * x) / np.linalg.norm(x)**2
        x = x + dt * v
        v = v + dt * a
    return (x_list, v_list)

def verlet(t_array, dt, m, x, v) -> tuple[list, list]:
    x_list = [x - dt * v]
    v_list = [v]
    for t in t_array:

        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -(G * m * x) / np.linalg.norm(x)**2
        x = 2 * x - x_list[-2] + dt**2 * a
        v = (x - x_list[-1]) / dt
    return (x_list[1:], v_list[1:])

m = 6.42 * 10**23
dt = 0.01

t_max=2
t_array = np.arange(0, t_max, dt)
x = np.array([10000000, 0, 0])
v = np.array([0, 3000000, 0])
x_list, v_list = verlet(t_array, dt, m, x, v)
fig = plt.figure()
ax = fig.add_subplot(111)
X, Y, Z = zip(*x_list)
ax.plot(t_array, X)

t_max=20
t_array = np.arange(0, t_max, dt)
x = np.array([10000000, 0, 0])
v = np.array([0, 4000000, 0])
x_list, v_list = verlet(t_array, dt, m, x, v)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X, Y, Z = zip(*x_list)
ax.plot(X, Y, Z)

t_max=20
t_array = np.arange(0, t_max, dt)
x = np.array([10000000, 0, 0])
v = np.array([0, 6500000, 0])
x_list, v_list = verlet(t_array, dt, m, x, v)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X, Y, Z = zip(*x_list)
ax.plot(X, Y, Z)

t_max=20
t_array = np.arange(0, t_max, dt)
x = np.array([10000000, 0, 0])
v = np.array([0, 20000000, 0])
x_list, v_list = verlet(t_array, dt, m, x, v)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X, Y, Z = zip(*x_list)
ax.plot(X, Y, Z)

plt.show()