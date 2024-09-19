# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

def euler(t_array, dt, m, k, x, v) -> tuple[list, list]:
    x_list = []
    v_list = []
    for t in t_array:

        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -k * x / m
        x = x + dt * v
        v = v + dt * a
    return (x_list, v_list)


def verlet(t_array, dt, m, k, x, v) -> tuple[list, list]:
    x_list = [x - dt * v]
    v_list = [v]
    for t in t_array:

        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -k * x / m
        x = 2 * x - x_list[-2] + dt**2 * a
        v = (x - x_list[-1]) / dt
    return (x_list[1:], v_list[1:])

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 1000
dt = 1.2
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list_e = []
v_list_e = []
x_list_v = []
v_list_v = []

# Euler integration
x_list_e, v_list_e = euler(t_array, dt, m, k, x, v)
x_list_v, v_list_v = verlet(t_array, dt, m, k, x, v)


# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array_e = np.array(x_list_e)
v_array_e = np.array(v_list_e)
x_array_v = np.array(x_list_v)
v_array_v = np.array(v_list_v)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
#plt.plot(t_array, x_array_e, label='Euler')
plt.plot(t_array, x_array_v, label='Verlet')
plt.plot(t_array, np.sin(t_array), label='Analytical')
plt.legend()
plt.figure(2)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
#plt.plot(t_array, v_array_e, label='Euler')
plt.plot(t_array, v_array_v, label='Verlet')
plt.plot(t_array, np.cos(t_array), label='Analytical')
plt.legend()
plt.show()
