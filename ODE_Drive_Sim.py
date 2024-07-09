import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


def diff_drive(state, t, v, omega):
    x, y, theta = state
    dxdt = v * np.cos(theta)
    dydt = v * np.sin(theta)
    dthetadt = omega
    return [dxdt, dydt, dthetadt]


# Initial state [x, y, θ]
initial_state = [0, 0, 0]
v = 1  # Linear velocity
omega = 0.3  # Angular velocity

# Time points
t = np.linspace(0, 10, 200)  # Simulate for 10 seconds

# Integrate the differential equations
result = odeint(diff_drive, initial_state, t, args=(v, omega)) # args is just passing extra arguments

# Extract the results
x = result[:, 0]
y = result[:, 1]
theta = result[:, 2]

# Plot the trajectory
plt.plot(x, y, label='Trajectory')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Robot Trajectory')
plt.legend()
plt.grid()
plt.axis('equal')
plt.show()
