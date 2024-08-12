import sys
import math
import numpy as np  # Added this line to import numpy
import matplotlib.pyplot as plt
from controller import Robot

webots_path = 'D:\\Apps\\Webots\\lib\\controller\\python'  # Corrected the path with double backslashes
sys.path.append(webots_path)

# Constants
r = 0.5  # radius of wheel in meters
l = 1.0  # distance between wheels in meters
P = 1.0  # constant speed in m/s

# Discretize time
dt = 0.01  # finer time step for better resolution
times = np.arange(0, 10, dt)  # simulate for 10 seconds

def plot_robot_path(x_start, y_start, theta_start):
    x, y, theta = [x_start], [y_start], [theta_start]
    
    for t in times:
        if 0 <= t < 2:
            # Only right wheel moving, robot describes a counterclockwise arc
            w = P / r  # Angular speed
            theta_dot = w / l
            theta_new = theta[-1] + theta_dot * dt
            
            # Use the kinematics of differential drive robots to compute new x and y
            x_new = x[-1] + (r/2) * (P + 0) * np.cos(theta_new) * dt
            y_new = y[-1] + (r/2) * (P + 0) * np.sin(theta_new) * dt
        else:
            # Both wheels moving forward
            dx = P * dt
            dy = 0
            theta_new = theta[-1]
            x_new = x[-1] + dx * np.cos(theta_new)
            y_new = y[-1] + dx * np.sin(theta_new)
            
        x.append(x_new)
        y.append(y_new)
        theta.append(theta_new)

    return x, y

# Create 2 random starting orientations (since the starting point is the same in the image)
thetas = np.random.uniform(0, 2 * np.pi, 5)

plt.figure(figsize=(10,10))
for theta_start in thetas:
    x_start, y_start = 0, 0  # All trajectories start from (0,0)
    x, y = plot_robot_path(x_start, y_start, theta_start)
    plt.plot(y, x)

plt.xlabel('X position [m]')
plt.ylabel('Y position [m]')
plt.title('Robot trajectory')
plt.grid(True)
plt.axis('equal')
plt.show()
