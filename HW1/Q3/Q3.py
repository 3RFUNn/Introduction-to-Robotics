import sys
import math
import matplotlib.pyplot as plt
from controller import Robot

webots_path = 'D:\Apps\Webots\lib\controller\python'
sys.path.append(webots_path)

def compute_new_position(x, y, theta, l, phi1, phi2, t):
    # Compute linear velocity V
    V = (phi1 + phi2) / 2

    # Compute angular velocity omega
    omega = (phi1 - phi2) / l

    # Compute new position
    x_n = x + V * math.cos(theta) * t
    y_n = y + V * math.sin(theta) * t
    theta_n = theta + omega * t
    
    return x_n, y_n, theta_n

# Initial position
x, y, theta = 1.5, 2, math.pi / 2

# Distance between the center of the robot and each wheel
l = 0.5  # This could be interpreted as the robot's radius

# Three sets of rotational velocities (assuming these are linear velocities)
c1 = (0.3, 0.3, 3)
c2 = (0.1, -0.1, 1)
c3 = (0.2, 0, 2)

# Prepare the plot
plt.figure(figsize=(8, 6))
plt.title('Robot Movement Plot')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.grid(True)

# Plot initial position
plt.plot(x, y, 'ko', label='Initial Position')

# Compute and plot new positions for each set
for i, c in enumerate([c1, c2, c3], start=1):
    xn, yn, thetan = compute_new_position(x, y, theta, l, c[0], c[1], c[2])
    plt.plot(xn, yn, 'o', label=f'Position after c{i}')
    plt.text(xn, yn, f' c{i}')

    # Optionally, draw an arrow showing the movement
    plt.arrow(x, y, xn-x, yn-y, head_width=0.05, head_length=0.1, fc='lightblue', ec='black')

# Show the legend
plt.legend()

# Show the plot
plt.show()
