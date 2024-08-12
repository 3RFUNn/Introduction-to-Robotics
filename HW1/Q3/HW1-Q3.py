import sys
import math
from controller import Robot

webots_path = 'D:\Apps\Webots\lib\controller\python'
sys.path.append(webots_path)

def compute_new_position(x, y, theta, l, phi1, phi2, t):
    # Assuming phi1 and phi2 are linear velocities
    # directly or they include the effect of wheel diameter implicitly
    
    # Compute linear velocity V (assuming it's the mean of the velocities of two wheels)
    V = (phi1 + phi2) / 2

    # Compute angular velocity omega (assuming l is half the distance between wheels)
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

# Compute new positions for each set
x1, y1, theta1 = compute_new_position(x, y, theta, l, c1[0], c1[1], c1[2])
x2, y2, theta2 = compute_new_position(x, y, theta, l, c2[0], c2[1], c2[2])
x3, y3, theta3 = compute_new_position(x, y, theta, l, c3[0], c3[1], c3[2])

print(f"For c1: x_n = {x1} m, y_n = {y1} m, theta_n = {theta1} rad")
print(f"For c2: x_n = {x2} m, y_n = {y2} m, theta_n = {theta2} rad")
print(f"For c3: x_n = {x3} m, y_n = {y3} m, theta_n = {theta3} rad")
