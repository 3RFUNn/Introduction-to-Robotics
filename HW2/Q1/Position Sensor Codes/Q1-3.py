import math
from math import cos, sin, pi
from controller import Robot, Motor, PositionSensor
import matplotlib.pyplot as plt

# Constants
TIME_STEP = 64
WHEEL_RADIUS = 0.02
WHEELBASE = 0.05
MAX_ANGLE = 2 * pi
MAX_STEPS = 700
RAD_TO_DEG = 180 / pi

# Kinematic function
def calculate_kinematics(phi_dot_left, phi_dot_right):
    velocity_left = phi_dot_left * WHEEL_RADIUS
    velocity_right = phi_dot_right * WHEEL_RADIUS
    linear_velocity = (velocity_left + velocity_right) / 2.0
    angular_velocity = (velocity_left - velocity_right) / WHEELBASE
    return linear_velocity, angular_velocity

# Initialize robot and devices
robot = Robot()
left_wheel = robot.getDevice('left wheel motor')
right_wheel = robot.getDevice('right wheel motor')
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))

left_sensor = robot.getDevice('left wheel sensor')
left_sensor.enable(TIME_STEP)
right_sensor = robot.getDevice('right wheel sensor')
right_sensor.enable(TIME_STEP)

# Lists for plotting
angle_list = []
time_list = []
x_positions = []
y_positions = []

# Current state
current_x = 0
current_y = 0
current_theta = 0
previous_left_sensor = 0
previous_right_sensor = 0

# Main loop
for _ in range(MAX_STEPS):
    if robot.step(TIME_STEP) == -1:
        break

    current_time = robot.getTime()
    time_list.append(current_time)

    left_wheel.setVelocity(-cos(current_time))
    right_wheel.setVelocity(sin(current_time))

    sensor_value_left = left_sensor.getValue()
    sensor_value_right = right_sensor.getValue()

    delta_right = sensor_value_right - previous_right_sensor
    delta_left = sensor_value_left - previous_left_sensor

    delta_x, delta_theta = calculate_kinematics(delta_right, delta_left)
    current_theta += delta_theta
    delta_x_i = delta_x * cos(current_theta)
    delta_y_i = delta_x * sin(current_theta)

    current_x += delta_x_i
    current_y += delta_y_i

    # Normalize the angle
    current_theta = (current_theta + MAX_ANGLE) % MAX_ANGLE

    previous_left_sensor = sensor_value_left
    previous_right_sensor = sensor_value_right

    angle_list.append(current_theta * RAD_TO_DEG)
    x_positions.append(current_x)
    y_positions.append(current_y)
    
    print(f"Time: {current_time}, Left Sensor: {sensor_value_left}, Right Sensor: {sensor_value_right}")
    print("#########################################################")

# Stop the wheels
left_wheel.setVelocity(0)
right_wheel.setVelocity(0)

# Plotting
plt.figure(1)
plt.plot(time_list, angle_list)
plt.title('Time vs Theta')
plt.xlabel('Time')
plt.ylabel('Theta (degrees)')

plt.figure(2)
plt.plot(x_positions, y_positions)
plt.title('X-Y Trajectory')
plt.xlabel('X')
plt.ylabel('Y')

plt.show()
