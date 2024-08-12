import math
from controller import Robot, Motor, PositionSensor
import matplotlib.pyplot as plt

TIME_STEP = 64
MAX_STEPS = 80
# in meter
wheel_radius = 0.02
wheelbase = 0.05

def kinematic(phi_dot_1, phi_dot_2):
    v1 = phi_dot_1 * wheel_radius
    v2 = phi_dot_2 * wheel_radius
    x_dot_r = (v1 + v2) / 2.0
    theta_dot = (v1 - v2) / wheelbase
    return x_dot_r, theta_dot

robot = Robot()


leftWheel = robot.getDevice('left wheel motor')
rightWheel = robot.getDevice('right wheel motor')

leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

ps_left = robot.getDevice('left wheel sensor')
ps_left.enable(TIME_STEP)

ps_right = robot.getDevice('right wheel sensor')
ps_right.enable(TIME_STEP)


# leftWheel.setVelocity(-1)
# rightWheel.setVelocity(1)

current_time = 0
theta = []
X = []
Y = []

left_sensor_value = 0
right_sensor_value = 0
curr_x = 0
curr_y = 0
curr_theta = 0

while robot.step(TIME_STEP) != -1:
    if current_time == MAX_STEPS:
        break
    
    left_speed = math.cos(math.radians(current_time))
    leftWheel.setVelocity(-left_speed)

    right_speed = math.sin(math.radians(current_time))
    rightWheel.setVelocity(right_speed)
    
    theta.append(curr_theta * (180/math.pi))
    X.append(curr_x)
    Y.append(curr_y)

    new_left_sensor_value = ps_left.getValue()
    new_right_sensor_value = ps_right.getValue()

    # Calculate change in sensor values
    delta_left = new_left_sensor_value - left_sensor_value
    delta_right = new_right_sensor_value - right_sensor_value

    # Update sensor values for next iteration
    left_sensor_value = new_left_sensor_value
    right_sensor_value = new_right_sensor_value

    dx, dtheta = kinematic(delta_right, delta_left)
    dx_i = dx * math.cos(curr_theta)
    dy_i = dx * math.sin(curr_theta)

    curr_x = curr_x + dx_i
    curr_y = curr_y + dy_i
    curr_theta = curr_theta + dtheta
    
    print("Time: ", current_time)
    print("Position Sensor Left Value: ", ps_left.getValue())
    print("Position Sensor Right Value: ", ps_right.getValue())
    print("#########################################################")

    current_time += 1


leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

t = list(range(current_time))

plt.figure(1)
plt.plot(t, theta)
plt.title('t-teta')
plt.xlabel('t')
plt.ylabel('teta')

plt.figure(2)
plt.xlim(-0.010, -0.002)
plt.ylim(-0.014, 0)
plt.plot(X, Y)
plt.title('X-Y')
plt.xlabel('X')
plt.ylabel('Y')

plt.show()