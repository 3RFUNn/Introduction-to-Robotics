import sys
import math
import matplotlib.pyplot as plt
from controller import Robot

webots_path = 'D:\\Apps\\Webots\\lib\\controller\\python'
sys.path.append(webots_path)

SPEED = 4
TIME_STEP = 32

# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# get a handle to the touch sensor and activate it
force = robot.getDevice('force')
force.enable(TIME_STEP)

# Initialize variables for the timer and data collection
start_time = robot.getTime() # Get the initial time
force_data = []
elapsed_time = []

while robot.step(TIME_STEP) != -1:
    t = robot.getTime()
    current_time = robot.getTime() # Get the current time
    elapsed = current_time - start_time
    elapsed_time.append(elapsed)

    # Collect force data
    force_value = force.getValue()
    force_data.append(force_value)

    # Normal robot operation
    left_speed = SPEED
    right_speed = SPEED

    # set the motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    if force_value >= 1:
        SPEED = -1

    if elapsed >= 6:
        # Calculate the maximum force value
        max_force = max(force_data)


        plt.plot(elapsed_time, force_data)
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.title('Force Sensor Data Over Time')

        # Show the plot
        plt.show()
        
        # Print the maximum force value
        print(f"The maximum of power is: {max_force}")

        break