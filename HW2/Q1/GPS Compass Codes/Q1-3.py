import math
import matplotlib.pyplot as plt
from controller import Robot, Motor

def initialize_device(robot, device_name, time_step):
    """Initializes and returns a Webots device."""
    device = robot.getDevice(device_name)
    device.enable(time_step)
    return device

def set_wheel_velocity(wheel, velocity):
    """Sets the velocity of a wheel."""
    wheel.setVelocity(velocity)

def calculate_bearing(compass_values):
    """Calculates and returns the bearing from compass values."""
    bearing = (math.atan2(compass_values[1], compass_values[0]) - math.pi/2) / math.pi * 180
    return (-bearing + 360) % 360

def main():
    TIME_STEP = 64
    MAX_STEPS = 700

    # Initialize the robot and its devices
    robot = Robot()
    
    gps = initialize_device(robot, 'gps', TIME_STEP)
    compass = initialize_device(robot, 'compass', TIME_STEP)
    
    left_wheel = robot.getDevice('left wheel motor')
    right_wheel = robot.getDevice('right wheel motor')
    
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))

    # Lists for plotting
    theta_list, times_list, X, Y = [], [], [], []

    for _ in range(MAX_STEPS):
        current_time = robot.getTime()
        times_list.append(current_time)
        
        # Set wheel velocities based on time
        set_wheel_velocity(left_wheel, -math.cos(current_time))
        set_wheel_velocity(right_wheel, math.sin(current_time))

        # Record sensor data
        bearing = calculate_bearing(compass.getValues())
        theta_list.append(bearing)
        X.append(gps.getValues()[0])
        Y.append(gps.getValues()[1])
        
        # Print sensor data
        print(f"Time: {current_time}, GPS: {gps.getValues()}, Compass: {compass.getValues()}")

        # Check for termination condition
        if robot.step(TIME_STEP) == -1:
            break

    # Stop the robot
    set_wheel_velocity(left_wheel, 0)
    set_wheel_velocity(right_wheel, 0)

    # Plotting
    plt.figure(1)
    plt.plot(times_list, theta_list)
    plt.title('t-theta')
    plt.xlabel('t')
    plt.ylabel('theta')

    plt.figure(2)
    plt.plot(X, Y)
    plt.title('X-Y')
    plt.xlabel('X')
    plt.ylabel('Y')

    plt.show()

if __name__ == "__main__":
    main()