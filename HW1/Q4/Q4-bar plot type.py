import matplotlib.pyplot as plt

def calculate_wheel_speeds(v, omega, wheel_radius, robot_width):
    """
    Calculate the speeds of the left and right wheels of a differential drive robot.
    """
    v_left = (v - (omega * robot_width / 2)) / wheel_radius
    v_right = (v + (omega * robot_width / 2)) / wheel_radius
    return v_left, v_right

def plot_wheel_speeds(velocities, wheel_radius, robot_width):
    """
    Plot the wheel speeds of the differential drive robot for different velocities.
    """
    fig, ax = plt.subplots()
    
    for i, (v, omega) in enumerate(velocities):
        # Calculate wheel speeds
        left_speed, right_speed = calculate_wheel_speeds(v, omega, wheel_radius, robot_width)
        
        # Plotting the wheel speeds
        ax.bar(f'Set {i+1}\nLeft Wheel', left_speed, color='blue')
        ax.bar(f'Set {i+1}\nRight Wheel', right_speed, color='red')
    
    ax.set_title('Wheel Speeds for Different Velocities')
    ax.set_ylabel('Speed (rad/s)')
    plt.show()

# Parameters
wheel_radius = 0.1  # Radius of the wheels in meters
robot_width = 0.5   # Distance between the wheels in meters

# Velocities to test
velocities = [
    (0.03, 0.1),  # v = 0.03 m/s, omega = 0.1 rad/s
    (0, 0.5)      # v = 0 m/s, omega = 0.5 rad/s
]

plot_wheel_speeds(velocities, wheel_radius, robot_width)
