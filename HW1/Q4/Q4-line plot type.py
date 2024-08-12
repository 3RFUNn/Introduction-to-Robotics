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
    Plot the wheel speeds of the differential drive robot for different velocities using a line plot.
    """
    # Prepare the figure and axis
    fig, ax = plt.subplots()
    
    # Define labels for the x-axis
    x_labels = ['Set 1', 'Set 2']
    
    # Initialize lists to store the speeds
    left_speeds = []
    right_speeds = []
    
    # Calculate the wheel speeds for each velocity set
    for v, omega in velocities:
        left_speed, right_speed = calculate_wheel_speeds(v, omega, wheel_radius, robot_width)
        left_speeds.append(left_speed)
        right_speeds.append(right_speed)
    
    # Plot the left and right wheel speeds
    ax.plot(x_labels, left_speeds, marker='o', label='Left Wheel Speed', color='blue')
    ax.plot(x_labels, right_speeds, marker='o', label='Right Wheel Speed', color='red')
    
    # Add title and labels
    ax.set_title('Wheel Speeds for Different Velocities')
    ax.set_ylabel('Speed (rad/s)')
    
    # Show legend
    ax.legend()

    # Display the plot
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
