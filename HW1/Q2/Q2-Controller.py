import sys
import math
import matplotlib.pyplot as plt
from controller import Robot

webots_path = 'D:\Apps\Webots\lib\controller\python'
sys.path.append(webots_path)

def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[0], compass_value[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    
    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading

robot = Robot()

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

sampling_period = 1
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(sampling_period)
compass.enable(sampling_period)

robot.step(1000)
initial_gps_value = gps.getValues()
initial_compass_value = compass.getValues()

destination_coordinate = [-0.209, -0.249]
direction_vector = [destination_coordinate[0] - initial_gps_value[0], 
                    destination_coordinate[1] - initial_gps_value[1]]
degree_to_target = math.atan2(direction_vector[0], direction_vector[1]) * 180 / math.pi
degree_to_target = round(degree_to_target) % 360
initial_degree = round(get_robot_heading(initial_compass_value))

print(f'Target Angle: {degree_to_target}, Robot Heading: {initial_degree}')

# Data collection for plotting
x_coords = []
y_coords = []
theta_minus_t = []

start_time = robot.getTime() # Get the initial time

# Update velocity based on the provided instructions
while True:
    t = robot.getTime()
    current_time = robot.getTime() # Get the current time
    elapsed_time = current_time - start_time # Calculate elapsed time since the loop started
    
    # Velocity 1
    # left_motor.setVelocity(1)
    # right_motor.setVelocity(-1)
    
    # Uncomment the desired velocity logic
    # For Velocity 2
    # left_motor.setVelocity(1)
    # right_motor.setVelocity(1)
    
    # For Velocity 3
    left_motor.setVelocity(math.sin(t))
    right_motor.setVelocity(-math.cos(t))
    
    robot.step()

    current_coordinate = gps.getValues()
    current_heading = get_robot_heading(compass.getValues())
    
    # Collect data for plotting
    x_coords.append(current_coordinate[0])
    y_coords.append(current_coordinate[1])
    theta_minus_t.append(current_heading - t)
    
    reach_condition = elapsed_time >= 10
    
    if reach_condition:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        robot.step(1000)  
        print('Finished.')
        break

# Plotting the graphs
plt.figure()
plt.plot(x_coords, y_coords)
plt.title('Movement in (x-y)')
plt.xlabel('x-coordinate')
plt.ylabel('y-coordinate')
plt.show()

plt.figure()
plt.plot(theta_minus_t)
plt.title('Robot Head Angle (θ - t)')
plt.xlabel('Time Steps')
plt.ylabel('θ - t')
plt.show()
