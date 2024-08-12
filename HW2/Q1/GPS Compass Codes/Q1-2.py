import math
from controller import Robot, Motor
import matplotlib.pyplot as plt

TIME_STEP = 64
MAX_TIMES = 80

robot = Robot()

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

leftWheel = robot.getDevice('left wheel motor')
rightWheel = robot.getDevice('right wheel motor')

leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))


leftWheel.setVelocity(-1)
rightWheel.setVelocity(1)

current_time = 0
theta_list = []
X = []
Y = []


while robot.step(TIME_STEP) != -1:
    if current_time == MAX_TIMES:
        break
    
    bearing = (math.atan2(compass.getValues()[1], compass.getValues()[0]) - 1.5708) / math.pi * 180
    if bearing < 0:
        bearing += 360
    bearing = -bearing + 360
    
    theta_list.append(bearing)
    X.append(gps.getValues()[0])
    Y.append(gps.getValues()[1])

    print("Time: ", current_time)
    print("GPS: ", gps.getValues()[0], gps.getValues()[1])
    print("Compass: ", compass.getValues()[0], compass.getValues()[1])
    print("#########################################################")

    current_time += 1


leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

t = list(range(current_time))

plt.figure(1)
plt.plot(t, theta_list)
plt.title('t-teta')
plt.xlabel('t')
plt.ylabel('teta')

plt.figure(2)
plt.xlim(-5, 5)
plt.ylim(-5, 5)
plt.plot(X, Y)
plt.title('X-Y')
plt.xlabel('X')
plt.ylabel('Y')

plt.show()