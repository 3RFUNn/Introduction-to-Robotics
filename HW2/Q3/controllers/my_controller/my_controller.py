from controller import Robot


robot = Robot()
timestep = int(robot.getBasicTimeStep())
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
    
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

sensor = robot.getDevice('ps0')
sensor.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(10)


with open('distance_data.txt', 'w') as file:
    for i in range(360):
        
        distance = sensor.getValue()
        compass_value = compass.getValues()
        print(distance)
        print(compass_value)
        print('-'*10)
        left_motor.setVelocity(0.1)
        right_motor.setVelocity(-0.1)     
        for _ in range(10):
            robot.step(10)
        file.write(f"{compass_value[0]}, {compass_value[1]}, {distance}\n")


left_motor.setVelocity(0)
right_motor.setVelocity(0)
  
