# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.0866  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793 

"""lab2_task4corridor controller."""
from controller import Robot, Camera, DistanceSensor
import sys

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftposition_sensor = robot.getDevice('left wheel sensor')
leftposition_sensor.enable(timestep)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


# proportional gain test
Kp = [0.1, 0.5, 1.0, 2.0, 2.5, 5.0]
#proportional_gain = Kp[0] 
#proportional_gain = Kp[1] 
#proportional_gain = Kp[2] 
proportional_gain = Kp[3] 
#proportional_gain = Kp[4] 
#proportional_gain = Kp[5] 
wall_distance = 6.9

def turnRight90degrees():
    START_THETA = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
    endpointRIGHT = (START_THETA - 90) % 360
    endpointLEFT = (START_THETA + 90) % 360
    while robot.step(timestep) != -1:
        left =  leftDistanceSensor.getValue() / 0.0254 
        right = rightDistanceSensor.getValue() / 0.0254  
        front = frontDistanceSensor.getValue() / 0.0254     
        printTuple = (left, front, right)
        print(' ')
        print('....................................................................................')
        print('(left, front, right) | ', end ="")
        print(printTuple)
        reading = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
        error_left = wall_distance - -left
        new_speed_left = proportional_gain * error_left              
        print('proportional gain Kp: ' + str(proportional_gain))
        print('distance desired from wall r(t): ' + str(wall_distance))
        print('distance measured y(t): ' + str(left))
        print('distance error e(t): ' + str(error_left))
        print('new velocity: ' + str(new_speed_left))
        print('>>>>>>>>>>>>>>>>>>>>>FRONT WALL DETECTED '+ str(right) + '',end ="")      
        print(' [turning right 90 degrees]')
        leftMotor.setVelocity(MAX_VELOCITY)
        rightMotor.setVelocity(-MAX_VELOCITY) 
        if reading < endpointRIGHT:
            velocityTurnControl() 

def velocityTurnControl():
    new_speed_left = MAX_VELOCITY
    new_speed_right = MAX_VELOCITY
    while robot.step(timestep) != -1:
        position = leftposition_sensor.getValue()
        if position > 480:
            print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>start point reached!!!')
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            sys.exit(0)
        left =  leftDistanceSensor.getValue() / 0.0254 
        right = rightDistanceSensor.getValue() / 0.0254  
        front = frontDistanceSensor.getValue() / 0.0254  
        printTuple = (left, front, right)
        print(' ')
        print('....................................................................................')
        print('(left, front, right) | ', end ="")
        print(printTuple)
        wall_distance = 6.9
        error_left = wall_distance - -left
        new_speed_left = proportional_gain * error_left              
        print('proportional gain Kp: ' + str(proportional_gain))
        print('distance desired from wall r(t): ' + str(wall_distance))
        print('distance measured y(t): ' + str(left))
        print('distance error e(t): ' + str(error_left))
        print('new velocity: ' + str(new_speed_left))
        
        if front < 8:
            turnRight90degrees()
       
        if left < 6.9:
            #print('>>>LEFT WALL: ' + str(left))
            new_speed_left = MAX_VELOCITY
            new_speed_right = MAX_VELOCITY - .30
        elif right > 6.1:
            #print('>>>>>>>>>>>>>>>>>>>>>RIGHT WALL: ' + str(right))
            new_speed_left = MAX_VELOCITY - .30
            new_speed_right = MAX_VELOCITY
        else:
            if new_speed_left > MAX_VELOCITY:
                #print('LEFT TOO HIGH => setting velocity to 6.28')
                new_speed_left = MAX_VELOCITY
            elif new_speed_left < -MAX_VELOCITY:
                #print('LEFT TOO LOW => setting velocity to -6.28')
                new_speed_left = -MAX_VELOCITY  
            else: 
                new_speed_left = new_speed_left
                new_speed_right = new_speed_left
      
        leftMotor.setVelocity(new_speed_left)
        rightMotor.setVelocity(new_speed_right)
    
while robot.step(timestep) != -1:
    front = frontDistanceSensor.getValue() / 0.0254 
    velocityTurnControl()
    if front < 8:
        turn90degrees()
