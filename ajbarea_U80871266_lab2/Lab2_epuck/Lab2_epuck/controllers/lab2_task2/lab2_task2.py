# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.28  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793 


"""lab2_task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import time
import math
import sys
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#getting the position sensors

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)


############################################################################
############# these variables can be played with freely ####################
############################################################################
R1 = 10
R2 = 14
X = 4   #speed
Y = 40  #seconds
Y = 10  #seconds
############################################################################
############################################################################
############################################################################

def backwardsCircle(first_left_speed, first_right_speed):
    start_time = time.monotonic()
    while robot.step(timestep) != -1 and ((imu.getRollPitchYaw()[2] * (180/pi)) % 360) != 360 and start_time > 0:
        leftMotor.setVelocity(-first_left_speed)
        rightMotor.setVelocity(-first_right_speed)
        getIMUDegrees()
        if math.isclose(imu.getRollPitchYaw()[2], 0.1, abs_tol = .01) == True:
            return True

def circles(R1, R2, X, Y):
    start_time = time.monotonic()
    W = X/R1
    
    distanceR1 = 2 * pi * R1
    distanceR2 = 2 * pi * R2    
    distance = (distanceR1 + distanceR2)
    realTime = distance / X
    
    if(Y < realTime):
        print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        print('>>>Given time: ' + str(Y) + ' seconds')
        print('>>>Actual time: ' + str(round(realTime, 2)) + ' seconds')
        print('>>>Circle cannot be completed in ' + str(Y) + ' seconds.')
        print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        sys.exit(0)
            
    first_right_speed = W * (R1 + (d/2))
    first_left_speed = W * (R1 - (d/2)) 
    second_left_speed = W * (R2 + (d/2))
    second_right_speed = W * (R2 - (d/2)) 
    
    if first_left_speed > 6.28 or first_left_speed < -6.28 or first_right_speed > 6.28 or first_right_speed < -6.28:
        exitError()
    if second_left_speed > 6.28 or second_left_speed < -6.28 or second_right_speed > 6.28 or second_right_speed < -6.28:
        exitError()
        
    b = backwardsCircle(first_left_speed, first_right_speed)

    leftMotor.setVelocity(second_left_speed)
    rightMotor.setVelocity(second_right_speed)
    while robot.step(timestep) != -1 and (imu.getRollPitchYaw()[2] * (180/pi)) % 360 != 0 and start_time > 1:
        leftMotor.setVelocity(second_left_speed)
        rightMotor.setVelocity(second_right_speed)
        getIMUDegrees()
        if math.isclose(imu.getRollPitchYaw()[2], (pi/2), abs_tol = 0.02) == True:
            finishLoop(second_left_speed, second_right_speed)
            
def finishLoop(second_left_speed, second_right_speed): 
    while robot.step(timestep) != -1 and imu.getRollPitchYaw()[2] > 0:
        getIMUDegrees()
        leftMotor.setVelocity(second_left_speed)
        rightMotor.setVelocity(second_right_speed) 
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0) 
    
    travel_dist = (2 * pi * R1) + (2 * pi * R2) 
    actual_time = travel_dist / X
    print("distance: " + str(round(travel_dist, 2)) + ' inches')
    print("speed: " + str(round(X, 2)) + ' inches per second')
    print("time: " + str(round(actual_time, 2)) + ' seconds')
    sys.exit(0)

def getIMUDegrees():
    degrees  = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
    print(str(round(degrees)) + "° degrees")
    
def exitError():
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
    print('>>>ERROR: E-PUCK MAX_VELOCITY = 6.28')
    print('>>>Try a velocity in range [-6.28, 6.28]')
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
    sys.exit(0)   
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if X > 6.28 or X < -6.28:
        exitError()
    
    circles(R1, R2, X, Y)
