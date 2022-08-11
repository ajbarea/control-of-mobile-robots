# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.28  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793 


"""lab2_task1 controller."""

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

robot.step(timestep)





############################################################################
############# these variables can be played with freely ####################
############################################################################

H = 20              #length
W = 40              #width
X = MAX_VELOCITY + 1   #speed
Y = 60              #seconds
############################################################################
############################################################################
############################################################################





def rectangle(H, W, X, Y):
    if X > 6.28 or X < -6.28:
        exitError()
        
    #drive north H/2
    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (pos + H/2):
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(X)
        getIMUDegrees()
    
    #SPIN from 0 to -pi/2 [NORTH => EAST]
    start_time = time.monotonic()
    while robot.step(timestep) != -1 and imu.getRollPitchYaw()[2] > (-pi/2) and imu.getRollPitchYaw()[2] < (pi/2):
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(-X)
        getIMUDegrees()
    leftMotor.setVelocity(X)
    rightMotor.setVelocity(X)
    turn_time = time.monotonic() - start_time

    #drive east W
    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (W + pos):
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(X)
        getIMUDegrees()
    
    #SPIN from -pi/2 to -pi [EAST => SOUTH]
    while robot.step(timestep) != -1 and degreeIMU() > 180:
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(-X)
        getIMUDegrees()
    leftMotor.setVelocity(X)
    rightMotor.setVelocity(X)
    
    #drive down H
    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (pos + H):
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(X)
        getIMUDegrees()

    #SPIN from -pi to pi/2 [SOUTH => WEST]
    leftMotor.setVelocity(X)
    rightMotor.setVelocity(-X)
    while robot.step(timestep) != -1 and degreeIMU() > 90:
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(-X)
        getIMUDegrees()
    leftMotor.setVelocity(X)
    rightMotor.setVelocity(X)  
    
    #drive west W
    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (W + pos):   
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(X)
        getIMUDegrees()
    
    #SPIN from -pi to pi/2 [SOUTH => WEST]
    leftMotor.setVelocity(X)
    rightMotor.setVelocity(-X)
    while robot.step(timestep) != -1 and imu.getRollPitchYaw()[2] > 0:
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(-X)
        getIMUDegrees()
    leftMotor.setVelocity(X)
    rightMotor.setVelocity(X)    
    
    #drive north H/2 back to starting point
    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (pos + H/2):   
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(X)
        getIMUDegrees()
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    travel_dist = (2 * H) + (W * 2)
    travel_time = travel_dist/X + turn_time
    
    print("distance: " + str(round(travel_dist, 2)) + ' inches')
    print("speed: " + str(round(X,2)) + ' inches per second')
    print("time: " + str(round(travel_time, 2)) + ' seconds')
    
    if(Y < travel_time):
        print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        print('>>>Given time: ' + str(Y) + ' seconds')
        print('>>>Actual time: ' + str(round(travel_time, 2)) + ' seconds')
        print('>>>NOTE: Waypoints could not be completed in ' + str(Y) + ' seconds.')
        print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
        sys.exit(0)
        
    return

def degreeIMU():
    return (imu.getRollPitchYaw()[2] * (180/pi)) % 360
    
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
if __name__ == '__main__':
    if X > 6.28 or X < -6.28:
        exitError()
    
    rectangle(H,W,X,Y)