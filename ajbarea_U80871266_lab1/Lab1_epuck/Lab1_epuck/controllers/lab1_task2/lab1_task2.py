# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.28  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793 

from controller import Robot 
import time 
import sys

robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

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

robot.step(timestep)

############################################################################
############# these variables can be played with freely ####################
############################################################################
V = 5  # velocity [inches per second]
V = -5
V = 0
V = MAX_VELOCITY + 1
W = 0  # angular velocity [radians per second]
R1 = 0 # circle radius [inches]
R1 = 10
#ticks = 0

###############################################################`#############
############################################################################
############################################################################

def turnRV(R, V):
    print(f'MOVE ROBOT in a circle of radius {R} inches at a linear velocity of {V} inches per second.')
    START = leftposition_sensor.getValue() # starting position
    print('[Starting position] ' + str(START))
    timeSTART = time.monotonic() # start time  
    if R == 0: # robot spins in place for a full circle
        leftMotor.setVelocity(V)
        rightMotor.setVelocity(-V)
        circumference = 12 * pi
        while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (circumference + START):
            print('distance traveled>  ' + str(round(abs(leftposition_sensor.getValue() - START), 5)))
        
        travelTime = time.monotonic() - timeSTART # stop time  
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        distanceTraveled = leftposition_sensor.getValue() - START
        print('[Stopping position] ' + str(leftposition_sensor.getValue()))
        print('[Distance: 0 inches] actual distance traveled: ' +str(round(abs(distanceTraveled), 5)))
        print('[Time: 0 seconds] actual time traveled: ' + str(round(travelTime, 2)))
        print('[Left Motor Velocity: ' + str(round(V, 2)) + ' inches per second]')
        print('[Right Motor Velocity: ' + str(round(-V, 2)) + ' inches per second]') 
    else:    
        if V == 0: # robot motor has a velocity of zero and stays put
            distanceTraveled = 0
            print('[Stopping position] ' + str(rightposition_sensor.getValue()))
            print('[Distance: 0 inches] actual distance traveled: ' + str(round(distanceTraveled,4)) + ' inches')
            print('[Time: 0 seconds] actual time traveled: 0 seconds')
            print('[Velocity: ' + str(V) + ' inches per second] actual velocity: 0 inches per second')
            print('>>>HINT: That was boring! Try a velocity in range [-6.28, 6.28]')
        elif V > 0:
            W = V/R
            print('Angular Velocity: ' + str(round(W, 2)) + ' radians per second')
            leftVelocity = W * (R + (d/2))
            rightVelocity = W * (R - (d/2))
            circumference = (2 * pi * R) * (pi/2)
            
            
            if leftVelocity > 6.28 or leftVelocity < -6.28 or rightVelocity > 6.28 or rightVelocity < -6.28:
                exitError()
            
            leftMotor.setVelocity(leftVelocity)
            rightMotor.setVelocity(rightVelocity)
            getSpeeds()
            
            while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (START + circumference):
                print('distance traveled>  ' + str(round(abs(rightposition_sensor.getValue() - START), 5)))
            
            travelTime = time.monotonic() - timeSTART # stop time
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            distanceTraveled = rightposition_sensor.getValue() - START 
            timer = distanceTraveled / V
            print('[Stopping position] ' + str(rightposition_sensor.getValue()))
            print('[Distance: ' + str(round(distanceTraveled, 2)) + ' inches]')
            print('[Circle Radius: ' + str(round(R1, 2)) + ' inches]')
            print('[Time: ' + str(round(timer,2)) + ' seconds]')
            print('[Velocity: ' + str(round(V, 2)) + ' inches per second]')
            print('[Left Motor Velocity: ' + str(round(leftVelocity, 2)) + ' inches per second]')
            print('[Right Motor Velocity: ' + str(round(rightVelocity, 2)) + ' inches per second]')   
        else: # robot motor has a negative velocity and moves backwards
            W = V/R
            print('Angular Velocity: ' + str(W) + ' radians per second')
            leftVelocity = W * (R + (d/2))
            rightVelocity = W * (R - (d/2))
            circumference = (2 * pi * R) * (pi/2)
            timer = abs(circumference / V)
    
            if leftVelocity > 6.28 or leftVelocity < -6.28 or rightVelocity > 6.28 or rightVelocity < -6.28:
                exitError()
            
            leftMotor.setVelocity(leftVelocity)
            rightMotor.setVelocity(rightVelocity)
            getSpeeds()
            while robot.step(timestep) != -1 and leftposition_sensor.getValue() > (START - circumference):
                print('distance traveled>  ' + str(round(abs(rightposition_sensor.getValue() - START), 5)))
            
            travelTime = time.monotonic() - timeSTART # stop time
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            distanceTraveled = rightposition_sensor.getValue() - START
            print('[Stopping position] ' + str(rightposition_sensor.getValue()))
            print('[Distance: ' + str(round(circumference, 2)) + ' inches]')
            print('[Circle Radius: ' + str(round(R1, 2)) + ' inches]')
            print('[Time: ' + str(round(timer,2)) + ' seconds]')
            print('[Velocity: ' + str(round(V, 2)) + ' inches per second]')
            print('[Left Motor Velocity: ' + str(round(leftVelocity, 2)) + ' inches per second]')
            print('[Right Motor Velocity: ' + str(round(rightVelocity, 2)) + ' inches per second]')   
      
def resetCounts():
    print(f'resetting number of ticks to 0')
    ticks = 0
    

def getCounts():
    leftTicks = 0
    rightTicks = 0
    tickTuple = (leftTicks, rightTicks)
    print(f'Motor Ticks (Left , Right) : {tickTuple}')


def getSpeeds():
    leftSpeed = round(leftMotor.getVelocity() , 2)
    rightSpeed = round(rightMotor.getVelocity() , 2)
    speedTuple = (leftSpeed, rightSpeed)
    print(f'Motor Speed (Left , Right) : {speedTuple}')
    
    
def initEncoders():
    print('this function should contain all code necessary for encoder initialization')


def exitError():
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
    print('>>>ERROR: E-PUCK MAX_VELOCITY = 6.28')
    print('>>>Try a velocity in range [-6.28, 6.28]')
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
    sys.exit(0)


if __name__ == '__main__':
    if V > 6.28 or V < -6.28:
        exitError()

    turnRV(R1, V)
    #resetCounts()
    #getCounts()
    #getSpeeds()
    #initEncoders()
