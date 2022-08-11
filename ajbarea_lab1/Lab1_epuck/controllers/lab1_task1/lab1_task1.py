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

X = 30  # distance [inches]
V = -MAX_VELOCITY  # velocity [inches per second]
V = 0  # velocity [inches per second]
V = MAX_VELOCITY  # velocity [inches per second]
V = MAX_VELOCITY + 1 # velocity [inches per second]
#W = 36  # angular velocity [radians per second]
#rpsLeft = 100  # motor speed [radians per second]
#rpsRight = 80
#ipsLeft = 21  # motor speed [inches per second]
#ipsRight = 18

############################################################################
############################################################################
############################################################################

def moveXV(X, V):
    print(f'\nMOVE ROBOT straight {X} inches at {V} inches per second')
    START = leftposition_sensor.getValue() # starting position
    print('[Starting position] ' + str(START))
    timeSTART = time.monotonic() # start time
    if V == 0: # robot motor has a velocity of zero and stays put
        distanceTraveled = 0
        print('[Stopping position] ' + str(rightposition_sensor.getValue()))
        print('[Distance: ' + str(X) + ' inches] actual distance traveled: ' + str(round(distanceTraveled,4)) + ' inches')
        print('[Time: 0 seconds] actual time traveled: 0 seconds')
        print('[Velocity: ' + str(V) + ' inches per second] actual velocity: 0 inches per second')
        print('>>>HINT: That was boring! Try a velocity in range [-6.28, 6.28]')
    elif V > 0: # robot motor has a positive velocity and moves forward
        while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (START + X):  
            leftMotor.setVelocity(V)
            rightMotor.setVelocity(V)
            print('distance traveled>  ' + str(round(abs(rightposition_sensor.getValue() - START), 5)))
            
        travelTime = time.monotonic() - timeSTART # stop time
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        distanceTraveled = rightposition_sensor.getValue() - START
        print('[Stopping position] ' + str(rightposition_sensor.getValue()))
        print('[Distance: ' + str(round(X,2)) + ' inches] actual distance traveled: ' + str(round(distanceTraveled,4)) + ' inches')
        print('[Time: ' + str(round(X/V, 2)) + ' seconds] actual time traveled: ' +  str(round(travelTime, 4)) + ' seconds')
        print('[Velocity: ' + str(round(V,2)) + ' inches per second] actual velocity: ' + str(round((distanceTraveled/travelTime),4)) + ' inches per second')
    else: # robot motor has a negative velocity and moves backwards
        while robot.step(timestep) != -1 and leftposition_sensor.getValue() > (START - X):
            leftMotor.setVelocity(V)
            rightMotor.setVelocity(V)
            print('distance traveled>  ' + str(round(abs(rightposition_sensor.getValue() - START), 5)))
        
        travelTime = time.monotonic() - timeSTART # stop time
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        distanceTraveled = (rightposition_sensor.getValue() - START) * -1
        print('[Stopping position] ' + str(rightposition_sensor.getValue()))
        print('[Distance: ' + str(X) + ' inches] actual distance traveled: ' + str(round(distanceTraveled,4)) + ' inches')
        print('[Time: ' + str(round((X/V * -1), 2)) + ' seconds] actual time traveled: ' +  str(round(travelTime, 4)) + ' seconds')
        print('[Velocity: ' + str(V * -1) + ' inches per second] actual velocity: ' + str(round((distanceTraveled/travelTime),4)) + ' inches per second')

    

def setSpeedsRPS(rpsLeft, rpsRight):
    print(f'\nLEFT MOTOR ==> {rpsLeft} radians per second')
    print(f'RIGHT MOTOR ==> {rpsRight} radians per second')


def setSpeedsIPS(ipsLeft, ipsRight):
    print(f'\nLEFT MOTOR ==> {ipsLeft} inches per second')
    print(f'RIGHT MOTOR ==> {ipsRight} inches per second')


def setSpeedsVW(V, W):
    print(f'\nLINEAR velocity {V} inches per second')
    print(f'ANGULAR velocity {W} radians per second')


def exitError():
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
    print('>>>ERROR: E-PUCK MAX_VELOCITY = 6.28')
    print('>>>Try a velocity in range [-6.28, 6.28]')
    print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
    sys.exit(0)


if __name__ == '__main__':
    if V > 6.28 or V < -6.28:
        exitError()

    moveXV(X, V)
    #setSpeedsRPS(rpsLeft, rpsRight)
    #setSpeedsIPS(ipsLeft, ipsRight)
    #setSpeedsVW(V, W)
