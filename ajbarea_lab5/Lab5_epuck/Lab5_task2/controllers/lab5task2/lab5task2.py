"""lab5_task2 controller."""
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import sys

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#enable distance sensors
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

#enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#global arrays
visited_cells = []
already_visited = []
ps = []
MAX_VELOCITY = 6.28
kp_sides = 0.6
kp = 0.9

class Pose:
    def __init__(self, x, y, n):
        self.x = x
        self.y = y 
        self.n = n

robot_pose = Pose(-1, -1, -1)
for i in range(16):
    visited_cells.append('.')

psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
    
def getDirectionFacing():
    degrees = getIMUDegrees()
    if(degrees<45 or degrees>315):
        return 'W'
    if(degrees >45 and degrees < 135):
        return 'S'
    if(degrees >135 and degrees < 225):
        return 'E'
    if(degrees >225 and degrees <315):
        return 'N'
    return '?'
    
def getIMUDegrees(): #convert radians to degrees and make range 180
    return imu.getRollPitchYaw()[2]*180/math.pi + 180
    
def getDistanceSensors(): #function for getting distance sensors in inches. convert from meters
    return [leftDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701]

def getPositionSensors():#function for getting getting position sensors in inches. convert from radians
    return leftposition_sensor.getValue()*0.8

def setPose(n):
    visited_cells[n-1] = 'X'
    robot_pose.n = n

def endRun():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)  
    sys.exit(0) 
 
def PrintGrid(p):
    if(p == 0):
        print(' ')
        print(' ')
        print(' ')
        print('---------------------------------------------------------------------')
        print(' Visited Cells')
        print('  |',visited_cells[0], visited_cells[1], visited_cells[2], visited_cells[3] ,'|')
        print('  |',visited_cells[4], visited_cells[5], visited_cells[6], visited_cells[7] ,'|')
        print('  |',visited_cells[8], visited_cells[9], visited_cells[10], visited_cells[11] ,'|')
        print('  |',visited_cells[12], visited_cells[13], visited_cells[14], visited_cells[15] ,'|')  
        print(' ')
        print(' Robot Pose |', end='')  
        print(' ( x =',robot_pose.x, ', y =', robot_pose.y,', Grid Cell #' ,robot_pose.n, ',',round(getIMUDegrees(), 1), '° )')  
        print('---------------------------------------------------------------------')

def FindCoordinates(n):
    # general X coordinate
    if n == 1 or n == 5 or n == 9 or n == 13:
        robot_pose.x = -15
    elif n == 2 or n == 6 or n == 10 or n == 14:
        robot_pose.x = -5
    elif n == 3 or n == 7 or n == 11 or n == 15:
        robot_pose.x = 5
    elif n == 4 or n == 8 or n == 12 or n == 16:
        robot_pose.x = 15
    
    # general Y coordinate
    if n == 1 or n == 2 or n == 3 or n == 4:
        robot_pose.y = 15
    elif n == 5 or n == 6 or n == 7 or n == 8:
        robot_pose.y = 5
    elif n == 9 or n == 10 or n == 11 or n == 12:
        robot_pose.y = -5
    elif n == 13 or n == 14 or n == 15 or n == 16:
        robot_pose.y = -15
        
def CheckWalls(L,F,R):
    #facing west
    if getDirectionFacing() == 'W': 
        if not L and not F and R: #OOW
            if visited_cells[2] == '.':
                setPose(3)            
        if L and not F and R: #WOW
            if visited_cells[1] == '.':
                setPose(2)  
            elif visited_cells[14] == '.' and pos > 118:
                setPose(15)  
            elif visited_cells[13] == '.' and pos > 135:
                setPose(14)                       
        if L and F and R: #WWW
            if visited_cells[0] == '.': 
                setPose(1)                 
        if L and F and not R: #WWO
            if visited_cells[12] == '.' and pos > 142:   
                setPose(13)   
    #facing north                   
    elif getDirectionFacing() == 'N': 
        if not L and not F and R: #OOW
            if visited_cells[15] == '.':  
                setPose(16)
            elif visited_cells[10] == '.' and pos > 30:
                setPose(11)        
        if L and not F and R: #WOW
            if visited_cells[11] == '.':    
                setPose(12)
            elif visited_cells[7] == '.' and pos > 15:   
                setPose(8)
            elif visited_cells[8] == '.'and pos > 30:   
                setPose(9)
            elif visited_cells[6] == '.'and pos > 180: 
                setPose(7)            
        if not L and F and R: #OWW
            if visited_cells[3] == '.':   
                setPose(4)
        if L and F and not R: #WWO
            if visited_cells[4] == '.' and pos > 164:   
                setPose(5)
    #facing east            
    elif getDirectionFacing() == 'E': 
        if L and F and not R: #WWO
            if visited_cells[5] == '.' and pos > 178:    
                setPose(6)
    #facing south           
    elif getDirectionFacing() == 'S':
        if not L and F and R: #OWW
            if visited_cells[9] == '.' and pos > 190: 
                setPose(10)
    FindCoordinates(robot_pose.n)
                    
while robot.step(timestep) != -1:
    left_speed  = kp_sides * MAX_VELOCITY
    right_speed = kp_sides * MAX_VELOCITY
    
    left = getDistanceSensors()[0]
    front = getDistanceSensors()[1]
    right = getDistanceSensors()[2]
        
    pos = getPositionSensors()
    #print(pos)
    if pos > 210:
        endRun()
   
    newpos = round(pos) % 10
    PrintGrid(newpos)
        
    position_sensor = []
    for i in range(8):
        position_sensor.append(ps[i].getValue())
        
    front_wall = getDistanceSensors()[1] < 5
    left_wall = position_sensor[5] > 90.0 or position_sensor[6] > 90.0 or position_sensor[7] > 90.0
    right_wall = position_sensor[0] > 90.0 or position_sensor[1] > 90.0 or position_sensor[2] > 90.0

    if front_wall:
        if right > left:
            left_speed  = kp * MAX_VELOCITY
            right_speed = -kp * MAX_VELOCITY
        else:
            left_speed  = -kp * MAX_VELOCITY
            right_speed = kp * MAX_VELOCITY
    elif left_wall:
        left_speed  = kp_sides * MAX_VELOCITY
        right_speed = -kp_sides * MAX_VELOCITY
    elif right_wall:
        left_speed  = -kp_sides * MAX_VELOCITY
        right_speed = kp_sides * MAX_VELOCITY
        
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
    
    L = 1 if left < 7 else 0
    F = 1 if front < 7 else 0
    R = 1 if right < 7 else 0
    #print('L(', L,') F(', F,') R(', R,')') 
    CheckWalls(L,F,R)