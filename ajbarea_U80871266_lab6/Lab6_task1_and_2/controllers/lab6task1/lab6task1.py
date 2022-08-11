"""lab6_task1 controller."""
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
left_try = False
done = False
already_visited = []

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

#[Visited, North, East, South, West]
grid = [[0, 1, 0, 0, 1], [0, 1, 0, 0, 0], [0, 1, 0, 0, 0],[0, 1, 1, 0, 0], #starting grid
            [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
            [0, 0, 0, 1, 1], [0, 0, 0, 1, 0], [0, 0, 0, 1, 0],[0, 0, 1, 1, 0]]

def printMaze(maze):
    print("__________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x+1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x+2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x+3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
        print("|  "+ str(maze[x][1]) +"\t  " +str(maze[x+1][1])+"\t  " +str(maze[x+2][1])
              +"\t  " +str(maze[x+3][1])+ "    |")
        print("|" +str(maze[x][4]) + " " +v1+" " + str(maze[x][2])+"\t" +str(maze[x+1][4])+ " " +v2+" " + str(maze[x+1][2])
              +"\t" +str(maze[x+2][4])+ " " +v3+" " + str(maze[x+2][2])
              +"\t" +str(maze[x+3][4]) + " " +v4+" " + str(maze[x+3][2]) +"  |")
        print("|  "+str(maze[x][3]) +"\t  " +str(maze[x+1][3])+"\t  " +str(maze[x+2][3])
              +"\t  " +str(maze[x+3][3])+"    |")
        if(i==3):
            print("|_________________________________|\n")
        else:
            print("|                                 |")
                
def getDirectionFacing():
    degrees = getIMUDegrees()
    if(degrees<45 or degrees>315):
        return 'S'
    if(degrees >45 and degrees < 135):
        return 'E'
    if(degrees >135 and degrees < 225):
        return 'N'
    if(degrees >225 and degrees <315):
        return 'W'
    return '?'
    
def getIMUDegrees(): #convert radians to degrees and make range 180
    return imu.getRollPitchYaw()[2]*180/math.pi + 180
    
def getDistanceSensors(): #function for getting distance sensors in inches. convert from meters
    return [leftDistanceSensor.getValue()*39.3701, frontDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701]

def getPositionSensors():#function for getting getting position sensors in inches. convert from radians
    return leftposition_sensor.getValue()*0.8, rightposition_sensor.getValue()*0.8

def turn(degree): #turns to face another cardinal direction.
    dir = getDirectionFacing()
    if(dir == 'S'):
        target= 0+degree
    elif(dir == 'E'):
        target = 90+degree
    elif(dir=='N'):
        target = 180+degree
    elif(dir=='W'):
        target =270 + degree  
    #ensure are targetdirection is (0-359)
    if(target>=360):
        target=target%360
    elif(target <0):
        target = target+360   
    upperBound = target+1
    lowerBound = target-1
    setSpeedsIPS(-2,2)
    if(degree<0): #if we are turning to the left
        setSpeedsIPS(2,-2)      
    if(lowerBound < 0):
        #if target is 0, we want our imu to give reading of <1 or >359 to stop as IMU rolls over
        lowerBound+=360
        while(robot.step(timestep) != -1 and not (lowerBound < getIMUDegrees() or  getIMUDegrees() < upperBound )):        
            if(abs(lowerBound - getIMUDegrees())< 30 or abs(upperBound - getIMUDegrees())< 30):
                setSpeedsIPS(-1,1)
                if(degree<0): #if we are turning to the left
                    setSpeedsIPS(1,-1)
    else:# we want to stop when our IMU reads lowerBound<imu<upperBound
        while(robot.step(timestep) != -1 and not (lowerBound< getIMUDegrees()  and getIMUDegrees() <upperBound)): 
            if(abs(lowerBound - getIMUDegrees())< 30 or abs(upperBound - getIMUDegrees())< 30):
                setSpeedsIPS(-1,1)
                if(degree<0): #if we are turning to the left
                    setSpeedsIPS(1,-1)          
    setSpeedsIPS(0,0)

def setPose(n):
    visited_cells[n-1] = 'X'
    robot_pose.n = n

def endRun():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)  
    sys.exit(0) 
 
def UpdateMaze(n, L, F, R, direction):
    grid[n - 1][0] = 1 #mark cell visited
    
    if n not in already_visited:
        # mark internal walls based on sensor readings
        if direction == 'N':
            grid[n - 1][4] = L       # LEFT sensor = WEST wall
            grid[n - 1][1] = F       # FRONT sensor = NORTH wall
            grid[n - 1][2] = R       # RIGHT sensor = WEST wall
        elif direction == 'E':
            grid[n - 1][1] = L       # LEFT sensor = NORTH wall
            grid[n - 1][2] = F       # FRONT sensor = EAST wall
            grid[n - 1][3] = R       # RIGHT sensor = SOUTH wall
        elif direction == 'S':
            grid[n - 1][2] = L       # LEFT sensor = EAST wall
            grid[n - 1][3] = F       # FRONT sensor = SOUTH wall
            grid[n - 1][4] = R       # RIGHT sensor = WEST wall
        elif direction == 'W':
            grid[n - 1][3] = L       # LEFT sensor = SOUTH wall
            grid[n - 1][4] = F       # FRONT sensor = WEST wall
            grid[n - 1][1] = R       # RIGHT sensor = NORTH wall
        already_visited.append(n)
        
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

def setSpeedsIPS(Vl,Vr):#function for getting setting motors in inches
    left_speed = (Vl/.8) #convert to radians/sec
    right_speed = (Vr/.8)
    if left_speed > 6.28:
        left_speed = 6.28
    if left_speed < -6.28:
        left_speed = -6.28
    if right_speed > 6.28:
        right_speed = 6.28
    if right_speed < -6.28:
        right_speed = -6.28
    leftMotor.setVelocity(left_speed) 
    rightMotor.setVelocity(right_speed)

def Forward(distance): #distance in inches, no time requirement, run max speed ~ 5 inches
    setSpeedsIPS(5,5)
    startDistance = getPositionSensors()
    curDistance = getPositionSensors()
    while(robot.step(timestep) != -1 and abs(curDistance[0] - startDistance[0]) < distance and abs(curDistance[1] - startDistance[1]) < distance):
        curDistance = getPositionSensors()
    
def ObstacleAvoidance():
    left = getDistanceSensors()[0]
    front = getDistanceSensors()[1]
    right = getDistanceSensors()[2]
    L = 1 if left < 7 else 0
    F = 1 if front < 7 else 0
    R = 1 if right < 7 else 0
    #print('L(', L,') F(', F,') R(', R,')')
    return (L,F,R)
  
def SearchForUnvisitedCells():
    for i in range(len(visited_cells)):
            if visited_cells[i] == '.':
                return False
    else: 
        return True
            
def CheckWalls(L,F,R):
    pos = getPositionSensors()[0]
    #facing west
    if getDirectionFacing() == 'W': 
        if not L and not F and R: #OOW
            if visited_cells[2] == '.':
                setPose(3)            
        if L and not F and R: #WOW
            if visited_cells[1] == '.':
                setPose(2)  
            elif visited_cells[14] == '.': 
                setPose(15)  
            elif visited_cells[13] == '.': 
                setPose(14)                       
        if L and F and R: #WWW
            if visited_cells[0] == '.': 
                setPose(1)                 
        if L and F and not R: #WWO
            if visited_cells[12] == '.':   
                setPose(13)   
    #facing north                   
    elif getDirectionFacing() == 'N': 
        if not L and not F and R: #OOW
            if visited_cells[15] == '.':  
                setPose(16)
            elif visited_cells[10] == '.':
                setPose(11)        
        if L and not F and R: #WOW
            if visited_cells[11] == '.':    
                setPose(12)
            elif visited_cells[7] == '.':  
                setPose(8)
            elif visited_cells[8] == '.':   
                setPose(9)
            elif visited_cells[6] == '.': 
                setPose(7)            
        if not L and F and R: #OWW
            if visited_cells[3] == '.':   
                setPose(4)
        if L and F and not R: #WWO
            if visited_cells[4] == '.':    
                setPose(5)
    #facing east            
    elif getDirectionFacing() == 'E': 
        if L and F and not R: #WWO
            if visited_cells[5] == '.' and pos > 178:    
                setPose(6)
        if not L and F and R: #OOW
            if visited_cells[10] == '.': 
                setPose(11)
    #facing south           
    elif getDirectionFacing() == 'S':
        if not L and F and R: #OWW
            if visited_cells[9] == '.': 
                setPose(10)
    FindCoordinates(robot_pose.n)
    print(' ')
    print(' ')
    print(' ')
    UpdateMaze(robot_pose.n, L, F, R, getDirectionFacing())
    printMaze(grid)
    print(' ')
    print(' Robot Pose |', end='')  
    print(' ( x =',robot_pose.x, ', y =', robot_pose.y,', Grid Cell #' ,robot_pose.n, ',',round(getIMUDegrees(), 1), '° )')
    print('---------------------------------------------------------------------')
    done = SearchForUnvisitedCells()
    if done:
        print(' ')
        print('ALL CELLS VISITED')
        print(' ')
        endRun() 
              
while robot.step(timestep) != -1:
    position_sensor = []
    for i in range(8):
        position_sensor.append(ps[i].getValue())    
        
    readings = ObstacleAvoidance()
    CheckWalls(readings[0],readings[1],readings[2])
    
    if readings == (0,1,0): # swap left or right!
        if left_try == False:
            turn(90)
            left_try = True
        else:
            turn(-90)
            left_try = False
		
    if readings == (1,1,1): # dead end!
        turn(180)
    if readings == (0,1,1): # only left open!
        turn(90)  
    if readings == (1,1,0): # only right open!
        turn(-90)
    Forward(10)