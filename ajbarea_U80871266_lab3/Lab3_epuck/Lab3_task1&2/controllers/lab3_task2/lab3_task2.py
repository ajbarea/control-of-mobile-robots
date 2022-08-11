# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.0866  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793 

"""lab3_task2 controller."""
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#initialization of motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#distance sensors 
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# proportional gain test
Kp = [0.1, 0.5, 1.0, 2.0, 2.5, 5.0]
#proportional_gain = Kp[0] # too slow
#proportional_gain = Kp[1] # back and forth too much
#proportional_gain = Kp[2] 
proportional_gain = Kp[4] # best
#proportional_gain = Kp[4] 
#proportional_gain = Kp[5] 

desired_distance = 10

def velocityTurnControl():
    left = leftDistanceSensor.getValue() / 0.0254
    right = rightDistanceSensor.getValue() / 0.0254
    front = frontDistanceSensor.getValue() / 0.0254
    wall_distance = (left + right) / 2
    error_left = wall_distance - -left
    error_right = wall_distance - -right
    new_speed_left = proportional_gain * error_left
    new_speed_right = proportional_gain * error_right
              
    print('proportional gain Kp: ' + str(proportional_gain))
    print('distance desired from wall r(t): ' + str(wall_distance))
    print('distance measured LEFT y(t): ' + str(left))
    print('distance error LEFT e(t): ' + str(error_left))
    print('new LEFT velocity: ' + str(new_speed_left))
    print('distance measured RIGHT y(t): ' + str(right))
    print('distance error RIGHT e(t): ' + str(error_right))
    print('new RIGHT velocity: ' + str(new_speed_right))
    
    if front < 5:
        new_speed_left = MAX_VELOCITY
        new_speed_right = -MAX_VELOCITY
    if left < 3.2:
        print('>>>LEFT WALL: ' + str(left))
        new_speed_left = MAX_VELOCITY
        new_speed_right = 0
    elif right < 3.2:
        print('>>>>>>>>>>>>>>>>>>>>>RIGHT WALL: ' + str(right))
        new_speed_left = 0
        new_speed_right = MAX_VELOCITY
    else:
        if new_speed_left > MAX_VELOCITY:
            print('LEFT TOO HIGH => setting velocity to 6.28')
            new_speed_left = MAX_VELOCITY
        elif new_speed_left < -MAX_VELOCITY:
            print('LEFT TOO LOW => setting velocity to -6.28')
            new_speed_left = -MAX_VELOCITY  
        else: 
            new_speed_left = new_speed_left
            
        if new_speed_right > MAX_VELOCITY:
            print('RIGHT TOO HIGH => setting velocity to 6.28')
            new_speed_right = MAX_VELOCITY
        elif new_speed_right < -MAX_VELOCITY:
            print('RIGHT TOO LOW => setting velocity to -6.28')
            new_speed_right = -MAX_VELOCITY
        else: 
            new_speed_right = new_speed_right
  
    leftMotor.setVelocity(new_speed_left)
    rightMotor.setVelocity(new_speed_right)
    
def velocityControlFunction(front):
    print('proportional gain Kp: ' + str(proportional_gain))
    print('distance desired r(t): ' + str(desired_distance))
    print('distance measured y(t): ' + str(front))
    distance_error = -desired_distance - -front
    print('distance error e(t): ' + str(distance_error))
    new_speed = proportional_gain * distance_error
    print('new velocity: ' + str(new_speed))
    
    if new_speed > MAX_VELOCITY:
        print('TOO HIGH => setting velocity to 6.28')
        leftMotor.setVelocity(MAX_VELOCITY)
        rightMotor.setVelocity(MAX_VELOCITY)
    elif new_speed < -MAX_VELOCITY:
        print('TOO LOW => setting velocity to -6.28')
        leftMotor.setVelocity(-MAX_VELOCITY)
        rightMotor.setVelocity(-MAX_VELOCITY)
    else: 
        leftMotor.setVelocity(new_speed)
        rightMotor.setVelocity(new_speed)
    
def sensorRead(): 
    while robot.step(timestep) != -1:
        front = frontDistanceSensor.getValue() / 0.0254
        left = leftDistanceSensor.getValue() / 0.0254
        right = rightDistanceSensor.getValue() / 0.0254
        tupleInches = (left, front, right)
        print(' ')
        print('....................................................................................')
        print('(left, front, right) | ', end ="")
        print(tupleInches)
        velocityTurnControl()       
    
while robot.step(timestep) != -1:
    sensorRead()
    velocityTurnControl()