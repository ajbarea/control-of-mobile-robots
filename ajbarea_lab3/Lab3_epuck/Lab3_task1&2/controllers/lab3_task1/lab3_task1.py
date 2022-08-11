# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.0866  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793 

"""lab3_task1 controller."""
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
#proportional_gain = Kp[1] # [90 seconds]
#proportional_gain = Kp[2] # [45 seconds]
#proportional_gain = Kp[3] # [25 seconds]
#proportional_gain = Kp[4] # [20 seconds] <== 2.5 showed the best results
#proportional_gain = Kp[5] # too fast

desired_distance = 10
proportional_gain = 2.5

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
        front = frontDistanceSensor.getValue() / .0254
        left = leftDistanceSensor.getValue() / .0254
        right = rightDistanceSensor.getValue() / .0254 
        tupleInches = (left, front, right)
        print('...................................................................................')
        print('(left, front, right) | ', end ="")
        print(tupleInches)
        velocityControlFunction(front)       
    
while robot.step(timestep) != -1:
    sensorRead()