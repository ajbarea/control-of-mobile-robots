# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.0866  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793 

"""lab4_task1 controller."""
from controller import Robot, Camera, DistanceSensor
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)
                  
def getIMUDegrees(): #convert radians to degrees and make range 180
    return imu.getRollPitchYaw()[2]*180/math.pi + 180

def getDirectionFacing():
    degrees = getIMUDegrees()
    if(degrees < 22.5 or degrees > 337.5):
        return 'N'
    if(degrees < 337.5 and degrees > 292.5):
        return 'NE'
    if(degrees < 292.5 and degrees > 247.5):
        return 'E'
    if(degrees < 247.5 and degrees > 202.5):
        return 'SE'
    if(degrees < 202.5 and degrees > 157.5):
        return 'S'
    if(degrees < 157.5 and degrees > 112.5):
        return 'SW'
    if(degrees < 112.5 and degrees > 67.5):
        return 'W'
    if(degrees < 67.5 and degrees > 22.5):
        return 'NW'
    return '?'
     
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

def getDistanceSensors(): #function for getting distance sensors in inches. convert from meters
    return [round(leftDistanceSensor.getValue()*39.3701, 5), round(frontDistanceSensor.getValue()*39.3701, 5), round(rightDistanceSensor.getValue()*39.3701, 5)]

def IMUPrint(): # print robot orientation in degrees
    print(' ')
    print('[IMU: '+ str(round(getIMUDegrees(), 1)) + '° ' + getDirectionFacing() + ']')
    dist = getDistanceSensors()
    print('[left, front, right] | ', end='')
    print(dist)

def GoalFacing(): # return object position, orientiation, size
    setSpeedsIPS(2,-2)
    recognized_object_array = camera.getRecognitionObjects()
    array_len = len(recognized_object_array)
    if array_len == 0:
        print('searching for goal...')
    else: 
        print('goal found!')
        recognized_object = camera.getRecognitionObjects()[0]
        image_position = recognized_object.get_position_on_image()
        if image_position[0] > 41:
            setSpeedsIPS(1, -1) #left spin
        if image_position[0] < 39:
            setSpeedsIPS(-1, 1) #right spin
        if image_position[0] == 39 or image_position[0] == 40 or image_position[0] == 41:
            setSpeedsIPS(0,0)  
        return [recognized_object.get_position(),recognized_object.get_orientation(), recognized_object.get_size()]
     
while robot.step(timestep) != -1:
    GoalFacing()
    IMUPrint()