"""lab4_task2 controller."""

import math

from controller import Robot

# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.0866  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793

robot = Robot()
timestep = int(robot.getBasicTimeStep())

frontDistanceSensor = robot.getDevice("front_ds")
leftDistanceSensor = robot.getDevice("left_ds")
rightDistanceSensor = robot.getDevice("right_ds")
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

camera = robot.getDevice("camera1")
camera.enable(timestep)
camera.recognitionEnable(timestep)

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

leftposition_sensor = robot.getDevice("left wheel sensor")
rightposition_sensor = robot.getDevice("right wheel sensor")
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice("inertial unit")
imu.enable(timestep)


def getIMUDegrees():  # convert radians to degrees and make range 180
    return imu.getRollPitchYaw()[2] * 180 / math.pi + 180


def getDirectionFacing():
    degrees = getIMUDegrees()
    if degrees < 22.5 or degrees > 337.5:
        return "N"
    if degrees < 292.5 and degrees > 247.5:
        return "E"
    if degrees < 202.5 and degrees > 157.5:
        return "S"
    if degrees < 112.5 and degrees > 67.5:
        return "W"
    return "?"


def setSpeedsIPS(Vl, Vr):  # function for getting setting motors in inches
    left_speed = Vl / 0.8  # convert to radians/sec
    right_speed = Vr / 0.8
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


def getPositionSensors():  # function for getting getting position sensors in inches. convert from radians
    return [leftposition_sensor.getValue() * 0.8, rightposition_sensor.getValue() * 0.8]


def getDistanceSensors():  # function for getting distance sensors in inches. convert from meters
    return [
        round(leftDistanceSensor.getValue() * 39.3701, 8),
        round(frontDistanceSensor.getValue() * 39.3701, 8),
        round(rightDistanceSensor.getValue() * 39.3701, 8),
    ]


def Forward(
    distance,
):  # distance in inches, no time requirement, run max speed ~ 5 inches
    v = 5
    setSpeedsIPS(v, v)
    startDistance = getPositionSensors()
    curDistance = getPositionSensors()
    while (
        robot.step(timestep) != -1
        and abs(curDistance[0] - startDistance[0]) < distance
        and abs(curDistance[1] - startDistance[1]) < distance
    ):
        IMUPrint()
        # print('FORWARD')
        curDistance = getPositionSensors()
        front = getDistanceSensors()[1]
        right = getDistanceSensors()[2]
        front_too_close = front < 5.01
        right_too_close = right < 5.01
        if not front_too_close and right_too_close:
            setSpeedsIPS(v, v)
        if front_too_close and not right_too_close:
            turnCardinal(90)
        if front_too_close and right_too_close:
            turnCardinal(90)
        if not front_too_close and not front_too_close:
            setSpeedsIPS(v, v)
    setSpeedsIPS(0, 0)


def IMUPrint():  # print robot orientation in degrees
    print(" ")
    print("[IMU: " + str(round(getIMUDegrees(), 1)) + "° " + getDirectionFacing() + "]")
    dist = getDistanceSensors()
    print("[left, front, right] | ", end="")
    print(dist)


def GoalFacing():  # return object position, orientiation, size
    IMUPrint()
    # print('GOAL FACING')
    Forward(34.5)
    front = getDistanceSensors()[1]
    right = getDistanceSensors()[2]
    front_too_close = front < 5.01
    right_too_close = right < 5.01

    if front_too_close:
        turnCardinal(90)
        # print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> FRONT WALL')
    if right_too_close:
        turnCardinal(90)
        # print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> RIGHT WALL')

    setSpeedsIPS(2, -2)
    recognized_object_array = camera.getRecognitionObjects()
    array_len = len(recognized_object_array)
    if array_len == 0:
        # print('searching for goal...')
        MotionToGoal()
    else:
        # print('goal found!')
        recognized_object = camera.getRecognitionObjects()[0]
        image_position = recognized_object.get_position_on_image()
        if image_position[0] > 41:
            setSpeedsIPS(1, -1)  # left spin
        if image_position[0] < 39:
            setSpeedsIPS(-1, 1)  # right spin
        if (
            image_position[0] == 39
            or image_position[0] == 40
            or image_position[0] == 41
        ):
            setSpeedsIPS(6, 6)
        else:
            MotionToGoal()
        return [
            recognized_object.get_position(),
            recognized_object.get_orientation(),
            recognized_object.get_size(),
        ]


def MotionToGoal():
    IMUPrint()
    # print('MOTION TO GOAL')
    front = getDistanceSensors()[1]
    right = getDistanceSensors()[2]
    front_too_close = front < 5.01
    right_too_close = right < 5.01
    proportional_gain = 1.5
    desired_distance = 5
    distance_error = -desired_distance - -front
    new_speed = proportional_gain * distance_error
    setSpeedsIPS(new_speed, new_speed)

    if not front_too_close and not right_too_close:
        turnCardinal(-90)
    if not front_too_close and right_too_close:
        setSpeedsIPS(5, 5)  # use default speed
    if front_too_close and not right_too_close:
        turnCardinal(90)
    if front_too_close and right_too_close:
        turnCardinal(90)


def turnCardinal(
    degree,
):  # this function assumes we are facing a cardinal direction, turn
    # print('TURN CARDINAL')
    dir = getDirectionFacing()
    if dir == "N":
        target = 0 + degree
    elif dir == "W":
        target = 90 + degree
    elif dir == "S":
        target = 180 + degree
    elif dir == "E":
        target = 270 + degree

    # ensure are targetdirection is (0-359)
    if target >= 360:
        target = target % 360
    elif target < 0:
        target = target + 360

    upperBound = target + 1
    lowerBound = target - 1

    setSpeedsIPS(-2, 2)

    if degree < 0:  # if we are turning to the left
        setSpeedsIPS(2, -2)

    if lowerBound < 0:
        # if target is 0, we want our imu to give reading of <1 or >359 to stop as IMU rolls over
        lowerBound += 360
        while robot.step(timestep) != -1 and not (
            lowerBound < getIMUDegrees() or getIMUDegrees() < upperBound
        ):
            IMUPrint()
            if (
                abs(lowerBound - getIMUDegrees()) < 30
                or abs(upperBound - getIMUDegrees()) < 30
            ):
                setSpeedsIPS(-1, 1)
                if degree < 0:  # if we are turning to the left
                    setSpeedsIPS(1, -1)
    else:  # we want to stop when our IMU reads lowerBound<imu<upperBound
        while robot.step(timestep) != -1 and not (
            lowerBound < getIMUDegrees() and getIMUDegrees() < upperBound
        ):
            IMUPrint()
            if (
                abs(lowerBound - getIMUDegrees()) < 30
                or abs(upperBound - getIMUDegrees()) < 30
            ):
                setSpeedsIPS(-1, 1)
                if degree < 0:  # if we are turning to the left
                    setSpeedsIPS(1, -1)

    setSpeedsIPS(0, 0)


def wallFollow(wall, targetDistance, Kp_side):
    # print('WALL FOLLOW')
    front = getDistanceSensors()[1]
    right = getDistanceSensors()[2]
    front_too_close = front < 5.01
    right_too_close = right < 5.01

    v = 5
    if wall == "r":
        error = getDistanceSensors()[2] - targetDistance
        if error < 0:
            setSpeedsIPS(v - abs(error) * Kp_side, v)  # turn away from right wall
        else:
            setSpeedsIPS(v, v - abs(error) * Kp_side)  # turn towards right wall

    elif wall == "l":
        error = getDistanceSensors()[0] - targetDistance
        if error < 0:
            setSpeedsIPS(v, v - abs(error) * Kp_side)  # turn away from left wall
        else:
            setSpeedsIPS(v - abs(error) * Kp_side, v)  # turn towards left wall

    if not front_too_close and not right_too_close:
        Forward(5, 10)
    if not front_too_close and right_too_close:
        setSpeedsIPS(5, 5)  # use default speed
    if front_too_close and not right_too_close:
        turnCardinal(90)
    if front_too_close and right_too_close:
        turnCardinal(90)


while robot.step(timestep) != -1:
    GoalFacing()
