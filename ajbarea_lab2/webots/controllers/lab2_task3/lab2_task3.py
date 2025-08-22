"""lab2_task3 controller."""

import math
import sys

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# e-puck robot specs and constants
r = 0.8  # radius of epuck wheel [inches]
d = 2.28  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
pi = 3.141592653589793

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# getting the position sensors

leftposition_sensor = robot.getDevice("left wheel sensor")
rightposition_sensor = robot.getDevice("right wheel sensor")
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice("inertial unit")
imu.enable(timestep)


# These variables can be played with freely
R1 = 10
D1 = 40
X = 5.6  # speed
X = MAX_VELOCITY + 1  # speed
Y = 60  # seconds
# End of configurable variables


def waypoints(R1, D1, X, Y):
    a = True
    W = X / R1
    left_speed = W * (R1 + (d / 2))
    right_speed = W * (R1 - (d / 2))

    travel_dist = (2 * pi * R1) + (D1 * 2)
    real_time = travel_dist / X

    if Y < real_time:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        print(">>>Given time: " + str(Y) + " seconds")
        print(">>>Actual time: " + str(round(real_time, 2)) + " seconds")
        print(">>>Waypoints cannot be completed in " + str(Y) + " seconds.")
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        sys.exit(0)

    if (
        left_speed > 6.28
        or left_speed < -6.28
        or right_speed > 6.28
        or right_speed < -6.28
    ):
        exitError()

    while (
        robot.step(timestep) != -1
        and imu.getRollPitchYaw()[2] > (-pi / 2)
        and imu.getRollPitchYaw()[2] < (pi / 2)
    ):
        leftMotor.setVelocity(left_speed)
        rightMotor.setVelocity(right_speed)
        getIMUDegrees()
    leftMotor.setVelocity(X)
    rightMotor.setVelocity(X)

    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (D1 + pos):
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(X)
        getIMUDegrees()

    while robot.step(timestep) != -1 and imu.getRollPitchYaw()[2] != pi / 2:
        leftMotor.setVelocity(left_speed)
        rightMotor.setVelocity(right_speed)
        getIMUDegrees()
        if math.isclose(imu.getRollPitchYaw()[2], (pi / 2), abs_tol=0.02) == a:
            finishLoop(left_speed, right_speed)


def finishLoop(left_speed, right_speed):
    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (D1 + pos):
        leftMotor.setVelocity(X)
        rightMotor.setVelocity(X)
        getIMUDegrees()

    while robot.step(timestep) != -1 and imu.getRollPitchYaw()[2] > 0:
        leftMotor.setVelocity(left_speed)
        rightMotor.setVelocity(right_speed)
        getIMUDegrees()

    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    travel_dist = (2 * pi * R1) + (D1 * 2)
    real_time = travel_dist / X
    print("distance: " + str(round(travel_dist, 2)) + " inches")
    print("speed: " + str(round(X, 2)) + " inches per second")
    print("time: " + str(round(real_time, 2)) + " seconds")
    sys.exit(0)


def getIMUDegrees():
    degrees = (imu.getRollPitchYaw()[2] * (180 / pi)) % 360
    print(str(round(degrees)) + "° degrees")


def exitError():
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print(">>>ERROR: E-PUCK MAX_VELOCITY = 6.28")
    print(">>>Try a velocity in range [-6.28, 6.28]")
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    sys.exit(0)


# Main loop:
while robot.step(timestep) != -1:
    if X > 6.28 or X < -6.28:
        exitError()

    waypoints(R1, D1, X, Y)
