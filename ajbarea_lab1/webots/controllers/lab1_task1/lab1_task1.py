# Lab 1 Task 1: Straight-Line Motion Control
# Tests linear motion with velocity control and encoder feedback

# E-puck robot physical specifications and constants
WHEEL_RADIUS = 0.8  # radius of epuck wheel [inches]
WHEEL_BASE = 2.28  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]

from controller import Robot
import time
import sys
import math

# Initialize robot and simulation parameters
robot = Robot()
timestep = int(robot.getBasicTimeStep())  # simulation time step

# Configure wheel motors for velocity control
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))  # enable velocity control mode
rightMotor.setPosition(float("inf"))  # enable velocity control mode
leftMotor.setVelocity(0)  # start stationary
rightMotor.setVelocity(0)  # start stationary

# Initialize wheel encoders for position feedback
leftposition_sensor = robot.getDevice("left wheel sensor")
rightposition_sensor = robot.getDevice("right wheel sensor")
leftposition_sensor.enable(timestep)  # enable encoder readings
rightposition_sensor.enable(timestep)  # enable encoder readings

robot.step(timestep)

############################################################################
############# Experiment Parameters - Modify these for testing #############
############################################################################

X = 60  # distance [inches]
V = -MAX_VELOCITY  # velocity [inches per second]
V = 0  # velocity [inches per second]
V = MAX_VELOCITY  # velocity [inches per second]
# V = MAX_VELOCITY + 1 # velocity [inches per second]
# W = 36  # angular velocity [radians per second]
# rpsLeft = 100  # motor speed [radians per second]
# rpsRight = 80
# ipsLeft = 21  # motor speed [inches per second]
# ipsRight = 18

############################################################################
############################################################################
############################################################################


def moveXV(X, V):
    """Move robot straight for X inches at velocity V inches/second.

    Args:
        X (float): Target distance in inches
        V (float): Linear velocity in inches/second
    """
    print(f"\nMOVE ROBOT straight {X} inches at {V} inches per second")
    START = leftposition_sensor.getValue()  # starting position
    print(f"[Starting position] {START}")
    timeSTART = time.monotonic()  # start time
    if V == 0:  # robot motor has a velocity of zero and stays put
        distanceTraveled = 0
        print(f"[Stopping position] {rightposition_sensor.getValue()}")
        print(f"[Distance: {X} inches] actual distance traveled: {distanceTraveled:.4f} inches")
        print("[Time: 0 seconds] actual time traveled: 0 seconds")
        print(f"[Velocity: {V} inches per second] actual velocity: 0 inches per second")
        print(">>>HINT: That was boring! Try a velocity in range [-6.28, 6.28]")
    elif V > 0:  # robot motor has a positive velocity and moves forward
        step_count = 0
        while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (
            START + X
        ):
            leftMotor.setVelocity(V)
            rightMotor.setVelocity(V)
            step_count += 1
            if step_count % 50 == 0:  # Print every 50 timesteps
                distance = abs(rightposition_sensor.getValue() - START)
                print(f"Progress: {distance:.3f}/{X} inches ({distance/X*100:.1f}%)")

        travelTime = time.monotonic() - timeSTART  # stop time
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        distanceTraveled = rightposition_sensor.getValue() - START
        print(f"[Stopping position] {rightposition_sensor.getValue()}")
        print(f"[Distance: {X:.2f} inches] actual distance traveled: {distanceTraveled:.4f} inches")
        print(f"[Time: {X/V:.2f} seconds] actual time traveled: {travelTime:.4f} seconds")
        print(f"[Velocity: {V:.2f} inches per second] actual velocity: {(distanceTraveled/travelTime):.4f} inches per second")
    else:  # robot motor has a negative velocity and moves backwards
        step_count = 0
        while robot.step(timestep) != -1 and leftposition_sensor.getValue() > (
            START - X
        ):
            leftMotor.setVelocity(V)
            rightMotor.setVelocity(V)
            step_count += 1
            if step_count % 50 == 0:  # Print every 50 timesteps
                distance = abs(rightposition_sensor.getValue() - START)
                print(f"Progress: {distance:.3f}/{X} inches ({distance/X*100:.1f}%)")

        travelTime = time.monotonic() - timeSTART  # stop time
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        distanceTraveled = (rightposition_sensor.getValue() - START) * -1
        print(f"[Stopping position] {rightposition_sensor.getValue()}")
        print(f"[Distance: {X} inches] actual distance traveled: {distanceTraveled:.4f} inches")
        print(f"[Time: {(X/V*-1):.2f} seconds] actual time traveled: {travelTime:.4f} seconds")
        print(f"[Velocity: {V*-1} inches per second] actual velocity: {(distanceTraveled/travelTime):.4f} inches per second")


def setSpeedsRPS(rpsLeft, rpsRight):
    print(f"\nLEFT MOTOR ==> {rpsLeft} radians per second")
    print(f"RIGHT MOTOR ==> {rpsRight} radians per second")


def setSpeedsIPS(ipsLeft, ipsRight):
    print(f"\nLEFT MOTOR ==> {ipsLeft} inches per second")
    print(f"RIGHT MOTOR ==> {ipsRight} inches per second")


def setSpeedsVW(V, W):
    print(f"\nLINEAR velocity {V} inches per second")
    print(f"ANGULAR velocity {W} radians per second")


def exitError():
    """Display velocity limit error and exit program."""
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print(">>>ERROR: E-PUCK MAX_VELOCITY = 6.28")
    print(">>>Try a velocity in range [-6.28, 6.28]")
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    sys.exit(0)


if __name__ == "__main__":
    if V > MAX_VELOCITY or V < -MAX_VELOCITY:
        exitError()

    moveXV(X, V)
    # setSpeedsRPS(rpsLeft, rpsRight)
    # setSpeedsIPS(ipsLeft, ipsRight)
    # setSpeedsVW(V, W)
