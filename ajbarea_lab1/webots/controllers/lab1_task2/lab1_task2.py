# Lab 1 Task 2: Circular Motion Control
# Tests differential drive kinematics for circular trajectories

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
V = 5  # linear velocity [inches per second]
V = -5  # negative velocity for reverse circular motion
V = 0  # zero velocity - robot stays stationary
V = MAX_VELOCITY + 1  # test velocity limit error handling
W = 0  # angular velocity [radians per second] - calculated from V and R
R1 = 0  # circle radius [inches] - 0 means turn in place
R1 = 10  # circle radius [inches] - positive value for curved path
# ticks = 0  # encoder tick count (unused)

############################################################################
############################################################################
############################################################################


def turnRV(R, V):
    """Execute circular motion with radius R at linear velocity V.

    Args:
        R (float): Circle radius in inches (0 = turn in place)
        V (float): Linear velocity in inches/second

    Uses differential drive kinematics to calculate individual wheel speeds:
    - Angular velocity: W = V / R
    - Left wheel speed: W * (R + d/2)
    - Right wheel speed: W * (R - d/2)
    """
    print(
        f"MOVE ROBOT in a circle of radius {R} inches at a linear velocity of {V} inches per second."
    )
    START = leftposition_sensor.getValue()  # starting position
    print(f"[Starting position] {START}")
    timeSTART = time.monotonic()  # start time
    if R == 0:  # robot spins in place for a full circle
        leftMotor.setVelocity(V)
        rightMotor.setVelocity(-V)
        circumference = 12 * math.pi
        step_count = 0
        while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (
            circumference + START
        ):
            step_count += 1
            if step_count % 50 == 0:  # Print every 50 timesteps
                distance = abs(leftposition_sensor.getValue() - START)
                print(f"Spin progress: {distance:.3f}/{circumference:.3f} ({distance/circumference*100:.1f}%)")

        travelTime = time.monotonic() - timeSTART  # stop time
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        distanceTraveled = leftposition_sensor.getValue() - START
        print(f"[Stopping position] {leftposition_sensor.getValue()}")
        print(
            "[Distance: 0 inches] actual distance traveled: "
            + str(round(abs(distanceTraveled), 5))
        )
        print("[Time: 0 seconds] actual time traveled: " + str(round(travelTime, 2)))
        print("[Left Motor Velocity: " + str(round(V, 2)) + " inches per second]")
        print("[Right Motor Velocity: " + str(round(-V, 2)) + " inches per second]")
    else:
        if V == 0:  # robot motor has a velocity of zero and stays put
            distanceTraveled = 0
            print(f"[Stopping position] {rightposition_sensor.getValue()}")
            print(
                "[Distance: 0 inches] actual distance traveled: "
                + str(round(distanceTraveled, 4))
                + " inches"
            )
            print("[Time: 0 seconds] actual time traveled: 0 seconds")
            print(
                "[Velocity: "
                + str(V)
                + " inches per second] actual velocity: 0 inches per second"
            )
            print(">>>HINT: That was boring! Try a velocity in range [-6.28, 6.28]")
        elif V > 0:
            W = V / R
            print("Angular Velocity: " + str(round(W, 2)) + " radians per second")
            leftVelocity = W * (R + (WHEEL_BASE / 2))
            rightVelocity = W * (R - (WHEEL_BASE / 2))
            circumference = (2 * math.pi * R) * (math.pi / 2)

            if (
                leftVelocity > 6.28
                or leftVelocity < -6.28
                or rightVelocity > 6.28
                or rightVelocity < -6.28
            ):
                exitError()

            leftMotor.setVelocity(leftVelocity)
            rightMotor.setVelocity(rightVelocity)
            getSpeeds()

            step_count = 0
            while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (
                START + circumference
            ):
                step_count += 1
                if step_count % 50 == 0:  # Print every 50 timesteps
                    distance = abs(rightposition_sensor.getValue() - START)
                    print(f"Circle progress: {distance:.3f}/{circumference:.3f} ({distance/circumference*100:.1f}%)")

            travelTime = time.monotonic() - timeSTART  # stop time
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            distanceTraveled = rightposition_sensor.getValue() - START
            timer = distanceTraveled / V
            print(f"[Stopping position] {rightposition_sensor.getValue()}")
            print("[Distance: " + str(round(distanceTraveled, 2)) + " inches]")
            print("[Circle Radius: " + str(round(R1, 2)) + " inches]")
            print("[Time: " + str(round(timer, 2)) + " seconds]")
            print("[Velocity: " + str(round(V, 2)) + " inches per second]")
            print(
                "[Left Motor Velocity: "
                + str(round(leftVelocity, 2))
                + " inches per second]"
            )
            print(
                "[Right Motor Velocity: "
                + str(round(rightVelocity, 2))
                + " inches per second]"
            )
        else:  # robot motor has a negative velocity and moves backwards
            W = V / R
            print("Angular Velocity: " + str(W) + " radians per second")
            leftVelocity = W * (R + (WHEEL_BASE / 2))
            rightVelocity = W * (R - (WHEEL_BASE / 2))
            circumference = (2 * math.pi * R) * (math.pi / 2)
            timer = abs(circumference / V)

            if (
                leftVelocity > 6.28
                or leftVelocity < -6.28
                or rightVelocity > 6.28
                or rightVelocity < -6.28
            ):
                exitError()

            leftMotor.setVelocity(leftVelocity)
            rightMotor.setVelocity(rightVelocity)
            getSpeeds()
            step_count = 0
            while robot.step(timestep) != -1 and leftposition_sensor.getValue() > (
                START - circumference
            ):
                step_count += 1
                if step_count % 50 == 0:  # Print every 50 timesteps
                    distance = abs(rightposition_sensor.getValue() - START)
                    print(f"Circle progress: {distance:.3f}/{circumference:.3f} ({distance/circumference*100:.1f}%)")

            travelTime = time.monotonic() - timeSTART  # stop time
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            distanceTraveled = rightposition_sensor.getValue() - START
            print(f"[Stopping position] {rightposition_sensor.getValue()}")
            print("[Distance: " + str(round(circumference, 2)) + " inches]")
            print("[Circle Radius: " + str(round(R1, 2)) + " inches]")
            print("[Time: " + str(round(timer, 2)) + " seconds]")
            print("[Velocity: " + str(round(V, 2)) + " inches per second]")
            print(
                "[Left Motor Velocity: "
                + str(round(leftVelocity, 2))
                + " inches per second]"
            )
            print(
                "[Right Motor Velocity: "
                + str(round(rightVelocity, 2))
                + " inches per second]"
            )


def resetCounts():
    """Reset encoder tick counter (placeholder function)."""
    print(f"resetting number of ticks to 0")
    ticks = 0


def getCounts():
    """Get current encoder tick counts (placeholder function)."""
    leftTicks = 0
    rightTicks = 0
    tickTuple = (leftTicks, rightTicks)
    print(f"Motor Ticks (Left , Right) : {tickTuple}")


def getSpeeds():
    """Display current motor velocities for both wheels."""
    leftSpeed = round(leftMotor.getVelocity(), 2)
    rightSpeed = round(rightMotor.getVelocity(), 2)
    speedTuple = (leftSpeed, rightSpeed)
    print(f"Motor Speed (Left , Right) : {speedTuple}")


def initEncoders():
    """Initialize encoder system (placeholder function)."""
    print("this function should contain all code necessary for encoder initialization")


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

    turnRV(R1, V)
    # resetCounts()
    # getCounts()
    # getSpeeds()
    # initEncoders()
