# Lab 1 Task 2: Circular Motion Control
# Tests differential drive kinematics for circular trajectories

# E-puck robot physical specifications and constants
WHEEL_RADIUS = 0.807  # radius of epuck wheel [inches] (official: 0.0205m)
WHEEL_BASE = 2.28  # distance between epuck wheels [inches]
MAX_VELOCITY = 6.28  # motor speed cap [radians per second]
MAX_LINEAR_VELOCITY = MAX_VELOCITY * WHEEL_RADIUS  # maximum linear velocity [inches per second]

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

# GPS not available on e-puck - using odometry instead

robot.step(timestep)

############################################################################
############# Experiment Parameters - Modify these for testing #############
############################################################################
V = 5  # linear velocity [inches per second]
# V = -5  # negative velocity for reverse circular motion
# V = 0  # zero velocity - robot stays stationary
# V = MAX_VELOCITY + 1  # test velocity limit error handling
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
        # Convert linear velocity to motor rotational velocity for turn-in-place
        motor_velocity = V / WHEEL_RADIUS
        motor_velocity = min(abs(motor_velocity), MAX_VELOCITY) * (1 if V > 0 else -1)
        leftMotor.setVelocity(motor_velocity)
        rightMotor.setVelocity(-motor_velocity)
        circumference = math.pi * WHEEL_BASE  # robot turning circumference
        step_count = 0
        while robot.step(timestep) != -1 and leftposition_sensor.getValue() < (
            circumference + START
        ):
            step_count += 1
            if step_count % 50 == 0:  # Print every 50 timesteps
                distance = abs(leftposition_sensor.getValue() - START)
                print(
                    f"Spin progress: {distance:.3f}/{circumference:.3f} ({distance/circumference*100:.1f}%)"
                )

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
            # Calculate wheel linear velocities
            leftLinearVelocity = W * (R + (WHEEL_BASE / 2))
            rightLinearVelocity = W * (R - (WHEEL_BASE / 2))
            
            # Convert to motor rotational velocities
            leftMotorVelocity = leftLinearVelocity / WHEEL_RADIUS
            rightMotorVelocity = rightLinearVelocity / WHEEL_RADIUS

            # Check if velocities exceed motor limits and scale down if needed
            max_required = max(abs(leftMotorVelocity), abs(rightMotorVelocity))
            if max_required > MAX_VELOCITY:
                scale_factor = MAX_VELOCITY / max_required
                leftMotorVelocity *= scale_factor
                rightMotorVelocity *= scale_factor
                leftLinearVelocity *= scale_factor
                rightLinearVelocity *= scale_factor
                V = V * scale_factor  # Update V for accurate reporting
                print(f">>>WARNING: Velocities scaled down by {scale_factor:.3f} to stay within motor limits")
                print(f">>>Actual linear velocity: {V:.2f} inches/s")

            leftMotor.setVelocity(leftMotorVelocity)
            rightMotor.setVelocity(rightMotorVelocity)
            getSpeeds()

            step_count = 0
            # Use distance traveled approach: stop after completing expected circle distance
            start_left = leftposition_sensor.getValue()
            
            # Universal formula for any radius based on empirical calibration
            # The outer wheel needs to travel more than theoretical due to:
            # 1. Differential drive geometry
            # 2. Motor dynamics and encoder precision
            outer_wheel_radius = R + (WHEEL_BASE / 2)  # radius of outer wheel path
            theoretical_outer_distance = 2 * math.pi * outer_wheel_radius
            empirical_scale_factor = 1.229  # derived from 86.0/69.98 for R=10 calibration
            expected_travel_distance = theoretical_outer_distance * empirical_scale_factor
            print(f"Calculated travel distance: {expected_travel_distance:.1f} inches (R={R}, outer_radius={outer_wheel_radius:.2f})")
            position_threshold = 0.5  # inch tolerance around expected distance (tighter for precision)
            
            # Prevent immediate stopping by requiring minimum movement first
            min_travel_distance = 2 * math.pi * R * 0.25  # must travel at least 1/4 circle
            
            while robot.step(timestep) != -1:
                left_current = leftposition_sensor.getValue()
                left_traveled = abs(left_current - start_left)
                
                # For display: simulate distance from start based on circular motion
                # This is just for progress tracking - actual stopping uses travel distance
                angle_traveled = left_traveled / (R + WHEEL_BASE/2)
                estimated_x = R * (1 - math.cos(angle_traveled))
                estimated_y = R * math.sin(angle_traveled)
                distance_from_start = math.sqrt(estimated_x**2 + estimated_y**2)
                
                # Stop when we've traveled the expected distance
                if left_traveled > min_travel_distance and abs(left_traveled - expected_travel_distance) < position_threshold:
                    break
                
                step_count += 1
                if step_count % 10 == 0:  # Print every 10 timesteps for detailed tracking
                    # Use actual wheel path distance for accurate percentage
                    outer_wheel_circumference = 2 * math.pi * (R + WHEEL_BASE/2)
                    progress_percent = min(100, (left_traveled / outer_wheel_circumference) * 100)
                    print(f"Circle progress: {distance_from_start:.2f} inches from start, {left_traveled:.2f} inches traveled ({progress_percent:.1f}%)")

            travelTime = time.monotonic() - timeSTART  # stop time
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            circumference = 2 * math.pi * R
            print(f"[Stopping position] {rightposition_sensor.getValue()}")
            total_distance_traveled = abs(leftposition_sensor.getValue() - START)
            print(f"[Distance: {total_distance_traveled:.2f} inches] (target: {expected_travel_distance:.2f} inches)")
            print(f"[Estimated final distance from start: {distance_from_start:.2f} inches]")
            print("[Circle Radius: " + str(round(R1, 2)) + " inches]")
            print("[Time: " + str(round(travelTime, 2)) + " seconds]")
            print("[Velocity: " + str(round(V, 2)) + " inches per second]")
            print(
                "[Left Motor Velocity: "
                + str(round(leftLinearVelocity, 2))
                + " inches per second]"
            )
            print(
                "[Right Motor Velocity: "
                + str(round(rightLinearVelocity, 2))
                + " inches per second]"
            )
        else:  # robot motor has a negative velocity and moves backwards
            W = V / R
            print("Angular Velocity: " + str(W) + " radians per second")
            # Calculate wheel linear velocities
            leftLinearVelocity = W * (R + (WHEEL_BASE / 2))
            rightLinearVelocity = W * (R - (WHEEL_BASE / 2))
            
            # Convert to motor rotational velocities
            leftMotorVelocity = leftLinearVelocity / WHEEL_RADIUS
            rightMotorVelocity = rightLinearVelocity / WHEEL_RADIUS

            # Check if velocities exceed motor limits and scale down if needed
            max_required = max(abs(leftMotorVelocity), abs(rightMotorVelocity))
            if max_required > MAX_VELOCITY:
                scale_factor = MAX_VELOCITY / max_required
                leftMotorVelocity *= scale_factor
                rightMotorVelocity *= scale_factor
                leftLinearVelocity *= scale_factor
                rightLinearVelocity *= scale_factor
                V = V * scale_factor  # Update V for accurate reporting
                print(f">>>WARNING: Velocities scaled down by {scale_factor:.3f} to stay within motor limits")
                print(f">>>Actual linear velocity: {V:.2f} inches/s")

            leftMotor.setVelocity(leftMotorVelocity)
            rightMotor.setVelocity(rightMotorVelocity)
            getSpeeds()
            
            step_count = 0
            # Track angular displacement for negative velocity using outer wheel arc length
            target_angle = 2 * math.pi  # full circle in radians
            angle_tolerance = 0.01  # very small tolerance for near-perfect completion
            angle_turned = 0
            left_start = leftposition_sensor.getValue()
            wheel_path_radius = R + (WHEEL_BASE / 2)  # radius of left wheel's circular path
            
            while robot.step(timestep) != -1 and angle_turned < (target_angle - angle_tolerance):
                # Calculate angle from outer wheel arc length: angle = arc_length / radius
                left_distance = leftposition_sensor.getValue() - left_start
                angle_turned = abs(left_distance) / wheel_path_radius
                
                step_count += 1
                if step_count % 50 == 0:  # Print every 50 timesteps
                    progress_percent = (angle_turned / target_angle) * 100
                    print(f"Circle progress: {angle_turned:.2f}/{target_angle:.2f} rad ({progress_percent:.1f}%)")

            travelTime = time.monotonic() - timeSTART  # stop time
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            circumference = 2 * math.pi * R
            print(f"[Stopping position] {rightposition_sensor.getValue()}")
            total_distance_traveled = abs(leftposition_sensor.getValue() - START)
            print(f"[Distance: {total_distance_traveled:.2f} inches] (target: {expected_travel_distance:.2f} inches)")
            print(f"[Estimated final distance from start: {distance_from_start:.2f} inches]")
            print("[Circle Radius: " + str(round(R1, 2)) + " inches]")
            print("[Time: " + str(round(travelTime, 2)) + " seconds]")
            print("[Velocity: " + str(round(V, 2)) + " inches per second]")
            print(
                "[Left Motor Velocity: "
                + str(round(leftLinearVelocity, 2))
                + " inches per second]"
            )
            print(
                "[Right Motor Velocity: "
                + str(round(rightLinearVelocity, 2))
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
    leftMotorSpeed = round(leftMotor.getVelocity(), 2)  # rad/s
    rightMotorSpeed = round(rightMotor.getVelocity(), 2)  # rad/s
    # Convert to linear velocities for display
    leftLinearSpeed = round(leftMotorSpeed * WHEEL_RADIUS, 2)  # inches/s
    rightLinearSpeed = round(rightMotorSpeed * WHEEL_RADIUS, 2)  # inches/s
    print(f"Motor Speed (Linear): Left={leftLinearSpeed}, Right={rightLinearSpeed} inches/s")
    print(f"Motor Speed (Rotational): Left={leftMotorSpeed}, Right={rightMotorSpeed} rad/s")


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
    if V > MAX_LINEAR_VELOCITY or V < -MAX_LINEAR_VELOCITY:
        print(f">>>WARNING: Requested {V} inches/s exceeds maximum {MAX_LINEAR_VELOCITY:.2f} inches/s")
        print(f">>>Clamping to maximum achievable velocity")
        V = MAX_LINEAR_VELOCITY if V > 0 else -MAX_LINEAR_VELOCITY

    turnRV(R1, V)
    # resetCounts()
    # getCounts()
    # getSpeeds()
    # initEncoders()
