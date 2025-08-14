"""lab5_task1 controller."""

import math
import sys

from controller import Robot

sys.setrecursionlimit(10000)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# enable distance sensors
frontDistanceSensor = robot.getDevice("front_ds")
leftDistanceSensor = robot.getDevice("left_ds")
rightDistanceSensor = robot.getDevice("right_ds")
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# getting the position sensors
leftposition_sensor = robot.getDevice("left wheel sensor")
rightposition_sensor = robot.getDevice("right wheel sensor")
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

# enable camera and recognition
camera = robot.getDevice("camera1")
camera.enable(timestep)
camera.recognitionEnable(timestep)

# enable imu
imu = robot.getDevice("inertial unit")
imu.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# global arrays
centered_scans = []
pillars = []
visited_cells = []


# Pillar objects to keep track of all data
class Pillar:
    def __init__(self, color, x, y, ID, dist, radius):
        self.color = color
        self.x = x
        self.y = y
        self.ID = ID
        self.dist = dist
        self.radius = radius


p06 = Pillar("Red", 20, 20, -1, -1, -1)
p14 = Pillar("Yellow", -20, 20, -1, -1, -1)
p30 = Pillar("Blue", 20, -20, -1, -1, -1)
p22 = Pillar("Green", -20, -20, -1, -1, -1)
robotPose = Pillar("ePuck", -1, -1, -1, -1, -1)  # borrow pillar struct for pose data

for i in range(16):
    visited_cells.append(".")


def getIMUDegrees():  # convert radians to degrees and make range 180
    return imu.getRollPitchYaw()[2] * 180 / math.pi + 180


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


def getDistanceSensors():  # function for getting distance sensors in inches. convert from meters
    return [
        round(leftDistanceSensor.getValue() * 39.3701, 8),
        round(frontDistanceSensor.getValue() * 39.3701, 8),
        round(rightDistanceSensor.getValue() * 39.3701, 8),
    ]


def getPositionSensors():  # function for getting getting position sensors in inches. convert from radians
    return [leftposition_sensor.getValue() * 0.8, rightposition_sensor.getValue() * 0.8]


def Forward(
    distance,
):  # distance in inches, no time requirement, run max speed ~ 5 inches
    setSpeedsIPS(5, 5)
    startDistance = getPositionSensors()
    curDistance = getPositionSensors()
    while (
        robot.step(timestep) != -1
        and abs(curDistance[0] - startDistance[0]) < distance
        and abs(curDistance[1] - startDistance[1]) < distance
    ):
        curDistance = getPositionSensors()
    setSpeedsIPS(0, 0)


def ScanEnvironment():  # return object position, orientiation, size
    setSpeedsIPS(1, -1)
    object_array = camera.getRecognitionObjects()
    if len(object_array) > 0:
        object = camera.getRecognitionObjects()[0]
        image_position = object.get_position_on_image()
        if image_position[0] == 39:
            centered_scans.append(object.get_id())
            if object.get_id() == 6 or object.get_id() == 5:
                p06.dist = getDistanceSensors()[1]
                p06.ID = object.get_id()
                pillars.append(p06)
            elif object.get_id() == 14 or object.get_id() == 13:
                p14.dist = getDistanceSensors()[1]
                p14.ID = object.get_id()
                pillars.append(p14)
            elif object.get_id() == 30 or object.get_id() == 29:
                p30.dist = getDistanceSensors()[1]
                p30.ID = object.get_id()
                pillars.append(p30)
            elif object.get_id() == 22 or object.get_id() == 21:
                p22.dist = getDistanceSensors()[1]
                p22.ID = object.get_id()
                pillars.append(p22)


def findGridCell(x, y):
    column = -1
    row = -1
    gridCell = -1
    # find column
    if x > -25 and x < -10:
        column = 1
    elif x > -10 and x < 0:
        column = 2
    elif x > 0 and x < 10:
        column = 3
    elif x > 10 and x < 25:
        column = 4
    # find row
    if y > -25 and y < -10:
        row = 4
    elif y > -10 and y < 0:
        row = 3
    elif y > 0 and y < 10:
        row = 2
    elif y > 10 and y < 25:
        row = 1
    # find Grid Cell
    if column == 1 and row == 1:  # column 1
        gridCell = 1
    elif column == 1 and row == 2:
        gridCell = 5
    elif column == 1 and row == 3:
        gridCell = 9
    elif column == 1 and row == 4:
        gridCell = 13
    elif column == 2 and row == 1:  # column 2
        gridCell = 2
    elif column == 2 and row == 2:
        gridCell = 6
    elif column == 2 and row == 3:
        gridCell = 10
    elif column == 2 and row == 4:
        gridCell = 14
    elif column == 3 and row == 1:  # column 3
        gridCell = 3
    elif column == 3 and row == 2:
        gridCell = 7
    elif column == 3 and row == 3:
        gridCell = 11
    elif column == 3 and row == 4:
        gridCell = 15
    elif column == 4 and row == 1:  # column 4
        gridCell = 4
    elif column == 4 and row == 2:
        gridCell = 8
    elif column == 4 and row == 3:
        gridCell = 12
    elif column == 4 and row == 4:
        gridCell = 16

    robotPose.x = x
    robotPose.y = y
    robotPose.ID = gridCell
    return gridCell


def robotTrilateration():
    for pillar in pillars:  # r^2 = distance^2
        pillar.radius = pillar.dist**2

    # Ax + By = C                 # Dx + Ey = F
    A = (-2 * pillars[0].x) + (2 * pillars[1].x)
    B = (-2 * pillars[0].y) + (2 * pillars[1].y)
    C = (
        (pillars[0].radius)
        - (pillars[1].radius)
        - (pillars[0].x ** 2)
        + (pillars[1].x ** 2)
        - (pillars[0].y ** 2)
        + (pillars[1].y ** 2)
    )
    D = (-2 * pillars[1].x) + (2 * pillars[2].x)
    E = (-2 * pillars[1].y) + (2 * pillars[2].y)
    F = (
        pillars[1].radius
        - pillars[2].radius
        - (pillars[1].x ** 2)
        + (pillars[2].x ** 2)
        - (pillars[1].y ** 2)
        + (pillars[2].y ** 2)
    )
    if (E * A) != (B * D):
        robotX = (C * E - F * B) / (E * A - B * D)
        robotY = (C * D - A * F) / (B * D - A * E)
    else:
        print("ERROR - DivideByZero: (E * A) = (B * D)")
    return findGridCell(robotX, robotY)


def visitedCells(cell):
    if visited_cells[cell - 1] != "X":
        visited_cells[cell - 1] = "X"
        print(" ")
        print(" ")
        print(" ")
        print("---------------------------------------------------------------------")
        print(" Visited Cells")
        print(
            "  |",
            visited_cells[0],
            visited_cells[1],
            visited_cells[2],
            visited_cells[3],
            "|",
        )
        print(
            "  |",
            visited_cells[4],
            visited_cells[5],
            visited_cells[6],
            visited_cells[7],
            "|",
        )
        print(
            "  |",
            visited_cells[8],
            visited_cells[9],
            visited_cells[10],
            visited_cells[11],
            "|",
        )
        print(
            "  |",
            visited_cells[12],
            visited_cells[13],
            visited_cells[14],
            visited_cells[15],
            "|",
        )
        print(" ")
        print(" Robot Pose |", end="")
        print(
            " ( x =",
            round(robotPose.x, 2),
            ", y =",
            round(robotPose.y, 2),
            ", Grid Cell #",
            robotPose.ID,
            ",",
            round(getIMUDegrees(), 1),
            "° )",
        )
        print("---------------------------------------------------------------------")

    if "." not in visited_cells:  # exit condition
        setSpeedsIPS(0, 0)
        sys.exit(0)
    return


def nextColumn():  # spin then move left 1
    setSpeedsIPS(1, -1)
    pos = leftposition_sensor.getValue()
    while robot.step(timestep) != -1:
        # print(pos)
        if getIMUDegrees() > 270 and getIMUDegrees() < 271:  # face west
            while (
                robot.step(timestep) != -1 and leftposition_sensor.getValue() < 14 + pos
            ):
                # print(pos)
                if leftposition_sensor.getValue() > 383:
                    visitedCells(2)
                setSpeedsIPS(3, 3)
                if getDistanceSensors()[1] < 3:  # front wall
                    nextRow()
                    ScanEnvironment()
                    cell = robotTrilateration()
                    visitedCells(cell)
        ScanEnvironment()
        if len(centered_scans) > 0 and len(centered_scans) % 3 == 0:
            cell = robotTrilateration()
            visitedCells(cell)
            nextColumn()


def nextRow():  # move down 1 and right 4
    setSpeedsIPS(1, -1)
    position = leftposition_sensor.getValue()
    while robot.step(timestep) != -1:
        if getIMUDegrees() > 0 and getIMUDegrees() < 2:  # face south
            while (
                robot.step(timestep) != -1
                and leftposition_sensor.getValue() < 10 + position
            ):
                # print(leftposition_sensor.getValue(), 'next row')
                if leftposition_sensor.getValue() > 200:
                    visitedCells(3)
                setSpeedsIPS(3, 3)
        setSpeedsIPS(-1, 1)
        if getIMUDegrees() > 90 and getIMUDegrees() < 91:
            Forward(30)
            return


while robot.step(timestep) != -1:
    ScanEnvironment()
    if len(centered_scans) > 0 and len(centered_scans) % 3 == 0:
        nextColumn()
