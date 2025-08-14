"""lab6_task1 controller."""

import math
import sys

from controller import Robot

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


class Cell:
    def __init__(self, num, n, s, e, w, value, visited, neighbors):
        self.num = num
        self.n = n
        self.s = s
        self.e = e
        self.w = w
        self.value = value
        self.visited = visited
        self.neighbors = neighbors


class Pose:
    def __init__(self, x, y, n):
        self.x = x
        self.y = y
        self.n = n


planner = []
cells = []
path = []
visited_cells = []

robot_pose = Pose(-1, -1, -1)
for i in range(16):
    visited_cells.append(".")


# [Visited, North, East, South, West]
grid = [
    [0, 1, 0, 0, 1],
    [0, 1, 0, 0, 0],
    [0, 1, 0, 0, 0],
    [0, 1, 1, 0, 0],  # starting grid
    [0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 1, 1],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 1, 1, 0],
]


def printMaze(maze):
    print("__________________________________")
    for i in range(4):
        x = i * 4
        if maze[x][0] == 0:
            v1 = "?"
        else:
            v1 = "V"
        if maze[x + 1][0] == 0:
            v2 = "?"
        else:
            v2 = "V"
        if maze[x + 2][0] == 0:
            v3 = "?"
        else:
            v3 = "V"
        if maze[x + 3][0] == 0:
            v4 = "?"
        else:
            v4 = "V"
        print(
            "|  "
            + str(maze[x][1])
            + "\t  "
            + str(maze[x + 1][1])
            + "\t  "
            + str(maze[x + 2][1])
            + "\t  "
            + str(maze[x + 3][1])
            + "    |"
        )
        print(
            "|"
            + str(maze[x][4])
            + " "
            + v1
            + " "
            + str(maze[x][2])
            + "\t"
            + str(maze[x + 1][4])
            + " "
            + v2
            + " "
            + str(maze[x + 1][2])
            + "\t"
            + str(maze[x + 2][4])
            + " "
            + v3
            + " "
            + str(maze[x + 2][2])
            + "\t"
            + str(maze[x + 3][4])
            + " "
            + v4
            + " "
            + str(maze[x + 3][2])
            + "  |"
        )
        print(
            "|  "
            + str(maze[x][3])
            + "\t  "
            + str(maze[x + 1][3])
            + "\t  "
            + str(maze[x + 2][3])
            + "\t  "
            + str(maze[x + 3][3])
            + "    |"
        )
        if i == 3:
            print("|_________________________________|\n")
        else:
            print("|                                 |")


def setPose(n):
    visited_cells[n - 1] = "X"
    robot_pose.n = n


# initialize grid for robot traversal
def Setup():  # 0 1 2 3 4 5 6 7 8
    known_grid = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1],  # 0
        [1, 0, 0, 0, 0, 0, 0, 0, 1],  # 1
        [1, 1, 1, 1, 1, 0, 1, 0, 1],  # 2
        [1, 0, 0, 0, 1, 0, 1, 0, 1],  # 3
        [1, 0, 1, 0, 1, 0, 1, 0, 1],  # 4
        [1, 0, 1, 0, 0, 0, 1, 0, 1],  # 5
        [1, 0, 1, 1, 1, 1, 1, 0, 1],  # 6
        [1, 0, 0, 0, 0, 0, 0, 0, 1],  # 7
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
    ]  # 8

    ########################################################################################
    known_grid[5][3] = 2  # set GOAL
    # START position is at grid[7][7]
    ########################################################################################
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print("                 Wavefront Planner")
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    for row in known_grid:
        print(row)
    print(">> initial grid")
    return known_grid


# set start and goal | calls function to print path plan
def PathFinder():
    ########################################################################################
    goal_cell = 48  # set GOAL
    starting_cell = 70  # set START
    ########################################################################################

    if starting_cell == goal_cell:
        print(" ")
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        print(" ")
        print("   :)                            GOOOOOOAAAALLLLLL!!!!")
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        print(" ")
        sys.exit(0)
    print(" ")
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print("                  Planned Path")
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print(" ")
    new_index = PathPrint(starting_cell)
    while new_index != goal_cell:
        new_index = PathPrint(new_index)


# uses the information in the planner to update the cells object list
def CreatePlanner(planner):
    counter = 0
    for i in range(len(planner)):
        for j in range(len(planner)):
            if planner[i][j] != 1:
                up = planner[i - 1][j]
                down = planner[i + 1][j]
                left = planner[i][j - 1]
                right = planner[i][j + 1]
                new_cell_object = Cell(
                    counter, up, down, right, left, planner[i][j], False, []
                )
                cells.append(new_cell_object)
            else:
                cells.append(Cell(counter, -1, -1, -1, -1, -1, False, []))
            counter = counter + 1


# search planner for any 0's to see if wavefront planning is finished
def SearchForZeroes(planner):
    for i in range(len(planner)):
        for j in range(len(planner[i])):
            if planner[i][j] == 0:
                return False
    else:
        return True


# calculate and print the new grid at each step of wavefront planning
def IncrementNeighbors(planner):
    counter = 0
    index = 0
    done = False
    previous_maximum = 2
    maximum = 2
    while not done:
        done = SearchForZeroes(planner)
        for i in range(len(planner)):
            for j in range(len(planner[i])):
                index = index + 1
                if planner[i][j] == maximum:
                    if planner[i - 1][j] != 1 and planner[i - 1][j] != previous_maximum:
                        planner[i - 1][j] = planner[i][j] + 1
                    if planner[i + 1][j] != 1 and planner[i + 1][j] != previous_maximum:
                        planner[i + 1][j] = planner[i][j] + 1
                    if planner[i][j + 1] != 1 and planner[i][j + 1] != previous_maximum:
                        planner[i][j + 1] = planner[i][j] + 1
                    if planner[i][j - 1] != 1 and planner[i][j - 1] != previous_maximum:
                        planner[i][j - 1] = planner[i][j] + 1
        previous_maximum = maximum
        maximum = maximum + 1
        done = SearchForZeroes(planner)
        print(" ")
        for row in planner:
            print(row)
        counter = counter + 1
        print(">> step", counter)


# update all of a cell's neighbor values
def UpdateNeighbors():
    east_column = [8, 17, 26, 35, 44, 53, 62, 71, 80]
    west_column = [0, 9, 18, 27, 36, 45, 54, 63, 72]
    for i in range(len(cells)):
        if i > 8:
            cells[i].n = cells[i - 9].value
        if i not in east_column:
            cells[i].e = cells[i + 1].value
        if i < 72:
            cells[i].s = cells[i + 9].value
        if i not in west_column:
            cells[i].w = cells[i - 1].value


# use planner to update cell values
def UpdateCellValues(planner):
    index = 0
    for i in range(len(planner)):
        for j in range(len(planner[i])):
            cells[index].value = planner[i][j]
            index = index + 1
    UpdateNeighbors()
    PathFinder()


# print path plan before robot motion begins
def PathPrint(current):
    print(">> from cell", cells[current].num, end=" ")
    if cells[current].n == cells[current].value - 1:
        print("drive UP to cell", end=" ")
        current = current - 9
        print(cells[current].num)
        path.append("UP")
    elif cells[current].e == cells[current].value - 1:
        print("drive RIGHT to cell", end=" ")
        current = current + 1
        print(cells[current].num)
        path.append("RIGHT")
    elif cells[current].s == cells[current].value - 1:
        print("drive DOWN to cell", end=" ")
        current = current + 9
        print(cells[current].num)
        path.append("DOWN")
    elif cells[current].w == cells[current].value - 1:
        print("drive LEFT to cell", end=" ")
        current = current - 1
        print(cells[current].num)
        path.append("LEFT")
    return current


# call all initialization functions for grid, planner and cells
def Initialize():
    global path
    planner = Setup()
    CreatePlanner(planner)
    IncrementNeighbors(planner)
    UpdateCellValues(planner)
    path = RobotController()


# controls robot turning based on generated plan and cardinal direction
def RobotMotion(d, m):
    if (
        d == "W"
        and m == "UP"
        or d == "N"
        and m == "RIGHT"
        or d == "E"
        and m == "DOWN"
        or d == "S"
        and m == "LEFT"
    ):
        turn(-90)
    elif (
        d == "W"
        and m == "DOWN"
        or d == "N"
        and m == "LEFT"
        or d == "E"
        and m == "UP"
        or d == "S"
        and m == "RIGHT"
    ):
        turn(90)
    elif (
        d == "W"
        and m == "RIGHT"
        or d == "N"
        and m == "DOWN"
        or d == "E"
        and m == "LEFT"
        or d == "S"
        and m == "UP"
    ):
        turn(180)


def ObstacleAvoidance():
    left = getDistanceSensors()[0]
    front = getDistanceSensors()[1]
    right = getDistanceSensors()[2]
    L = 1 if left < 7 else 0
    F = 1 if front < 7 else 0
    R = 1 if right < 7 else 0
    # print('L(', L,') F(', F,') R(', R,')')
    return (L, F, R)


def PrintGrid():
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
        robot_pose.x,
        ", y =",
        robot_pose.y,
        ", Grid Cell #",
        robot_pose.n,
        ",",
        round(getIMUDegrees(), 1),
        "° )",
    )
    print("---------------------------------------------------------------------")


def CheckWalls(L, F, R):
    getPositionSensors()[
        0
    ]  # position sensor reading (unused but called for side effects)
    # facing west
    if getDirectionFacing() == "W":
        if not L and not F and R:  # OOW
            if visited_cells[2] == ".":
                setPose(3)
        if L and not F and R:  # WOW
            if visited_cells[1] == ".":
                setPose(2)
            elif visited_cells[14] == ".":
                setPose(15)
            elif visited_cells[13] == ".":
                setPose(14)
        if L and F and R:  # WWW
            if visited_cells[0] == ".":
                setPose(1)
        if L and F and not R:  # WWO
            if visited_cells[12] == ".":
                setPose(13)
    # facing north
    elif getDirectionFacing() == "N":
        if not L and not F and R:  # OOW
            if visited_cells[15] == ".":
                setPose(16)
            elif visited_cells[10] == ".":
                setPose(11)
        if L and not F and R:  # WOW
            if visited_cells[11] == ".":
                setPose(12)
            elif visited_cells[7] == ".":
                setPose(8)
            elif visited_cells[8] == ".":
                setPose(9)
            elif visited_cells[6] == ".":
                setPose(7)
        if not L and F and R:  # OWW
            if visited_cells[3] == ".":
                setPose(4)
        if L and F and not R:  # WWO
            if visited_cells[4] == ".":
                setPose(5)
    # facing east
    elif getDirectionFacing() == "E":
        if L and F and not R:  # WWO
            if visited_cells[5] == ".":
                setPose(6)
        if not L and F and R:  # OOW
            if visited_cells[10] == ".":
                setPose(11)
    # facing south
    elif getDirectionFacing() == "S":
        if not L and F and R:  # OWW
            if visited_cells[9] == ".":
                setPose(10)

    FindCoordinates(robot_pose.n)
    UpdateMaze(robot_pose.n, L, F, R, getDirectionFacing())


# store every other grid cell movement in wavefront plan for 4x4 robot map
def RobotController():
    every_other_step = []
    for i in range(len(path)):
        if i % 2 == 0:
            every_other_step.append(path[i])
    return every_other_step


def getDirectionFacing():
    degrees = getIMUDegrees()
    if degrees < 45 or degrees > 315:
        return "S"
    if degrees > 45 and degrees < 135:
        return "E"
    if degrees > 135 and degrees < 225:
        return "N"
    if degrees > 225 and degrees < 315:
        return "W"
    return "?"


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


def turn(degree):  # turns to face another cardinal direction.
    dir = getDirectionFacing()
    if dir == "S":
        target = 0 + degree
    elif dir == "E":
        target = 90 + degree
    elif dir == "N":
        target = 180 + degree
    elif dir == "W":
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
            if (
                abs(lowerBound - getIMUDegrees()) < 30
                or abs(upperBound - getIMUDegrees()) < 30
            ):
                setSpeedsIPS(-1, 1)
                if degree < 0:  # if we are turning to the left
                    setSpeedsIPS(1, -1)
    setSpeedsIPS(0, 0)


def UpdateMaze(n, L, F, R, direction):
    grid[n - 1][0] = 1  # mark cell visited
    # mark internal walls based on sensor readings
    if direction == "N":
        grid[n - 1][4] = L  # LEFT sensor = WEST wall
        grid[n - 1][1] = F  # FRONT sensor = NORTH wall
        grid[n - 1][2] = R  # RIGHT sensor = WEST wall
    elif direction == "E":
        grid[n - 1][1] = L  # LEFT sensor = NORTH wall
        grid[n - 1][2] = F  # FRONT sensor = EAST wall
        grid[n - 1][3] = R  # RIGHT sensor = SOUTH wall
    elif direction == "S":
        grid[n - 1][2] = L  # LEFT sensor = EAST wall
        grid[n - 1][3] = F  # FRONT sensor = SOUTH wall
        grid[n - 1][4] = R  # RIGHT sensor = WEST wall
    elif direction == "W":
        grid[n - 1][3] = L  # LEFT sensor = SOUTH wall
        grid[n - 1][4] = F  # FRONT sensor = WEST wall
        grid[n - 1][1] = R  # RIGHT sensor = NORTH wall


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


def getIMUDegrees():  # convert radians to degrees and make range 180
    return imu.getRollPitchYaw()[2] * 180 / math.pi + 180


def getPositionSensors():  # function for getting getting position sensors in inches. convert from radians
    return [leftposition_sensor.getValue() * 0.8, rightposition_sensor.getValue() * 0.8]


def getDistanceSensors():  # function for getting distance sensors in inches. convert from meters
    return [
        leftDistanceSensor.getValue() * 39.3701,
        frontDistanceSensor.getValue() * 39.3701,
        rightDistanceSensor.getValue() * 39.3701,
    ]


# implement and print robot motions generated by wavefront planning
def FollowPath(path):
    print(" ")
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print("                        Robot Motion to Goal")
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print(" ")
    print("Shortest Path =>", path)
    for motion in path:  # FIXME ADD REAL ROBOT POSE
        readings = ObstacleAvoidance()
        CheckWalls(readings[0], readings[1], readings[2])
        direction = getDirectionFacing()
        # PrintGrid()
        # printMaze(grid)
        print(" ")
        print(round(getIMUDegrees()), "°", direction, "| driving", motion, "...")
        RobotMotion(direction, motion)
        Forward(10)
    print(" ")
    print("   :)                                           GOOOOOOAAAALLLLLL!!!!")
    print("---------------------------------------------------------------------")
    print(" ")


while robot.step(timestep) != -1:
    Initialize()
    FollowPath(path)
    sys.exit(0)
