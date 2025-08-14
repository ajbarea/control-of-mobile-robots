# 🤖 Lab 1: Mobile Robot Kinematics

## 🎯 Overview

This lab explores fundamental concepts of mobile robot kinematics using the E-puck robot in Webots simulation. Students implement and test straight-line motion control and circular trajectory following using differential drive kinematics.

## 📋 Lab Structure

### 🚀 Task 1: Straight-Line Motion Control (`lab1_task1.py`)

- **🎯 Objective**: Move the robot in a straight line for a specified distance at a given velocity
- **🧠 Key Concepts**:
  - ⚙️ Velocity control using wheel motors
  - 📡 Position feedback from wheel encoders
  - 📏 Distance measurement and tracking
- **⚡ Parameters**:
  - `X`: Target distance in inches
  - `V`: Linear velocity in inches/second (range: -6.28 to 6.28)

### 🔄 Task 2: Circular Motion Control (`lab1_task2.py`)

- **🎯 Objective**: Execute circular trajectories using differential drive kinematics
- **🧠 Key Concepts**:
  - ⚖️ Differential wheel speed calculation
  - 🌀 Angular velocity relationships: `W = V / R`
  - 🧮 Kinematic equations for curved motion
- **⚡ Parameters**:
  - `R1`: Circle radius in inches (0 = turn in place)
  - `V`: Linear velocity in inches/second

## 🔧 Robot Specifications

- **🛞 Wheel Radius**: 0.8 inches
- **📐 Wheelbase**: 2.28 inches (distance between wheels)
- **⚡ Max Velocity**: 6.28 rad/s per wheel

## 🌍 World File

The simulation uses `lab1.wbt` which includes:

- 🤖 E-puck robot with wheel encoders
- 📶 Distance sensors (front, left, right) - configured for Webots R2025a
- 🏟️ Basic rectangular arena environment

## 🚀 Running the Lab

1. 🔧 Open Webots R2025a
2. 📂 Load `webots/worlds/lab1.wbt`
3. ▶️ The simulation starts with Task 1 controller by default
4. 🔄 To switch to Task 2:
   - ⏹️ Stop simulation
   - ⚙️ Change controller in robot properties from "lab1_task1" to "lab1_task2"
   - 🔄 Restart simulation

## 🧪 Experiment Guidelines

1. **⚙️ Modify parameters** in the designated sections of each controller
2. **🏃‍♂️ Test different velocities** within the motor limits (-6.28 to 6.28)
3. **👀 Observe encoder feedback** and actual vs. expected performance
4. **📊 Document results** comparing theoretical and measured values

## 🎓 Key Learning Outcomes

- 📡 Understanding of encoder-based position feedback
- ⚙️ Implementation of velocity control for mobile robots
- 🧮 Application of differential drive kinematics
- 📈 Analysis of motion accuracy and error sources
- 🔧 Real-world calibration and empirical corrections

## ⚗️ Theory vs. Reality: Why Empirical Factors Matter

While theoretical kinematics provide the foundation for robot motion control, real-world implementations require **empirical corrections** due to several physical factors:

### 🏗️ **Physical Limitations**

- **🛞 Wheel Slippage**: Wheels don't grip perfectly, especially during turns
- **⚙️ Mechanical Play**: Gear backlash and mounting tolerances affect precision
- **🔧 Manufacturing Variations**: Actual wheel radius and wheelbase differ slightly from specifications

### 📊 **Sensor & Control Issues**

- **📡 Encoder Quantization**: Discrete encoder ticks vs. continuous motion
- **⏱️ Control Loop Dynamics**: Acceleration/deceleration effects during start/stop
- **🕐 Timing Precision**: Simulation timestep discretization

### 🧮 **Mathematical Approximations**

- **📏 Distance Measurement**: Using one wheel to estimate robot center motion
- **🌀 Circular Motion**: Assuming perfect differential drive kinematics
- **📐 Coordinate Frames**: Differences between theoretical and actual robot geometry

### 💡 **Lab Implementation**

In **Task 2 (Circular Motion)**, we use an **empirical scaling factor** (~1.22) to correct the theoretical travel distance:

```python
# Theoretical: Left wheel travels 2π(R + wheelbase/2)
theoretical_distance = 2 * math.pi * (R + WHEEL_BASE/2)

# Reality: Need extra distance due to physical factors
empirical_factor = 1.25  # Determined through testing (R=6,15)
actual_distance_needed = theoretical_distance * empirical_factor
```

This demonstrates the **essential engineering principle**: **theory guides design, but empirical testing ensures performance**. 🎯

## 🧪 Experimental Results Summary

### 📊 Task 1: Straight-Line Motion Performance
- **Accuracy**: High precision for distances > 6 inches
- **Velocity Control**: Consistent performance across velocity range
- **Encoder Feedback**: Reliable distance measurement using wheel encoders
- **Key Finding**: Motion accuracy improves with longer distances due to reduced impact of acceleration/deceleration phases

### 📊 Task 2: Circular Motion Performance  
- **Circle Completion**: Excellent trajectory following with empirical factor (1.25)
- **Differential Drive**: Successful implementation of kinematic equations
- **Turn-in-Place**: Precise rotation using opposite wheel velocities
- **Key Finding**: Real-world physics require calibration factors beyond theoretical calculations

### 🔧 Implementation Highlights
- **Robust Input Validation**: Parameter checking with helpful warnings
- **Velocity Clamping**: Automatic scaling to stay within motor limits
- **Progress Tracking**: Real-time feedback during motion execution
- **Error Handling**: Graceful handling of edge cases and invalid inputs

## 📁 Files

```text
ajbarea_lab1/
├── 📚 docs/
│   └── 📄 ajbarea_lab1_report.pdf - Lab report with experimental results
├── 🤖 webots/
│   ├── 🎮 controllers/
│   │   ├── lab1_task1/
│   │   │   └── 🐍 lab1_task1.py - Straight-line motion controller
│   │   └── lab1_task2/
│   │       └── 🐍 lab1_task2.py - Circular motion controller
│   └── 🌍 worlds/
│       └── 🗺️ lab1.wbt - Webots simulation world
└── 📖 README.md - This file
```

---

✨ **Happy coding and experimenting with mobile robot kinematics!** 🤖✨
