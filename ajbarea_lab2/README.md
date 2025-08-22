# 🧭 Module 2: Autonomous Navigation with IMU Control

## 🎯 Overview

This module explores advanced autonomous navigation behaviors for an e-puck robot using IMU-based orientation control. You'll implement complex path-following algorithms including rectangular patterns, figure-8 trajectories, and oval waypoint navigation using precise orientation feedback.

## 📋 Module Structure

### 📐 Task 1: Rectangle Navigation (`lab2_task1.py`)

**🎯 Objective**: Navigate a precise rectangular path using IMU-controlled turns

**🔑 Key Concepts**:

- 🧭 IMU-based orientation control and feedback
- 📐 Precise 90° turn execution using yaw measurements
- 📏 Distance tracking with wheel encoders
- ⚡ Velocity validation and error handling

**⚙️ Parameters**:

- `H`: Rectangle height (20 inches)
- `W`: Rectangle width (40 inches)
- `X`: Linear velocity (must be ≤ 6.28 rad/s)
- `Y`: Time constraint (60 seconds)

### 🔄 Task 2: Figure-8 Circles (`lab2_task2.py`)

**🎯 Objective**: Execute two connected clockwise circles forming a figure-8 pattern

**🔑 Key Concepts**:

- 🔄 Differential wheel speeds for circular motion
- ⏪ Reverse motion control for first circle
- 🧭 IMU-guided transitions between circles
- ⚖️ Kinematic calculations for dual-radius trajectories

**⚙️ Parameters**:

- `R1`: First circle radius (10 inches) - executed in reverse
- `R2`: Second circle radius (14 inches) - executed forward
- `X`: Linear velocity (4.0 inches/second)
- `Y`: Time constraint (40 seconds)

### 🏁 Task 3: Oval Track Navigation (`lab2_task3.py`)

**🎯 Objective**: Navigate an oval "running track" with curved ends and straight sections

**🔑 Key Concepts**:

- 🔄 Combined circular and linear motion patterns
- 🧭 IMU-controlled transitions between motion types
- 📏 Waypoint-based navigation system
- ⚡ Continuous motion without stops

**⚙️ Parameters**:

- `R1`: Curve radius (10 inches)
- `D1`: Straight section distance (40 inches)
- `X`: Linear velocity (5.6 inches/second)
- `Y`: Time constraint (60 seconds)

## 🤖 Robot Specifications

- **🛞 Wheel Radius**: 0.8 inches
- **📐 Wheelbase**: 2.28 inches (distance between wheels)
- **⚡ Max Velocity**: 6.28 rad/s per wheel
- **🧭 IMU**: Inertial Measurement Unit for orientation feedback
- **📡 Sensors**: Wheel position encoders, IMU (roll, pitch, yaw)

## 🌐 World File

The simulation uses `webots/worlds/lab2.wbt` which includes:

- 🤖 e-puck robot with IMU and wheel encoders
- 🧭 Calibrated inertial measurement unit
- 🏟️ Open arena environment for complex path navigation
- 📊 Real-time orientation and position feedback

## 🎮 Running the Module

1. 🌐 Open Webots R2025a
2. 📁 Load `webots/worlds/lab2.wbt`
3. 🔧 Select desired controller:
   - `lab2_task1` for rectangle navigation
   - `lab2_task2` for figure-8 circles
   - `lab2_task3` for oval track navigation
4. ▶️ Run simulation and observe autonomous navigation

## 🧪 Experiment Guidelines

1. **⚙️ Adjust parameters** in the controller configuration sections
2. **🧭 Monitor IMU feedback** for orientation accuracy
3. **⏱️ Test time constraints** and velocity limits
4. **📊 Analyze path accuracy** against intended trajectories
5. **🔧 Fine-tune motion parameters** for optimal performance

## 🎓 Key Learning Outcomes

- 🧭 **IMU Integration**: Using inertial sensors for precise orientation control
- 📐 **Complex Path Planning**: Implementing multi-segment navigation algorithms
- ⚖️ **Differential Drive Control**: Advanced kinematic calculations for curved paths
- ⏱️ **Time-Constrained Navigation**: Optimizing motion within temporal limits
- 🔧 **Real-World Calibration**: Handling IMU coordinate transformations and scaling

## 🧭 IMU Coordinate System & Calibration

The IMU provides orientation feedback in radians within the range `[-π/2, π/2]`, requiring conversion to a full `[0°, 360°]` range:

```python
def degreeIMU():
    return (imu.getRollPitchYaw()[2] * (180 / pi)) % 360
```

### 🔧 Key Implementation Details

- **📐 Yaw Angle Conversion**: Transform from radians to degrees with modulo operation
- **🎯 Turn Precision**: Use `math.isclose()` for floating-point angle comparisons  
- **⚡ Velocity Validation**: Ensure all motor speeds stay within e-puck limits
- **⏱️ Time Management**: Calculate and validate motion timing constraints

## 📊 Advanced Navigation Techniques

### 📐 Rectangle Navigation Strategy

- **Phase 1**: Drive north (H/2 distance)
- **Phase 2**: Turn 90° east using IMU feedback
- **Phase 3**: Drive east (W distance)
- **Phase 4**: Continue pattern for complete rectangle
- **🔧 Precision**: IMU-controlled turns ensure accurate angles

### 🔄 Figure-8 Implementation

- **Reverse Circle**: First circle executed backwards with differential speeds
- **Forward Circle**: Second circle executed forward to complete figure-8
- **🧭 Transition**: IMU detects completion point for seamless motion change
- **⚖️ Speed Calculation**: `W = X/R`, then differential wheel speeds

### 🏁 Oval Track Execution  

- **Curved Sections**: Use circular motion kinematics
- **Straight Sections**: Linear motion with encoder feedback
- **🔄 Transitions**: IMU-guided switches between motion types
- **⚡ Continuous Motion**: No stops between segments

## 📁 Files Structure

```text
ajbarea_lab2/
├── docs/
│   └── 📄 ajbarea_lab2_report.pdf
├── webots/
│   ├── controllers/
│   │   ├── lab2_task1/
│   │   │   └── 🐍 lab2_task1.py
│   │   ├── lab2_task2/
│   │   │   └── 🐍 lab2_task2.py
│   │   └── lab2_task3/
│   │       └── 🐍 lab2_task3.py
│   └── worlds/
│       └── 🌐 lab2.wbt
└── 📖 README.md
```
