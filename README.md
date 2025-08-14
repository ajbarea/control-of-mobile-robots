# Control of Mobile Robots

A collection of mobile robotics control system implementations using Webots simulation environment. Learn robot navigation, localization, and autonomous behavior development through hands-on examples and practical implementations.

> **📝 Migration Notice**: Currently migrating source code from Webots R2022a to Webots R2025a. Some implementations may be in transition.

## Project Overview

Mobile robotics control systems design and implementation covering:

- Microcontroller programming and sensor integration
- Actuator control processes for precise movement
- Localization algorithms and navigation systems
- Autonomous behavior development
- Software interface design for robot control

[![Control of Mobile Robots](https://res.cloudinary.com/marcomontalbano/image/upload/v1671483418/video_to_markdown/images/youtube--MX-L8MLTDGI-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=MX-L8MLTDGI&list=PLmQVFU1FBDddYV_4IRW1zfXH6CAKuZjIM&index=2 "Control of Mobile Robots")

## Repository Structure

```text
├── ajbarea_lab1/          # Basic robot movement and control
├── ajbarea_lab2/          # Sensor integration and feedback systems
├── ajbarea_lab3/          # Navigation and obstacle avoidance
├── ajbarea_lab4/          # Advanced control algorithms
├── ajbarea_lab5/          # Localization techniques
├── ajbarea_lab6/          # Autonomous behavior systems
├── tests/                 # Unit tests and validation
└── lint.py               # Code quality tools
```

## Technology Stack

- **Simulation Environment**: Cyberbotics Webots R2025a
- **Programming Language**: Python 3.13+
- **Robot Platform**: e-puck differential drive robot
- **Control Systems**: PID controllers, state machines
- **Sensors**: Distance sensors, encoders, camera

## Quick Start

### Prerequisites

- Webots R2025a or later
- Python 3.13+
- Git

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/ajbarea/control-of-mobile-robots.git
   cd control-of-mobile-robots
   ```

2. Install development dependencies:

   ```bash
   pip install -e ".[testing]"
   ```

3. Open any module world file in Webots and run the corresponding controller.

## Tutorial Modules

### Module 1: Robot Fundamentals

- Basic movement control
- Velocity and angular velocity manipulation
- Circular and linear motion patterns

### Module 2: Sensor Integration

- Distance sensor calibration
- Feedback control systems
- Reactive behaviors

### Module 3: Navigation Systems

- Wall following algorithms
- Corridor navigation
- Maze solving strategies

### Module 4: Advanced Control

- PID controller implementation
- Trajectory following
- Precision movement control

### Module 5: Localization

- Odometry calculations
- Position estimation
- Sensor fusion techniques

### Module 6: Autonomous Behavior

- Decision-making algorithms
- Multi-task coordination
- Complex navigation scenarios

## Development

### Running Tests

```bash
python -m pytest tests/ -v
```

### Code Quality

```bash
python lint.py
```

### Webots Integration

Each module contains Webots world files (`.wbt`) and corresponding Python controllers. Open the world file in Webots and the controller will automatically load.

## Usage & Learning

This repository provides practical implementations of mobile robotics algorithms. Each module builds upon previous concepts, making it suitable for:

- Students learning robotics fundamentals
- Developers implementing mobile robot control systems  
- Researchers prototyping navigation algorithms
- Anyone interested in autonomous robot behavior

All implementations are thoroughly documented and tested.
