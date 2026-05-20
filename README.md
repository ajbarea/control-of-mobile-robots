<div align="center">

# 🤖 Control of Mobile Robots

### Mobile robotics in Webots: PID control, localization, and maze-solving with e-puck robots

*A six-module tutorial series with [YouTube walkthroughs](https://www.youtube.com/playlist?list=PLmQVFU1FBDddYV_4IRW1zfXH6CAKuZjIM) covering everything from basic differential drive to autonomous goal-seeking.*

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg?style=flat-square)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.13+-3776AB?style=flat-square&logo=python&logoColor=white)](https://python.org)
[![Webots](https://img.shields.io/badge/Webots-R2025a-blueviolet?style=flat-square)](https://cyberbotics.com)
[![YouTube](https://img.shields.io/badge/YouTube-tutorial%20series-FF0000?style=flat-square&logo=youtube&logoColor=white)](https://www.youtube.com/playlist?list=PLmQVFU1FBDddYV_4IRW1zfXH6CAKuZjIM)

[![Control of Mobile Robots](https://res.cloudinary.com/marcomontalbano/image/upload/v1671483418/video_to_markdown/images/youtube--MX-L8MLTDGI-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=MX-L8MLTDGI&list=PLmQVFU1FBDddYV_4IRW1zfXH6CAKuZjIM&index=2 "Control of Mobile Robots — watch the playlist")

</div>

---

## Project Overview

Hands-on mobile robotics control covering:

- Microcontroller programming and sensor integration
- Actuator control for precise movement
- Localization algorithms and navigation systems
- Autonomous behavior development
- Software interface design for robot control

## Repository Structure

```text
├── ajbarea_lab1/          # Basic robot movement and control
├── ajbarea_lab2/          # Sensor integration and feedback systems
├── ajbarea_lab3/          # Navigation and obstacle avoidance
├── ajbarea_lab4/          # Advanced control algorithms
├── ajbarea_lab5/          # Localization techniques
├── ajbarea_lab6/          # Autonomous behavior systems
├── tests/                 # Unit tests and validation
└── lint.py                # Code quality tools
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

```bash
git clone https://github.com/ajbarea/control-of-mobile-robots.git
cd control-of-mobile-robots
pip install -e ".[testing]"
```

Open any module's world file (`.wbt`) in Webots and run the corresponding controller.

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

## Audience

Practical implementations of mobile robotics algorithms. Each module builds on the previous one, suitable for:

- Students learning robotics fundamentals
- Developers implementing mobile robot control systems
- Researchers prototyping navigation algorithms
- Anyone interested in autonomous robot behavior

All implementations are documented and tested.

## License

[MIT](LICENSE)

---

<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://res.cloudinary.com/dumwa1w5x/image/upload/q_auto,f_auto,e_negate/v1779302138/brand_gwqy8l.png">
  <img src="https://res.cloudinary.com/dumwa1w5x/image/upload/q_auto,f_auto/v1779302138/brand_gwqy8l.png" alt="" height="16" />
</picture>&nbsp;&nbsp;2026 <a href="https://ajbarea.github.io/">AJ Barea</a>

</div>
