# Robotics Playground Skills

A collection of agent skills for robotic programming. These skills extend AI coding assistants with specialized capabilities for robotics development, simulation, and deployment.

## Overview

This repository provides reusable skills that help developers with common robotics programming tasks, including:

- Robot motion planning and control
- Sensor data processing and fusion
- Simulation environment setup
- ROS (Robot Operating System) integration
- Hardware interface programming
- Path planning and navigation algorithms

## Available Skills

Each skill is a self-contained module under `skills/`.

| Skill | What it does |
|---|---|
| [`cpp-embedded`](skills/cpp-embedded/) | Expert guidance for C/C++ on embedded systems and microcontrollers (STM32 / ESP32 / Arduino, FreeRTOS / Zephyr, bare-metal firmware, MISRA, debugging HardFaults). |
| [`cv-detection`](skills/cv-detection/) | Computer vision + object detection in ROS 2 with OpenCV, YOLO, TensorRT, OAK-D, RealSense — defect detection and depth-camera pipelines. |
| [`gazebo-sim`](skills/gazebo-sim/) | Gazebo Harmonic simulation for ROS 2: SDF worlds, sensor plugins, `ros_gz_bridge`, physics tuning, sim-to-real config portability. |
| [`isaac-rl`](skills/isaac-rl/) | NVIDIA Isaac Sim/Lab and reinforcement learning for robotics — PPO/SAC, reward design, sim-to-real, Unitree humanoids. |
| [`nav-slam`](skills/nav-slam/) | SLAM (`slam_toolbox`) + Nav2 navigation: costmaps, planners, controllers, AMCL, EKF, behavior trees, recovery behaviors. |
| [`ros2-design`](skills/ros2-design/) | **Pre-implementation design contract.** Takes a PRD + architecture + hardware/wiring doc and produces a complete `designs/` tree — per-node specs, custom interface specs, package-structure document, and 7 PlantUML data-flow + node-map diagrams (rendered to SVG). Use before writing any node code. |
| [`ros2-dev`](skills/ros2-dev/) | ROS 2 (Jazzy) development in Python (`rclpy`) and C++ (`rclcpp`) — topics/services/actions, QoS, lifecycle nodes, tf2, URDF, launch files, debugging DDS. |

## Getting Started

### Prerequisites

- A compatible AI coding assistant that supports agent skills (e.g., Claude Code)

### Installation

```bash
npx skills add robotics-playground/skills
```

### Usage

Refer to individual skill directories for specific usage instructions and documentation.

## Contributing

Contributions are welcome! To add a new skill:

1. Fork the repository
2. Create a new directory under `skills/` for your skill
3. Include a README with usage instructions
4. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
