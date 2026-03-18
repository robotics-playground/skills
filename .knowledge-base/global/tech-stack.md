---
title: "Tech Stack Reference"
category: "global"
tags: ["tech-stack", "infrastructure", "frameworks", "tooling", "architecture", "robotics", "ros2", "python", "gazebo", "jetson"]
description: "Robotics project technical stack including ROS 2, simulation, perception, navigation, and AI/RL frameworks"
last_updated: "2026-03-18"
---

## Tech Stack

### Robot Middleware & Runtime

- **Middleware:** ROS 2 Jazzy Jalisco (LTS, EOL 2029)
- **Primary Language:** Python 3.11 (rclpy)
- **Secondary Language:** C++17 (rclcpp) — for performance-critical nodes only
- **Build System:** colcon + ament_python / ament_cmake
- **DDS Implementation:** Cyclone DDS (default for Jazzy) or Fast-DDS
- **Package Manager:** rosdep + pip (Python), apt (system)

### Simulation

- **Primary Simulator:** Gazebo Harmonic (Gazebo 8, paired with Jazzy)
- **RL Training Simulator:** NVIDIA Isaac Sim 4.x / Isaac Lab
- **Scene Format:** SDF (Gazebo), USD (Isaac Sim)
- **ROS-Gazebo Bridge:** ros_gz (ros_gz_bridge, ros_gz_sim, ros_gz_image)
- **Visualization:** RViz2, rqt, Foxglove Studio

### Navigation & SLAM

- **SLAM:** slam_toolbox (online and async modes)
- **Navigation:** Nav2 (Navigation2 stack)
- **Localization:** AMCL (known maps), robot_localization (EKF sensor fusion)
- **Map Server:** nav2_map_server

### Perception & Computer Vision

- **Image Processing:** OpenCV 4.x (cv_bridge, image_transport)
- **Object Detection:** YOLOv8/YOLOv11 (Ultralytics)
- **Inference Runtime:** TensorRT (Jetson), ONNX Runtime (development)
- **Depth Processing:** DepthAI (OAK-D Lite), depth_image_proc
- **Point Clouds:** PCL (Point Cloud Library)

### AI & Reinforcement Learning

- **RL Framework:** Isaac Lab (PPO via rsl_rl / rl_games)
- **ML Framework:** PyTorch 2.x
- **Experiment Tracking:** TensorBoard, Weights & Biases (optional)

### Hardware Platforms

- **Mobile Robot:** [Your robot platform — e.g., tracked chassis, differential drive, Ackermann]
- **Humanoid (optional):** [e.g., Unitree G1, H1, or other humanoid platform]
- **Edge Compute:** NVIDIA Jetson Orin NX/Nano (JetPack 7, L4T)
- **Sensors:** [e.g., RPLiDAR A1/A2, OAK-D Lite, IMU, GPS]
- **Development PC:** Ubuntu 24.04 LTS (Noble Numbat)

### Development Environment

- **OS:** Ubuntu 24.04 LTS (native or WSL2)
- **Containerization:** Docker (NVIDIA Container Toolkit for Jetson/GPU)
- **IDE:** VS Code + ROS extension
- **Version Control:** Git (this repository, branch-per-experiment model)

### Testing & Quality

- **Python Testing:** pytest, launch_testing
- **C++ Testing:** gtest (via ament_cmake_gtest)
- **Linting:** flake8, ament_flake8, ament_cpplint
- **Type Checking:** mypy (Python)
- **CI/CD:** GitHub Actions with colcon test

### Third-Party ROS 2 Packages

- **Robot Description:** robot_state_publisher, joint_state_publisher
- **Transforms:** tf2_ros, tf2_geometry_msgs
- **Teleop:** teleop_twist_keyboard, teleop_twist_joy
- **Diagnostics:** diagnostic_updater, diagnostic_aggregator
- **Recording:** ros2bag (rosbag2)
