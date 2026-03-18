---
stepsCompleted: [1]
inputDocuments: []
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: 'ROS 2 Node Architecture Best Practices'
research_goals: 'Research best practices for architecture of ROS2 nodes, topics, subscribers, publishers, and overall system design patterns'
user_name: 'TanNT'
date: '2026-03-18'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-03-18
**Author:** TanNT
**Research Type:** technical

---

## Research Overview

[Research overview and methodology will be appended here]

---

## Technical Research Scope Confirmation

**Research Topic:** ROS 2 Node Architecture Best Practices
**Research Goals:** Research best practices for architecture of ROS2 nodes, topics, subscribers, publishers, and overall system design patterns

**Technical Research Scope:**

- Architecture Analysis - design patterns, frameworks, system architecture
- Implementation Approaches - development methodologies, coding patterns
- Technology Stack - languages, frameworks, tools, platforms
- Integration Patterns - APIs, protocols, interoperability
- Performance Considerations - scalability, optimization, patterns

**Research Methodology:**

- Current web data with rigorous source verification
- Multi-source validation for critical technical claims
- Confidence level framework for uncertain information
- Comprehensive technical coverage with architecture-specific insights

**Scope Confirmed:** 2026-03-18

## Technology Stack Analysis

### Programming Languages

ROS 2 is built on a layered client library architecture with the core `rcl` library written in **C**, providing the foundational API. On top of this, two primary client libraries are officially supported:

- **C++ (rclcpp)**: The primary language for performance-critical robotics applications. Provides full access to all ROS 2 features including zero-copy intra-process communication, real-time capable executors, and advanced lifecycle management. Recommended for production robotic systems where latency and throughput matter.
- **Python (rclpy)**: The preferred language for rapid prototyping, scripting, launch file configuration, and non-performance-critical nodes. As of Kilted Kaiju (May 2025), the events executor has been ported to Python with benchmarks showing up to **10x speed improvements**.

_Community Client Libraries:_ Additional client libraries exist for **Java**, **Go**, **Rust**, and **C#**, though these are community-maintained and not Tier 1 supported.

_Language Evolution:_ The trend is toward C++ for production nodes and Python for tooling, testing, and orchestration. Rust is an emerging language in the ROS 2 ecosystem for safety-critical applications.

_Source: [ROS 2 Design](https://design.ros2.org/), [Henki Robotics - ROS 2 Best Practices](https://henkirobotics.com/ros-2-best-practices/)_

### Development Frameworks and Libraries

**Core ROS 2 Frameworks:**

- **rclcpp / rclpy**: The primary client libraries providing node creation, pub/sub, services, actions, parameters, timers, and lifecycle management.
- **tf2**: Transform library for managing coordinate frame transformations — essential for any multi-frame robotic system.
- **nav2**: The Navigation 2 stack for autonomous mobile robot navigation.
- **MoveIt 2**: Motion planning framework for robotic manipulators.
- **ros2_control**: Hardware abstraction layer for robot controllers.
- **micro-ROS (rclc)**: Lightweight framework for microcontroller-based ROS 2 nodes, enabling embedded systems integration.

**Simulation Frameworks:**

- **Gazebo (Harmonic)**: Primary physics simulator with tight ROS 2 integration.
- **NVIDIA Isaac Sim**: GPU-accelerated simulation with ROS 2 reference architecture support.

_Ecosystem Maturity:_ The ROS 2 package ecosystem has matured significantly, with Jazzy Jalisco (LTS, supported until May 2029) and Kilted Kaiju (released May 2025) representing the current stable distributions.

_Source: [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html), [Isaac Sim ROS 2 Reference Architecture](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/ros2_reference_architecture.html), [Kilted Kaiju Release](https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released)_

### Middleware and Communication Technologies

ROS 2's communication layer is built on an abstracted **ROS Middleware Interface (RMW)**, allowing pluggable middleware implementations:

**DDS-Based Middleware (Traditional):**

- **eProsima Fast DDS**: Default RMW for most ROS 2 distributions. Mature, well-tested, full DDS compliance.
- **Eclipse Cyclone DDS**: Lightweight alternative, previously default in some distributions.
- **RTI Connext DDS**: Commercial-grade DDS with enterprise support.
- **GurumNetworks GurumDDS**: Additional supported vendor.

**Zenoh Middleware (New — Tier 1 as of Kilted Kaiju):**

- **Eclipse Zenoh**: First supported as Tier 1 in ROS 2 Kilted Kaiju (May 2025). More lightweight than DDS with minimal wire overhead, flexible routing, and better suited for challenging network conditions. Expected to be more efficient and secure compared to DDS.

_Key DDS Features:_ Distributed discovery (no central broker unlike ROS 1), configurable Quality of Service (QoS) profiles, and support for intra-process zero-copy communication to reduce CPU load for high-bandwidth data streams.

_Confidence Level:_ HIGH — verified against official ROS 2 documentation and release notes.

_Source: [ROS 2 Middleware Interface Design](https://design.ros2.org/articles/ros_middleware_interface.html), [ROS 2 Different Middleware Vendors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Different-Middleware-Vendors.html), [Kilted Kaiju Release Details](https://www.therobotreport.com/kilted-kaiju-ros-2-release-details/)_

### Development Tools and Platforms

**Build System:**

- **colcon**: The meta-build tool for ROS 2 workspaces, succeeding catkin_make and ament_tools. Supports out-of-source builds, creates separate `build/`, `install/`, and `log/` directories. Can build ROS 1, ROS 2, and Gazebo workspaces.
- **ament_cmake**: CMake-based build type for C++ packages with ROS 2 extensions.
- **ament_python**: setuptools-based build type for pure Python packages.

**IDE and Editors:**

- **VS Code** with ROS extensions is the most widely used development environment.
- **CLion** with ROS 2 plugin support for C++ development.
- **Eclipse** with ROS 2 integration (historical, declining usage).

**Testing Frameworks:**

- **launch_testing**: Primary integration testing framework, extends Python launch files with active and post-shutdown tests. Uses Python `unittest` under the hood.
- **launch_pytest**: pytest-based alternative for integration tests.
- **gtest / gmock**: For C++ unit testing of ROS 2 nodes.
- **pytest**: For Python unit testing.
- **ament_cmake_ros**: Provides `run_test_isolated.py` for applying unique ROS domain IDs to prevent test crosstalk.
- **colcon test**: Unified test runner that orchestrates all registered tests.

**Debugging and Visualization:**

- **RViz2**: 3D visualization tool for sensor data, robot models, and planning.
- **rqt**: Modular GUI framework with plugins for topic monitoring, node graphs, and parameter tuning.
- **ros2 CLI tools**: Command-line introspection (`ros2 topic`, `ros2 node`, `ros2 service`, `ros2 action echo` — new in Kilted).

_Source: [colcon Documentation](https://colcon.readthedocs.io/en/released/user/quick-start.html), [ament Build System Design](https://design.ros2.org/articles/ament.html), [ROS 2 Integration Testing](https://arnebaeyens.com/blog/2024/ros2-integration-testing/), [ROS 2 Launch Testing Tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Integration.html)_

### Cloud Infrastructure and Deployment

**Containerization:**

- **Docker**: Standard approach for packaging ROS 2 nodes. Official `ros:jazzy` and `ros:kilted` base images available. Enables reproducible builds and deployment across different hardware platforms.
- **Kubernetes**: Used for orchestrating multi-node ROS 2 systems in cloud/edge environments. Each ROS 2 node can run as a separate container in a Kubernetes Deployment. K3S (lightweight Kubernetes) is particularly relevant for resource-constrained robots.

**Cloud Robotics Platforms:**

- **AWS RoboMaker**: Cloud robotics service with ROS 2 support for simulation and fleet management.
- **Google Cloud Robotics Core**: Provides Kubernetes-based deployment patterns for ROS 2 nodes.
- **NVIDIA Isaac**: GPU-accelerated cloud simulation and deployment.

**Challenges:**

- Real-time responsiveness requirements complicate direct adoption of standard container orchestration.
- DDS discovery mechanisms may need special configuration in containerized/cloud environments.
- Network latency between cloud and edge nodes requires careful QoS tuning.

_Confidence Level:_ MEDIUM-HIGH — cloud robotics with ROS 2 is an active area of research and development, with patterns still maturing.

_Source: [ROS with Kubernetes Discourse](https://discourse.ros.org/t/ros-with-kubernetes/28107), [Deploying ROS 2 on IBM Cloud](https://docs.ros.org/en/foxy/Tutorials/Miscellaneous/Deploying-ROS-2-on-IBM-Cloud.html), [ROS 2 and Kubernetes - Ubuntu](https://ubuntu.com/blog/exploring-ros-2-with-kubernetes), [Enhancing Resilience of ROS 2 with Kubernetes](https://pmc.ncbi.nlm.nih.gov/articles/PMC12390455/)_

### Technology Adoption Trends

_Current Active Distributions:_

| Distribution | Release Date | Support End | Type |
|---|---|---|---|
| **Humble Hawksbill** | May 2022 | May 2027 | LTS |
| **Jazzy Jalisco** | May 2024 | May 2029 | LTS |
| **Kilted Kaiju** | May 2025 | Nov 2026 | Standard |

_Migration Patterns:_

- Organizations are migrating from ROS 1 to ROS 2 at an accelerating pace, with ROS 1 Noetic reaching EOL in May 2025.
- Zenoh adoption as a DDS alternative is expected to grow significantly following its Tier 1 status in Kilted Kaiju.
- Increasing adoption of component-based (composable) node architectures over standalone process-per-node patterns.

_Emerging Technologies:_

- **Zenoh middleware**: Lightweight alternative to DDS, Tier 1 since May 2025.
- **Events executor in Python**: 10x performance improvement enables more viable Python-based production nodes.
- **Action introspection CLI**: New `ros2 action echo` command for runtime debugging.

_Legacy Technology Being Phased Out:_

- ROS 1 (Noetic EOL May 2025) — full migration to ROS 2 expected.
- OpenSplice DDS — discontinued, removed from supported vendors.
- catkin build system — fully replaced by colcon/ament.

_Source: [ROS 2 Distributions](https://docs.ros.org/en/jazzy/Releases.html), [Kilted Kaiju Release](https://docs.ros.org/en/kilted/Releases/Release-Kilted-Kaiju.html)_
