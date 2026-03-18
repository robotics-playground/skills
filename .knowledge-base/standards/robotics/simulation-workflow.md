---
title: "Simulation Workflow Standards"
category: "robotics"
tags: ["ros2", "gazebo", "simulation", "isaac-sim", "ci-cd", "testing", "reproducibility"]
description: "Simulation workflow standards including Gazebo Harmonic configuration, world management, sim/real launch patterns, and CI integration"
last_updated: "2026-03-18"
---

# Simulation Workflow Standards

> **Philosophy:** Simulation is the primary development environment. All algorithms MUST work in simulation before touching real hardware. Treat simulation configs as production code — version controlled and reproducible.

## Simulator Selection

| Simulator | Use Case | When to Use |
|-----------|----------|-------------|
| **Gazebo Harmonic** (primary) | Physics simulation, sensor simulation, navigation | Default for all development |
| **Isaac Sim** | RL training, domain randomization, photorealistic rendering | ML/RL experiments only |
| **RViz2** | Visualization only (no physics) | Quick topic inspection, TF debugging |

**Gazebo Harmonic** (formerly Ignition Gazebo) is the MANDATORY primary simulator for all ROS 2 development. It integrates natively via `ros_gz_bridge`.

## Workspace Structure for Simulation

```
ros2_ws/src/
├── my_robot_description/               # Robot URDF/SDF and meshes
│   ├── urdf/
│   │   ├── robot.urdf.xacro       # Robot description (xacro format)
│   │   └── sensors/
│   │       ├── lidar.urdf.xacro
│   │       └── camera.urdf.xacro
│   ├── meshes/
│   │   ├── visual/                 # Detailed meshes for rendering
│   │   └── collision/              # Simplified meshes for physics
│   └── config/
│       └── joint_limits.yaml
├── my_robot_simulation/                 # Simulation-specific package
│   ├── worlds/
│   │   ├── empty_test.sdf          # Minimal world for unit tests
│   │   ├── inspection_tunnel.sdf          # Inspection environment
│   │   └── obstacles_course.sdf    # Navigation testing
│   ├── models/
│   │   ├── inspection_tunnel_straight/
│   │   │   ├── model.sdf
│   │   │   └── model.config
│   │   └── inspection_tunnel_bend/
│   │       ├── model.sdf
│   │       └── model.config
│   ├── launch/
│   │   ├── gazebo.launch.py        # Gazebo standalone
│   │   └── spawn_robot.launch.py   # Spawn robot into running sim
│   └── config/
│       ├── bridge_params.yaml      # ros_gz_bridge topic mappings
│       └── sensor_noise.yaml       # Noise profiles for sensors
└── my_robot_bringup/
    ├── launch/
    │   ├── robot.launch.py          # Real hardware launch
    │   ├── robot_sim.launch.py      # Simulation launch
    │   └── robot_common.launch.py   # Shared between real and sim
    └── config/
        ├── nav2_params.yaml
        └── slam_params.yaml
```

## World File Version Control

**Rules:**
- ALL world files (`.sdf`) MUST be version controlled
- World files go in `my_robot_simulation/worlds/`
- Custom models go in `my_robot_simulation/models/`
- Use descriptive names: `inspection_tunnel_t_junction.sdf`, not `world1.sdf`
- Include a comment header in each world file describing the scenario

```xml
<?xml version="1.0" ?>
<!--
  World: inspection_tunnel.sdf
  Description: 10m straight inspection tunnel with 0.6m diameter.
  Purpose: Basic inspection navigation testing.
  Author: Your Team Name
-->
<sdf version="1.9">
  <world name="inspection_tunnel">
    <physics type="odes">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- ... world content ... -->
  </world>
</sdf>
```

## Sim vs Real Launch Configuration

The key pattern: shared logic in a common launch file, with sim/real variants that set the appropriate parameters.

### Common Launch (shared)

```python
"""robot_common.launch.py — shared between sim and real."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    scan_processor = Node(
        package='my_robot_inspection',
        executable='scan_processor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_range': 30.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        scan_processor,
    ])
```

### Simulation Launch

```python
"""robot_sim.launch.py — simulation-specific launch."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r worlds/inspection_tunnel.sdf',
        }.items(),
    )

    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-topic', 'robot_description'],
    )

    # Bridge Gazebo topics to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[PathJoinSubstitution([
            FindPackageShare('my_robot_simulation'), 'config', 'bridge_params.yaml'
        ])],
    )

    # Include common launch with use_sim_time=true
    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'), 'launch', 'robot_common.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([gazebo, spawn, bridge, common])
```

### Real Hardware Launch

```python
"""robot.launch.py — real hardware launch."""

def generate_launch_description():
    # Hardware drivers (not needed in sim)
    motor_driver = Node(
        package='my_robot_motor_driver',
        executable='motor_driver_node',
        parameters=[{'serial_port': '/dev/ttyMotor'}],
    )

    lidar_driver = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        parameters=[{'serial_port': '/dev/ttyLidar'}],
    )

    # Include common launch with use_sim_time=false
    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([...]),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([motor_driver, lidar_driver, common])
```

## use_sim_time Parameter (CRITICAL)

**Every node** that uses time MUST respect the `use_sim_time` parameter. In simulation, Gazebo publishes the `/clock` topic and all nodes must use simulated time.

```python
# In your node — always accept use_sim_time
self.declare_parameter('use_sim_time', False)
```

| Mode | `use_sim_time` | Clock Source | When |
|------|---------------|--------------|------|
| Real hardware | `false` | System clock | Running on robot |
| Simulation | `true` | `/clock` topic from Gazebo | Development/testing |
| Bag replay | `true` | `--clock` flag on `ros2 bag play` | Offline analysis |

**Failure mode:** If `use_sim_time` is not set in simulation, TF lookups will fail with "extrapolation into the future" errors because node time and sim time diverge.

## Sensor Noise Profiles

Simulate realistic sensor noise to avoid algorithms that only work with perfect data.

```yaml
# config/sensor_noise.yaml
lidar:
  type: gaussian
  mean: 0.0
  stddev: 0.01           # 1cm noise on LiDAR
  range_noise_percent: 0.5

imu:
  accelerometer:
    noise_density: 0.004     # m/s^2/sqrt(Hz)
    random_walk: 0.006       # m/s^2*sqrt(Hz)
  gyroscope:
    noise_density: 0.0002    # rad/s/sqrt(Hz)
    random_walk: 0.00004     # rad/s*sqrt(Hz)

camera:
  type: gaussian
  mean: 0.0
  stddev: 0.007            # Image noise
```

Configure these in Gazebo sensor plugins in your URDF/SDF files. Never test with zero-noise sensors — it hides fragility in your algorithms.

## CI/CD with Simulation

### Headless Gazebo in CI

```yaml
# GitHub Actions step for simulation tests
- name: Simulation tests
  env:
    DISPLAY: ""
    QT_QPA_PLATFORM: offscreen
    GZ_SIM_RESOURCE_PATH: ${{ github.workspace }}/ros2_ws/src/my_robot_simulation/models
  run: |
    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    # Run Gazebo headless with -s (server only, no GUI)
    colcon test --packages-select my_robot_inspection \
      --pytest-args 'test/test_sim_integration.py'
```

**Key flags for headless:**
- Gazebo: `-s` flag (server only) or `--headless-rendering`
- Qt apps: `QT_QPA_PLATFORM=offscreen`
- No `DISPLAY` variable needed

## Reproducibility Requirements

| Requirement | How |
|------------|-----|
| Deterministic physics | Set `max_step_size` explicitly in world file |
| Repeatable sensor data | Set random seed in sensor noise configuration |
| Fixed initial conditions | Specify exact spawn pose in launch file |
| Version-locked Gazebo | Pin Gazebo version in Docker/CI image |
| Documented scenarios | Each world file includes header comment with purpose |

## Performance Benchmarks

| Metric | Target | How to Measure |
|--------|--------|---------------|
| Real-time factor | > 0.8 (sim runs at 80%+ real speed) | Gazebo GUI status bar or `gz sim --info` |
| Topic bridge latency | < 5ms | `ros2 topic delay` on bridged topics |
| Physics step size | 1ms (0.001s) default | Set in world file `<max_step_size>` |
| Max entities | Keep under 100 for real-time | Monitor Gazebo entity count |

If simulation runs slower than 0.5x real-time, simplify the world (reduce mesh complexity, fewer dynamic objects).

## Quick Checklist

- [ ] Gazebo Harmonic is the primary simulator
- [ ] All world files and models are version controlled
- [ ] World files have descriptive names and header comments
- [ ] Sim and real use shared `robot_common.launch.py` with `use_sim_time` parameter
- [ ] ALL nodes declare and respect `use_sim_time`
- [ ] Sensor noise profiles are configured (never test with zero noise)
- [ ] CI runs simulation tests headless (`-s` flag, no DISPLAY)
- [ ] Physics step size is explicitly set in world files
- [ ] Spawn poses are deterministic (specified in launch files)
- [ ] Algorithms validated in simulation BEFORE hardware testing
