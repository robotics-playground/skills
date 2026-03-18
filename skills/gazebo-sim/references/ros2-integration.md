# ROS 2 Integration with Gazebo Harmonic

This reference covers bridging Gazebo Harmonic topics to ROS 2, spawning models, clock
synchronization, and writing launch files that tie everything together.

---

## ros_gz_bridge: Connecting Gazebo and ROS 2

Gazebo Harmonic uses its own transport layer (gz-transport), which is separate from ROS 2's
DDS-based transport. The `ros_gz_bridge` node translates messages between the two systems.

Think of it like a protocol adapter: Gazebo publishes sensor data on gz-transport topics, and
the bridge republishes that data as standard ROS 2 messages that your nodes can subscribe to.

### How It Works

```
Gazebo gz-transport          ros_gz_bridge          ROS 2 DDS
  /world/my_world/     -->   translates     -->   /scan
   model/robot/             message types         sensor_msgs/LaserScan
   link/lidar_link/
   sensor/lidar/scan
```

### Bridge Direction Types

| Direction | Meaning | Use case |
|-----------|---------|----------|
| `GZ_TO_ROS` | Gazebo -> ROS 2 | Sensor data (LiDAR, camera, IMU) |
| `ROS_TO_GZ` | ROS 2 -> Gazebo | Commands (cmd_vel, joint positions) |
| `BIDIRECTIONAL` | Both directions | Clock, transforms |

---

## Bridge Configuration (YAML Format)

The recommended way to configure the bridge is with a YAML file. This is cleaner than
command-line arguments and easier to maintain.

### Basic Bridge Config

```yaml
# bridge_config.yaml
---
- ros_topic_name: "/scan"
  gz_topic_name: "/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

### Running the Bridge with Config

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config.yaml
```

### Common Bridge Topic Mappings

| Sensor / Data | Gazebo Type | ROS 2 Type | Direction |
|--------------|-------------|------------|-----------|
| LiDAR (2D) | `gz.msgs.LaserScan` | `sensor_msgs/msg/LaserScan` | GZ_TO_ROS |
| LiDAR (3D) | `gz.msgs.PointCloudPacked` | `sensor_msgs/msg/PointCloud2` | GZ_TO_ROS |
| IMU | `gz.msgs.IMU` | `sensor_msgs/msg/Imu` | GZ_TO_ROS |
| Camera (RGB) | `gz.msgs.Image` | `sensor_msgs/msg/Image` | GZ_TO_ROS |
| Camera Info | `gz.msgs.CameraInfo` | `sensor_msgs/msg/CameraInfo` | GZ_TO_ROS |
| Depth Image | `gz.msgs.Image` | `sensor_msgs/msg/Image` | GZ_TO_ROS |
| Point Cloud | `gz.msgs.PointCloudPacked` | `sensor_msgs/msg/PointCloud2` | GZ_TO_ROS |
| Velocity cmd | `gz.msgs.Twist` | `geometry_msgs/msg/Twist` | ROS_TO_GZ |
| Joint states | `gz.msgs.Model` | `sensor_msgs/msg/JointState` | GZ_TO_ROS |
| Clock | `gz.msgs.Clock` | `rosgraph_msgs/msg/Clock` | GZ_TO_ROS |
| Odometry | `gz.msgs.Odometry` | `nav_msgs/msg/Odometry` | GZ_TO_ROS |
| TF | `gz.msgs.Pose_V` | `tf2_msgs/msg/TFMessage` | GZ_TO_ROS |
| Contacts | `gz.msgs.Contacts` | `gazebo_msgs/msg/ContactsState` | GZ_TO_ROS |

---

## Gazebo Topic Names

Gazebo Harmonic uses long, structured topic names. The pattern depends on the world name,
model name, and sensor name.

### Finding Gazebo Topic Names

```bash
# List all active Gazebo topics
gz topic -l

# Get info about a specific topic (message type, publishers, subscribers)
gz topic -i -t /world/my_world/model/robot/link/base_link/sensor/imu_sensor/imu

# Echo topic data (like rostopic echo)
gz topic -e -t /world/my_world/model/robot/link/base_link/sensor/imu_sensor/imu
```

### Typical Gazebo Topic Name Patterns

```
# Sensor data
/world/{world_name}/model/{model_name}/link/{link_name}/sensor/{sensor_name}/{data_type}

# Examples:
/world/my_world/model/robot/link/base_link/sensor/imu_sensor/imu
/world/my_world/model/robot/link/lidar_link/sensor/lidar/scan
/world/my_world/model/robot/link/camera_link/sensor/camera/image

# Model command topics (set by plugins)
/model/{model_name}/cmd_vel          # DiffDrive plugin
/model/{model_name}/odometry         # DiffDrive plugin
```

**Important:** The exact topic names depend on your world name, model name, and sensor
configuration. Always use `gz topic -l` to discover the actual topic names after launching
the simulation.

---

## ros_gz_sim: Spawning Models from ROS 2

The `ros_gz_sim` package provides a `create` node that spawns models into a running
Gazebo simulation.

### Spawning a URDF Model

```bash
# From a file
ros2 run ros_gz_sim create \
  -name my_robot \
  -file /path/to/robot.urdf \
  -x 0 -y 0 -z 0.5

# From the robot_description topic (common pattern with robot_state_publisher)
ros2 run ros_gz_sim create \
  -name my_robot \
  -topic robot_description \
  -x 0 -y 0 -z 0.5
```

### Spawning an SDF Model

```bash
# From a file
ros2 run ros_gz_sim create \
  -name my_robot \
  -file /path/to/model.sdf \
  -x 0 -y 0 -z 0.5

# From Gazebo Fuel (online model repository)
ros2 run ros_gz_sim create \
  -name my_robot \
  -string '<include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri></include>'
```

### Spawn Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `-name` | Unique model name in simulation | `-name robot_1` |
| `-file` | Path to SDF or URDF file | `-file robot.urdf` |
| `-topic` | ROS 2 topic with model description | `-topic robot_description` |
| `-string` | Inline SDF/URDF string | `-string '<sdf>...</sdf>'` |
| `-x`, `-y`, `-z` | Spawn position (meters) | `-x 1.0 -y 0 -z 0.5` |
| `-R`, `-P`, `-Y` | Spawn orientation (roll, pitch, yaw in radians) | `-Y 1.5708` |
| `-world` | Target world name | `-world my_world` |

---

## Clock Synchronization (use_sim_time)

When running with simulation, all ROS 2 nodes should use the simulated clock instead of
the system wall clock. This ensures consistent timestamps and proper timing.

### Why It Matters

Without `use_sim_time`, your ROS 2 nodes use the system clock while Gazebo uses simulation
time. If the simulation runs slower than real-time (RTF < 1.0), timestamps will not match,
causing issues with TF transforms, sensor data fusion, and navigation.

### Setting use_sim_time

In a launch file (recommended):
```python
# Set for all nodes
Node(
    package='my_package',
    executable='my_node',
    parameters=[{'use_sim_time': True}],
)
```

From the command line:
```bash
ros2 run my_package my_node --ros-args -p use_sim_time:=true
```

### Bridging the Clock

The clock bridge must be configured so ROS 2 receives the simulation clock:

```yaml
# In bridge_config.yaml
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

**Every ROS 2 node that uses time (which is almost all of them) must have
`use_sim_time: True` when running with Gazebo.**

---

## Sensor Data Bridges

### IMU Bridge

```yaml
- ros_topic_name: "/imu/data"
  gz_topic_name: "/world/my_world/model/robot/link/base_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

### LiDAR Bridge (2D LaserScan)

```yaml
- ros_topic_name: "/scan"
  gz_topic_name: "/world/my_world/model/robot/link/lidar_link/sensor/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

### Camera Bridge

```yaml
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/world/my_world/model/robot/link/camera_link/sensor/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/world/my_world/model/robot/link/camera_link/sensor/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
```

### Depth Camera Bridge

```yaml
- ros_topic_name: "/depth_camera/image_raw"
  gz_topic_name: "/world/my_world/model/robot/link/depth_link/sensor/depth_camera/depth_image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/depth_camera/points"
  gz_topic_name: "/world/my_world/model/robot/link/depth_link/sensor/depth_camera/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```

---

## Joint State Publisher from Simulation

Gazebo can publish joint states for your robot, replacing the need for `joint_state_publisher`
in simulation.

Add this plugin to your robot model (URDF or SDF):

```xml
<!-- In URDF (inside <gazebo> tag) -->
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
  </plugin>
</gazebo>
```

Then bridge the joint states:

```yaml
- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/my_world/model/robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
```

---

## Sending Commands to Simulated Robots

### Velocity Commands (cmd_vel)

For differential drive or tracked robots, bridge the `/cmd_vel` topic:

```yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

The robot model must have the appropriate drive plugin (DiffDrive, TrackedVehicle, etc.)
configured to listen on this topic. See `references/robot-models.md` for plugin setup.

### Testing Commands

```bash
# Send a velocity command from the terminal
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Or use teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Launch File Patterns for Gazebo + ROS 2

### Pattern 1: Minimal Launch (World + Spawn + Bridge)

```python
# launch/simulation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    # Read URDF for robot_state_publisher
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Set Gazebo resource path
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share, 'models')
        ),

        # Start Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
            }.items(),
        ),

        # Publish robot description to /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # Spawn robot in Gazebo from /robot_description topic
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'robot',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.2',
            ],
            output='screen',
        ),

        # Bridge Gazebo topics to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': True,
            }],
            output='screen',
        ),
    ])
```

### Pattern 2: Launch with Arguments

```python
# Allow choosing the world at launch time
DeclareLaunchArgument(
    'world',
    default_value='my_world',
    description='Name of the Gazebo world to load'
),

# Use the argument
world_file = PathJoinSubstitution([
    pkg_share, 'worlds',
    [LaunchConfiguration('world'), '.sdf']
])
```

Run with:
```bash
ros2 launch my_robot_sim simulation.launch.py world:=my_world
```

### Pattern 3: Headless Simulation (No GUI)

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={
        'gz_args': f'-r -s {world_file}',  # -s = server only (no GUI)
    }.items(),
),
```

---

## Complete Example: Launch File for Inspection Robot

This launch file starts Gazebo with a custom world, spawns the robot, starts
the bridge, and configures all sensor topics.

```python
# launch/simulation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),

        # Gazebo resource path
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share, 'models')
        ),

        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
            }.items(),
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', 'robot_description',
                '-x', '-8.0',
                '-y', '0.0',
                '-z', '0.2',
            ],
            output='screen',
        ),

        # Bridge: sensor and command topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # RViz (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'my_robot.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
```

### Running the Complete Simulation

```bash
# Full simulation with GUI and RViz
ros2 launch my_robot_sim simulation.launch.py

# Headless (no GUI, no RViz)
ros2 launch my_robot_sim simulation.launch.py gui:=false rviz:=false

# With GUI but no RViz
ros2 launch my_robot_sim simulation.launch.py rviz:=false
```

### Verifying the Bridge Is Working

```bash
# Check ROS 2 topics are being published
ros2 topic list

# Check data is flowing
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /camera/image_raw

# View sensor data
ros2 topic echo /scan --once
ros2 topic echo /imu/data --once
```
