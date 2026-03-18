# Launch Files Reference

> ROS 2 Jazzy | Python launch files, arguments, composition, namespaces

---

## 1. Why Launch Files?

A robotics system runs many nodes simultaneously. Launch files define which nodes
start, with what parameters, how they connect, and in what order.

Launch files declare the system topology. Instead of starting 15 terminals and
running 15 commands, you run one launch file.

```bash
# Without launch file:
ros2 run my_robot_perception camera_node &
ros2 run my_robot_perception lidar_node &
ros2 run my_robot_navigation planner_node &
ros2 run my_robot_control motor_controller &
# ... 10 more commands

# With launch file:
ros2 launch my_robot_bringup robot.launch.py
```

---

## 2. Python Launch File Structure

ROS 2 launch files are Python scripts. They return a `LaunchDescription` containing
launch actions.

### Minimal Launch File

```python
# my_robot_bringup/launch/minimal.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_perception',
            executable='camera_node',
            name='camera',
            output='screen',
        ),
        Node(
            package='my_robot_perception',
            executable='lidar_node',
            name='lidar',
            output='screen',
        ),
    ])
```

**Run it:**
```bash
ros2 launch my_robot_bringup minimal.launch.py
```

### Node Configuration Options

```python
Node(
    package='my_robot_perception',       # Package containing the executable
    executable='camera_node',         # Entry point name from setup.py
    name='front_camera',              # Override node name (optional)
    namespace='sensors',              # Namespace prefix (optional)
    output='screen',                  # 'screen' shows logs, 'log' writes to file
    parameters=[                      # Parameter values
        {'device_id': 0},
        {'fps': 30.0},
        '/path/to/params.yaml',      # Can also load from YAML file
    ],
    remappings=[                      # Remap topic names
        ('image_raw', '/sensors/front_camera/image'),
        ('camera_info', '/sensors/front_camera/info'),
    ],
    arguments=['--ros-args', '--log-level', 'debug'],  # CLI arguments
    respawn=True,                     # Restart if the node crashes
    respawn_delay=2.0,                # Wait 2s before restarting
)
```

---

## 3. Launch Arguments

Launch arguments make launch files configurable at runtime. Like CLI flags for
your system deployment.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments with defaults
    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot_1',
        description='Name of the robot instance'
    )

    camera_fps = DeclareLaunchArgument(
        'camera_fps',
        default_value='30.0',
        description='Camera frame rate'
    )

    # Use arguments in node configuration
    camera_node = Node(
        package='my_robot_perception',
        executable='camera_node',
        name='camera',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'fps': LaunchConfiguration('camera_fps'),
            'use_sim': LaunchConfiguration('use_sim'),
        }],
        output='screen',
    )

    return LaunchDescription([
        use_sim,
        robot_name,
        camera_fps,
        camera_node,
    ])
```

**Override at launch time:**
```bash
ros2 launch my_robot_bringup robot.launch.py use_sim:=true camera_fps:=15.0
```

---

## 4. Conditionals

Control which nodes launch based on arguments.

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    use_sim = DeclareLaunchArgument('use_sim', default_value='false')
    enable_rviz = DeclareLaunchArgument('enable_rviz', default_value='true')

    # Launch real camera driver only if NOT in simulation
    real_camera = Node(
        package='my_robot_perception',
        executable='camera_driver',
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
    )

    # Launch simulated camera only if in simulation
    sim_camera = Node(
        package='my_robot_simulation',
        executable='sim_camera',
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    # Launch RViz only if enabled
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/path/to/config.rviz'],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription([use_sim, enable_rviz, real_camera, sim_camera, rviz])
```

---

## 5. Including Other Launch Files

Break complex systems into composable launch files.

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim = DeclareLaunchArgument('use_sim', default_value='false')

    # Include perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_perception'),
                'launch',
                'perception.launch.py'
            )
        ),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'camera_fps': '30.0',
        }.items(),
    )

    # Include navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_navigation'),
                'launch',
                'navigation.launch.py'
            )
        ),
    )

    return LaunchDescription([use_sim, perception_launch, navigation_launch])
```

**Composition pattern:**
```
my_robot_bringup/launch/
  robot.launch.py          # Top-level: includes everything
  hardware.launch.py       # Camera, LiDAR, IMU drivers
  perception.launch.py     # Image processing, point cloud processing
  navigation.launch.py     # Path planning, obstacle avoidance
  teleop.launch.py         # Manual control
  simulation.launch.py     # Gazebo + sim-specific nodes
```

---

## 6. Remapping Topics and Parameters

Remapping lets you connect nodes without modifying their code. A node publishes to
`image_raw`, but your pipeline expects `/sensors/camera/image` -- remap it.

```python
Node(
    package='my_robot_perception',
    executable='camera_node',
    remappings=[
        # (original_name, new_name)
        ('image_raw', '/sensors/front_camera/image'),
        ('camera_info', '/sensors/front_camera/info'),
    ],
)

Node(
    package='my_robot_perception',
    executable='detector',
    remappings=[
        # Connect detector input to camera output
        ('input_image', '/sensors/front_camera/image'),
        ('detections', '/perception/detections'),
    ],
)
```

### Loading Parameters from YAML

```yaml
# config/camera_params.yaml
camera:  # Must match the node name
  ros__parameters:
    device_id: 0
    fps: 30.0
    resolution_width: 1920
    resolution_height: 1080
    exposure: -1  # auto
```

```python
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'camera_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_perception',
            executable='camera_node',
            name='camera',
            parameters=[config],  # Load from YAML
            output='screen',
        ),
    ])
```

---

## 7. Composable Node Containers (Zero-Copy)

When two nodes in the same process exchange large messages (images, point clouds),
composable nodes enable zero-copy transfer -- the subscriber gets a pointer to the
publisher's message buffer instead of a serialized copy.

**Use this when:** profiling shows serialization/deserialization is a bottleneck,
typically for image or point cloud pipelines.

```python
from launch_ros.actions import ComposableNodeContainer, LoadComposableNode
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_robot_perception',
                plugin='my_robot_perception::CameraDriver',
                name='camera',
                parameters=[{'fps': 30.0}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='my_robot_perception',
                plugin='my_robot_perception::DefectDetector',
                name='detector',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

**Note:** Composable nodes require C++ rclcpp components. Python nodes cannot
be loaded into composable containers. This is a valid reason to write
performance-critical nodes in C++.

---

## 8. Namespace Management

Namespaces group related nodes and topics. Essential for multi-robot systems or
when running multiple instances of the same node.

```python
# All topics from this node will be prefixed with /robot1/
Node(
    package='my_robot_control',
    executable='motor_controller',
    namespace='robot1',
)

# Same node for a second robot
Node(
    package='my_robot_control',
    executable='motor_controller',
    namespace='robot2',
)

# Result:
# /robot1/cmd_vel
# /robot1/odom
# /robot2/cmd_vel
# /robot2/odom
```

### Group Actions for Shared Namespace

```python
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        Node(package='my_robot_control', executable='motor_controller'),
        Node(package='my_robot_perception', executable='camera_node'),
        Node(package='my_robot_perception', executable='lidar_node'),
    ])

    return LaunchDescription([robot1_group])
```

---

## 9. Event Handlers

React to process lifecycle events -- useful for ordered startup and shutdown.

```python
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown

def generate_launch_description():
    driver_node = Node(
        package='my_robot_control',
        executable='motor_driver',
        name='motor_driver',
    )

    # Start processing only after driver is up
    processing_node = Node(
        package='my_robot_control',
        executable='velocity_smoother',
        name='velocity_smoother',
    )

    # When driver exits, shut down everything
    on_driver_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=driver_node,
            on_exit=[
                LogInfo(msg='Motor driver exited! Shutting down...'),
                EmitEvent(event=Shutdown(reason='Motor driver crashed')),
            ],
        )
    )

    # Log when driver starts
    on_driver_start = RegisterEventHandler(
        OnProcessStart(
            target_action=driver_node,
            on_start=[LogInfo(msg='Motor driver started successfully')],
        )
    )

    return LaunchDescription([
        driver_node,
        processing_node,
        on_driver_exit,
        on_driver_start,
    ])
```

---

## 10. Environment Variable Substitution

```python
from launch.substitutions import EnvironmentVariable

Node(
    package='my_robot_perception',
    executable='camera_node',
    parameters=[{
        'model_path': EnvironmentVariable('ROBOT_MODEL_PATH',
                                          default_value='/opt/models/detector_v1'),
    }],
)
```

---

## 11. Complete Example: Multi-Node Robot Launch

```python
#!/usr/bin/env python3
"""Main launch file for the robot.

Launches all nodes for a complete robot system.
Supports simulation mode and configurable parameters.

Usage:
  ros2 launch my_robot_bringup robot.launch.py
  ros2 launch my_robot_bringup robot.launch.py use_sim:=true
  ros2 launch my_robot_bringup robot.launch.py enable_rviz:=true camera_fps:=15
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Declare arguments ---
    args = [
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='Use Gazebo simulation'),
        DeclareLaunchArgument('enable_rviz', default_value='false',
                              description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('camera_fps', default_value='30.0',
                              description='Camera frame rate'),
        DeclareLaunchArgument('robot_name', default_value='my_robot',
                              description='Robot instance name'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Logging level'),
    ]

    # --- Config file paths ---
    bringup_share = FindPackageShare('my_robot_bringup')
    camera_config = PathJoinSubstitution([bringup_share, 'config', 'camera.yaml'])
    nav_config = PathJoinSubstitution([bringup_share, 'config', 'navigation.yaml'])

    # --- Hardware drivers (real robot only) ---
    hardware_nodes = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
        actions=[
            Node(
                package='my_robot_perception',
                executable='camera_node',
                name='camera',
                parameters=[camera_config, {
                    'fps': LaunchConfiguration('camera_fps'),
                }],
                respawn=True,
                respawn_delay=2.0,
                output='screen',
            ),
            Node(
                package='my_robot_perception',
                executable='lidar_node',
                name='lidar',
                output='screen',
                respawn=True,
            ),
            Node(
                package='my_robot_control',
                executable='motor_controller',
                name='motors',
                output='screen',
            ),
        ],
    )

    # --- Processing pipeline (always runs) ---
    processing_nodes = GroupAction(actions=[
        Node(
            package='my_robot_inspection',
            executable='detector',
            name='detector',
            parameters=[{
                'confidence_threshold': 0.7,
                'model_path': '/opt/models/detector_v2.onnx',
            }],
            remappings=[
                ('input_image', '/camera/image_raw'),
                ('detections', '/perception/detections'),
            ],
            output='screen',
        ),
        Node(
            package='my_robot_navigation',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            parameters=[nav_config],
            output='screen',
        ),
    ])

    # --- Static transforms ---
    static_transforms = GroupAction(actions=[
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0',
                       'base_link', 'camera_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.15', '0', '0', '0',
                       'base_link', 'lidar_link'],
        ),
    ])

    # --- RViz (optional) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [bringup_share, 'config', 'robot.rviz']
        )],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        output='screen',
    )

    # --- Robot state publisher (URDF) ---
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'launch',
                'description.launch.py'
            ])
        ),
    )

    return LaunchDescription(
        args + [
            robot_description,
            static_transforms,
            hardware_nodes,
            processing_nodes,
            rviz_node,
        ]
    )
```

---

## 12. Launch File Best Practices

| Do | Don't |
|---|---|
| Use `get_package_share_directory()` for file paths | Hardcode absolute paths |
| Declare all configurable values as launch arguments | Bury magic numbers in node configs |
| Use YAML files for complex parameter sets | Put 20 parameters inline in Node() |
| Break large launch files into composable pieces | Put everything in one 500-line file |
| Use `respawn=True` for critical drivers | Assume nodes never crash |
| Document launch arguments with descriptions | Leave arguments undocumented |
| Use namespaces for multi-robot deployments | Rely on unique topic names |
| Test launch files: `ros2 launch --print` to preview | Deploy untested launch configs |
