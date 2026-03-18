# TF2, URDF, and Coordinate Frames Reference

> ROS 2 Jazzy | REP 103, REP 105, tf2, URDF, Xacro

---

## 1. REP 103: Coordinate Frame Conventions

Every measurement in ROS 2 uses SI units and right-hand coordinate frames.

### Units

| Quantity | Unit | Notes |
|---|---|---|
| Distance | meters (m) | Not centimeters, not inches |
| Angle | radians (rad) | Not degrees |
| Time | seconds (s) | Use ROS time, not system time |
| Mass | kilograms (kg) | |
| Velocity | m/s | |
| Angular velocity | rad/s | |
| Force | Newtons (N) | |
| Frequency | Hertz (Hz) | |

### Body Frame Orientation (Right-Hand Rule)

For a robot or sensor:
- **X axis**: points **forward** (direction of travel)
- **Y axis**: points **left** (when facing forward)
- **Z axis**: points **up**

```
        Z (up)
        |
        |
        +------- Y (left)
       /
      /
     X (forward)
```

**This catches everyone.** If you're used to screen coordinates (Y-down) or
game engine coordinates (Z-forward), you MUST adjust. In ROS 2:
- A robot facing east has X pointing east
- A camera looking forward has Z pointing forward in the camera's optical frame
  (exception to the body frame convention -- camera_optical_link uses Z-forward)

### Rotation Convention

Rotations use quaternions (x, y, z, w) in message types. For human-readable
angles, the convention is Roll-Pitch-Yaw (RPY):
- **Roll**: rotation around X (forward axis)
- **Pitch**: rotation around Y (left axis)
- **Yaw**: rotation around Z (up axis)

```python
# Convert RPY to quaternion (for message construction)
from tf_transformations import quaternion_from_euler
import math

q = quaternion_from_euler(
    0.0,                    # roll
    0.0,                    # pitch
    math.radians(90.0)      # yaw: 90 degrees left turn
)
# q = [x, y, z, w]
```

---

## 2. REP 105: Standard Frame Names

REP 105 defines the standard coordinate frame hierarchy for mobile robots.
This is non-negotiable -- all ROS 2 navigation and localization tools expect
these frames.

### The Frame Tree

```
map
 └── odom
      └── base_link
           ├── base_footprint   (optional: projection on ground plane)
           ├── camera_link
           │    └── camera_optical_link
           ├── lidar_link
           ├── imu_link
           └── left_wheel_link / right_wheel_link / ...
```

### Frame Definitions

| Frame | Definition | Published By | Static? |
|---|---|---|---|
| `map` | Global fixed frame, aligned with the world | SLAM / localization | No -- updated as localization improves |
| `odom` | Odometry frame, continuous but drifts over time | Wheel odometry / visual odometry | No -- continuous motion |
| `base_link` | Rigidly attached to the robot body center | Robot state publisher | No -- moves with robot |
| `base_footprint` | Projection of base_link on the ground plane | Optional convention | No |
| `camera_link` | Camera mounting point on robot | Static TF publisher / URDF | Yes (fixed to base_link) |
| `camera_optical_link` | Camera optical center (Z-forward, X-right, Y-down) | Static TF publisher / URDF | Yes (fixed to camera_link) |
| `lidar_link` | LiDAR sensor mounting point | Static TF publisher / URDF | Yes |
| `imu_link` | IMU mounting point | Static TF publisher / URDF | Yes |

### Transform Chain

To know where the camera sees something in the map:

```
map -> odom -> base_link -> camera_link -> camera_optical_link
```

Each arrow is a transform published by some node. The tf2 library chains them
automatically.

**Key insight:** The `map -> odom` transform corrects for odometry drift. SLAM
or localization publishes this. The `odom -> base_link` transform is smooth and
continuous from wheel encoders. Together they give a globally-corrected but
locally-smooth position.

---

## 3. TF2 Concepts

tf2 is the transform library. It maintains a tree of coordinate frames and
lets you query the transform between any two frames at any time.

tf2 is essentially a graph of spatial relationships. You insert edges (transforms
between frames), and it can compute paths (transforms between any two connected
frames).

### Static vs Dynamic Transforms

| Type | When to Use | Example |
|---|---|---|
| **Static** | Fixed relationship that never changes | Camera mounted on robot body |
| **Dynamic** | Relationship that changes over time | Robot position in the world |

### Static Transform Publisher

For sensors rigidly mounted on the robot. Publish once, tf2 remembers forever.

```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math
from tf_transformations import quaternion_from_euler

class SensorFramePublisher(Node):
    def __init__(self):
        super().__init__('sensor_frame_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

    def publish_static_transforms(self):
        transforms = []

        # Camera: 10cm forward, 15cm up from base_link, tilted 15 degrees down
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.10   # 10cm forward
        camera_tf.transform.translation.y = 0.0     # centered
        camera_tf.transform.translation.z = 0.15    # 15cm up
        q = quaternion_from_euler(0, math.radians(-15), 0)  # 15 deg pitch down
        camera_tf.transform.rotation.x = q[0]
        camera_tf.transform.rotation.y = q[1]
        camera_tf.transform.rotation.z = q[2]
        camera_tf.transform.rotation.w = q[3]
        transforms.append(camera_tf)

        # LiDAR: 5cm forward, 20cm up, flat
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = self.get_clock().now().to_msg()
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'lidar_link'
        lidar_tf.transform.translation.x = 0.05
        lidar_tf.transform.translation.z = 0.20
        lidar_tf.transform.rotation.w = 1.0  # No rotation
        transforms.append(lidar_tf)

        # IMU: at center of base_link
        imu_tf = TransformStamped()
        imu_tf.header.stamp = self.get_clock().now().to_msg()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.rotation.w = 1.0  # Identity -- co-located
        transforms.append(imu_tf)

        self.static_broadcaster.sendTransform(transforms)
```

**Or use the CLI tool** (simpler for quick setup):
```bash
# static_transform_publisher x y z yaw pitch roll parent child
ros2 run tf2_ros static_transform_publisher \
  0.10 0.0 0.15 0.0 -0.26 0.0 base_link camera_link
```

### Dynamic Transform Publisher

For transforms that change over time (e.g., odometry).

```python
from tf2_ros import TransformBroadcaster

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_odom)  # 50 Hz
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def publish_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
```

### Transform Listener (Looking Up Transforms)

```python
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class TransformUser(Node):
    def __init__(self):
        super().__init__('transform_user')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.lookup_transform)

    def lookup_transform(self):
        try:
            # Get transform from camera_link to map
            transform = self.tf_buffer.lookup_transform(
                'map',           # target frame
                'camera_link',   # source frame
                rclpy.time.Time(),  # latest available
                timeout=Duration(seconds=1.0)
            )
            pos = transform.transform.translation
            self.get_logger().info(
                f'Camera in map: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}'
            )
        except Exception as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')
```

---

## 4. URDF Basics

URDF (Unified Robot Description Format) defines the robot's physical structure.
It describes links (rigid bodies) and joints (connections between links).

URDF is a declarative format for describing robot structure. Links are the rigid
bodies, joints define the parent-child relationships between them, and visual/collision
properties describe their physical appearance and shape.

### Link Element

```xml
<link name="base_link">
  <!-- What it looks like (for RViz) -->
  <visual>
    <geometry>
      <box size="0.3 0.2 0.1"/>  <!-- length width height in meters -->
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </visual>

  <!-- What the physics engine uses for collisions -->
  <collision>
    <geometry>
      <box size="0.3 0.2 0.1"/>
    </geometry>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </collision>

  <!-- Mass and inertia (required for Gazebo simulation) -->
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.01"/>
  </inertial>
</link>
```

### Joint Types

| Type | Motion | Example |
|---|---|---|
| `fixed` | No motion | Sensor mounted on body |
| `continuous` | Unlimited rotation | Wheel |
| `revolute` | Rotation with limits | Arm joint |
| `prismatic` | Linear sliding with limits | Elevator |
| `floating` | 6-DOF free movement | Drone (rarely used in URDF) |
| `planar` | Movement in a plane | Rare |

```xml
<!-- Fixed joint: camera mounted on body -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0.0 0.15" rpy="0 -0.26 0"/>
</joint>

<!-- Continuous joint: wheel -->
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel_link"/>
  <origin xyz="0.0 0.12 0.0" rpy="-1.5708 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Rotation axis -->
</joint>
```

---

## 5. Xacro: Parameterized URDF

Xacro adds macros, properties, and math to URDF. Without Xacro, URDF files
for real robots become thousands of lines of repetitive XML.

Xacro adds macros and variables to URDF, reducing repetition and making robot
descriptions maintainable.

### Properties (Variables)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Properties (reusable variables) -->
  <xacro:property name="chassis_length" value="0.30"/>
  <xacro:property name="chassis_width" value="0.20"/>
  <xacro:property name="chassis_height" value="0.10"/>
  <xacro:property name="wheel_radius" value="0.04"/>
  <xacro:property name="wheel_width" value="0.03"/>
  <xacro:property name="wheel_offset_y" value="${chassis_width/2 + wheel_width/2}"/>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### Macros (Reusable Components)

```xml
<!-- Define a wheel macro -->
<xacro:macro name="wheel" params="name x_offset y_reflect">
  <link name="${name}_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="${name}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${name}_wheel_link"/>
    <origin xyz="${x_offset} ${y_reflect * wheel_offset_y} 0"
            rpy="${-pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</xacro:macro>

<!-- Use the macro -->
<xacro:wheel name="front_left"  x_offset="0.10"  y_reflect="1"/>
<xacro:wheel name="front_right" x_offset="0.10"  y_reflect="-1"/>
<xacro:wheel name="rear_left"   x_offset="-0.10" y_reflect="1"/>
<xacro:wheel name="rear_right"  x_offset="-0.10" y_reflect="-1"/>
```

### Including Files

```xml
<!-- Main robot file -->
<xacro:include filename="$(find my_robot_description)/urdf/chassis.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/sensors.xacro"/>
<xacro:include filename="$(find my_robot_description)/urdf/tracks.xacro"/>
```

### Processing Xacro to URDF

```bash
# Generate URDF from Xacro
xacro my_robot.urdf.xacro > my_robot.urdf

# Or in a launch file (preferred):
```

```python
import xacro
from launch_ros.actions import Node

xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
robot_description = xacro.process_file(xacro_file).toxml()

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}],
)
```

---

## 6. Adding Sensor Frames to URDF

Every sensor on the robot needs a frame in the URDF.

```xml
<!-- Camera with optical frame -->
<link name="camera_link">
  <visual>
    <geometry><box size="0.02 0.05 0.02"/></geometry>
    <material name="gray"><color rgba="0.5 0.5 0.5 1.0"/></material>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0.0 0.10" rpy="0 ${-15*pi/180} 0"/>
</joint>

<!-- Optical frame: Z-forward, X-right, Y-down (camera convention) -->
<link name="camera_optical_link"/>
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_link"/>
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>

<!-- LiDAR -->
<link name="lidar_link">
  <visual>
    <geometry><cylinder radius="0.03" length="0.04"/></geometry>
    <material name="dark_gray"><color rgba="0.3 0.3 0.3 1.0"/></material>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.0 0.0 0.18" rpy="0 0 0"/>
</joint>

<!-- IMU -->
<link name="imu_link"/>
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
</joint>
```

---

## 7. Common Frame Mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| Wrong frame orientation | LiDAR data appears rotated 90 degrees in RViz | Check REP 103: X-forward, Y-left, Z-up for the sensor |
| Missing camera_optical_link | Image processing gives wrong coordinates | Add optical frame with RPY rotation from camera_link |
| base_link at wrong height | Robot appears to float or sink in simulation | Set base_link at the rotation center of the robot |
| Forgot static transform publisher | `view_frames` shows disconnected trees | Ensure all URDF joints have corresponding TF publishers |
| Frame name typo | "Could not find a connection between X and Y" | Use consistent naming; check `ros2 run tf2_tools view_frames` |
| Wrong transform direction | Sensor data appears mirrored | Verify parent/child order in joints and TF broadcasts |
| Using wrong time for lookups | "Extrapolation into the future" error | Use `rclpy.time.Time()` for latest, or match message timestamps |

---

## 8. Debugging TF2

### Essential Commands

```bash
# View the complete frame tree (generates PDF)
ros2 run tf2_tools view_frames
# Open the generated frames.pdf

# Echo a specific transform
ros2 run tf2_ros tf2_echo map base_link
# Output: Translation: [x, y, z], Rotation: [x, y, z, w]

# Monitor transform rates and delays
ros2 run tf2_ros tf2_monitor
# Shows update rate and delay for each transform

# List all frames
ros2 run tf2_ros tf2_echo --help  # then check available frames in view_frames
```

### Debugging "Could not find a connection" Error

1. Run `ros2 run tf2_tools view_frames` and check for:
   - Disconnected subtrees (missing transform between them)
   - Missing frames (sensor frame not being published)
   - Typos in frame names

2. Check what transforms are being published:
   ```bash
   ros2 topic echo /tf --field transforms
   ros2 topic echo /tf_static --field transforms
   ```

3. Verify all static transforms are being published by your launch file
   or URDF robot_state_publisher.

---

## 9. Complete Example: Tracked Robot URDF with Sensors

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- ===== Properties ===== -->
  <xacro:property name="pi" value="3.14159265359"/>
  <xacro:property name="chassis_length" value="0.40"/>
  <xacro:property name="chassis_width" value="0.25"/>
  <xacro:property name="chassis_height" value="0.08"/>
  <xacro:property name="track_width" value="0.05"/>
  <xacro:property name="track_radius" value="0.04"/>
  <xacro:property name="camera_offset_x" value="0.18"/>
  <xacro:property name="camera_offset_z" value="0.12"/>
  <xacro:property name="lidar_offset_z" value="0.16"/>

  <!-- ===== Materials ===== -->
  <material name="chassis_color"><color rgba="0.2 0.2 0.2 1.0"/></material>
  <material name="track_color"><color rgba="0.1 0.1 0.1 1.0"/></material>
  <material name="sensor_color"><color rgba="0.5 0.5 0.5 1.0"/></material>

  <!-- ===== Track Macro ===== -->
  <xacro:macro name="track" params="name y_offset">
    <link name="${name}_track_link">
      <visual>
        <geometry>
          <box size="${chassis_length} ${track_width} ${track_radius*2}"/>
        </geometry>
        <material name="track_color"/>
      </visual>
      <collision>
        <geometry>
          <box size="${chassis_length} ${track_width} ${track_radius*2}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
      </inertial>
    </link>
    <joint name="${name}_track_joint" type="fixed">
      <parent link="base_link"/>
      <child link="${name}_track_link"/>
      <origin xyz="0 ${y_offset} 0" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <!-- ===== Base Link ===== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <origin xyz="0 0 ${chassis_height/2 + track_radius}" rpy="0 0 0"/>
      <material name="chassis_color"/>
    </visual>
    <collision>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <origin xyz="0 0 ${chassis_height/2 + track_radius}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 ${chassis_height/2 + track_radius}"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- ===== Tracks ===== -->
  <xacro:track name="left"  y_offset="${chassis_width/2 + track_width/2}"/>
  <xacro:track name="right" y_offset="${-(chassis_width/2 + track_width/2)}"/>

  <!-- ===== Camera ===== -->
  <link name="camera_link">
    <visual>
      <geometry><box size="0.025 0.06 0.025"/></geometry>
      <material name="sensor_color"/>
    </visual>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${camera_offset_x} 0 ${camera_offset_z}" rpy="0 ${-15*pi/180} 0"/>
  </joint>

  <!-- Camera optical frame (Z-forward for image processing) -->
  <link name="camera_optical_link"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <!-- ===== LiDAR ===== -->
  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="0.035" length="0.04"/></geometry>
      <material name="sensor_color"/>
    </visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 ${lidar_offset_z}" rpy="0 0 0"/>
  </joint>

  <!-- ===== IMU ===== -->
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${chassis_height/2 + track_radius}" rpy="0 0 0"/>
  </joint>

</robot>
```

### Publishing the Robot Description

```python
# In launch file
import xacro, os
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('my_robot_description')
xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
robot_description = xacro.process_file(xacro_file).toxml()

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}],
    output='screen',
)
```

### Visualizing in RViz

```bash
# Launch with RViz
ros2 launch my_robot_description display.launch.py

# Or manually:
ros2 run rviz2 rviz2
# Add RobotModel display, set Description Topic to /robot_description
# Add TF display to see all frames
```
