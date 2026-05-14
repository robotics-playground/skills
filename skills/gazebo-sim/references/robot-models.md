# Robot Models for Gazebo Harmonic

This reference covers creating robot models that work in Gazebo Harmonic, including URDF
extensions, SDF models, collision geometry, inertia calculation, joint configuration, and
adding Gazebo-specific plugins.

---

## URDF with Gazebo Extensions

If your robot uses URDF (recommended for ROS 2 toolchain compatibility), you add Gazebo-
specific features using `<gazebo>` extension blocks. These blocks are stripped by ROS 2
tools and only used when the URDF is converted to SDF for Gazebo.

### Basic Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Standard URDF links and joints -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>

  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>
      <ambient>0.5 0.5 0.5 1</ambient>
      <diffuse>0.7 0.7 0.7 1</diffuse>
    </material>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
    </sensor>
  </gazebo>

  <!-- Gazebo plugins (no reference = model-level) -->
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <topic>cmd_vel</topic>
      <odom_topic>odometry</odom_topic>
    </plugin>
  </gazebo>

</robot>
```

### `<gazebo reference="link_name">` vs `<gazebo>`

| Tag | Scope | Use for |
|-----|-------|---------|
| `<gazebo reference="link_name">` | Applies to a specific link | Material, sensors, surface properties, self-collide |
| `<gazebo reference="joint_name">` | Applies to a specific joint | Joint friction, damping, spring stiffness |
| `<gazebo>` (no reference) | Applies to the entire model | Plugins (drive, joint state publisher), static flag |

---

## SDF Model Format

SDF is Gazebo's native format and provides more control than URDF+extensions. Use SDF when
building Gazebo-only models (environment objects, props) or when you need features that
URDF cannot express.

### Minimal SDF Model

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="simple_box">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.00667</ixx>
          <iyy>0.00667</iyy>
          <izz>0.00667</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.2 0.2 0.2</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.2 0.2 0.2</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

---

## Collision Geometry

### Primitive vs Mesh Collision

| Approach | Pros | Cons | Use when |
|----------|------|------|----------|
| **Primitives** (box, cylinder, sphere) | Fast physics, stable contacts | Does not match complex shapes | Always prefer for collision |
| **Simplified mesh** | Better shape approximation | Slower, can cause instability | Robot chassis with complex profile |
| **Full mesh** | Exact shape | Very slow, contact issues | Avoid for collision geometry |

**Rule of thumb:** Use the simplest collision geometry that prevents the robot from passing
through obstacles it should not. Use primitives wherever possible. Save detailed meshes for
visual geometry only.

### Example: Simplified Collision with Detailed Visual

```xml
<link name="chassis">
  <!-- Visual: detailed mesh for appearance -->
  <visual name="visual">
    <geometry>
      <mesh>
        <uri>meshes/chassis_detailed.dae</uri>
        <scale>1 1 1</scale>
      </mesh>
    </geometry>
  </visual>

  <!-- Collision: simple box approximation -->
  <collision name="collision">
    <geometry>
      <box><size>0.5 0.3 0.15</size></box>
    </geometry>
  </collision>

  <inertial>
    <mass>5.0</mass>
    <inertia>
      <ixx>0.052</ixx>
      <iyy>0.115</iyy>
      <izz>0.142</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
    </inertia>
  </inertial>
</link>
```

---

## Inertia Calculation

Inertia is the #1 cause of physics issues in simulation. Incorrect inertia values cause
models to sink, fly, oscillate, or behave unpredictably.

### What Is Inertia?

Inertia describes how a body resists rotational acceleration. It depends on both mass and
how that mass is distributed. A long thin rod is easy to spin around its length axis but
hard to spin end-over-end. The inertia tensor captures this in a 3x3 matrix (6 unique values
due to symmetry).

### Formulas for Primitive Shapes

| Shape | Ixx | Iyy | Izz |
|-------|-----|-----|-----|
| **Box** (w x d x h) | m/12 * (d^2 + h^2) | m/12 * (w^2 + h^2) | m/12 * (w^2 + d^2) |
| **Cylinder** (r, h, Z-axis) | m/12 * (3r^2 + h^2) | m/12 * (3r^2 + h^2) | m/2 * r^2 |
| **Sphere** (r) | 2/5 * m * r^2 | 2/5 * m * r^2 | 2/5 * m * r^2 |

### Example: Calculating Inertia for a Chassis

For a box-shaped chassis: width=0.5m, depth=0.3m, height=0.15m, mass=5.0kg

```
Ixx = 5.0/12 * (0.3^2 + 0.15^2) = 5.0/12 * (0.09 + 0.0225) = 0.0469
Iyy = 5.0/12 * (0.5^2 + 0.15^2) = 5.0/12 * (0.25 + 0.0225) = 0.1135
Izz = 5.0/12 * (0.5^2 + 0.3^2)  = 5.0/12 * (0.25 + 0.09)   = 0.1417
```

```xml
<inertial>
  <mass>5.0</mass>
  <inertia>
    <ixx>0.0469</ixx>
    <iyy>0.1135</iyy>
    <izz>0.1417</izz>
    <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
  </inertia>
</inertial>
```

### Using gz sdf for Inertia Verification

```bash
# Check inertia values for an SDF model
gz sdf --inertial-stats model.sdf
```

### Common Inertia Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| All inertia values = 0 | Model flies away or explodes | Calculate proper values using formulas above |
| Inertia too small for mass | Model oscillates or vibrates | Recalculate; inertia should be proportional to mass and dimensions |
| Inertia from wrong shape | Unrealistic rotation behavior | Match inertia calculation to actual geometry |
| Copy-pasted identical inertia on all links | Unnatural movement | Calculate per-link based on each link's geometry and mass |
| Center of mass offset wrong | Model tips over unexpectedly | Set `<pose>` in `<inertial>` to match actual CoM |

---

## Mesh File Formats

| Format | Extension | Pros | Cons | Recommendation |
|--------|-----------|------|------|----------------|
| **STL** | `.stl` | Simple, widely supported | No color/texture, large file size | Use for collision geometry |
| **Collada** | `.dae` | Colors, textures, materials | Complex format, parser issues sometimes | Preferred for visual geometry |
| **OBJ** | `.obj` | Simple, materials via .mtl | Limited features | Good alternative to DAE |
| **glTF** | `.glb`/`.gltf` | Modern, compact, PBR materials | Newer, less tool support | Supported in Harmonic |

### Mesh Best Practices

- Keep mesh files under 50,000 triangles for visual geometry.
- Keep collision meshes under 5,000 triangles (or use primitives).
- Place mesh files in a `meshes/` directory relative to your model file.
- Use relative paths in SDF/URDF: `<uri>meshes/chassis.dae</uri>`.
- Export meshes with Z-up orientation (Gazebo convention).
- Set scale to meters (1 unit = 1 meter).

### Never Use a Heavy Mesh as Collision Geometry

This is worth calling out on its own because it is silent and catastrophic.
A detailed mesh (a CAD-exported chassis STL is often tens of thousands of
triangles, multiple MB) used as a `<collision>` shape forces the physics
engine to test every contact against every triangle. There is no error — the
simulation just *crawls*: real-time factor can drop from ~1.0 to ~0.04, and
then everything downstream looks broken (sensors publish at a fraction of
their configured rate, odometry stutters, control loops miss deadlines).

The fix is always the same: keep the detailed mesh for `<visual>`, and give
`<collision>` a cheap primitive (a box bounding the chassis, a cylinder, a
sphere). The visual is what the user sees; the collision only needs to stop
the robot passing through obstacles. They do not have to match.

If you copied a robot model from another project and the sim is suddenly
slow, check the collision geometry *first* — `gz sdf -p model.sdf` and look
for `<mesh>` inside any `<collision>`.

---

## Joint Types

| Type | Motion | Use case | Example |
|------|--------|----------|---------|
| `revolute` | Rotation around one axis, with limits | Steering, arms, flippers | Steering servo: -0.5 to 0.5 rad |
| `continuous` | Unlimited rotation around one axis | Wheels, rollers | Drive wheel |
| `prismatic` | Linear motion along one axis, with limits | Telescoping, linear actuators | Camera boom: 0 to 0.3 m |
| `fixed` | No motion | Rigidly attaching sensors/parts | Camera mount to chassis |
| `ball` | Rotation around all axes | Caster wheels (simplified) | Rear caster |
| `universal` | Rotation around two axes | Suspension | Not commonly used |

### Joint Configuration Example

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent>base_link</parent>
  <child>left_wheel</child>
  <axis>
    <xyz>0 0 1</xyz>
    <dynamics>
      <damping>0.1</damping>
      <friction>0.05</friction>
    </dynamics>
  </axis>
</joint>
```

### Joint Dynamics Parameters

| Parameter | What it does | Typical range | Too low | Too high |
|-----------|-------------|---------------|---------|----------|
| `damping` | Resists joint velocity (viscous friction) | 0.01 - 1.0 | Oscillation | Sluggish movement |
| `friction` | Static/kinetic friction at joint | 0.0 - 0.5 | Joints spin freely | Joints do not move |
| `spring_stiffness` | Restorative force toward rest position | 0.0 - 100.0 | No spring effect | Violent oscillation |
| `spring_reference` | Rest position for spring | Joint-dependent | N/A | N/A |

---

## Gazebo Plugins for Robot Models

### Choosing a Drive Plugin — Read This First

How you drive the robot determines whether odometry can ever disagree with
physics. Pick deliberately.

| Plugin | How it moves the robot | How odometry is produced | Can odom decouple from physics? |
|--------|------------------------|--------------------------|----------------------------------|
| `DiffDrive` / `TrackedVehicle` | Applies torque to wheel/track joints; the body moves only via wheel-ground contact | Dead-reckons the *commanded* wheel joint velocities | **Yes** — if contact is imperfect (slip, unstable footprint, a dragging collision shape), the body goes one way while odom integrates another |
| `VelocityControl` + `OdometryPublisher` | Sets the body's twist directly from `/cmd_vel` — no wheel joints involved | `OdometryPublisher` reads the body's *actual* pose | **No** — odom is measured from the same body the plugin moves; they are equal by construction |

**Default to `VelocityControl` + `OdometryPublisher`** for any robot whose
job is navigation/SLAM testing rather than drivetrain fidelity. Wheel-joint
plugins are physically realistic but introduce a whole class of bugs where
`/odom` lies — and because `/odom` is the root of the TF tree, a lying `/odom`
makes the laser scan, the map, and Nav2 all wrong, with no error anywhere.
The symptom users report is "the scan drifts/rotates when the robot turns."

Use a wheel-joint plugin (`DiffDrive`, `TrackedVehicle`) only when you
specifically need to study traction, wheel slip, or drivetrain dynamics — and
then expect to spend time on footprint stability and friction tuning. See
`references/debugging.md` for diagnosing odom/physics decoupling.

### Velocity Control + Odometry Publisher (Recommended for Navigation)

Body-driven control. The robot has no wheel joints that bear drive load;
`VelocityControl` sets the canonical link's twist directly, and
`OdometryPublisher` reports the body's true pose. Add a low-friction
center caster (and/or low-friction chassis underside) so the body slides
cleanly without contact forces fighting the commanded twist.

```xml
<!-- Drive the body directly from /cmd_vel -->
<gazebo>
  <plugin filename="gz-sim-velocity-control-system"
          name="gz::sim::systems::VelocityControl">
    <topic>cmd_vel</topic>
  </plugin>
</gazebo>

<!-- Publish odom from the body's actual pose -->
<gazebo>
  <plugin filename="gz-sim-odometry-publisher-system"
          name="gz::sim::systems::OdometryPublisher">
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
    <odom_topic>odometry</odom_topic>
    <tf_topic>tf</tf_topic>
    <odom_publish_frequency>50</odom_publish_frequency>
  </plugin>
</gazebo>
```

Note `robot_base_frame` is whatever frame you want odom to track —
conventionally `base_footprint` (ground projection). Make sure every
downstream consumer (EKF, SLAM, Nav2 costmaps, collision monitor) uses the
*same* base frame name, or TF lookups silently fail.

### Differential Drive (Wheeled Robots)

Wheel-joint drive. Realistic, but odometry is dead-reckoned — see the
decoupling warning above. If you use this, the footprint must be stable
(see "Caster wheel with low friction" below) and `wheel_separation` /
`wheel_radius` must exactly match the URDF joint geometry.

```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system"
          name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_radius>0.05</wheel_radius>
    <max_linear_acceleration>1.0</max_linear_acceleration>
    <max_angular_acceleration>2.0</max_angular_acceleration>
    <topic>cmd_vel</topic>
    <odom_topic>odometry</odom_topic>
    <tf_topic>tf</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_publish_frequency>30</odom_publish_frequency>
  </plugin>
</gazebo>
```

### Tracked Vehicle

For tracked/crawling robots, use the TrackedVehicle plugin instead of DiffDrive:

```xml
<gazebo>
  <plugin filename="gz-sim-tracked-vehicle-system"
          name="gz::sim::systems::TrackedVehicle">
    <body_link>base_link</body_link>
    <left_track>
      <link>left_track</link>
      <velocity_topic>/model/robot/track/left/cmd_vel</velocity_topic>
    </left_track>
    <right_track>
      <link>right_track</link>
      <velocity_topic>/model/robot/track/right/cmd_vel</velocity_topic>
    </right_track>
    <track_mu>1.0</track_mu>
    <track_mu2>0.5</track_mu2>
  </plugin>
</gazebo>
```

### Joint State Publisher

```xml
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
  </plugin>
</gazebo>
```

### Joint Position Controller

For controlling individual joints (camera pan/tilt, arm joints):

```xml
<gazebo>
  <plugin filename="gz-sim-joint-position-controller-system"
          name="gz::sim::systems::JointPositionController">
    <joint_name>camera_pan_joint</joint_name>
    <topic>camera_pan_cmd</topic>
    <p_gain>10.0</p_gain>
    <i_gain>0.0</i_gain>
    <d_gain>1.0</d_gain>
  </plugin>
</gazebo>
```

---

## Friction and Contact Parameters

### Surface Properties on Collision Geometry

```xml
<collision name="wheel_collision">
  <geometry>
    <cylinder><radius>0.05</radius><length>0.04</length></cylinder>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>     <!-- Primary friction coefficient -->
        <mu2>0.8</mu2>   <!-- Secondary friction coefficient -->
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e6</kp>      <!-- Contact stiffness (N/m) -->
        <kd>100</kd>       <!-- Contact damping (N*s/m) -->
        <max_vel>0.1</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

### Friction Tuning Guide

| Surface pair | mu (typical) | Notes |
|-------------|-------------|-------|
| Rubber on concrete | 0.8 - 1.2 | Robot wheels on concrete floor |
| Rubber on wet concrete | 0.4 - 0.7 | Wet indoor conditions |
| Plastic on concrete | 0.3 - 0.5 | Skids, chassis underside |
| Metal on concrete | 0.4 - 0.6 | Track links |
| Track treads on concrete | 1.0 - 1.5 | Tracked vehicles need high friction |

---

## Complete Example: Tracked Robot with LiDAR and Camera

This URDF defines a small tracked robot suitable for indoor inspection, with a 2D LiDAR,
forward-facing camera, and IMU.

```xml
<?xml version="1.0"?>
<robot name="inspection_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============ Base Link (Chassis) ============ -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.035" ixy="0" ixz="0"
               iyy="0.060" iyz="0" izz="0.083"/>
    </inertial>
  </link>

  <!-- ============ Left Wheel ============ -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.00026" ixy="0" ixz="0"
               iyy="0.00026" iyz="0" izz="0.000375"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.17 -0.03" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <!-- ============ Right Wheel ============ -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.00026" ixy="0" ixz="0"
               iyy="0.00026" iyz="0" izz="0.000375"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.17 -0.03" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <!-- ============ Caster Wheel (rear) ============ -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.000025" ixy="0" ixz="0"
               iyy="0.000025" iyz="0" izz="0.000025"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.15 0 -0.055" rpy="0 0 0"/>
  </joint>

  <!-- ============ LiDAR Mount ============ -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.00005" ixy="0" ixz="0"
               iyy="0.00005" iyz="0" izz="0.00009"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.07" rpy="0 0 0"/>
  </joint>

  <!-- ============ Camera Mount ============ -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.03 0.06 0.03"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.0 0.0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.03 0.06 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.00003" ixy="0" ixz="0"
               iyy="0.00002" iyz="0" izz="0.00004"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- ============ Gazebo Extensions ============ -->

  <!-- Wheel surface properties -->
  <gazebo reference="left_wheel">
    <surface>
      <friction>
        <ode><mu>1.0</mu><mu2>0.8</mu2></ode>
      </friction>
    </surface>
  </gazebo>
  <gazebo reference="right_wheel">
    <surface>
      <friction>
        <ode><mu>1.0</mu><mu2>0.8</mu2></ode>
      </friction>
    </surface>
  </gazebo>
  <gazebo reference="caster_wheel">
    <surface>
      <friction>
        <ode><mu>0.1</mu><mu2>0.1</mu2></ode>
      </friction>
    </surface>
  </gazebo>

  <!-- IMU Sensor -->
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
          <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
          <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></x>
          <y><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></y>
          <z><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

  <!-- 2D LiDAR Sensor -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0</mean>
          <stddev>0.005</stddev>
        </noise>
      </lidar>
    </sensor>
  </gazebo>

  <!-- Camera Sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>15</update_rate>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>50</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
    </sensor>
  </gazebo>

  <!-- Diff Drive Plugin -->
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.34</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <max_angular_acceleration>2.0</max_angular_acceleration>
      <topic>cmd_vel</topic>
      <odom_topic>odometry</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
    </plugin>
  </gazebo>

</robot>
```

### Key Design Decisions in This Model

1. **Differential drive** instead of tracked vehicle -- simpler to set up and debug. Switch to
   TrackedVehicle plugin when track-specific behavior is needed.
2. **Caster wheel with low friction** -- provides three-point contact without fighting the
   diff-drive controller.
3. **All links have inertia** -- calculated from geometry and realistic mass estimates.
4. **Sensor noise on all sensors** -- produces realistic data for algorithm testing.
5. **Simplified collision** -- all collision is primitive geometry (box, cylinder, sphere).
