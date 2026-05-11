# Standard ROS 2 Message Types — When to Use Each

Before designing a custom interface, always check whether a standard type fits. Custom types add build-time complexity and reduce interoperability. **Only create custom types when standard types genuinely cannot represent the data.**

## sensor_msgs — Sensor Data Types

Use these before creating custom equivalents:

| Type | When to use | Common pitfall |
|------|-------------|----------------|
| `sensor_msgs/Imu` | Raw or filtered inertial measurement (accelerometer + gyro ± magnetometer) | Don't use for magnetometer-only readings — use `MagneticField` |
| `sensor_msgs/MagneticField` | Magnetometer readings (compass/heading) | — |
| `sensor_msgs/JointState` | Physical joint positions, velocities, efforts | Don't use for motor encoders unless directly mapping |
| `sensor_msgs/LaserScan` | 2D LIDAR scans (single echo, single range) | **Do not use** `PointCloud2` adapter — see lessons/robotics-lio-adoption-path.md |
| `sensor_msgs/Range` | Single-value distance (ultrasonic, IR time-of-flight) | Only for single-point range; use `LaserScan` for scanning sensors |
| `sensor_msgs/BatteryState` | Battery status (voltage, current, percentage, power supply state) | Prefer over custom voltage-only messages |
| `sensor_msgs/Image` / `CompressedImage` | Camera frames | Use `CompressedImage` for bandwidth-constrained links |
| `sensor_msgs/CameraInfo` | Camera calibration (intrinsics, distortion) | Always pair with `Image` |
| `sensor_msgs/NavSatFix` | GPS/GNSS position (lat/lon/altitude) | Pair with `NavSatStatus` for fix validity |
| `sensor_msgs/Odometry` | 6-DOF pose + twist with covariance | For wheel-odom or EKF output; `Twist` + `Transform` for raw |
| `sensor_msgs/PointCloud2` | Dense 3D point clouds (from depth cameras, 3D LIDAR) | **Use with caution** — see robotics-lio-adoption-path lessons |
| `sensor_msgs/Joy` | Gamepad / joystick input | ROS 2 standard; use `joy_node` driver |
| `sensor_msgs/Temperature` | Environment or device temperature | — |
| `sensor_msgs/RelativeHumidity` | Humidity readings | — |
| `sensor_msgs/FluidPressure` | Barometric pressure | — |
| `sensor_msgs/Illuminance` | Light level (lux) | — |

## std_msgs — Primitive and Infrastructure Types

| Type | When to use | Don't use for |
|------|-------------|---------------|
| `std_msgs/Bool` | Binary on/off, enable/disable, emergency stop | Anything that needs a reason string — use a custom type with a reason field |
| `std_msgs/Empty` | Signal-only messages (no payload) | — |
| `std_msgs/String` | Human-readable strings (warnings, descriptions) | Machine-readable enums — use `uint8` constants or a custom enum type |
| `std_msgs/Float32` / `Float64` | Simple scalar readings (temperature, voltage, distance) | Structured data with units — use a custom type with explicit units comments |
| `std_msgs/Int32` / `Int64` | Integer counters, IDs | — |
| `std_msgs/Header` | Timestamps + frame_id (timestamp with coordinate reference) | Always embed via `builtin_interfaces/Time` in custom types as first field |

## geometry_msgs — Pose, Twist, and Transform Types

| Type | When to use |
|------|-------------|
| `geometry_msgs/Twist` | Velocity commands (linear + angular) — `/cmd_vel` standard type |
| `geometry_msgs/PoseStamped` | 3D pose with header (localization output, goal poses) |
| `geometry_msgs/Pose2D` | 2D planar pose (x, y, theta) — lighter than `PoseStamped` |
| `geometry_msgs/PoseWithCovarianceStamped` | 6-DOF pose with covariance (SLAM output) |
| `geometry_msgs/TransformStamped` | Single transform (parent frame, child frame, transform) — use `tf2_msgs/TFMessage` for broadcast |
| `geometry_msgs/Wrench` | Force + torque (end-effector force, contact force) |

## nav_msgs — Navigation Types

| Type | When to use |
|------|-------------|
| `nav_msgs/Odometry` | Wheel odometry or EKF odometry output |
| `nav_msgs/Path` | Planned or traveled paths (sequence of `PoseStamped`) |
| `nav_msgs/OccupancyGrid` | 2D costmap / occupancy grid |
| `nav2_msgs/action/NavigateToPose` | Single-goal Nav2 navigation action |
| `nav2_msgs/action/NavigateThroughPoses` | Multi-goal Nav2 navigation (patrol, survey) |
| `nav2_msgs/action/FollowPath` | Follow a pre-defined `nav_msgs/Path` |
| `nav2_msgs/srv/LoadMap` | Load a SLAM map from disk |
| `nav2_msgs/srv/ManageLifecycleNodes` | Manage Nav2 lifecycle nodes |

## actionlib_msgs / nav2_msgs — Action Types

| Type | When to use |
|------|-------------|
| `actionlib_msgs/GoalStatus` | Action goal state (PENDING, ACTIVE, SUCCEEDED, ABORTED, REJECTED) |
| `nav2_msgs/ParticleCloud` | Particle cloud for localization (AMCL) |
| `nav2_msgs/CollisionMonitorState` | Collision monitor state output |

## diagnostic_msgs — Diagnostics

| Type | When to use |
|------|-------------|
| `diagnostic_msgs/DiagnosticArray` | System health + diagnosticaggregator output |
| `diagnostic_msgs/KeyValue` | Key-value diagnostic data |

## vision_msgs — Vision Detection (use before custom detection types)

| Type | When to use |
|------|-------------|
| `vision_msgs/Detection2D` | Single 2D bounding-box detection |
| `vision_msgs/Detection2DArray` | Array of 2D detections |
| `vision_msgs/ObjectHypothesis` | Class probability for detected object |

## tf2_msgs — Transform Broadcast

| Type | When to use |
|------|-------------|
| `tf2_msgs/TFMessage` | Broadcast transform tree (subscribe to `/tf`, publish to `/tf_static`) |
| `tf2_msgs/TFMessage` with `stamp: zeros` | Static transforms (map→odom, odom→base_link for NED) |

## std_srvs — Standard Services

| Type | When to use |
|------|-------------|
| `std_srvs/srv/Trigger` | One-shot request → success + message (e.g., "start", "stop", "reset") |
| `std_srvs/srv/Empty` | No-request, no-response service (rare) |

## rule for custom types

If you need a struct that combines:
- `builtin_interfaces/Time timestamp`
- 1–2 scalar fields from `sensor_msgs` / `std_msgs`

→ **Compose it from standard types**, don't create a new `.msg`. Example: a battery node that publishes voltage + temperature + timestamp should publish `sensor_msgs/BatteryState` (already has `voltage`, `percentage`, `power_supply_status`) or a custom `BatteryStatus` only if the standard fields are insufficient.

If you need a struct that has:
- More than 3 fields beyond what standard types provide
- Domain-specific enums (mission states, safety events)
- Business logic constants (threshold names, fault codes)

→ **Create a custom interface** in `<project>_interfaces/`.

## Quick reference: standard types by domain

```
Sensors (non-vision) → sensor_msgs
  Imu, LaserScan, JointState, Range, BatteryState, NavSatFix, Joy

Vision → sensor_msgs + vision_msgs
  Image, CompressedImage, CameraInfo, PointCloud2
  Detection2D, Detection2DArray

Navigation → nav_msgs + nav2_msgs
  Odometry, Path, OccupancyGrid
  NavigateToPose, NavigateThroughPoses, FollowPath

Control → geometry_msgs
  Twist (cmd_vel), PoseStamped, TransformStamped, Wrench

Infrastructure → std_msgs + std_srvs + tf2_msgs + diagnostic_msgs
  Bool, String, Empty, Header
  Trigger, Empty
  TFMessage
  DiagnosticArray
```

## URL reference

- sensor_msgs: https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html
- std_msgs: https://docs.ros2.org/latest/api/std_msgs/index-msg.html
- geometry_msgs: https://docs.ros2.org/latest/api/geometry_msgs/index-msg.html
- nav_msgs: https://docs.ros2.org/latest/api/nav_msgs/index-msg.html
- diagnostic_msgs: https://docs.ros2.org/latest/api/diagnostic_msgs/index-msg.html
- vision_msgs: https://docs.ros2.org/latest/api/vision_msgs/index-msg.html
- std_srvs: https://docs.ros2.org/latest/api/std_srvs/index-srv.html
- actionlib_msgs: https://docs.ros2.org/latest/api/actionlib_msgs/index-msg.html