# QoS, Naming, and Schema Conventions

## QoS profiles by topic class

These are the conventional QoS classes for ROS 2 robotics — apply them consistently in the design unless the project's architecture explicitly chooses otherwise.

| Topic class | Reliability | Durability | History | Examples |
|---|---|---|---|---|
| **High-rate sensor streams** | Best Effort | Volatile | Keep Last 1 | `/imu/data_raw`, `/scan`, `/odom/unfiltered`, `/sonar/*`, `/tof/scan`, `/camera/*/image_raw` |
| **Control commands** | Reliable | Volatile | Keep Last 1 | `/cmd_vel`, `/servo/command`, `/laser/enable`, `/display/emotion` |
| **TF & static descriptions** | Reliable | Transient Local | Keep Last 1 | `/tf_static`, `/robot_description`, `/map` |
| **Mission/safety state (decisions)** | Reliable | Transient Local | Keep Last 1 | `/mission/status`, `/safety/override_event`, `/sensor_fault`, `/emergency_stop`, `/abort_mission` |
| **Diagnostics** | Reliable | Volatile | Keep Last 5 | `/diagnostics` |

Refer to the "decisions" QoS as `DECISION_QOS` in node specs — it's a recognised shorthand. Make sure the user's architecture or stdlib has a corresponding constant.

## micro-ROS exception

micro-ROS publishers use **Best Effort + Volatile** across the WiFi UDP boundary regardless of the ROS-side profile. This is required by `rmw_microxrcedds` for high-rate streams. Note this exception in the firmware-tier node spec.

## Naming conventions (mandatory)

- **ROS 2 nodes / topics / parameters:** `snake_case` — e.g., `motor_control`, `/imu/data_raw`, `wheel_radius_m`
- **Message / service / action types:** `PascalCase` — e.g., `MissionStatus`, `OverrideThreshold`, `EvaluateQuality`
- **Frame IDs:** `<sensor>_link` — e.g., `imu_link`, `lidar_link`, `tof_turret_link`. Standard: `base_link`, `odom`, `map`
- **Topic namespacing by sensor class:** `/sonar/{front,left,right}` not `/front_sonar`. `/imu/data_raw` (raw) vs `/imu/data` (filtered). `/odom/unfiltered` (encoder-only) vs `/odom` (EKF-fused) — **never** `/odom/filtered`, that breaks `robot_localization` defaults.
- **Custom interface package name:** `<project>_interfaces` (the `<project>` prefix is the snake_case codename; e.g. `acme_robot_interfaces`)
- **Custom domain packages:** `<project>_<domain>` — typical domains are `safety`, `mission`, `bt_nodes`, `perception`, `bringup`, `description`, `interfaces`, `diagnostics`, `<sensor_or_driver>`

## The robot base frame is a contract — pick one name, write it down

`odom → <base> → sensor frames` is the TF spine. Whatever `<base>` is called —
`base_link` or `base_footprint` — **every** consumer must use the *same* name:
the odometry source (firmware, EKF, or a sim drive plugin), `robot_state_publisher`,
SLAM, AMCL, every Nav2 costmap, the collision monitor. A mismatch does not
error — TF lookups just silently fail and the robot "can't localise".

State the chosen base frame name explicitly in the design (package-structure
doc or the description-package node spec) so downstream stories don't each
guess. If the project uses both `base_footprint` (ground projection) and
`base_link` (body origin), specify which one each component's `*base_frame*`
parameter points at — conventionally `base_footprint` for Nav2/SLAM/EKF, with
a fixed `base_footprint → base_link` joint carrying the offset.

## Simulation model: specify the drive mechanism in the design

If the project has a Gazebo sim tier, the description-package spec must say
*how* the simulated robot is driven, because the choice has architectural
consequences the implementer cannot undo cheaply:

- **Body-driven** (`VelocityControl` + `OdometryPublisher`): the plugin sets
  the body twist directly and odometry is *measured* from the body. Odometry
  and physics cannot diverge. **Default for nav/SLAM-focused sims.**
- **Wheel-joint** (`DiffDrive` / `TrackedVehicle`): realistic drivetrain, but
  odometry is *dead-reckoned* from commanded wheel velocity. If the footprint
  is at all unstable it decouples from physics and every downstream consumer
  (TF, SLAM, Nav2) is silently fed a lying pose. Specify this only when wheel
  slip / traction fidelity is itself a requirement.

This belongs in the design, not just the URDF, because "the sim robot uses
body-driven control with `base_footprint` as the odom child frame" is a
contract the EKF spec, the SLAM config, and the Nav2 params all depend on.

## Custom message field rules

- **First field convention:** every custom `.msg` type starts with `builtin_interfaces/Time timestamp` for audit traceability. Apply this consistently — it makes log replay much easier.
- **Field names:** `snake_case`
- **Enums:** prefer `string` over `uint8` for readability in logs (rejected `power_supply_status: 2`, prefer `power_supply_status: "DISCHARGING"`). When you do use `uint8` enums (e.g., for `MissionStatus.status`), declare the constants in the message file:
  ```
  uint8 STATUS_IDLE = 0
  uint8 STATUS_RUNNING = 2
  ...
  uint8 status
  ```
- **Units in field comments** if not obvious from the name: `float64 wait_seconds`, `float32 max_speed_mps`, `int32 cmd_vel_timeout_ms`.
- **Standard ROS types over custom:** never wrap a `std_msgs/Bool` in a custom 1-field type. Use the standard.

## Service rejection convention

Every service that can refuse a request returns:

```
bool    accepted          # or 'success'
<type>  effective_value   # current value if rejected, applied value if accepted
string  rejection_reason  # "" if accepted; one of an enumerated set
```

Enumerate rejection reasons. Examples for `OverrideThreshold.srv`: `"hard_threshold"`, `"out_of_range"`, `"invalid_duration"`, `"unknown_threshold"`. The enumeration goes in the service file's docstring AND in the calling node's spec.

## Standard ROS 2 types — use unchanged, never wrap

> **Important:** Before designing any custom interface, read `references/standard-ros2-types.md` to check whether a standard type already exists. Custom types add build complexity and reduce interoperability.

```
geometry_msgs/Twist · PoseStamped · Pose2D · PoseWithCovarianceStamped
nav_msgs/Odometry · OccupancyGrid · Path
sensor_msgs/Imu · LaserScan · Range · JointState · Image · CameraInfo · BatteryState
std_msgs/Float32 · Bool · UInt8 · String · Empty · Header
std_srvs/srv/Trigger · Empty
nav2_msgs/action/{NavigateToPose, NavigateThroughPoses, FollowPath}
nav2_msgs/srv/{LoadMap, ManageLifecycleNodes}
nav2_msgs/{CollisionMonitorState, ParticleCloud}
vision_msgs/{Detection2D, Detection2DArray}
diagnostic_msgs/DiagnosticArray
tf2_msgs/TFMessage
slam_toolbox/srv/{SaveMap, SerializePoseGraph}
```

For the full catalogue by domain (which type to use for IMU, LIDAR, battery, GPS, etc.), see `references/standard-ros2-types.md`.

If you find yourself reaching for a custom type that has a `std_msgs` / `sensor_msgs` / `geometry_msgs` equivalent, stop and reconsider.

## Topic rate table — produce one for the firmware tier

If there's a firmware tier with multiple publishers, produce a rate table in its node spec:

| Rate | Topics | Timer |
|---|---|---|
| 100 Hz | `/imu/data_raw` | T1 |
| 50 Hz | `/odom/unfiltered` | T2 |
| 20 Hz | `/cliff/status` | T3 |
| 10 Hz | `/tof/scan` | T4 |
| 4 Hz | `/sonar/{front,left,right}` (sequential 60 ms spacing) | T5 |
| 2 Hz | `/joint_states` | T6 |
| 1 Hz | `/battery_voltage`, `/ambient_light` | T7 |

Stagger publishers across timers — never publish two topics from the same timer simultaneously.

## Lifecycle nodes — when to use

Custom application nodes should be `LifecycleNode` (rclpy / rclcpp_lifecycle) if any of:

- They expose long-lived resources (file handles, sockets, GPU sessions)
- They can be paused and resumed (mission orchestrator, BT executor, perception)
- They are part of a managed bring-up sequence (Nav2 stack, all the `<project>_safety` nodes)

Pure passthroughs (heartbeat publisher, simple decoder) can be standalone. Document the choice in the node spec's Identity section.
