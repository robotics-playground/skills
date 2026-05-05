# Interfaces Index

<Project> has <no REST API in Phase 1 — operator UI is via Foxglove WebSocket / a REST API in `<package>_tablet`>.

(Phase 3 / future API surfaces note here, if applicable.)

---

## File index

| File | Description |
|------|-------------|
| [<project>_interfaces.md](<project>_interfaces.md) | Custom ROS 2 message, service, and action definitions in the `<project>_interfaces` package — `<list of types>`. (Phase 2 additions: ...) |

> **ROS 2 standard interfaces** used unchanged (no custom wrapper):
>
> - `geometry_msgs/Twist`, `geometry_msgs/PoseStamped`, `geometry_msgs/Pose2D`, `geometry_msgs/PoseWithCovarianceStamped`
> - `nav_msgs/Odometry`, `nav_msgs/OccupancyGrid`, `nav_msgs/Path`
> - `sensor_msgs/Imu`, `sensor_msgs/LaserScan`, `sensor_msgs/Range`, `sensor_msgs/JointState`, `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, `sensor_msgs/BatteryState`
> - `std_msgs/Float32`, `std_msgs/Bool`, `std_msgs/UInt8`, `std_msgs/String`, `std_msgs/Empty`
> - `std_srvs/srv/Trigger`, `std_srvs/srv/Empty`
> - `nav2_msgs/action/NavigateToPose`, `nav2_msgs/action/NavigateThroughPoses`, `nav2_msgs/action/FollowPath`, `nav2_msgs/srv/LoadMap`, `nav2_msgs/srv/ManageLifecycleNodes`
> - `vision_msgs/Detection2DArray` (if Phase 2 perception)
> - `diagnostic_msgs/DiagnosticArray`
> - `tf2_msgs/TFMessage`
>
> Per-node usage is documented in the corresponding node file under [`../nodes/`](../nodes/).

---

## Conventions

- **Custom interface package name:** `<project>_interfaces` (CMake-based ROS 2 package)
- **Naming:** `PascalCase` for type names; `snake_case` for fields
- **First field convention:** every custom `.msg` begins with `builtin_interfaces/Time timestamp` for audit traceability
- **Service rejection conventions:** boolean `accepted` / `success` field; `string rejection_reason`; current default value returned even on rejection so caller can re-issue with valid input

---

## Related Documentation

| Document | Path |
|----------|------|
| Per-node interface usage | [`../nodes/`](../nodes/) — every `Published Topics`, `Subscribed Topics`, `Services Served`, `Actions Served` section |
| Package structure | [`../package-structure.md`](../package-structure.md) — `<project>_interfaces` build details |
| Architecture: Communication Patterns | [`../../architecture.md`](../../architecture.md) §Communication Patterns |
