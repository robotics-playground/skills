# Node Design Index

Complete node reference for the **<project>** system.
Each file documents: identity, topics, services, actions, parameters, lifecycle behavior, and dependencies.

The system spans <N> execution tiers:

- **<Firmware tier>** (real-time, <RTOS>, micro-ROS over <transport>) — `<project>_firmware` is documented as a single conceptual node from the ROS 2 graph's perspective.
- **<Linux/PC tier>** (ROS 2 <distro>) — Cyclone DDS localhost-unicast, `ROS_DOMAIN_ID=<N>`.

> **Three-Layer Logical Architecture** (only include if the architecture uses this pattern) — see `../../architecture.md` §Three-Layer Logical Architecture.
> **Spinal Cord** = firmware atomic safety + PC-side `<project>_safety` package.
> **Cerebellum** = `<project>_bt_nodes` (BT.CPP) + Nav2 + EKF + SLAM.
> **Cortex** = `<project>_mission`, future `<project>_perception`, future `<project>_llm_behavior`.

---

## Custom Nodes

### Firmware Bridge

| Node | File | Description |
|------|------|-------------|
| `<project>_firmware` | [<project>_firmware.md](<project>_firmware.md) | Microcontroller micro-ROS client — motor PID + sensor publishers + cmd_vel/etc subscribers + firmware-local safety |
| `micro_ros_agent` | [micro_ros_agent.md](micro_ros_agent.md) | XRCE-DDS bridge to Cyclone DDS |

(Add more groups as needed: Spinal Cord, Cerebellum, Cortex, Phase 2 placeholder, etc.)

---

## External / Configured Nodes

### Sensor Fusion & TF

| Node | File | Description |
|------|------|-------------|
| `robot_state_publisher` | [robot_state_publisher.md](robot_state_publisher.md) | URDF → `/tf_static`, `/joint_states` → `/tf` |
| `ekf_node` | [ekf_node.md](ekf_node.md) | `robot_localization` 2D EKF |

### SLAM

| Node | File | Description |
|------|------|-------------|
| `async_slam_toolbox_node` | [async_slam_toolbox_node.md](async_slam_toolbox_node.md) | Real-time async SLAM mapping & loop closure |

### Nav2 Stack

| Node | File | Description |
|------|------|-------------|
| `nav2_bt_navigator` | [nav2_bt_navigator.md](nav2_bt_navigator.md) | Nav2 internal BT — serves `/navigate_to_pose` action |
| `nav2_planner_server` | [nav2_planner_server.md](nav2_planner_server.md) | Global planner (NavFn / Smac) |
| `nav2_controller_server` | [nav2_controller_server.md](nav2_controller_server.md) | DWB local controller |
| `nav2_collision_monitor` | [nav2_collision_monitor.md](nav2_collision_monitor.md) | Layer 2 safety |
| `map_server` | [map_server.md](map_server.md) | Static map loader |
| `amcl` | [amcl.md](amcl.md) | Particle filter localisation |
| `nav2_lifecycle_manager` | [nav2_lifecycle_manager.md](nav2_lifecycle_manager.md) | Nav2 lifecycle orchestrator |

### Visualization & Teleop

| Node | File | Description |
|------|------|-------------|
| `foxglove_bridge` | [foxglove_bridge.md](foxglove_bridge.md) | WebSocket :8765 — visualization + teleop |
| `joy_node` + `teleop_twist_joy` | [joy_teleop.md](joy_teleop.md) | Gamepad → `/cmd_vel` |

---

## Topic & QoS Conventions

| Topic class | QoS profile | Examples |
|---|---|---|
| High-rate sensor streams | **Best Effort, Volatile, Keep Last 1** | `/imu/data_raw`, `/scan`, `/odom/unfiltered` |
| Control commands | **Reliable, Volatile, Keep Last 1** | `/cmd_vel`, `/servo/command` |
| TF & static descriptions | **Reliable, Transient Local, Keep Last 1** | `/tf_static`, `/robot_description`, `/map` |
| Mission/safety state | **`DECISION_QOS` = Reliable + Transient Local, Keep Last 1** | `/mission/status`, `/safety/override_event`, `/sensor_fault` |
| Diagnostics | **Reliable, Volatile, Keep Last 5** | `/diagnostics` |

micro-ROS publishers use Best Effort + Volatile across the WiFi UDP boundary regardless of the ROS-side profile.

---

## Related Documentation

| Document | Path |
|----------|------|
| Architecture | [`../../architecture.md`](../../architecture.md) |
| PRD | [`../../prd.md`](../../prd.md) |
| Epics | [`../../epics.md`](../../epics.md) |
| Roadmap | [`../../roadmap.md`](../../roadmap.md) |
| Package structure | [`../package-structure.md`](../package-structure.md) |
| Custom interfaces | [`../interfaces/index.md`](../interfaces/index.md) |
| Diagrams | [`../diagrams/`](../diagrams/) |
| Wiring | [`<hardware-wiring-guide-path>`](<hardware-wiring-guide-path>) |
