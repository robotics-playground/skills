---
title: "ROS 2 Debugging Standards"
category: "robotics"
tags: ["ros2", "debugging", "diagnostics", "rqt", "rviz2", "dds", "rosbag", "troubleshooting"]
description: "Debugging tools, techniques, and common error solutions for ROS 2 development"
last_updated: "2026-03-18"
---

# ROS 2 Debugging Standards

> **Philosophy:** ROS 2 debugging is primarily about inspecting the communication graph. Most bugs are misconfigured topics, QoS mismatches, or transform errors — not logic bugs. Learn the CLI tools first.

## CLI Diagnostic Tools Reference

### Node Inspection

```bash
# List all running nodes
ros2 node list

# Show node details (publishers, subscribers, services, parameters)
ros2 node info /scan_processor

# Check if a specific node is running
ros2 node list | grep scan_processor
```

### Topic Inspection

```bash
# List all active topics with message types
ros2 topic list -t

# Show topic message type
ros2 topic type /scan

# Monitor messages in real-time
ros2 topic echo /scan

# Show publish rate (Hz) — first thing to check if "nothing works"
ros2 topic hz /scan

# Show bandwidth usage
ros2 topic bw /scan

# Show message count and delay
ros2 topic delay /scan

# Publish a test message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

### Service Inspection

```bash
# List available services
ros2 service list

# Show service type
ros2 service type /trigger_inspection

# Call a service
ros2 service call /trigger_inspection std_srvs/srv/Trigger
```

### Parameter Inspection

```bash
# List all parameters for a node
ros2 param list /scan_processor

# Get a parameter value
ros2 param get /scan_processor max_range

# Set a parameter at runtime
ros2 param set /scan_processor max_range 25.0

# Dump all parameters to YAML
ros2 param dump /scan_processor
```

### TF (Transform) Inspection

```bash
# View the full TF tree as text
ros2 run tf2_tools view_frames

# Monitor a specific transform
ros2 run tf2_ros tf2_echo map base_link

# Check if a transform exists and its latency
ros2 run tf2_ros tf2_monitor map base_link
```

## GUI Tools

### rqt_graph — Node/Topic Visualization

```bash
# Launch the computation graph viewer
ros2 run rqt_graph rqt_graph
```

**What to look for:**
- Disconnected nodes (publisher with no subscriber or vice versa)
- Unexpected topic remappings
- Missing nodes that should be running
- Circular dependencies

### rqt_console — Log Viewer

```bash
ros2 run rqt_console rqt_console
```

Filter by severity, node name, or message content. Essential for catching warnings from nodes that are not in the foreground terminal.

### rqt_topic — Topic Monitor

```bash
# Monitor topic rates and bandwidth in a GUI
rqt
# Then select Plugins → Topics → Topic Monitor
```

### rqt_tf_tree — Transform Tree Viewer

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

Shows the full transform tree with publish rates and latencies. First tool to open when transforms are broken.

### RViz2 — 3D Visualization

```bash
ros2 run rviz2 rviz2
```

**Essential displays to add:**
| Display Type | Topic | Purpose |
|-------------|-------|---------|
| LaserScan | `/scan` | Verify LiDAR data |
| TF | (auto) | Verify transform tree |
| RobotModel | `/robot_description` | Verify URDF |
| Map | `/map` | Verify SLAM output |
| Path | `/plan` | Verify navigation paths |
| PointCloud2 | `/points` | Verify depth data |
| Image | `/camera/image_raw` | Verify camera feed |

**Key setting:** Set the **Fixed Frame** to `map` or `odom` (must match your TF tree root). If RViz shows nothing, the fixed frame is almost always wrong.

## ros2 doctor

```bash
# Run system diagnostics
ros2 doctor

# Verbose report
ros2 doctor --report

# Check specific middleware
ros2 doctor --report | grep -A5 middleware
```

Checks: RMW implementation, network configuration, DDS discovery, running nodes.

## DDS Troubleshooting

### Common DDS Issues

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| Nodes can't discover each other | Different `ROS_DOMAIN_ID` | Ensure same ID on all machines |
| Topics visible but no data | QoS mismatch | Check reliability/durability settings |
| Slow discovery on startup | Large network, multicast blocked | Set `ROS_LOCALHOST_ONLY=1` for single-machine |
| Random disconnections | DDS lease duration too short | Increase liveliness lease duration |
| Works locally, fails across machines | Firewall blocking UDP | Open ports 7400-7500 (DDS discovery) |

### Domain ID

```bash
# Set domain ID (default is 0, all machines must match)
export ROS_DOMAIN_ID=42

# Restrict to localhost only (single machine development)
export ROS_LOCALHOST_ONLY=1
```

### QoS Debugging

```bash
# Check QoS settings on a topic (shows all publishers and subscribers)
ros2 topic info /scan --verbose
```

**Common QoS mismatch:** Publisher uses `BEST_EFFORT` but subscriber expects `RELIABLE` (or vice versa). Data flows but subscriber silently drops messages.

| Publisher QoS | Subscriber QoS | Result |
|--------------|----------------|--------|
| RELIABLE | RELIABLE | Works |
| BEST_EFFORT | BEST_EFFORT | Works |
| RELIABLE | BEST_EFFORT | Works (subscriber downgrades) |
| BEST_EFFORT | RELIABLE | **FAILS — no data received** |

## Common Error Messages Catalog

| Error Message | Cause | Solution |
|--------------|-------|----------|
| `Could not find requested resource` | Package not sourced | Run `source install/setup.bash` |
| `No executable found` | Entry point missing in setup.py | Add to `console_scripts` in `setup.py` |
| `Lookup would require extrapolation into the future` | TF timestamp mismatch, `use_sim_time` not set | Set `use_sim_time:=true` in simulation |
| `Transform timeout` | TF publisher not running or too slow | Check `ros2 run tf2_ros tf2_monitor` |
| `QoS incompatible` | Subscriber/publisher QoS mismatch | Match reliability and durability settings |
| `[WARN] subscription matched but no data` | QoS depth too small, publisher too fast | Increase queue depth or use KEEP_ALL |
| `Could not load the Qt platform plugin` | No display server (headless) | Set `export QT_QPA_PLATFORM=offscreen` |
| `Failed to find package` | Not built or not sourced | `colcon build --packages-select <pkg>` then source |
| `ModuleNotFoundError` | Python package not installed | Rebuild with `--symlink-install` |
| `Parameter not declared` | Parameter used before `declare_parameter()` | Declare all parameters in `__init__` |

## Logging Configuration

### Setting Log Levels

```bash
# Set log level for a specific node at launch
ros2 run my_robot_inspection scan_processor --ros-args --log-level debug

# Set log level at runtime (Jazzy+)
ros2 service call /scan_processor/set_logger_levels \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'scan_processor', level: 10}]}"
```

**Log levels:** DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50

### Enabling Console Output

```bash
# Colorized console output (default)
export RCUTILS_COLORIZED_OUTPUT=1

# Log to file
export ROS_LOG_DIR=/tmp/ros2_logs
```

## ros2 bag — Data Capture and Replay

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /cmd_vel /odom -o test_run_001

# Record with compression
ros2 bag record /scan /odom -o test_run --compression-mode file

# Show bag info
ros2 bag info test_run_001/

# Replay at normal speed
ros2 bag play test_run_001/

# Replay at half speed
ros2 bag play test_run_001/ --rate 0.5

# Replay with clock (for time-sensitive nodes)
ros2 bag play test_run_001/ --clock
```

**When to record bags:**
- Before any hardware test session (always)
- When reproducing a bug (capture the exact scenario)
- For regression testing (replay known-good data)
- For algorithm development (process offline)

## Network Debugging (Multi-Machine)

```bash
# Verify ROS_DOMAIN_ID matches on all machines
echo $ROS_DOMAIN_ID

# Check if DDS discovery is working
ros2 daemon status
ros2 daemon stop && ros2 daemon start

# Test basic connectivity
ros2 topic pub /test std_msgs/msg/String "data: 'hello'" --once
# On other machine:
ros2 topic echo /test

# Check multicast (required for DDS discovery)
ros2 multicast receive &
ros2 multicast send
```

## Performance Debugging

```bash
# Monitor callback execution frequency
ros2 topic hz /scan_processor/min_distance

# Monitor end-to-end latency (requires header timestamps)
ros2 topic delay /scan_processor/min_distance

# CPU/memory per node (standard Linux tools)
top -p $(pgrep -f scan_processor)
htop

# Trace callback execution with ros2_tracing (advanced)
ros2 trace start -s my_session
# ... run your test ...
ros2 trace stop my_session
```

## Systematic Debugging Workflow

When something is not working, follow this order:

1. **Is the node running?** → `ros2 node list`
2. **Are topics being published?** → `ros2 topic list -t` then `ros2 topic hz /topic`
3. **Is the data correct?** → `ros2 topic echo /topic`
4. **Are transforms available?** → `ros2 run tf2_tools view_frames`
5. **Are parameters correct?** → `ros2 param dump /node_name`
6. **Is there a QoS mismatch?** → `ros2 topic info /topic --verbose`
7. **Check logs** → `ros2 run rqt_console rqt_console`
8. **Visualize** → Open RViz2, add relevant displays

## Quick Checklist

- [ ] Know how to use `ros2 topic list/echo/hz/info` from memory
- [ ] Know how to check TF tree with `view_frames` and `tf2_echo`
- [ ] Know how to read QoS compatibility from `topic info --verbose`
- [ ] `ROS_DOMAIN_ID` set consistently across all machines
- [ ] RViz2 fixed frame set correctly (usually `map` or `odom`)
- [ ] Record ros2 bags before every hardware test session
- [ ] Check the common errors table before searching online
- [ ] Use the systematic debugging workflow (node → topic → data → TF → params → QoS)
