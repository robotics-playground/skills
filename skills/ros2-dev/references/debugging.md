# Debugging and Diagnostics Reference

> ROS 2 Jazzy | CLI tools, rqt, RViz, DDS, logging, ros2 bag

---

## 1. CLI Tools Overview

ROS 2 CLI tools are your primary debugging interface. Learn these before anything else.

### Node Inspection

```bash
# List all running nodes
ros2 node list

# Get detailed info about a node (publishers, subscribers, services, actions)
ros2 node info /camera_driver
# Output shows:
#   Subscribers: /parameter_events, /scan
#   Publishers: /camera/image_raw, /camera/camera_info
#   Service Servers: /camera_driver/describe_parameters, ...
#   Action Servers: (none)
```

### Topic Inspection

```bash
# List all active topics
ros2 topic list
ros2 topic list -t     # Show message types too

# See data on a topic
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel --once           # Show one message and exit
ros2 topic echo /cmd_vel --field linear.x # Show only one field
ros2 topic echo /scan --qos-reliability best_effort  # Match QoS

# Check message rate
ros2 topic hz /camera/image_raw
# Output: average rate: 30.02 Hz

# Check bandwidth
ros2 topic bw /camera/image_raw
# Output: average: 45.23 MB/s

# Check latency (delay between publish and receipt)
ros2 topic delay /camera/image_raw
# Requires messages with Header (stamped messages)

# Show topic info with QoS details
ros2 topic info /scan -v
# Shows publisher count, subscriber count, and QoS profiles for each

# Publish a test message from CLI
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Publish once and exit
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Service Inspection

```bash
# List all services
ros2 service list
ros2 service list -t    # Show service types

# Get service type
ros2 service type /get_status

# Call a service from CLI
ros2 service call /get_status my_robot_msgs/srv/GetStatus \
  "{item_id: 'item_001'}"
```

### Parameter Inspection

```bash
# List parameters for a node
ros2 param list /camera_driver

# Get a parameter value
ros2 param get /camera_driver fps

# Set a parameter at runtime
ros2 param set /camera_driver fps 15.0

# Dump all parameters to YAML
ros2 param dump /camera_driver

# Load parameters from YAML
ros2 param load /camera_driver camera_params.yaml
```

### Interface Inspection

```bash
# Show message definition
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show sensor_msgs/msg/LaserScan
ros2 interface show my_robot_msgs/srv/GetStatus

# List all available message types
ros2 interface list

# Find messages in a package
ros2 interface package sensor_msgs
```

---

## 2. rqt_graph: Visualizing Node Connections

rqt_graph shows a visual graph of all nodes and their topic connections.
This is the fastest way to understand how your system is wired.

```bash
# Launch rqt_graph
ros2 run rqt_graph rqt_graph
# Or: rqt  (then select Plugins > Introspection > Node Graph)
```

**What to look for:**
- Disconnected nodes (no edges) -- something isn't publishing or subscribing
- Missing connections -- topic name mismatch or QoS mismatch
- Unexpected connections -- nodes connected that shouldn't be
- Cycles -- usually indicates a design problem

### Other Useful rqt Plugins

| Plugin | Command | Purpose |
|---|---|---|
| `rqt_graph` | `rqt_graph` | Node/topic connection graph |
| `rqt_console` | `rqt_console` | Filter and view log messages |
| `rqt_topic` | `rqt` > Topics | Monitor topic data in real-time |
| `rqt_plot` | `rqt_plot` | Plot numeric topic values over time |
| `rqt_image_view` | `rqt_image_view` | View camera image topics |
| `rqt_reconfigure` | `rqt_reconfigure` | Change node parameters with GUI |
| `rqt_tf_tree` | `rqt_tf_tree` | View TF frame tree |

---

## 3. RViz2: Sensor Data Visualization

RViz2 is the primary visualization tool for 3D sensor data, robot models, and
coordinate frames.

```bash
# Launch RViz with default config
rviz2

# Launch with a saved config
rviz2 -d /path/to/config.rviz
```

### Essential RViz Displays

| Display Type | Visualizes | Configuration |
|---|---|---|
| RobotModel | URDF robot model | Set Description Topic to `/robot_description` |
| TF | Coordinate frames | Shows all frames as arrows |
| LaserScan | 2D LiDAR data | Set topic, adjust size/color by intensity |
| PointCloud2 | 3D point cloud | Set topic, adjust point size |
| Image | Camera image | Set topic |
| Marker | Custom shapes, text, arrows | Published programmatically |
| Path | Navigation paths | Set topic |
| Map | Occupancy grid | Set topic to `/map` |
| Odometry | Position + velocity arrows | Set topic to `/odom` |

### Creating an RViz Config for Your Robot

1. Open `rviz2`
2. Add displays: RobotModel, TF, LaserScan, Image
3. Set Fixed Frame to `map` or `odom`
4. Configure each display's topic
5. Save config: File > Save Config As > `robot.rviz`
6. Include in launch file for quick access

---

## 4. ros2 doctor: System Diagnostics

`ros2 doctor` checks your ROS 2 installation and runtime environment for
common problems.

```bash
# Quick check
ros2 doctor

# Full report
ros2 doctor --report

# Check specific aspects
ros2 doctor --report --include-warnings
```

**Common issues it catches:**
- Network configuration problems
- DDS middleware issues
- Missing environment variables
- Package installation problems
- RMW (middleware) mismatches

---

## 5. DDS Troubleshooting

DDS (Data Distribution Service) is the middleware layer. Most "can't see topics"
problems are DDS issues.

### Discovery Problems

```bash
# Check which DDS implementation is being used
echo $RMW_IMPLEMENTATION
# Default: rmw_fastrtps_cpp (Fast-DDS)
# Alternative: rmw_cyclonedds_cpp (Cyclone DDS)

# Set DDS implementation (must be same on all machines)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Domain ID

Nodes on different domain IDs cannot see each other. Default is 0.

```bash
# Check current domain ID
echo $ROS_DOMAIN_ID

# Set domain ID (all machines in the same robot must match)
export ROS_DOMAIN_ID=42

# Restrict to localhost (useful for development, avoids cross-machine interference)
export ROS_LOCALHOST_ONLY=1
```

### Multicast Issues

DDS uses multicast for discovery. If multicast is blocked (common on corporate
networks, VMs, Docker), nodes can't find each other.

```bash
# Test multicast connectivity
ros2 multicast receive &
ros2 multicast send

# If multicast is blocked, use Cyclone DDS with unicast config:
```

Create `cyclone_dds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.100"/>  <!-- Other machine IP -->
        <Peer address="localhost"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file://$(pwd)/cyclone_dds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Common DDS Issues

| Symptom | Cause | Fix |
|---|---|---|
| Can't see topics from another machine | Different `ROS_DOMAIN_ID` | Set same ID on all machines |
| Can't see topics from another machine | Firewall blocking UDP multicast | Open UDP ports 7400-7500 |
| Can't see topics from another machine | Different DDS implementations | Set same `RMW_IMPLEMENTATION` |
| Can't see topics in Docker | Multicast not forwarded | Use `--network=host` or configure unicast |
| Intermittent message loss | Network congestion or WiFi | Use wired connection; adjust QoS |
| High latency | DDS discovery overhead | Use Cyclone DDS; tune discovery interval |
| "RMW implementation not found" | Missing package | `sudo apt install ros-jazzy-rmw-cyclonedds-cpp` |

---

### micro-ROS + Docker: Topics Not Appearing in `ros2 topic list`

This is a multi-layer failure mode with three independent root causes. Work through them in order.

#### Root cause 1 — Wrong DDS domain (most common)

`ROS_DOMAIN_ID` env var is **ignored** by the XRCE bridge inside `micro_ros_agent`. The bridge creates Fast DDS participants using the `domain_id` sent by the firmware in the XRCE `CREATE PARTICIPANT` request — and the firmware defaults to domain 0.

**Diagnosis:** scan all domains from the sim container:
```bash
for d in 0 1 2 3 42; do
  echo "=== DOMAIN $d ===" && \
  docker exec tank-sim bash -c "source /opt/ros/jazzy/setup.bash && CYCLONEDDS_URI='' ROS_DOMAIN_ID=$d ros2 topic list --no-daemon 2>/dev/null | grep -v '^/parameter\|^/rosout'"
done
```
If `/heartbeat` appears on domain 0 but the stack uses domain 42, the firmware is using domain 0.

**Fix (firmware side — authoritative):** use `rcl_init_options_set_domain_id()` before `rclc_support_init_with_options()`:
```cpp
#include <rcl/init_options.h>

rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
rcl_init_options_init(&init_options, allocator);
rcl_init_options_set_domain_id(&init_options, 42);   // ← must match ROS_DOMAIN_ID
rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
rmw_uros_options_set_client_key(client_key, rmw_options);
rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator);
rcl_init_options_fini(&init_options);
```

**Alternative (Docker side):** set `XRCE_DOMAIN_ID_OVERRIDE=42` on the agent container AND send `domain_id=255` (`UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV`) from firmware. The firmware approach is cleaner.

#### Root cause 2 — Network interface mismatch (shared-namespace Docker)

When using `network_mode: "service:agent"` (shared network namespace), the micro-ros-agent entrypoint generates `/tmp/disable_fastdds_shm.xml` with no `interfaceWhiteList` — Fast DDS binds to **all interfaces** including `eth0`. If Cyclone DDS is configured to use `lo` only (a common "localhost-only" config), they're on different interfaces and SPDP multicast never reaches Cyclone DDS.

**Diagnosis:** test with Cyclone DDS unrestricted:
```bash
docker exec tank-sim bash -c "source /opt/ros/jazzy/setup.bash && CYCLONEDDS_URI='' ros2 topic list --no-daemon 2>/dev/null"
```
If the topic appears here but not with your `CYCLONEDDS_URI`, the interface is the problem.

**Fix:** change `cyclonedds.xml` to use `eth0` (the container's shared interface), not `lo`:
```xml
<General>
  <Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="true"/>
  </Interfaces>
  <AllowMulticast>true</AllowMulticast>
</General>
```

#### Root cause 3 — Entrypoint overrides FASTRTPS_DEFAULT_PROFILES_FILE

`microros/micro-ros-agent:jazzy` sets `MICROROS_DISABLE_SHM=1` and its entrypoint (`/micro-ros_entrypoint.sh`) generates `/tmp/disable_fastdds_shm.xml` at startup, then overwrites `FASTRTPS_DEFAULT_PROFILES_FILE` in the agent process environment. Any `FASTRTPS_DEFAULT_PROFILES_FILE` you set in `docker-compose.yml` is silently replaced.

**Diagnosis:**
```bash
docker exec tank-agent cat /proc/$(pgrep micro_ros_agent)/environ | tr '\0' '\n' | grep FASTRTPS
# Will show /tmp/disable_fastdds_shm.xml, not your file
```

**Consequence:** custom Fast DDS XML profiles (unicast initial peers, interfaceWhiteList) are ignored. The agent always uses the entrypoint-generated profile, which has plain UDPv4 on all interfaces.

**Fix options:**
- Patch the entrypoint (complex, brittle across image updates) — generally avoid
- Rely on the firmware `rcl_init_options_set_domain_id()` fix (root cause 1) so domain is correct, and fix `cyclonedds.xml` to match Fast DDS's interface (root cause 2)
- Set `ROS_LOCALHOST_ONLY=1` only if you want the entrypoint to apply `disable_fastdds_shm_localhost_only.xml` — but this also overrides `CYCLONEDDS_URI` for CLI tools, causing `rmw_create_node: failed to create domain`

#### XRCE session stability — client_key

If topics appear briefly then vanish, the XRCE session is churning. A random `client_key` per boot creates a new DDS participant on each reconnect; the old participant lingers past DDS liveliness timeouts, causing graph churn.

**Fix:** derive `client_key` from a stable MAC address:
```cpp
#include <esp_mac.h>
uint8_t mac[6];
esp_read_mac(mac, ESP_MAC_WIFI_STA);
uint32_t client_key = (uint32_t)mac[2] << 24 | (uint32_t)mac[3] << 16
                    | (uint32_t)mac[4] << 8  | (uint32_t)mac[5];
rmw_uros_options_set_client_key(client_key, rmw_options);
```
A stable key causes the agent to log `session re-established` (instant) instead of `session created` (triggers full RTPS discovery cycle).

---

## 6. Logging

### Python Logging API

```python
# In a node -- always use the node's logger
self.get_logger().debug('Detailed info for development')
self.get_logger().info('Normal operational info')
self.get_logger().warn('Something unexpected but not fatal')
self.get_logger().error('Something failed')
self.get_logger().fatal('Unrecoverable error')

# With throttling (to avoid log spam)
self.get_logger().info('Processing frame', throttle_duration_sec=5.0)
# Logs at most once every 5 seconds

# With skip first (don't log the first occurrence)
self.get_logger().warn('Sensor noise detected', skip_first=True)

# Once only
self.get_logger().info('Node initialized', once=True)
```

### Setting Log Levels

```bash
# At launch time
ros2 run my_package my_node --ros-args --log-level debug

# Per-logger at launch time
ros2 run my_package my_node --ros-args --log-level my_node:=debug

# At runtime (change while node is running, Jazzy+)
ros2 service call /my_node/set_logger_levels \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'my_node', level: 10}]}"
# Levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
```

### Log to File

```bash
# Logs go to ~/.ros/log/ by default
# Set custom log directory
export ROS_LOG_DIR=/path/to/logs

# View recent log files
ls ~/.ros/log/latest_build/
```

**Never use `print()` in ROS 2 nodes.** Always use `self.get_logger()`. The
logger integrates with ROS 2 tooling (rqt_console, log files, level filtering).

---

## 7. ros2 bag: Data Recording and Playback

ros2 bag records topic data to files for offline analysis, debugging, and
testing. It captures all messages on selected topics for later playback.

### Recording

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /camera/image_raw /odom /tf /tf_static

# Record with a name
ros2 bag record -o my_test_run /scan /odom

# Record with compression
ros2 bag record -o my_test_run --compression-mode message \
  --compression-format zstd /scan /odom

# Record for a duration
ros2 bag record -o timed_recording --max-duration 60 /scan /odom
```

### Playback

```bash
# Play back a recording
ros2 bag play my_test_run

# Play at half speed
ros2 bag play my_test_run --rate 0.5

# Play and loop
ros2 bag play my_test_run --loop

# Play starting from a specific time
ros2 bag play my_test_run --start-offset 30.0

# Remap topics during playback
ros2 bag play my_test_run --remap /scan:=/playback/scan
```

### Inspecting Bag Files

```bash
# Show bag info (topics, message counts, duration)
ros2 bag info my_test_run
# Output:
#   Duration: 120.5s
#   Topics:
#     /scan     sensor_msgs/msg/LaserScan    1205 msgs
#     /odom     nav_msgs/msg/Odometry        6025 msgs
#     /tf       tf2_msgs/msg/TFMessage       6025 msgs
```

### Bag Recording Best Practices

- **Always record /tf and /tf_static** -- without transforms, sensor data
  is useless for offline analysis
- **Record raw sensor data** (image_raw, scan) not processed data -- you can
  reprocess offline
- **Use compression** for long recordings or when disk space is limited
- **Name bags descriptively** -- `hallway_run_2026-04-15` not `test1`
- **Record system state** too -- `/diagnostics`, `/battery_state`

---

## 8. Network Debugging

### Environment Variables

| Variable | Default | Purpose |
|---|---|---|
| `ROS_DOMAIN_ID` | 0 | Isolate ROS 2 groups (0-232) |
| `ROS_LOCALHOST_ONLY` | 0 | Restrict to localhost when set to 1 |
| `RMW_IMPLEMENTATION` | rmw_fastrtps_cpp | DDS middleware selection |
| `ROS_LOG_DIR` | ~/.ros/log | Log file directory |
| `CYCLONEDDS_URI` | (none) | Cyclone DDS config file path |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | (none) | Fast-DDS config file path |

### Firewall Configuration

```bash
# Allow ROS 2 DDS traffic (Linux UFW)
sudo ufw allow 7400:7500/udp    # DDS discovery
sudo ufw allow 7400:7500/tcp    # DDS data
```

### Testing Cross-Machine Communication

```bash
# On machine A:
export ROS_DOMAIN_ID=42
ros2 topic pub /test std_msgs/msg/String "{data: 'hello from A'}"

# On machine B:
export ROS_DOMAIN_ID=42
ros2 topic echo /test
# Should see: data: 'hello from A'
```

---

## 9. Performance Profiling

### Callback Duration

```python
import time

class ProfilingNode(Node):
    def __init__(self):
        super().__init__('profiling_node')
        self.sub = self.create_subscription(
            LaserScan, 'scan', self.callback, 10
        )
        self._callback_durations = []

    def callback(self, msg):
        start = time.monotonic()

        # ... processing ...

        duration = time.monotonic() - start
        self._callback_durations.append(duration)

        if len(self._callback_durations) >= 100:
            avg = sum(self._callback_durations) / len(self._callback_durations)
            max_d = max(self._callback_durations)
            self.get_logger().info(
                f'Callback stats (last 100): avg={avg*1000:.1f}ms, '
                f'max={max_d*1000:.1f}ms'
            )
            self._callback_durations.clear()
```

### Message Latency

```python
from rclpy.time import Time

def callback(self, msg):
    if hasattr(msg, 'header'):
        now = self.get_clock().now()
        msg_time = Time.from_msg(msg.header.stamp)
        latency = (now - msg_time).nanoseconds / 1e6  # milliseconds
        self.get_logger().debug(f'Message latency: {latency:.1f}ms')
```

### System-Level Profiling

```bash
# CPU usage per node
top -p $(pgrep -f my_node)

# Memory usage
ps aux | grep ros

# CPU and memory for all ROS nodes
ros2 node list | while read node; do
  pid=$(ros2 node info $node 2>/dev/null | grep -oP 'PID: \K\d+')
  if [ -n "$pid" ]; then
    echo "$node (PID $pid):"
    ps -p $pid -o %cpu,%mem,rss
  fi
done
```

---

## 10. Diagnostic Flowchart

When something isn't working, follow this systematic approach:

```
PROBLEM: Node not receiving messages
│
├── 1. Is the publishing node running?
│   └── ros2 node list
│       ├── Not listed → Node crashed or not started. Check launch file.
│       └── Listed → Continue
│
├── 2. Is the topic being published?
│   └── ros2 topic list
│       ├── Topic missing → Publisher not created. Check node code.
│       └── Topic exists → Continue
│
├── 3. Is data flowing?
│   └── ros2 topic hz /my_topic
│       ├── No data → Publisher not calling publish(). Check timer/callback.
│       └── Data flowing → Continue
│
├── 4. Can you see the data?
│   └── ros2 topic echo /my_topic (try --qos-reliability best_effort)
│       ├── No output → QoS mismatch with echo tool
│       └── Data visible → Continue
│
├── 5. Is your subscriber connected?
│   └── ros2 node info /my_subscriber_node
│       ├── Not subscribed → Wrong topic name. Check create_subscription().
│       └── Subscribed → Continue
│
├── 6. QoS mismatch?
│   └── ros2 topic info /my_topic -v
│       ├── Publisher: BEST_EFFORT, Subscriber: RELIABLE → MISMATCH! Fix QoS.
│       └── QoS compatible → Continue
│
├── 7. Is the callback being called?
│   └── Add self.get_logger().info('CALLBACK FIRED') at top of callback
│       ├── Not firing → Executor not spinning. Check rclpy.spin().
│       └── Firing → Bug is in callback logic, not communication.
│
└── 8. Still stuck?
    ├── ros2 doctor --report
    ├── Check ROS_DOMAIN_ID matches
    ├── Check ROS_LOCALHOST_ONLY
    └── Try RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

```
PROBLEM: Transform lookup fails
│
├── 1. Are transforms being published?
│   └── ros2 topic echo /tf
│       ├── No data → No transform broadcaster running
│       └── Data flowing → Continue
│
├── 2. Is the frame tree connected?
│   └── ros2 run tf2_tools view_frames → check frames.pdf
│       ├── Disconnected tree → Missing broadcaster for connecting transform
│       └── Connected → Continue
│
├── 3. Frame name typo?
│   └── Compare frame names in view_frames with your code
│       ├── Mismatch → Fix frame name string
│       └── Match → Continue
│
├── 4. Timing issue?
│   └── ros2 run tf2_ros tf2_monitor
│       ├── Large delays → Transform broadcaster too slow
│       └── Normal → Check lookup time parameter
│
└── 5. Use rclpy.time.Time() for latest transform
```

---

## 11. Common Error Messages and What They Mean

| Error Message | Meaning | Fix |
|---|---|---|
| `[WARN] Subscription already connected to a publisher with incompatible QoS` | QoS mismatch (silent data loss) | Match QoS profiles |
| `Could not find requested resource` | Package or file not found during build | Check package.xml dependencies |
| `No executable found` | Missing entry point in setup.py | Add console_scripts entry; rebuild |
| `Lookup would require extrapolation into the future` | TF timestamp mismatch | Use `rclpy.time.Time()` for latest |
| `Lookup would require extrapolation into the past` | TF data not yet available or too old | Increase TF buffer timeout; check broadcaster rate |
| `Multiple publishers within a node on the same topic` | Two publishers on the same topic | Consolidate to one publisher |
| `Timer period must be >= 0` | Invalid timer period | Check parameter value (must be positive) |
| `Node name must not be empty` | Missing node name in constructor | Pass a name to `super().__init__('name')` |
| `parameter 'X' has already been declared` | Declaring the same parameter twice | Remove duplicate `declare_parameter()` |
| `Service server already exists` | Duplicate service name | Use unique service names |
