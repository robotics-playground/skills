# Debugging Gazebo Harmonic Simulations

This reference covers CLI tools for inspecting simulation state, diagnosing common physics
issues, debugging sensors, profiling performance, and a systematic diagnostic flowchart.

---

## gz topic CLI

The `gz topic` command inspects the internal Gazebo transport layer. This is your primary
tool for checking what data Gazebo is producing, even before the ROS 2 bridge is involved.

### Essential Commands

```bash
# List all active topics
gz topic -l

# Get type info for a topic
gz topic -i -t /world/my_world/model/robot/link/lidar_link/sensor/lidar/scan

# Echo (print) messages on a topic (like rostopic echo)
gz topic -e -t /world/my_world/model/robot/link/base_link/sensor/imu_sensor/imu

# Measure publication frequency
gz topic -hz -t /world/my_world/model/robot/link/lidar_link/sensor/lidar/scan

# Publish a message to a topic
gz topic -t /model/robot/cmd_vel -m gz.msgs.Twist \
  -p 'linear:{x:0.5} angular:{z:0.3}'
```

### When to Use gz topic vs ros2 topic

| Use `gz topic` when | Use `ros2 topic` when |
|---------------------|----------------------|
| Checking if Gazebo is producing data at all | Checking if the bridge is working |
| The ROS 2 bridge is not configured yet | Verifying ROS 2 nodes receive sensor data |
| Debugging sensor output before bridging | Testing end-to-end ROS 2 pipeline |
| Sending direct commands to Gazebo plugins | Sending commands through the ROS 2 stack |

**Debugging strategy:** If a ROS 2 topic has no data, first check with `gz topic -e` whether
Gazebo is publishing the data. If Gazebo publishes but ROS 2 does not receive, the problem
is in the bridge configuration. If Gazebo does not publish, the problem is in the model/sensor
configuration.

---

## gz service CLI

Gazebo services provide request/response communication (like ROS 2 services).

```bash
# List all services
gz service -l

# Get info about a service
gz service -i -s /world/my_world/scene/info

# Call a service
gz service -s /world/my_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf: "<sdf version=\"1.9\"><model name=\"box\"><static>true</static><link name=\"link\"><visual name=\"v\"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></sdf>"'
```

### Useful Services

| Service | What it does |
|---------|-------------|
| `/world/{name}/create` | Spawn a model at runtime |
| `/world/{name}/remove` | Remove a model |
| `/world/{name}/scene/info` | Get full scene state |
| `/world/{name}/set_physics` | Change physics parameters at runtime |
| `/world/{name}/control` | Pause/unpause simulation |

---

## gz model CLI

Inspect models currently in the simulation.

```bash
# List all models in the world
gz model --list --world my_world

# Get detailed info about a model (links, joints, poses)
gz model --model robot --world my_world

# Get the pose of a model
gz model --model robot --pose --world my_world
```

---

## Performance Profiling

### Real-Time Factor (RTF)

RTF indicates how fast the simulation runs relative to real time:
- **RTF = 1.0**: Real-time speed (ideal)
- **RTF = 0.5**: Simulation runs at half real-time speed (too slow)
- **RTF > 1.0**: Faster than real-time (possible in simple scenes or headless mode)

### Checking RTF

```bash
# From the GUI: look at the bottom status bar

# From the command line: echo the stats topic
gz topic -e -t /stats
```

The `/stats` topic publishes `gz.msgs.WorldStatistics` which includes:
- `real_time`: Wall clock time elapsed
- `sim_time`: Simulation time elapsed
- `real_time_factor`: Current RTF

### Performance Diagnosis Table

| RTF | Meaning | Action |
|-----|---------|--------|
| > 0.9 | Good performance | No action needed |
| 0.5 - 0.9 | Acceptable, but degraded | Check sensor load, reduce if needed |
| 0.2 - 0.5 | Slow, may affect control | Reduce LiDAR rays, camera resolution, or use headless |
| < 0.2 | Very slow, control may fail | Major simplification needed; check for mesh collision issues |

### Finding the Performance Bottleneck

```bash
# Run with verbose timing
GZ_VERBOSE=3 gz sim -r -s my_world.sdf 2>&1 | grep -i "time\|step\|render"
```

Common bottlenecks in order of likelihood:
1. **GPU LiDAR with too many rays** -- reduce horizontal/vertical samples
2. **Camera rendering** -- reduce resolution, reduce update rate
3. **Mesh collision** -- replace mesh collision with primitives
4. **Too many models** -- simplify environment, use fewer objects
5. **Physics solver** -- increase step size or reduce solver iterations

---

## Common Physics Issues and Solutions

### Model Sinks Through Floor

**Symptoms:** Robot falls through the ground plane or static models.

**Diagnostic steps:**
```bash
# Check if the model has collision elements
gz model --model robot --world my_world
# Look for <collision> elements in the output

# Check if the ground plane has collision
gz model --model ground_plane --world my_world
```

**Causes and fixes:**

| Cause | Fix |
|-------|-----|
| Missing `<collision>` on robot links | Add `<collision>` geometry matching the link shape |
| Missing `<collision>` on ground | Add `<collision>` to the ground plane model |
| Mass too high, default contact stiffness too low | Reduce mass or increase `<kp>` in surface contact parameters |
| Spawned below the ground surface | Increase spawn Z coordinate: `-z 0.5` |

### Model Flies Away / Explodes on Spawn

**Symptoms:** Robot shoots into the air or off to the side immediately after spawning.

**Causes and fixes:**

| Cause | Fix |
|-------|-----|
| Inertia values are zero or near-zero | Calculate proper inertia (see `references/robot-models.md`) |
| Inertia values are wildly mismatched to mass | Recalculate using the correct geometry dimensions |
| Model spawned inside another model (overlap) | Change spawn position to avoid overlap |
| Spring stiffness too high on a joint | Reduce `<spring_stiffness>` or remove it |
| Two links occupy the same space with no joint | Add a joint between them or fix link origins |

### Joints Oscillate Wildly

**Symptoms:** Robot joints vibrate, shake, or oscillate without external force.

**Causes and fixes:**

| Cause | Fix |
|-------|-----|
| Joint damping too low | Increase `<damping>` (try 0.1 - 1.0) |
| Physics step size too large | Decrease `<max_step_size>` (try 0.0005) |
| PID gains too high (position controller) | Reduce P and D gains |
| Joint limits conflicting with initial pose | Ensure initial joint positions are within limits |

### Robot Does Not Move When Commanded

**Symptoms:** Publishing to cmd_vel but robot stays still.

**Diagnostic steps:**
```bash
# Check if cmd_vel reaches Gazebo
gz topic -e -t /model/robot/cmd_vel

# Check if the drive plugin is loaded
gz model --model robot --world my_world
# Look for DiffDrive or TrackedVehicle in plugins

# Check if the model is static
gz model --model robot --world my_world
# Look for <static>true</static>
```

**Causes and fixes:**

| Cause | Fix |
|-------|-----|
| Model is `<static>true</static>` | Set `<static>false</static>` |
| Drive plugin not loaded | Add DiffDrive or TrackedVehicle plugin to model |
| Topic name mismatch | Check plugin `<topic>` matches what you publish to |
| Bridge not configured for cmd_vel | Add ROS_TO_GZ bridge entry for cmd_vel |
| Wheel joints not matching plugin config | Verify `<left_joint>` and `<right_joint>` names match URDF/SDF |
| Wheel friction too low | Increase `<mu>` on wheel collision surfaces |

### Odometry Disagrees With the Robot's Actual Pose (the "drift" bug)

**Symptoms:** The robot drives, but in a viewer the laser scan, the map,
and/or the robot model "drift" or "rotate with the robot" — most obvious
during turns. SLAM produces a distorted map. Nav2 cannot reach goals.

This is **odometry/physics decoupling**, and it is the highest-impact,
hardest-to-spot sim bug. A wheel-joint drive plugin (`DiffDrive`,
`TrackedVehicle`) computes `/odom` by dead-reckoning the *commanded* wheel
velocities — it never measures the body. If physics pushes the chassis
independently of the wheels, `/odom` keeps reporting a clean number while the
real body goes elsewhere. Because `/odom` is the root of the TF tree,
*everything* built on it (scan, map, costmaps) renders faithfully — and
faithfully wrong.

**How to confirm it (do this before touching sensors or TF):** compare
`/odom` against the simulator's ground-truth pose. Gazebo's
`SceneBroadcaster` publishes every dynamic entity's true pose on
`/world/<world>/dynamic_pose/info` (`gz.msgs.Pose_V`) — bridge it and diff it
against `/odom`. If they diverge while driving, this is your bug. (The
`ros2-dev` skill bundles ready-made diagnostic scripts for exactly this
comparison; see its `references/simulation-debugging.md`.)

**Causes and fixes:**

| Cause | Fix |
|-------|-----|
| A non-wheel collision shape (belly plate, chassis underside) drags on the ground | Remove it, or drop its `<mu>` near zero so it slides instead of dragging |
| Load-bearing caster spheres coplanar with the drive wheels, sharing weight | Raise casters slightly so wheels carry the load, or tune their friction so they neither drag nor let the body teeter |
| Unstable footprint (two wheels on one transverse axis, nothing fore/aft) | Add low-friction fore/aft support so the body cannot pitch |
| `wheel_separation` / `wheel_radius` in the plugin don't match URDF joint geometry | Make them match exactly — a mismatch makes odom wrong even when physics is stable |
| **The drivetrain isn't the point of the sim** | Switch to `VelocityControl` + `OdometryPublisher` — body-driven control cannot decouple. See `references/robot-models.md` |

The robust fix for navigation-focused sims is the last row: don't dead-reckon
at all. Reserve wheel-joint plugins for when drivetrain fidelity *is* the
thing under test.

---

## Sensor Debugging

### Sensor Not Publishing Data

**Diagnostic steps:**
```bash
# List all topics and look for sensor topics
gz topic -l | grep sensor

# If no sensor topics appear:
# 1. Check that gz-sim-sensors-system plugin is in the world file
# 2. Check that the sensor is defined in the model
# 3. Check that <always_on>true</always_on> is set
# 4. Check that <update_rate> is > 0
```

**Common causes:**

| Cause | Fix |
|-------|-----|
| Missing `Sensors` system plugin in world | Add `gz-sim-sensors-system` plugin to world SDF |
| `<always_on>` is false or missing | Add `<always_on>true</always_on>` |
| `<update_rate>` is 0 | Set a positive rate (e.g., 10 for LiDAR, 100 for IMU) |
| Sensor attached to wrong link | Verify `<gazebo reference="link_name">` matches the intended link |
| Render engine not specified for GPU sensors | Add `<render_engine>ogre2</render_engine>` to Sensors plugin |
| GPU sensor (`gpu_lidar`, depth cam) exists in `gz topic -l` but never publishes; data appears the instant you `gz topic -e` it | GPU sensors only tick while subscribed, and the lazy ROS bridge never subscribes. Set `lazy: false` on the bridge entry — see `references/ros2-integration.md` |

**Tell-tale for the lazy-bridge deadlock:** IMU/contact sensors work but every
GPU sensor is silent. `gz topic -e -t /lidar` shows data (your echo is the
subscriber waking it). The robot's data path is fine — the bridge just never
asked for it.

### Sensor Data Looks Wrong

| Issue | Likely cause | Fix |
|-------|-------------|-----|
| LiDAR shows max range everywhere | Sensor inside a collision mesh | Move sensor origin outside collision geometry |
| Camera image is black | No lighting in the world | Add lights; check `<scene><ambient>` values |
| IMU shows constant non-zero acceleration | IMU measures gravity (expected) | Subtract gravity component; Z should show ~9.81 |
| Sensor data has wrong frame_id | Frame not configured in bridge | Set correct `frame_id` in bridge config or sensor config |
| Sensor publishes at wrong rate | `<update_rate>` not set correctly | Check `<update_rate>` tag in sensor definition |
| Camera image is upside down | Camera frame orientation wrong | Check camera link orientation in URDF/SDF |

---

## Visual Debugging in Gazebo GUI

The Gazebo GUI provides visualization overlays accessible through the GUI menu.

### Useful Visualizations

| Visualization | How to enable | What it shows |
|--------------|---------------|---------------|
| Collision shapes | Right-click model -> View -> Collisions | Green wireframe of collision geometry |
| Joint axes | Right-click model -> View -> Joints | Colored axes at each joint |
| Center of mass | Right-click model -> View -> Center of Mass | Sphere at each link's CoM |
| Inertia | Right-click model -> View -> Inertias | Box representing inertia tensor |
| Frames | Right-click model -> View -> Frames | Coordinate frames on each link |
| Transparent | Right-click model -> View -> Transparent | See-through model to check internals |
| Wireframe | Right-click model -> View -> Wireframe | Wireframe rendering |

### GUI Plugins for Debugging

| Plugin | Access | Use |
|--------|--------|-----|
| Entity Tree | Window -> Plugins -> Entity Tree | Browse world hierarchy |
| Component Inspector | Click model -> Component Inspector | View/edit model properties |
| Transform Control | Top toolbar | Move/rotate models interactively |
| Plot | Window -> Plugins -> Plot | Graph topic data over time |

---

## Log Files and Verbosity

### Setting Verbosity

```bash
# Via environment variable (0=error, 1=warn, 2=info, 3=debug, 4=trace)
GZ_VERBOSE=3 gz sim my_world.sdf

# Or export for the session
export GZ_VERBOSE=3
gz sim my_world.sdf
```

### Log File Locations

```bash
# Gazebo logs
~/.gz/sim/log/

# System journal (if systemd)
journalctl -u gz-sim

# ROS 2 bridge logs
ros2 run ros_gz_bridge parameter_bridge --ros-args --log-level debug
```

### Reading Verbose Output

Key patterns to look for in verbose output:

| Log message pattern | Meaning |
|--------------------|---------|
| `[Err]` | Error -- something is broken |
| `[Wrn]` | Warning -- something may be wrong |
| `Failed to load plugin` | Plugin file not found or wrong name |
| `Model [X] missing inertial` | Link has no `<inertial>` element |
| `Sensor [X] not created` | Sensor configuration error |
| `Physics step time exceeded` | Simulation cannot keep up with requested rate |

---

## Complete Diagnostic Flowchart

```
Simulation problem
|
+-- Nothing appears in GUI
|   +-- Is the world file loading? (check terminal for errors)
|   |   +-- NO --> Fix SDF syntax (use `gz sdf -k world.sdf` to validate)
|   |   +-- YES --> Is SceneBroadcaster plugin in the world?
|   |       +-- NO --> Add gz-sim-scene-broadcaster-system
|   |       +-- YES --> Is the model spawned? (`gz model --list`)
|   |           +-- NO --> Check spawn command/launch file
|   |           +-- YES --> Model may be below ground or very small
|
+-- Model behaves badly (sinks, flies, oscillates)
|   +-- Check inertia values (see Physics Issues section above)
|   +-- Check collision geometry exists
|   +-- Check physics step size
|   +-- Enable collision visualization in GUI
|
+-- Sensor not producing data
|   +-- Is Sensors system plugin in world? (gz-sim-sensors-system)
|   +-- Is <always_on>true</always_on> set?
|   +-- Does gz topic -l show the sensor topic?
|   |   +-- NO --> Check sensor definition in model
|   |   +-- YES, but no messages --> Does `gz topic -e` make data appear?
|   |       +-- YES --> Lazy-bridge / GPU-sensor deadlock: set lazy:false on the bridge entry
|   |       +-- NO  --> Sensor mis-defined; check render engine for GPU sensors
|
+-- ROS 2 not receiving data
|   +-- Does gz topic show the data? (see above)
|   +-- Is the bridge running? (`ros2 node list | grep bridge`)
|   +-- Is the bridge config correct? (topic names, types, direction)
|   +-- Is the bridge lazy and nothing subscribes? (set lazy:false for GPU sensors)
|   +-- Is use_sim_time set on subscriber nodes?
|
+-- Scan / map / robot "drift" when turning
|   +-- Compare /odom against ground-truth pose (/world/<world>/dynamic_pose/info)
|   |   +-- They diverge --> odometry/physics decoupling (see Physics Issues section)
|   |   |   +-- Fix the model's footprint, or switch to VelocityControl
|   |   +-- They agree --> not a data bug; check the viewer's display frame
|
+-- Simulation too slow (RTF < 0.5)
|   +-- Check collision geometry FIRST (a heavy mesh as collision collapses RTF to ~0.04)
|   +-- Check LiDAR ray count (reduce to 360 horizontal)
|   +-- Check camera resolution (use 320x240 for dev)
|   +-- Try headless mode: gz sim -s (no GUI)
|   +-- Try Bullet physics if many objects
|
+-- Gazebo crashes on startup
    +-- Check GPU drivers (especially for GPU sensors)
    +-- Check SDF file syntax: gz sdf -k file.sdf
    +-- Try without GPU sensors first
    +-- Check disk space and memory
    +-- Run with GZ_VERBOSE=4 for detailed logs
```

---

## SDF Validation

Always validate your SDF files before loading them:

```bash
# Validate SDF syntax
gz sdf -k my_world.sdf
gz sdf -k my_model.sdf

# Print the fully resolved SDF (shows defaults and includes)
gz sdf -p my_world.sdf
```

Common SDF errors:
- Missing closing tags
- Wrong SDF version in header (use `version="1.9"` for Harmonic)
- Misspelled element names (SDF is case-sensitive)
- Missing required child elements (e.g., `<geometry>` without a shape inside)

---

## Quick Reference: Most Used Commands

```bash
# Start simulation
gz sim world.sdf              # With GUI
gz sim -s world.sdf           # Headless (server only)
gz sim -r world.sdf           # Start running (not paused)
gz sim -r -s world.sdf        # Headless + running

# Inspect simulation
gz topic -l                   # List topics
gz topic -e -t /topic_name    # Echo topic data
gz topic -hz -t /topic_name   # Check publish rate
gz model --list               # List models
gz model --model name         # Model details
gz service -l                 # List services
gz sdf -k file.sdf            # Validate SDF

# Send commands
gz topic -t /model/robot/cmd_vel -m gz.msgs.Twist -p 'linear:{x:0.5}'

# Debug
GZ_VERBOSE=4 gz sim world.sdf  # Maximum verbosity
gz sdf --inertial-stats model.sdf  # Check inertia values
```
