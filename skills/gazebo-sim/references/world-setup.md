# World Setup and Configuration

This reference covers creating simulation worlds in Gazebo Harmonic, configuring physics
engines, adding environmental elements, and building custom worlds for robotics simulation
scenarios.

---

## SDF World File Structure

An SDF world file defines everything in the simulation environment: physics, lighting, ground,
models, and plugins.

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="my_world">

    <!-- Physics engine configuration -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Scene (rendering settings) -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Plugins (system-level) -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sun light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Your models go here -->

  </world>
</sdf>
```

### Mandatory World Plugins

These plugins must be included in every world file or the simulation will not function correctly:

| Plugin | Purpose | What breaks without it |
|--------|---------|----------------------|
| `gz-sim-physics-system` | Runs the physics engine | Nothing moves, no collisions |
| `gz-sim-scene-broadcaster-system` | Broadcasts scene state to GUI | GUI shows empty scene |
| `gz-sim-user-commands-system` | Handles spawn/delete commands | Cannot spawn models at runtime |
| `gz-sim-sensors-system` | Runs sensor computations | Sensors do not publish data |

---

## Physics Engine Configuration

### DART (Default -- Recommended)

```xml
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
  </dart>
</physics>
```

### Bullet

```xml
<physics name="bullet_physics" type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### TPE (Trivial Physics Engine -- Kinematic Only)

```xml
<physics name="tpe_physics" type="tpe">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

### Physics Parameter Tuning Guide

| Parameter | Default | Increase when | Decrease when |
|-----------|---------|---------------|---------------|
| `max_step_size` | 0.001 s | Simulation too slow and accuracy is acceptable | Joints oscillate, objects pass through each other |
| `real_time_factor` | 1.0 | You want faster-than-realtime (training) | You want slow-motion for debugging |
| `real_time_update_rate` | 1000 Hz | Generally leave at default | CPU cannot keep up (lower to 500) |

**Relationship:** `real_time_update_rate * max_step_size = real_time_factor` at full speed.
If the simulation cannot keep up, the actual RTF will drop below the target.

---

## Gravity Settings

Standard Earth gravity (default):
```xml
<gravity>0 0 -9.81</gravity>
```

The values are X, Y, Z in meters per second squared. Gazebo uses Z-up coordinate convention.

For underwater or reduced gravity simulations:
```xml
<!-- Moon gravity -->
<gravity>0 0 -1.62</gravity>

<!-- Zero gravity (space) -->
<gravity>0 0 0</gravity>
```

---

## Lighting

### Light Types

| Type | Use case | Example |
|------|----------|---------|
| `directional` | Sun, overall scene illumination | Outdoor scenes |
| `point` | Light bulb, omnidirectional | Indoor rooms, corridor lights |
| `spot` | Flashlight, focused beam | Robot headlight, inspection light |

### Point Light (for indoor scenes)

```xml
<light type="point" name="corridor_light_1">
  <pose>5 0 2 0 0 0</pose>
  <diffuse>0.6 0.5 0.4 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <attenuation>
    <range>20</range>
    <constant>0.5</constant>
    <linear>0.1</linear>
    <quadratic>0.02</quadratic>
  </attenuation>
  <cast_shadows>false</cast_shadows>
</light>
```

### Spot Light (for robot headlight simulation)

```xml
<light type="spot" name="robot_headlight">
  <pose>0 0 0.3 0 0 0</pose>
  <diffuse>1.0 1.0 0.9 1</diffuse>
  <specular>0.3 0.3 0.3 1</specular>
  <attenuation>
    <range>30</range>
    <constant>0.3</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <spot>
    <inner_angle>0.5</inner_angle>
    <outer_angle>1.0</outer_angle>
    <falloff>1.0</falloff>
  </spot>
  <cast_shadows>true</cast_shadows>
</light>
```

---

## Adding Static Models (Walls, Obstacles)

Static models do not move and do not participate in dynamic physics (they have infinite mass).
Use them for walls, floors, obstacles, and environment structures.

### Simple Wall

```xml
<model name="wall_1">
  <static>true</static>
  <pose>5 0 1 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>0.2 10 2</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>0.2 10 2</size></box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Cylindrical Obstacle

```xml
<model name="cylinder_obstacle">
  <static>true</static>
  <pose>3 2 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder><radius>0.15</radius><length>1.0</length></cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder><radius>0.15</radius><length>1.0</length></cylinder>
      </geometry>
      <material>
        <ambient>0.4 0.3 0.2 1</ambient>
        <diffuse>0.6 0.5 0.3 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

---

## Sensor Plugins in World File

Sensors are typically attached to robot models (see `references/robot-models.md`), but some
sensors can be placed in the world as standalone models.

### Available Sensor Types in Gazebo Harmonic

| Sensor type string | What it simulates |
|-------------------|-------------------|
| `imu` | Inertial measurement unit (acceleration + angular velocity) |
| `gpu_lidar` | 2D or 3D LiDAR (GPU-accelerated ray casting) |
| `camera` | RGB camera |
| `depth_camera` | Depth camera (returns distance per pixel) |
| `rgbd_camera` | Combined RGB + depth camera |
| `contact` | Contact/touch sensor on a collision surface |
| `altimeter` | Height above reference |
| `magnetometer` | Magnetic field measurement |
| `air_pressure` | Barometric pressure |
| `force_torque` | Force/torque at a joint |
| `logical_camera` | Returns model names/poses in frustum (no rendering) |
| `thermal_camera` | Thermal/IR camera |

---

## Environment Variables

Set these before running `gz sim` to configure model and plugin paths.

```bash
# Add custom model directories
export GZ_SIM_RESOURCE_PATH=$HOME/my_gazebo_models:$GZ_SIM_RESOURCE_PATH

# Add custom plugin directories
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/my_plugins/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Increase log verbosity for debugging (0=error, 1=warn, 2=info, 3=debug, 4=trace)
export GZ_VERBOSE=3

# Isolate simulation transport (useful for multiple simulations)
export GZ_PARTITION=my_experiment
```

For ROS 2 launch files, set these in the launch file:
```python
import os
from launch.actions import SetEnvironmentVariable

SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    os.path.join(get_package_share_directory('my_pkg'), 'models')
)
```

---

## Custom World Creation Workflow

1. **Start with a template** -- copy the basic world structure from the top of this file.
2. **Choose your physics engine** -- DART for accuracy, Bullet for speed, TPE for kinematic-only.
3. **Add the ground plane** -- always include collision geometry.
4. **Set up lighting** -- directional for outdoor, point/spot for indoor environments.
5. **Add static environment models** -- walls, obstacles, terrain.
6. **Test with `gz sim`** before adding the robot:

```bash
# Test your world file
gz sim my_world.sdf

# Server-only (no GUI, faster)
gz sim -s my_world.sdf
```

7. **Add robot via launch file** (see `references/ros2-integration.md`) rather than embedding
   the robot in the world file. This keeps the world reusable across different robots.

---

## Complete Example: Indoor Corridor World

This world creates a straight enclosed corridor section with dim lighting, suitable for testing
an inspection robot with LiDAR and camera.

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="indoor_corridor">

    <!-- Physics -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Dark scene for indoor environment -->
    <scene>
      <ambient>0.05 0.05 0.05 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>true</shadows>
    </scene>

    <gravity>0 0 -9.81</gravity>

    <!-- Required plugins -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact">
    </plugin>

    <!-- Floor (concrete-like surface) -->
    <model name="corridor_floor">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>20 2 0.1</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>20 2 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Left wall -->
    <model name="wall_left">
      <static>true</static>
      <pose>0 -1.0 1.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>20 0.2 2.0</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>20 0.2 2.0</size></box>
          </geometry>
          <material>
            <ambient>0.25 0.2 0.15 1</ambient>
            <diffuse>0.35 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Right wall -->
    <model name="wall_right">
      <static>true</static>
      <pose>0 1.0 1.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>20 0.2 2.0</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>20 0.2 2.0</size></box>
          </geometry>
          <material>
            <ambient>0.25 0.2 0.15 1</ambient>
            <diffuse>0.35 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Ceiling -->
    <model name="ceiling">
      <static>true</static>
      <pose>0 0 2.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>20 2.2 0.2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>20 2.2 0.2</size></box>
          </geometry>
          <material>
            <ambient>0.2 0.18 0.12 1</ambient>
            <diffuse>0.3 0.25 0.18 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle: debris pile (simulated as box) -->
    <model name="debris_1">
      <static>true</static>
      <pose>5 0.3 0.15 0 0 0.2</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.4 0.3 0.3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.4 0.3 0.3</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.25 0.15 1</ambient>
            <diffuse>0.4 0.35 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle: cylindrical beam crossing the corridor -->
    <model name="cross_beam">
      <static>true</static>
      <pose>8 0 1.5 1.5708 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>2.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>2.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.5 0.3 0.15 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Dim interior lighting: spaced point lights -->
    <light type="point" name="corridor_light_0">
      <pose>0 0 1.8 0 0 0</pose>
      <diffuse>0.4 0.35 0.25 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.3</constant>
        <linear>0.1</linear>
        <quadratic>0.05</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    <light type="point" name="corridor_light_1">
      <pose>5 0 1.8 0 0 0</pose>
      <diffuse>0.3 0.25 0.2 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.3</constant>
        <linear>0.1</linear>
        <quadratic>0.05</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    <light type="point" name="corridor_light_2">
      <pose>10 0 1.8 0 0 0</pose>
      <diffuse>0.3 0.25 0.2 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.3</constant>
        <linear>0.1</linear>
        <quadratic>0.05</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

  </world>
</sdf>
```

### Running the Custom World

```bash
# With GUI
gz sim indoor_corridor.sdf

# Headless (for testing)
gz sim -s indoor_corridor.sdf
```

The corridor is 20 meters long, 2 meters wide, 2 meters tall. It includes debris and a
crossing beam as obstacles. Lighting is dim with warm-toned point lights to simulate
low-light indoor conditions. The robot's own headlight (see `references/robot-models.md`)
will be the primary light source for camera-based inspection.
