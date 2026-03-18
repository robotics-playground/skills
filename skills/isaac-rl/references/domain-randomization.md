# Domain Randomization

## What Is Domain Randomization and Why It Matters

Domain randomization is the practice of randomly varying simulation parameters during training
so the learned policy is robust to the differences between simulation and reality.

The core insight: **if the policy works across a wide range of simulated physics parameters,
there's a good chance the real world falls somewhere within that range.**

### Why It Works

- **No randomization** = training with a single set of physics parameters. The policy overfits
  to those exact parameters and fails when deployed to the real robot where conditions differ.
- **Domain randomization** = training across a wide range of physics parameters (friction, mass,
  motor strength, sensor noise). The policy that works across all these variations is robust
  enough for real-world deployment.

---

## Physics Randomization

The most important type of randomization for sim-to-real transfer.

### What to Randomize

| Parameter | Default | Randomization Range | Why |
|---|---|---|---|
| **Friction** | 1.0 | [0.5, 2.0] | Floor surfaces vary (tile, carpet, concrete) |
| **Mass** | Per URDF | [0.8x, 1.2x] per link | Model mass is approximate; payload varies |
| **Center of mass** | Per URDF | +/- 5cm per axis | Manufacturing tolerances |
| **Motor strength** | 100% | [80%, 120%] | Motor degradation, temperature effects |
| **Joint damping** | Per URDF | [0.5x, 2.0x] | Hard to measure accurately |
| **Joint stiffness** | Per config | [0.8x, 1.2x] | PD gains may differ from sim |
| **Restitution** | 0.0 | [0.0, 0.3] | Surface bounciness varies |

### Isaac Lab Configuration

```python
from isaaclab.managers import EventTermCfg
import isaaclab.envs.mdp as mdp

@configclass
class EventsCfg:
    """Domain randomization events."""

    # === Applied on reset ===

    # Randomize friction
    physics_material = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "static_friction_range": (0.5, 2.0),
            "dynamic_friction_range": (0.5, 2.0),
            "restitution_range": (0.0, 0.3),
        },
    )

    # Randomize robot mass
    add_base_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (-1.0, 3.0),  # Add -1 to +3 kg to base
            "operation": "add",
        },
    )

    # Randomize center of mass
    randomize_com = EventTermCfg(
        func=mdp.randomize_rigid_body_com,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "com_distribution_params": {
                "x": (-0.05, 0.05),  # +/- 5cm
                "y": (-0.05, 0.05),
                "z": (-0.03, 0.03),
            },
        },
    )

    # Randomize joint properties
    randomize_joint_parameters = EventTermCfg(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "stiffness_distribution_params": (0.8, 1.2),  # 80-120% of nominal
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )

    # === Applied every step (interval) ===

    # External push forces (random perturbations)
    push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),  # Push every 10-15 seconds
        params={
            "velocity_range": {
                "x": (-0.5, 0.5),  # m/s
                "y": (-0.5, 0.5),
                "z": (0.0, 0.0),
            },
        },
    )
```

---

## Sensor Noise Randomization

Real sensors are noisy. Adding noise during training helps the policy handle imperfect
observations.

### Observation Noise

```python
@configclass
class PolicyCfg(ObservationGroupCfg):
    """Policy observations with sensor noise."""

    # Joint position noise: encoder noise
    joint_pos = ObservationTermCfg(
        func=mdp.joint_pos_rel,
        noise={
            "type": "gaussian",
            "mean": 0.0,
            "std": 0.01,  # ~0.6 degrees noise
        },
    )

    # Joint velocity noise: derivative of noisy position
    joint_vel = ObservationTermCfg(
        func=mdp.joint_vel_rel,
        noise={
            "type": "gaussian",
            "mean": 0.0,
            "std": 1.5,  # rad/s noise
        },
    )

    # IMU angular velocity noise
    base_ang_vel = ObservationTermCfg(
        func=mdp.base_ang_vel,
        noise={
            "type": "gaussian",
            "mean": 0.0,
            "std": 0.2,  # rad/s
        },
    )

    # IMU linear acceleration / velocity noise
    base_lin_vel = ObservationTermCfg(
        func=mdp.base_lin_vel,
        noise={
            "type": "gaussian",
            "mean": 0.0,
            "std": 0.1,  # m/s
        },
    )

    # Gravity direction noise (IMU orientation estimate)
    projected_gravity = ObservationTermCfg(
        func=mdp.projected_gravity,
        noise={
            "type": "gaussian",
            "mean": 0.0,
            "std": 0.05,
        },
    )
```

### Communication Latency

Real robots have communication delays between sensors, computation, and actuators. Simulate
this by adding observation and action delays:

```python
# Observation latency: use observation from 1-3 steps ago
# This is typically implemented by maintaining an observation buffer
# and randomly selecting a past observation

# Action latency: delay action execution by 1-2 steps
# Implemented in the action manager
@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.25,
        use_default_offset=True,
        # Simulate 1-step action delay (common in real systems)
        # The action computed at time t is applied at time t+1
    )
```

---

## Visual Randomization

Visual randomization (changing textures, lighting, colors) is mainly relevant for
**vision-based policies** that use cameras. For proprioceptive locomotion (joint states + IMU),
visual randomization is not needed.

### When Visual Randomization Matters

| Policy Type | Uses Cameras? | Need Visual Randomization? |
|---|---|---|
| Proprioceptive locomotion | No | No |
| Vision-based locomotion (terrain awareness) | Yes (depth) | Minimal (depth is less affected) |
| Object manipulation with vision | Yes (RGB + depth) | Yes (textures, lighting, colors) |
| Navigation | Yes (RGB) | Yes (full visual randomization) |

For the Unitree G1 locomotion task, **skip visual randomization**. Focus on physics and
sensor noise randomization instead.

---

## Scheduling: Uniform vs Curriculum-Based Randomization

### Uniform Randomization

Every episode samples random parameters independently from the full range:

```python
# Friction is uniformly random between 0.5 and 2.0 for every episode
"static_friction_range": (0.5, 2.0),
```

**Pros:** Simple, covers the full range from the start.
**Cons:** If the range is too wide, the agent may struggle to learn anything.

### Curriculum-Based Randomization

Start with narrow ranges and widen them as the agent improves:

```python
# Phase 1 (iterations 0-500): Easy
# friction: (0.8, 1.2), mass_offset: (-0.5, 0.5)

# Phase 2 (iterations 500-1000): Medium
# friction: (0.6, 1.5), mass_offset: (-1.0, 2.0)

# Phase 3 (iterations 1000+): Hard
# friction: (0.5, 2.0), mass_offset: (-1.0, 3.0)
```

**Pros:** Agent learns basic skills first, then robustness.
**Cons:** More complex to implement and tune.

### Recommendation

**Start with uniform randomization** using moderate ranges. If the agent can't learn (reward
stays flat), narrow the ranges. If the policy doesn't transfer to real, widen the ranges.

---

## Parameter Ranges: Finding the Sweet Spot

| Too Little Randomization | Sweet Spot | Too Much Randomization |
|---|---|---|
| Policy overfits to sim defaults | Policy handles range of conditions | Policy can't learn anything |
| Fast training, poor transfer | Moderate training time, good transfer | Training never converges |
| Sim-to-real gap remains | Sim-to-real gap mostly closed | Sim-to-real irrelevant (nothing works) |

### Recommended Starting Ranges for Humanoid Locomotion

| Parameter | Conservative | Moderate | Aggressive |
|---|---|---|---|
| Friction | [0.8, 1.2] | [0.5, 2.0] | [0.3, 3.0] |
| Base mass offset | [-0.5, 0.5] kg | [-1.0, 3.0] kg | [-2.0, 5.0] kg |
| CoM offset | +/- 2 cm | +/- 5 cm | +/- 10 cm |
| Motor strength | [90%, 110%] | [80%, 120%] | [70%, 130%] |
| Joint damping | [90%, 110%] | [50%, 200%] | [30%, 300%] |
| Observation noise | std * 0.5 | std * 1.0 | std * 2.0 |
| Push force | 0.2 m/s | 0.5 m/s | 1.0 m/s |
| Push interval | 15-20s | 10-15s | 5-10s |

**Start with "Moderate" and adjust based on results.**

---

## Actuator Modeling

The biggest sim-to-real gap often comes from actuator (motor) modeling. Simulated motors
respond instantly; real motors have dynamics, delays, and nonlinearities.

### Simple PD Model (Default)

```python
# Isaac Lab default: ideal PD controller
# torque = Kp * (target_pos - current_pos) + Kd * (target_vel - current_vel)
actuators = {
    "legs": ImplicitActuatorCfg(
        joint_names_expr=[".*"],
        stiffness=80.0,  # Kp
        damping=4.0,     # Kd
    ),
}
```

**Problem:** Real motors don't behave like ideal PD controllers. They have:
- Response delay (5-20ms)
- Torque limits
- Velocity-dependent torque
- Friction and backlash

### Improved Actuator Model

```python
# More realistic actuator configuration
actuators = {
    "legs": DCMotorCfg(
        joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
        stiffness=80.0,
        damping=4.0,
        saturation_effort=33.5,  # Max torque (Nm) -- match real motor specs
        # Motor dynamics
        velocity_limit=21.0,     # Max joint velocity (rad/s)
    ),
}
```

### Actuator Network (Advanced)

For the best sim-to-real transfer, train a separate neural network to model the actuator
dynamics from real data:

1. Command the real robot to perform joint trajectories
2. Record: commanded position, actual position, actual torque, actual velocity
3. Train a small network: `(commanded_pos, current_pos, current_vel) -> actual_torque`
4. Use this network as the actuator model in simulation

This is covered in detail in `sim-to-real.md`.

---

## Ground Terrain Randomization

Training on varied terrain produces policies that handle real-world ground irregularities.

### Terrain Types in Isaac Lab

```python
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
import isaaclab.terrains as terrain_gen

terrain_cfg = TerrainGeneratorCfg(
    size=(8.0, 8.0),           # Terrain patch size (meters)
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    curriculum=True,            # Enable curriculum (start easy)
    sub_terrains={
        # Flat ground (easiest)
        "flat": terrain_gen.MeshPlaneTerrainCfg(
            proportion=0.2,
        ),
        # Random rough terrain
        "rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2,
            noise_range=(0.02, 0.10),   # 2-10 cm height variation
            noise_step=0.02,
        ),
        # Slopes
        "slope_up": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1,
            slope_range=(0.0, 0.4),     # Up to ~22 degrees
        ),
        # Stairs (up)
        "stairs_up": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.15,
            step_height_range=(0.05, 0.23),  # 5-23 cm steps
            step_width=0.31,
        ),
        # Stairs (down)
        "stairs_down": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.15,
            step_height_range=(0.05, 0.23),
            step_width=0.31,
        ),
        # Discrete obstacles (stepping stones)
        "stepping_stones": terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.1,
            obstacle_height_mode="fixed",
            obstacle_height_range=(0.05, 0.15),
            platform_width=1.5,
        ),
        # Random uniform (hardest)
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.1,
            noise_range=(0.05, 0.20),
            noise_step=0.02,
        ),
    },
)
```

### Terrain Curriculum Flow

```
Level 0: Flat ground
Level 1: Slight roughness (2cm)
Level 2: Moderate roughness (5cm)
Level 3: Gentle slopes (10 degrees)
Level 4: Rough terrain (10cm)
Level 5: Steeper slopes (20 degrees)
Level 6: Small stairs (10cm)
Level 7: Large stairs (20cm)
Level 8: Mixed obstacles
Level 9: Everything at maximum difficulty
```

The agent automatically advances to harder terrain as its performance improves.

---

## External Force Perturbations

Pushing the robot during training teaches push recovery, which is essential for real-world
deployment where the robot may be bumped or encounter unexpected forces.

```python
# Random velocity pushes
push_robot = EventTermCfg(
    func=mdp.push_by_setting_velocity,
    mode="interval",
    interval_range_s=(10.0, 15.0),  # Push every 10-15 seconds
    params={
        "velocity_range": {
            "x": (-0.5, 0.5),    # Forward/backward push
            "y": (-0.5, 0.5),    # Lateral push
            "z": (0.0, 0.0),     # No vertical push
            "roll": (-0.25, 0.25),
            "pitch": (-0.25, 0.25),
            "yaw": (-0.25, 0.25),
        },
    },
)

# Curriculum: start gentle, increase force
# Phase 1: velocity range (-0.2, 0.2) -- gentle nudges
# Phase 2: velocity range (-0.5, 0.5) -- moderate pushes
# Phase 3: velocity range (-1.0, 1.0) -- aggressive pushes
```

---

## Complete Example: Domain Randomization for Bipedal Locomotion

```python
"""Full domain randomization config for Unitree G1 walking."""

from isaaclab.managers import EventTermCfg, SceneEntityCfg
import isaaclab.envs.mdp as mdp
from isaaclab.utils import configclass


@configclass
class G1DomainRandEventsCfg:
    """Comprehensive domain randomization for sim-to-real transfer."""

    # ========================================
    # STARTUP (applied once at beginning)
    # ========================================

    # Randomize motor strength (simulates motor-to-motor variation)
    motor_strength = EventTermCfg(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )

    # ========================================
    # RESET (applied each time an env resets)
    # ========================================

    # Reset robot position with randomization
    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.25, 0.25),
                "pitch": (-0.25, 0.25),
                "yaw": (-0.25, 0.25),
            },
        },
    )

    # Reset joints with randomization
    reset_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.8, 1.2),
            "velocity_range": (-0.5, 0.5),
        },
    )

    # Randomize ground friction
    ground_friction = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "static_friction_range": (0.5, 2.0),
            "dynamic_friction_range": (0.5, 2.0),
            "restitution_range": (0.0, 0.2),
        },
    )

    # Randomize base mass (simulate payload or modeling error)
    base_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
        },
    )

    # Randomize center of mass
    base_com = EventTermCfg(
        func=mdp.randomize_rigid_body_com,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "com_distribution_params": {
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.03, 0.03),
            },
        },
    )

    # ========================================
    # INTERVAL (applied periodically during episode)
    # ========================================

    # External push forces (push recovery training)
    push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.0),
                "roll": (-0.25, 0.25),
                "pitch": (-0.25, 0.25),
                "yaw": (0.0, 0.0),
            },
        },
    )
```

### Tuning Checklist

Before deploying to real hardware, verify these randomization levels:

- [ ] Friction range includes observed real-world surfaces
- [ ] Mass range includes expected payload and modeling error
- [ ] Observation noise matches measured sensor noise on real robot
- [ ] Push forces match expected disturbances
- [ ] Actuator model matches measured motor response
- [ ] Policy is trained for at least 500 iterations after adding all randomization
- [ ] Policy evaluation shows robust behavior across randomization extremes
