# Isaac Sim Basics

## What Is Isaac Sim?

Isaac Sim is NVIDIA's robot simulator built on the Omniverse platform. Think of it as a
physics-accurate video game engine designed specifically for robots. It simulates:

- **Physics** (PhysX 5): rigid bodies, joints, contacts, friction — all on the GPU
- **Sensors**: cameras (RGB, depth), LiDAR, IMU, contact sensors
- **Rendering**: photorealistic scenes with RTX ray tracing (useful for vision tasks)
- **Parallel worlds**: thousands of copies of the same scene running simultaneously on one GPU

For our use case (humanoid locomotion with RL), the physics simulation is what matters most.
We rarely need photorealistic rendering because locomotion policies typically use proprioceptive
observations (joint angles, IMU), not camera images.

---

## Installation and System Requirements

### Hardware Requirements

| Component | Minimum | Recommended | Why |
|---|---|---|---|
| GPU | RTX 2070 (8 GB) | RTX 4090 (24 GB) | PhysX runs on GPU; more VRAM = more parallel environments |
| CPU | Intel i7 / AMD Ryzen 7 | Intel i9 / AMD Ryzen 9 | Scene loading and data processing |
| RAM | 32 GB | 64 GB | Isaac Sim loads many assets into memory |
| Disk | 50 GB SSD | 100 GB+ NVMe | Installation is ~30 GB; cache and logs add up |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS | Primary supported platform; Windows works but is secondary |
| Driver | NVIDIA 525.60+ | Latest stable | Older drivers cause cryptic failures |

### Installation Methods

**Method 1: pip install (recommended for Isaac Lab RL workflows)**

```bash
# Create a conda environment
conda create -n isaaclab python=3.11
conda activate isaaclab

# Install Isaac Sim Python packages
pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
```

**Note:** Install Isaac Sim via pip (Omniverse Launcher is deprecated since October 2025).
The pip install method above is the only supported installation path for Isaac Lab RL workflows.

### Verifying Installation

```python
# Test that Isaac Sim loads correctly
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import isaacsim.core.utils.stage as stage_utils  # omni.isaac.core in Isaac Sim 4.x
print("Isaac Sim loaded successfully")
print(f"Stage: {stage_utils.get_current_stage()}")

simulation_app.close()
```

If this script runs without errors, your installation is correct.

---

## USD (Universal Scene Description) Basics

USD is the file format used by Isaac Sim for everything: robots, environments, materials,
physics properties.

### Key Concepts

| USD Concept | Description |
|---|---|
| **Stage** | The root container for the entire scene |
| **Prim** | Any object in the scene (robot, ground, light) |
| **Xform** | A prim with position/rotation/scale |
| **Mesh** | A 3D shape (visual representation) |
| **Material** | Surface appearance (color, texture, roughness) |
| **Reference** | Embedding one USD file inside another |
| **Layer** | Overlapping edits; stronger layers override weaker |

### Common USD File Types

| Extension | Description |
|---|---|
| `.usd` | Binary USD (most common for robots) |
| `.usda` | ASCII USD (human-readable, good for debugging) |
| `.usdc` | Crate (binary, optimized for performance) |
| `.usdz` | Zipped package (USD + textures in one file) |

### Robot Models in USD

Robot models (URDF, MJCF) are converted to USD for use in Isaac Sim. Isaac Lab provides
pre-converted models for common robots including the Unitree G1.

```python
# Loading a robot USD in Isaac Lab
import isaaclab.sim as sim_utils

# Unitree G1 robot (pre-packaged in Isaac Lab)
from isaaclab_assets.unitree import G1_CFG

robot_cfg = G1_CFG.replace(prim_path="/World/Robot")
robot = robot_cfg.spawn("/World/Robot")
```

---

## Creating Scenes

A scene in Isaac Sim contains the ground, robots, objects, lighting, and physics configuration.

### Minimal Scene: Ground + Robot

```python
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg

# Ground plane
ground_cfg = AssetBaseCfg(
    prim_path="/World/Ground",
    spawn=sim_utils.GroundPlaneCfg(),
)

# Lighting
light_cfg = AssetBaseCfg(
    prim_path="/World/Light",
    spawn=sim_utils.DomeLightCfg(
        intensity=2000.0,
        color=(0.8, 0.8, 0.8),
    ),
)
```

### Scene with Multiple Robots (Vectorized)

For RL training, you spawn thousands of copies of the same scene. Isaac Lab handles this
automatically via the `num_envs` parameter:

```python
# This creates 4096 identical scenes, each with one robot
# All running in parallel on the GPU
# You don't write a loop -- Isaac Lab does the replication

@configclass
class MySceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",  # {ENV_REGEX_NS} is replaced per environment
        spawn=sim_utils.UsdFileCfg(
            usd_path="path/to/unitree_g1.usd",
        ),
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
                stiffness=80.0,
                damping=4.0,
            ),
        },
    )
```

---

## Physics Configuration (PhysX 5)

Isaac Sim uses NVIDIA PhysX 5 for physics simulation. Key parameters to understand:

### Simulation Timestep

```python
from isaaclab.sim import SimulationCfg

sim_cfg = SimulationCfg(
    dt=0.005,              # Physics timestep: 5ms = 200 Hz
    render_interval=4,     # Render every 4th physics step (50 Hz visual)
    gravity=(0.0, 0.0, -9.81),
    physx=PhysxCfg(
        solver_type=1,                  # TGS solver (better for articulations)
        num_position_iterations=4,      # More = more accurate, slower
        num_velocity_iterations=1,
        max_depenetration_velocity=1.0, # Prevents explosion on deep penetration
        bounce_threshold_velocity=0.2,
    ),
)
```

**Timestep matters for RL:**
- Too large (>10ms): physics becomes inaccurate, joints oscillate, contacts are unstable
- Too small (<1ms): simulation is slow, training takes forever
- Sweet spot for humanoids: **2-5ms** (200-500 Hz physics)

The **decimation** parameter controls how many physics steps happen per RL step:

```python
# RL step frequency = physics_dt * decimation
# Example: 0.005s * 4 = 0.02s = 50 Hz RL control
decimation = 4  # 4 physics steps per RL action
```

### Contact and Friction

```python
# Physics material for ground
from isaaclab.sim import RigidBodyMaterialCfg

ground_material = RigidBodyMaterialCfg(
    static_friction=1.0,    # Friction when not sliding
    dynamic_friction=1.0,   # Friction when sliding
    restitution=0.0,        # Bounciness (0 = no bounce)
)
```

---

## Sensor Simulation

Isaac Sim can simulate various sensors. For locomotion, we primarily use:

### IMU (Inertial Measurement Unit)

```python
from isaaclab.sensors import ImuCfg

imu_cfg = ImuCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link",
    update_period=0.02,  # 50 Hz, matching RL step
    # Outputs: linear acceleration, angular velocity, orientation
)
```

### Contact Sensors (Foot Contact)

```python
from isaaclab.sensors import ContactSensorCfg

foot_contact_cfg = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/.*_foot",
    update_period=0.02,
    # Outputs: contact forces, contact normals, binary contact state
)
```

### Camera (for vision-based policies)

```python
from isaaclab.sensors import CameraCfg

camera_cfg = CameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/head_camera",
    update_period=0.1,  # 10 Hz (cameras are expensive)
    height=240,
    width=320,
    data_types=["rgb", "depth"],
)
```

**For locomotion:** Start with proprioceptive observations only (joint states + IMU).
Add cameras later if needed for terrain-aware walking. Camera-based policies are significantly
harder to train and transfer to real hardware.

---

## Headless Mode for Training

During RL training, you don't need the GUI. Headless mode disables rendering and runs
physics-only, which is significantly faster.

```bash
# Run training without GUI (headless)
python train.py --task MyTask --headless --num_envs 4096

# If you need to occasionally visualize during training
python train.py --task MyTask --num_envs 4096 --enable_cameras
# Then connect via Isaac Sim Streaming (browser-based viewer)
```

### Headless vs GUI Performance

| Mode | Envs | Approx. Steps/sec (RTX 4090) | Use Case |
|---|---|---|---|
| GUI | 64 | ~5,000 | Debugging, visual inspection |
| Headless | 4096 | ~200,000 | Training |
| Headless + cameras | 4096 | ~50,000 | Vision-based training |

**Always train headless.** Use the GUI only for debugging reward functions and environment setup
with a small number of environments (16-64).

---

## Cloud GPU Options for Isaac Sim

When your local GPU isn't enough, these cloud options support Isaac Sim:

### Recommended Cloud Providers

| Provider | Best GPU | Instance | Approx. $/hr | Isaac Sim Compatible |
|---|---|---|---|---|
| Lambda Labs | A100 80GB | gpu_1x_a100_sxm4 | $1.10 | Yes, pre-installed images available |
| Vast.ai | RTX 4090 / A100 | Varies | $0.30-1.50 | Yes, Docker images available |
| AWS | A10G | g5.xlarge | $1.00 | Yes, with setup |
| AWS | A100 | p4d.24xlarge | $32.77 | Yes, 8xA100 for large-scale training |
| GCP | A100 | a2-highgpu-1g | $3.67 | Yes, with setup |
| RunPod | RTX 4090 | 1x4090 | $0.40 | Yes, community images |

### Docker for Cloud Deployment

```dockerfile
# Isaac Lab Docker image (official)
FROM nvcr.io/nvidia/isaac-lab:latest

# Your custom training code
COPY my_locomotion/ /workspace/my_locomotion/

# Default training command
CMD ["python", "my_locomotion/train.py", \
     "--task", "G1-Walking-v0", \
     "--headless", \
     "--num_envs", "4096"]
```

```bash
# Run with NVIDIA container runtime
docker run --gpus all --rm \
    -v $(pwd)/logs:/workspace/logs \
    my-isaac-training:latest
```

---

## Isaac Sim vs Gazebo Comparison

| Feature | Isaac Sim | Gazebo (Classic/Harmonic) |
|---|---|---|
| Physics engine | PhysX 5 (GPU-accelerated) | ODE, Bullet, DART, or TPE (CPU) |
| Parallel envs | Thousands on one GPU | One per process (or Gazebo Transport) |
| RL training speed | ~200K steps/sec (headless, 4096 envs) | ~1K steps/sec (single env) |
| Rendering | RTX ray tracing (photorealistic) | OpenGL / Ogre (functional) |
| Sensor sim | High-fidelity cameras, LiDAR, IMU | Good cameras, LiDAR, IMU |
| Scene format | USD | SDF / URDF |
| ROS integration | Yes (ROS 2 bridge) | Native (ROS 1/2) |
| Cost | Free for individual use | Free and open source |
| Learning curve | Steep (Omniverse ecosystem) | Moderate (well-documented) |
| GPU required | Yes (NVIDIA only) | No (CPU-based) |
| Best for | RL training, sim-to-real at scale | ROS development, multi-robot sims |

**For RL training:** Isaac Sim wins decisively due to GPU-accelerated parallel simulation.
Training that takes days in Gazebo takes hours in Isaac Sim.

**For ROS integration and prototyping:** Gazebo is simpler to set up and doesn't require
an NVIDIA GPU. Useful for testing ROS nodes before moving to Isaac Sim for RL.

---

## Performance Tips

### GPU Instancing

Isaac Lab automatically instances robot meshes across parallel environments. To maximize
performance:

```python
# Use fewer unique meshes in the scene
# BAD: 10 different rock meshes scattered around
# GOOD: 1-2 rock meshes reused across environments

# Reduce collision mesh complexity
# Use convex decomposition instead of triangle mesh for collision
spawn=sim_utils.UsdFileCfg(
    usd_path="robot.usd",
    activate_contact_sensors=True,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
    ),
    collision_props=sim_utils.CollisionPropertiesCfg(
        contact_offset=0.02,
        rest_offset=0.0,
    ),
)
```

### Reducing Rendering Overhead

```python
# During training, disable unnecessary rendering features
sim_cfg = SimulationCfg(
    dt=0.005,
    render_interval=0,      # 0 = no rendering at all (fastest)
    disable_contact_processing=False,  # Keep contacts (needed for foot sensors)
)
```

### Memory Management

```python
# Tune environment count based on available VRAM
# Monitor with: nvidia-smi -l 1

# If you run out of VRAM:
# 1. Reduce num_envs
# 2. Simplify robot mesh (fewer visual polygons)
# 3. Remove cameras if not needed
# 4. Use smaller observation/action buffers
```

---

## Complete Example: Simple Environment with a Robot

This creates a minimal Isaac Lab scene with a Unitree G1 on flat ground:

```python
"""Minimal Isaac Sim scene with Unitree G1."""

from isaaclab.app import AppLauncher

# Launch Isaac Sim (headless for training, GUI for debugging)
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationCfg, SimulationContext

# Physics configuration
sim_cfg = SimulationCfg(
    dt=0.005,
    render_interval=4,
)
sim = SimulationContext(sim_cfg)

# Set main camera (for GUI mode)
sim.set_camera_view(eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.5])

# Spawn ground plane
cfg = sim_utils.GroundPlaneCfg()
cfg.func("/World/Ground", cfg)

# Spawn dome light
cfg = sim_utils.DomeLightCfg(intensity=2000.0)
cfg.func("/World/Light", cfg)

# Spawn Unitree G1 robot
from isaaclab_assets.unitree import G1_CFG

robot_cfg: ArticulationCfg = G1_CFG.replace(
    prim_path="/World/Robot",
)
robot = Articulation(robot_cfg)

# Reset simulation
sim.reset()

# Run simulation loop
for i in range(1000):
    # Apply zero actions (robot stands with default PD control)
    robot.set_joint_position_target(robot.data.default_joint_pos)
    robot.write_data_to_sim()

    # Step physics
    sim.step()

    # Read robot state
    robot.update(sim_cfg.dt)

    if i % 100 == 0:
        base_pos = robot.data.root_pos_w[0]
        print(f"Step {i}: Robot position = {base_pos}")

simulation_app.close()
```

This example demonstrates:
1. Launching Isaac Sim in headless mode
2. Creating a ground plane and lighting
3. Spawning a Unitree G1 robot
4. Running a physics loop with position control
5. Reading robot state data

From here, you would wrap this in an Isaac Lab environment class (see `isaac-lab-envs.md`)
to add observations, rewards, and resets for RL training.
