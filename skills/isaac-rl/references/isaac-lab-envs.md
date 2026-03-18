# Isaac Lab Environments

## Overview

Isaac Lab is the RL framework built on top of Isaac Sim. It provides the abstractions you
need to define RL environments: observations, actions, rewards, resets, and parallel simulation.

Isaac Lab handles the boilerplate (parallel envs, GPU tensor management, simulation stepping)
so you can focus on the RL-specific design (what the robot observes, how it acts, what it gets
rewarded for).

### Key Classes

| Class | Purpose |
|---|---|
| `ManagerBasedRLEnv` | Base environment class |
| `ManagerBasedRLEnvCfg` | Environment configuration |
| `ObservationManager` | Collects observations |
| `ActionManager` | Applies actions |
| `RewardManager` | Computes rewards |
| `TerminationManager` | Checks if episode is done |
| `EventManager` | Handles randomization events |
| `SceneCfg` | Defines the physical scene |

---

## Environment Class Structure

An Isaac Lab RL environment is defined by a configuration dataclass. Here is the skeleton:

```python
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    TerminationTermCfg,
    SceneEntityCfg,
)
from isaaclab.utils import configclass
from dataclasses import MISSING

@configclass
class MyLocomotionEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion environment."""

    # Scene definition
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)

    # Observation space
    observations: ObservationsCfg = ObservationsCfg()

    # Action space
    actions: ActionsCfg = ActionsCfg()

    # Reward function
    rewards: RewardsCfg = RewardsCfg()

    # Termination conditions
    terminations: TerminationsCfg = TerminationsCfg()

    # Domain randomization events
    events: EventsCfg = EventsCfg()

    # Timing
    sim: SimulationCfg = SimulationCfg(dt=0.005, render_interval=4)
    decimation: int = 4  # RL runs at 50 Hz (0.005 * 4 = 0.02s per step)
    episode_length_s: float = 20.0  # Max 20 seconds per episode
```

---

## Observation Space Design

Observations are what the robot "sees." For locomotion, these are numbers representing the
robot's internal state (proprioception). The agent receives a flat vector of floats.

### What to Include (Locomotion)

| Observation | Shape per Env | Why It Matters |
|---|---|---|
| Base linear velocity (local frame) | (3,) | Robot needs to know how fast it's moving |
| Base angular velocity (local frame) | (3,) | Needed for balance (detecting rotation) |
| Projected gravity (base frame) | (3,) | Tells the robot which way is "up" |
| Velocity command (target) | (3,) | What velocity we want (vx, vy, yaw_rate) |
| Joint positions (normalized) | (num_joints,) | Current pose of all joints |
| Joint velocities (normalized) | (num_joints,) | How fast joints are moving |
| Previous actions | (num_joints,) | What the agent did last step (helps smooth policy) |

**Total observation dimension for G1:** ~3 + 3 + 3 + 3 + 23 + 23 + 23 = **81** floats

### Observation Configuration

```python
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg
import isaaclab.envs.mdp as mdp

@configclass
class ObservationsCfg:
    """Observation specifications."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for the policy (actor network)."""

        # Base velocity in local frame
        base_lin_vel = ObservationTermCfg(
            func=mdp.base_lin_vel,
            noise={"type": "gaussian", "mean": 0.0, "std": 0.05},
        )
        base_ang_vel = ObservationTermCfg(
            func=mdp.base_ang_vel,
            noise={"type": "gaussian", "mean": 0.0, "std": 0.02},
        )

        # Projected gravity tells the robot its orientation
        projected_gravity = ObservationTermCfg(
            func=mdp.projected_gravity,
            noise={"type": "gaussian", "mean": 0.0, "std": 0.05},
        )

        # Velocity command (what we want the robot to do)
        velocity_commands = ObservationTermCfg(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )

        # Joint state
        joint_pos = ObservationTermCfg(
            func=mdp.joint_pos_rel,  # Relative to default pose
            noise={"type": "gaussian", "mean": 0.0, "std": 0.01},
        )
        joint_vel = ObservationTermCfg(
            func=mdp.joint_vel_rel,
            noise={"type": "gaussian", "mean": 0.0, "std": 0.05},
        )

        # Previous action (helps smooth the policy)
        actions = ObservationTermCfg(func=mdp.last_action)

    policy: PolicyCfg = PolicyCfg()
```

### Observation Design Tips

1. **Normalize everything.** Joint positions should be relative to default pose and scaled.
   Joint velocities should be divided by max velocity. Raw values have wildly different scales
   and make training harder.

2. **Add noise during training.** Real sensors are noisy. Adding Gaussian noise to observations
   during training helps the policy be robust on real hardware.

3. **Start proprioceptive.** Don't add cameras or terrain height maps initially. Get walking
   working with joint state + IMU first, then add complexity.

4. **Include previous actions.** This gives the policy implicit velocity information and
   helps produce smoother motions.

5. **Don't include everything.** More observations = harder to train. Include only what the
   policy needs to make decisions.

---

## Action Space Design

Actions are what the agent commands the robot to do each step. For locomotion, this is
typically target joint positions or torques.

### Control Modes

| Mode | Action Meaning | Pros | Cons | Recommended For |
|---|---|---|---|---|
| **Position targets** | Desired joint angles | Stable, PD controller handles dynamics | Less dynamic motions | Start here. Most common for locomotion. |
| **Velocity targets** | Desired joint velocities | Good for continuous motion | Can be unstable without limits | When position control feels too stiff. |
| **Torques** | Direct motor torques | Maximum control authority | Hard to train, requires good actuator model | Advanced users only. |

### Action Configuration (Position Control)

```python
from isaaclab.managers import ActionTermCfg
import isaaclab.envs.mdp as mdp

@configclass
class ActionsCfg:
    """Action specifications."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],  # All joints
        scale=0.25,          # Scale actions: network outputs [-1, 1], scaled to [-0.25, 0.25] rad
        use_default_offset=True,  # Actions are offsets from default standing pose
    )
```

### Action Space Design Tips

1. **Scale actions appropriately.** Neural networks output values roughly in [-1, 1]. Scale
   these to reasonable joint angle changes. Too large (>0.5 rad) = jerky, unstable. Too small
   (<0.05 rad) = slow, limited motion.

2. **Use position offsets from default pose.** The agent outputs deltas from the standing
   pose rather than absolute joint angles. This means zero action = standing, which is a
   reasonable starting point.

3. **Start with position control.** It's the most forgiving. The built-in PD controller
   handles the low-level dynamics, and the RL policy just needs to output target angles.

4. **Action rate limiting.** You can limit how fast actions change between steps to prevent
   jerky motion:

```python
joint_pos = mdp.JointPositionActionCfg(
    asset_name="robot",
    joint_names=[".*"],
    scale=0.25,
    use_default_offset=True,
    # Limit action change rate (optional)
    # This smooths the policy output
)
```

---

## Reward Function Design

Reward design is the most important and most difficult part of RL. The reward function
defines what behavior the agent learns. **Bad rewards = bad behavior, even if training
"succeeds."**

### Reward Philosophy

Think of reward design like writing a performance review rubric. You need to:

1. **Clearly define success** (forward velocity)
2. **Penalize bad behavior** (falling, excessive energy, jerky motion)
3. **Balance competing objectives** (fast walking vs energy efficiency)
4. **Avoid loopholes** (the agent WILL find any exploit in your reward)

### Reward Components for Locomotion

| Component | Type | Purpose | Typical Weight |
|---|---|---|---|
| Forward velocity tracking | Positive | Walk at commanded speed | 1.0 - 2.0 |
| Angular velocity tracking | Positive | Turn at commanded rate | 0.5 - 1.0 |
| Upright bonus | Positive | Stay upright (don't lean) | 0.2 - 0.5 |
| Alive bonus | Positive | Small reward each step for not falling | 0.1 - 0.5 |
| Energy penalty | Negative | Penalize excessive motor torques | -0.001 to -0.01 |
| Action smoothness | Negative | Penalize jerky motions (action rate) | -0.01 to -0.05 |
| Joint acceleration | Negative | Penalize sudden joint movements | -0.001 to -0.01 |
| Joint limit penalty | Negative | Penalize being near joint limits | -0.1 to -1.0 |
| Foot air time | Positive | Encourage proper swing phase (not shuffling) | 0.1 - 0.5 |
| Orientation penalty | Negative | Penalize body roll/pitch | -0.1 to -0.5 |
| Base height | Positive | Maintain desired standing height | 0.1 - 0.5 |

### Sparse vs Dense Rewards

**Sparse reward:** "You get +1 when you reach the goal, 0 otherwise."
Problem: The agent may never accidentally reach the goal, so it never learns.

**Dense reward:** "You get reward proportional to how close you are to the goal at every step."
This guides the agent toward the goal incrementally.

**For locomotion: Always use dense rewards.** Sparse rewards don't work for continuous control
tasks. The forward velocity reward is already dense (reward every step the robot moves forward).

### Reward Configuration

```python
from isaaclab.managers import RewardTermCfg
import isaaclab.envs.mdp as mdp

@configclass
class RewardsCfg:
    """Reward specifications."""

    # === Positive rewards (encourage good behavior) ===

    # Track commanded forward velocity
    track_lin_vel_xy_exp = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.25},
    )

    # Track commanded angular velocity
    track_ang_vel_z_exp = RewardTermCfg(
        func=mdp.track_ang_vel_z_exp,
        weight=0.75,
        params={"command_name": "base_velocity", "std": 0.25},
    )

    # Stay alive bonus
    is_alive = RewardTermCfg(
        func=mdp.is_alive,
        weight=0.25,
    )

    # === Negative rewards (penalize bad behavior) ===

    # Penalize z-axis (vertical) linear velocity (bouncing)
    lin_vel_z_l2 = RewardTermCfg(
        func=mdp.lin_vel_z_l2,
        weight=-2.0,
    )

    # Penalize angular velocity on roll/pitch axes
    ang_vel_xy_l2 = RewardTermCfg(
        func=mdp.ang_vel_xy_l2,
        weight=-0.05,
    )

    # Penalize large joint torques (energy efficiency)
    joint_torques_l2 = RewardTermCfg(
        func=mdp.joint_torques_l2,
        weight=-0.0001,
    )

    # Penalize jerky actions (action rate = change between steps)
    action_rate_l2 = RewardTermCfg(
        func=mdp.action_rate_l2,
        weight=-0.01,
    )

    # Penalize being near joint limits
    joint_pos_limits = RewardTermCfg(
        func=mdp.joint_pos_limits,
        weight=-1.0,
    )

    # Penalize feet sliding on ground (should lift feet to walk)
    feet_slide = RewardTermCfg(
        func=mdp.feet_slide,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*_foot")},
    )
```

### Custom Reward Functions

When built-in rewards aren't enough, write custom ones:

```python
from __future__ import annotations
import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def foot_clearance_reward(env: ManagerBasedRLEnv, threshold: float = 0.05) -> torch.Tensor:
    """Reward for lifting feet during swing phase.

    Encourages the robot to lift its feet above a threshold height during
    the swing phase, preventing shuffling gaits.

    Args:
        env: The RL environment.
        threshold: Minimum foot height (meters) to get reward.

    Returns:
        Reward tensor of shape (num_envs,).
    """
    robot = env.scene["robot"]

    # Get foot positions in world frame
    # Shape: (num_envs, num_feet, 3)
    foot_pos = robot.data.body_pos_w[:, foot_indices, :]
    foot_heights = foot_pos[:, :, 2]  # z-coordinate

    # Get foot contact state
    contact_forces = env.scene["contact_sensor"].data.net_forces_w
    in_contact = contact_forces.norm(dim=-1) > 1.0  # 1N threshold

    # Reward feet that are not in contact AND above threshold
    swing_mask = ~in_contact  # Feet in swing phase
    above_threshold = foot_heights > threshold
    clearance_reward = (swing_mask & above_threshold).float().mean(dim=-1)

    return clearance_reward


# Register in reward config
foot_clearance = RewardTermCfg(
    func=foot_clearance_reward,
    weight=0.5,
    params={"threshold": 0.05},
)
```

### Common Reward Pitfalls

| Pitfall | What Happens | Fix |
|---|---|---|
| **Reward hacking** | Robot finds an exploit (e.g., vibrating in place to maximize velocity reward) | Add smoothness penalties; penalize high joint velocities/accelerations |
| **Competing rewards** | Positive and negative rewards cancel out; agent learns to stand still | Ensure forward velocity reward dominates penalties; tune weights |
| **Scale mismatch** | One reward dominates because its magnitude is 100x larger | Normalize all reward components to similar scales before weighting |
| **Missing penalty** | Robot slides on knees because there's no knee-ground contact penalty | Add contact penalty for non-foot body parts touching ground |
| **Too many penalties** | Agent avoids all penalties by standing still (safest option) | Increase positive reward for forward velocity; add time penalty (small negative per step) |
| **Reward not differentiable** | Binary reward (1 or 0) gives no gradient information | Use exponential or Gaussian shaping: `exp(-error^2 / std)` instead of `if error < threshold` |

---

## Task Registration and Configuration

Isaac Lab uses a registry to manage tasks. Register your environment so it can be launched
by name:

```python
import gymnasium as gym
from isaaclab.envs import ManagerBasedRLEnv

# Register the environment
gym.register(
    id="MyLab-G1-Walking-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": "my_locomotion.config.g1_walking:G1WalkingEnvCfg",
    },
    disable_env_checker=True,
)

# Now you can create it by name
env = gym.make("MyLab-G1-Walking-v0", num_envs=4096)
```

---

## Vectorized Environments

Isaac Lab runs thousands of environment copies in parallel on the GPU. This is the key to
fast training.

### How It Works

```
Traditional RL (CPU):                   Isaac Lab (GPU):

env_1.step(a1) -> obs_1, r_1           All 4096 envs step simultaneously
env_2.step(a2) -> obs_2, r_2           on the GPU in a single call:
env_3.step(a3) -> obs_3, r_3
...                                     env.step(actions)  # actions: (4096, num_joints)
env_N.step(aN) -> obs_N, r_N           # returns:
                                        #   obs:    (4096, obs_dim)
Sequential: O(N)                        #   reward: (4096,)
                                        #   done:   (4096,)

                                        Parallel: O(1) GPU operations
```

### Environment Count Guidelines

| VRAM | Max Humanoid Envs | Notes |
|---|---|---|
| 8 GB | ~512 | Tight; reduce observation size if needed |
| 12 GB | ~1024 | Comfortable for development |
| 24 GB | ~4096 | Standard training setup |
| 48 GB | ~8192 | Fast training |
| 80 GB | ~16384 | Research-grade throughput |

### Data Flow

All data in Isaac Lab is stored as PyTorch tensors on the GPU:

```python
# Everything is a GPU tensor -- no CPU-GPU transfer during training
observations = env.obs_buf        # Shape: (num_envs, obs_dim), device='cuda:0'
actions = policy(observations)     # Shape: (num_envs, act_dim), device='cuda:0'
env.step(actions)                  # All physics on GPU
rewards = env.reward_buf          # Shape: (num_envs,), device='cuda:0'
```

---

## Scene Creation with Isaac Lab API

The scene defines what exists in the simulation: robots, ground, sensors, objects.

```python
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
import isaaclab.sim as sim_utils

@configclass
class G1SceneCfg(InteractiveSceneCfg):
    """Scene with Unitree G1 on flat ground."""

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # Dome light
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)),
    )

    # Unitree G1 robot
    robot: ArticulationCfg = G1_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
    )

    # Contact sensor on feet
    contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*_foot",
        update_period=0.0,  # Every physics step
        history_length=3,
        track_air_time=True,
    )
```

---

## Reset Conditions

Episodes end (and reset) when certain conditions are met. Isaac Lab distinguishes between:

- **Termination**: Episode ends because something bad happened (robot fell)
- **Truncation**: Episode ends because of time limit (max episode length)

```python
from isaaclab.managers import TerminationTermCfg
import isaaclab.envs.mdp as mdp

@configclass
class TerminationsCfg:
    """Termination conditions."""

    # Time limit (truncation)
    time_out = TerminationTermCfg(
        func=mdp.time_out,
        time_out=True,  # This is truncation, not failure
    )

    # Robot fell (base height too low)
    base_contact = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names="base_link"),
            "threshold": 1.0,  # Contact force > 1N = fell
        },
    )

    # Robot orientation too far from upright
    bad_orientation = TerminationTermCfg(
        func=mdp.bad_orientation,
        params={"limit_angle": 0.5},  # ~29 degrees
    )
```

### Reset Behavior

When an environment resets:
1. Robot is placed back at initial position with some randomization
2. Joint positions are set to default standing pose (with optional noise)
3. Velocities are zeroed (or set to small random values)
4. Observations are recomputed
5. The episode counter increments

```python
# Configuring reset randomization
@configclass
class EventsCfg:
    """Randomization events."""

    # On reset, randomize robot initial state
    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5), "pitch": (-0.5, 0.5), "yaw": (-0.5, 0.5),
            },
        },
    )

    reset_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.8, 1.2),  # 80-120% of default position
            "velocity_range": (-0.1, 0.1),
        },
    )
```

---

## Complete Example: Walking Task Environment

Putting it all together -- a complete environment config for Unitree G1 walking:

```python
"""Unitree G1 walking environment configuration."""

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from isaaclab.utils import configclass
import isaaclab.envs.mdp as mdp
from isaaclab.sim import SimulationCfg
from isaaclab_assets.unitree import G1_CFG


@configclass
class G1WalkingEnvCfg(ManagerBasedRLEnvCfg):
    """Config for Unitree G1 flat-ground walking."""

    # --- Scene ---
    scene = G1SceneCfg(num_envs=4096, env_spacing=2.5)

    # --- Simulation ---
    sim = SimulationCfg(dt=0.005, render_interval=4)
    decimation = 4  # RL at 50 Hz
    episode_length_s = 20.0

    # --- Observations ---
    @configclass
    class ObservationsCfg:
        @configclass
        class PolicyCfg(ObservationGroupCfg):
            base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel)
            base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel)
            projected_gravity = ObservationTermCfg(func=mdp.projected_gravity)
            velocity_commands = ObservationTermCfg(
                func=mdp.generated_commands,
                params={"command_name": "base_velocity"},
            )
            joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel)
            joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel)
            actions = ObservationTermCfg(func=mdp.last_action)
        policy: PolicyCfg = PolicyCfg()

    observations = ObservationsCfg()

    # --- Actions ---
    @configclass
    class ActionsCfg:
        joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            scale=0.25,
            use_default_offset=True,
        )

    actions = ActionsCfg()

    # --- Rewards ---
    @configclass
    class RewardsCfg:
        # Positive
        track_lin_vel_xy_exp = RewardTermCfg(
            func=mdp.track_lin_vel_xy_exp, weight=1.5,
            params={"command_name": "base_velocity", "std": 0.25},
        )
        track_ang_vel_z_exp = RewardTermCfg(
            func=mdp.track_ang_vel_z_exp, weight=0.75,
            params={"command_name": "base_velocity", "std": 0.25},
        )
        is_alive = RewardTermCfg(func=mdp.is_alive, weight=0.25)

        # Negative
        lin_vel_z_l2 = RewardTermCfg(func=mdp.lin_vel_z_l2, weight=-2.0)
        ang_vel_xy_l2 = RewardTermCfg(func=mdp.ang_vel_xy_l2, weight=-0.05)
        joint_torques_l2 = RewardTermCfg(func=mdp.joint_torques_l2, weight=-0.0001)
        action_rate_l2 = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01)

    rewards = RewardsCfg()

    # --- Terminations ---
    @configclass
    class TerminationsCfg:
        time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)
        base_contact = TerminationTermCfg(
            func=mdp.illegal_contact,
            params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names="base_link"), "threshold": 1.0},
        )

    terminations = TerminationsCfg()

    # --- Events (randomization) ---
    @configclass
    class EventsCfg:
        reset_base = EventTermCfg(
            func=mdp.reset_root_state_uniform, mode="reset",
            params={"pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)}},
        )
        reset_joints = EventTermCfg(
            func=mdp.reset_joints_by_scale, mode="reset",
            params={"position_range": (0.9, 1.1), "velocity_range": (-0.1, 0.1)},
        )

    events = EventsCfg()
```

This is a complete, trainable environment. The next step is to pair it with a PPO
training config (see `rl-training.md`).
