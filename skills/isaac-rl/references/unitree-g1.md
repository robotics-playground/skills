# Unitree G1

## Hardware Overview

The Unitree G1 is a humanoid robot designed for research and development. It stands about
127 cm tall and weighs approximately 35 kg.

### Key Specifications

| Specification | Value |
|---|---|
| Height | ~127 cm |
| Weight | ~35 kg (without battery) |
| Degrees of Freedom | 23 (standard), up to 43 (dexterous hands) |
| Compute | NVIDIA Jetson Orin (onboard) |
| Battery | ~2 hours runtime (walking) |
| Max walking speed | ~2 m/s |
| Payload capacity | ~3 kg (carried) |
| Communication | Ethernet, WiFi, CycloneDDS |
| Sensors | IMU, joint encoders, foot force sensors, optional stereo camera, LiDAR |

### Joint Configuration

The G1 has the following joint groups:

```
HEAD (2 DOF):
  - head_yaw, head_pitch

LEFT ARM (6 DOF):
  - left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw
  - left_elbow_pitch, left_wrist_yaw, left_wrist_roll

RIGHT ARM (6 DOF):
  - right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw
  - right_elbow_pitch, right_wrist_yaw, right_wrist_roll

WAIST (3 DOF):
  - waist_yaw, waist_roll, waist_pitch

LEFT LEG (6 DOF):
  - left_hip_yaw, left_hip_roll, left_hip_pitch
  - left_knee_pitch
  - left_ankle_pitch, left_ankle_roll

RIGHT LEG (6 DOF):
  - right_hip_yaw, right_hip_roll, right_hip_pitch
  - right_knee_pitch
  - right_ankle_pitch, right_ankle_roll
```

**For locomotion:** The 12 leg joints (6 per leg) are the primary actuated joints. Arms
and waist joints can be used for balance but are often locked in initial experiments.

### Joint Limits and Torques

| Joint Group | Position Range (rad) | Max Torque (Nm) | Max Velocity (rad/s) |
|---|---|---|---|
| Hip yaw | [-0.8, 0.8] | 88 | 21 |
| Hip roll | [-0.5, 0.5] | 88 | 21 |
| Hip pitch | [-1.6, 1.6] | 88 | 21 |
| Knee | [-0.1, 2.1] | 139 | 16 |
| Ankle pitch | [-0.8, 0.8] | 50 | 30 |
| Ankle roll | [-0.3, 0.3] | 50 | 30 |

---

## unitree_sdk2 Python API

The `unitree_sdk2` is the official Python SDK for controlling Unitree robots. It uses
CycloneDDS for real-time communication.

### Installation

```bash
pip install unitree_sdk2py
# Also available: unitree_sdk2 (C++ version) for lower latency
```

### Communication Architecture

```
+------------------+     CycloneDDS      +------------------+
|  Your Controller |  <===============>  |  Unitree G1      |
|  (PC or Jetson)  |    UDP/Ethernet     |  (onboard MCU)   |
+------------------+                     +------------------+
```

CycloneDDS is a real-time communication middleware. Think of it as a pub/sub system
(similar to pub/sub messaging) but designed for robotics with microsecond latency.

### High-Level vs Low-Level Control

| Mode | What You Control | Latency | Use For |
|---|---|---|---|
| **High-level** | Velocity commands (vx, vy, yaw_rate) | ~10ms | Walking, navigation, teleoperation |
| **Low-level** | Individual joint positions/torques | ~2ms | RL policy deployment, custom gaits |

**For RL deployment: Use low-level mode.** Your trained policy outputs joint position targets
that must be sent directly to the motors.

### Low-Level Control Example

```python
"""Deploy a policy to Unitree G1 using low-level control."""

import time
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC


class G1PolicyRunner:
    """Runs a trained RL policy on the real Unitree G1."""

    CONTROL_FREQ = 50  # Hz, matching training
    DT = 1.0 / CONTROL_FREQ

    def __init__(self, policy_path: str):
        # Initialize DDS communication
        ChannelFactoryInitialize(0, "enp3s0")  # Network interface

        # Create publisher for motor commands
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()

        # Create subscriber for robot state
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._state_callback)

        # Load trained policy (ONNX or PyTorch)
        import torch
        self.policy = torch.jit.load(policy_path)
        self.policy.eval()

        self.latest_state = None
        self.crc = CRC()

    def _state_callback(self, msg: LowState_):
        """Called when new robot state arrives."""
        self.latest_state = msg

    def get_observations(self) -> np.ndarray:
        """Extract observation vector from robot state.

        Must match the observation space used during training.
        """
        state = self.latest_state
        if state is None:
            return None

        obs = []

        # Base angular velocity (from IMU)
        obs.extend([
            state.imu_state.gyroscope[0],
            state.imu_state.gyroscope[1],
            state.imu_state.gyroscope[2],
        ])

        # Projected gravity (from IMU quaternion)
        quat = state.imu_state.quaternion  # w, x, y, z
        gravity = self._project_gravity(quat)
        obs.extend(gravity)

        # Joint positions (relative to default)
        for i in range(12):  # 12 leg joints
            obs.append(state.motor_state[i].q - self.default_joint_pos[i])

        # Joint velocities
        for i in range(12):
            obs.append(state.motor_state[i].dq)

        # Previous action
        obs.extend(self.prev_action.tolist())

        return np.array(obs, dtype=np.float32)

    def send_joint_targets(self, positions: np.ndarray):
        """Send joint position targets to the robot."""
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF  # Low-level mode

        for i in range(12):
            cmd.motor_cmd[i].mode = 0x0A  # Position control
            cmd.motor_cmd[i].q = float(positions[i])
            cmd.motor_cmd[i].kp = 80.0    # Must match training PD gains
            cmd.motor_cmd[i].kd = 4.0
            cmd.motor_cmd[i].tau = 0.0    # No feedforward torque
            cmd.motor_cmd[i].dq = 0.0

        cmd.crc = self.crc.Crc(cmd)
        self.cmd_pub.Write(cmd)

    def run(self, duration: float = 30.0):
        """Run the policy loop."""
        import torch

        start_time = time.time()
        while time.time() - start_time < duration:
            loop_start = time.time()

            # Get observation
            obs = self.get_observations()
            if obs is None:
                time.sleep(self.DT)
                continue

            # Run policy
            with torch.no_grad():
                obs_tensor = torch.tensor(obs).unsqueeze(0)
                action = self.policy(obs_tensor).squeeze(0).numpy()

            # Scale action and add to default pose
            joint_targets = self.default_joint_pos + action * 0.25

            # Safety: clip to joint limits
            joint_targets = np.clip(joint_targets, self.joint_lower, self.joint_upper)

            # Send to robot
            self.send_joint_targets(joint_targets)
            self.prev_action = action

            # Maintain control frequency
            elapsed = time.time() - loop_start
            if elapsed < self.DT:
                time.sleep(self.DT - elapsed)
```

---

## Unitree G1 in Isaac Lab

Isaac Lab provides pre-configured robot models for the Unitree G1.

### Built-In Config

```python
from omni.isaac.lab_assets.unitree import UNITREE_G1_CFG

# This provides:
# - USD model with accurate link masses and inertias
# - Joint limits matching hardware specs
# - Default standing pose
# - Actuator configuration

robot_cfg = UNITREE_G1_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot",
)
```

### Customizing the G1 Config

```python
from omni.isaac.lab.actuators import ImplicitActuatorCfg

# Override actuator settings to match your specific robot
custom_g1_cfg = UNITREE_G1_CFG.copy()
custom_g1_cfg.actuators = {
    "legs": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_hip_yaw", ".*_hip_roll", ".*_hip_pitch",
            ".*_knee",
            ".*_ankle_pitch", ".*_ankle_roll",
        ],
        stiffness=80.0,   # PD gain Kp
        damping=4.0,       # PD gain Kd
        # Match these to your robot's actual PD gains
    ),
    "arms": ImplicitActuatorCfg(
        joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"],
        stiffness=40.0,
        damping=2.0,
    ),
}
```

### Built-In G1 Tasks

Isaac Lab includes pre-built tasks for the G1:

```bash
# List available G1 tasks
python -c "import gymnasium as gym; [print(s) for s in gym.registry.keys() if 'G1' in s]"

# Common tasks:
# Isaac-Velocity-Flat-Unitree-G1-v0    -- Walk on flat ground
# Isaac-Velocity-Rough-Unitree-G1-v0   -- Walk on rough terrain
```

---

## Standing, Walking, and Balance Tasks

### Standing (First Goal)

Before walking, get the robot to stand stably. This is a simpler task that verifies your
setup is correct.

```python
# Standing task reward: minimize motion, stay upright
@configclass
class StandingRewardsCfg:
    # Reward being upright
    upright = RewardTermCfg(
        func=mdp.upright_posture,
        weight=1.0,
    )
    # Penalize any joint velocity (should be still)
    joint_vel = RewardTermCfg(
        func=mdp.joint_vel_l2,
        weight=-0.1,
    )
    # Penalize base linear velocity (should not move)
    base_lin_vel = RewardTermCfg(
        func=mdp.lin_vel_l2,
        weight=-0.5,
    )
    # Maintain target height
    base_height = RewardTermCfg(
        func=mdp.base_height_l2,
        weight=-1.0,
        params={"target_height": 0.72},  # G1 standing height
    )
```

### Walking (Primary Task)

See `isaac-lab-envs.md` for the complete walking environment configuration.

### Balance Recovery

Once walking works, train push recovery:

```python
# Add to events config
push_robot = EventTermCfg(
    func=mdp.push_by_setting_velocity,
    mode="interval",
    interval_range_s=(8.0, 12.0),
    params={
        "velocity_range": {
            "x": (-1.0, 1.0),
            "y": (-1.0, 1.0),
        },
    },
)
```

---

## Teleoperation for Data Collection

Teleoperation (manual control) is useful for collecting demonstration data or testing
hardware before deploying RL policies.

### Keyboard Teleoperation

```python
"""Simple keyboard teleop for Unitree G1 high-level control."""

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeCmd_
import threading
import sys
import tty
import termios

class KeyboardTeleop:
    def __init__(self):
        ChannelFactoryInitialize(0, "enp3s0")
        self.cmd_pub = ChannelPublisher("rt/sportmodecmd", SportModeCmd_)
        self.cmd_pub.Init()

        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        print("Keyboard Teleop:")
        print("  W/S: Forward/Backward")
        print("  A/D: Left/Right")
        print("  Q/E: Rotate Left/Right")
        print("  Space: Stop")
        print("  Esc: Quit")

        while True:
            key = self.get_key()
            if key == 'w': self.vx = min(self.vx + 0.1, 1.0)
            elif key == 's': self.vx = max(self.vx - 0.1, -0.5)
            elif key == 'a': self.vy = min(self.vy + 0.1, 0.5)
            elif key == 'd': self.vy = max(self.vy - 0.1, -0.5)
            elif key == 'q': self.vyaw = min(self.vyaw + 0.1, 1.0)
            elif key == 'e': self.vyaw = max(self.vyaw - 0.1, -1.0)
            elif key == ' ': self.vx = self.vy = self.vyaw = 0.0
            elif key == '\x1b': break

            cmd = SportModeCmd_()
            cmd.velocity = [self.vx, self.vy, self.vyaw]
            self.cmd_pub.Write(cmd)

            print(f"\rVx: {self.vx:.1f}  Vy: {self.vy:.1f}  Vyaw: {self.vyaw:.1f}", end="")
```

---

## Safety Considerations

**The G1 is a 35kg bipedal robot. It can fall and cause injury or damage.** Take safety
seriously.

### Essential Safety Rules

1. **Always have someone ready to catch the robot.** During early testing, have a person
   standing behind/beside the robot with hands near it.

2. **Use the emergency stop.** The G1 has a hardware e-stop button. Know where it is.
   Test it before every session.

3. **Start with slow commands.** Never go from standing to full speed. Ramp up gradually:
   0.1 m/s, 0.2 m/s, 0.3 m/s, etc.

4. **Test in a safe area.** Padded floor or outdoor grass. No obstacles, no stairs, no people
   nearby.

5. **Have a kill script ready.** A script that immediately sends zero torques to all joints:

```python
"""Emergency stop: zero all motor torques."""

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

ChannelFactoryInitialize(0, "enp3s0")
pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()

cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0] = 0xFE
cmd.head[1] = 0xEF
cmd.level_flag = 0xFF

crc = CRC()
for i in range(23):
    cmd.motor_cmd[i].mode = 0x00  # Disable motor
    cmd.motor_cmd[i].q = 0.0
    cmd.motor_cmd[i].kp = 0.0
    cmd.motor_cmd[i].kd = 0.0
    cmd.motor_cmd[i].tau = 0.0

cmd.crc = crc.Crc(cmd)

# Send multiple times to ensure delivery
import time
for _ in range(10):
    pub.Write(cmd)
    time.sleep(0.01)

print("Emergency stop sent. All motors disabled.")
```

### Safety Checklist Before Each Test

- [ ] E-stop tested and within reach
- [ ] Kill script ready on a separate terminal
- [ ] Safe testing area clear of obstacles and people
- [ ] Robot battery charged (low battery = unpredictable behavior)
- [ ] Policy tested in simulation with similar conditions
- [ ] Speed limits set in policy deployment script
- [ ] Spotter present with hands near robot

---

## Deployment Workflow: Sim Policy to Real Robot

```
1. TRAIN in Isaac Lab (headless, 4096+ envs)
        |
        v
2. EVALUATE in Isaac Lab (visual, 16 envs, verify behavior)
        |
        v
3. EXPORT policy (PyTorch JIT or ONNX)
        |
        v
4. TEST on real robot with STAND command first (verify communication)
        |
        v
5. DEPLOY walking policy at LOW SPEED (0.2 m/s, with spotter)
        |
        v
6. GRADUALLY increase speed and reduce support
        |
        v
7. ADD perturbations (gentle pushes) to test robustness
        |
        v
8. ITERATE: if failures, go back to simulation, adjust randomization
```

### Exporting the Policy

```python
"""Export trained policy for deployment."""

import torch

# Load trained checkpoint
checkpoint = torch.load("logs/g1_walking/model_1500.pt")

# Extract just the actor (policy) network
actor_state_dict = {
    k.replace("actor.", ""): v
    for k, v in checkpoint["model_state_dict"].items()
    if k.startswith("actor.")
}

# Create a standalone policy module
class DeployablePolicy(torch.nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(obs_dim, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 64),
            torch.nn.ELU(),
            torch.nn.Linear(64, act_dim),
        )

    def forward(self, obs):
        return self.net(obs)

policy = DeployablePolicy(obs_dim=81, act_dim=12)
policy.load_state_dict(actor_state_dict)
policy.eval()

# Export as TorchScript (can run without Python)
scripted = torch.jit.script(policy)
scripted.save("g1_walking_policy.pt")
print("Policy exported to g1_walking_policy.pt")

# Export as ONNX (for TensorRT optimization on Jetson)
dummy_obs = torch.randn(1, 81)
torch.onnx.export(
    policy, dummy_obs, "g1_walking_policy.onnx",
    input_names=["observations"],
    output_names=["actions"],
    dynamic_axes={"observations": {0: "batch"}, "actions": {0: "batch"}},
)
print("Policy exported to g1_walking_policy.onnx")
```

See `sim-to-real.md` for detailed deployment steps and safety protocols.

---

## Complete Example: G1 Standing Balance Task Config

```python
"""Unitree G1 standing balance task for Isaac Lab.

A good first task: train the robot to stand still and maintain balance.
Simpler than walking, verifies your environment setup is correct.
"""

from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from omni.isaac.lab.utils import configclass
import omni.isaac.lab.envs.mdp as mdp
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab_assets.unitree import UNITREE_G1_CFG


@configclass
class G1StandingEnvCfg(ManagerBasedRLEnvCfg):
    """Config for G1 standing balance."""

    # Scene
    scene = G1SceneCfg(num_envs=4096, env_spacing=2.0)

    # Simulation
    sim = SimulationCfg(dt=0.005, render_interval=4)
    decimation = 4
    episode_length_s = 10.0  # Shorter episodes for standing

    # Observations
    @configclass
    class ObservationsCfg:
        @configclass
        class PolicyCfg(ObservationGroupCfg):
            base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel)
            projected_gravity = ObservationTermCfg(func=mdp.projected_gravity)
            joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel)
            joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel)
            actions = ObservationTermCfg(func=mdp.last_action)
        policy: PolicyCfg = PolicyCfg()
    observations = ObservationsCfg()

    # Actions: only leg joints
    @configclass
    class ActionsCfg:
        joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                ".*_hip_yaw", ".*_hip_roll", ".*_hip_pitch",
                ".*_knee",
                ".*_ankle_pitch", ".*_ankle_roll",
            ],
            scale=0.1,  # Small actions for standing
            use_default_offset=True,
        )
    actions = ActionsCfg()

    # Rewards: stay upright and still
    @configclass
    class RewardsCfg:
        # Stay upright
        upright = RewardTermCfg(func=mdp.flat_orientation_l2, weight=-1.0)
        # Minimize base velocity (stand still)
        base_vel = RewardTermCfg(func=mdp.lin_vel_l2, weight=-0.5)
        base_ang_vel = RewardTermCfg(func=mdp.ang_vel_xy_l2, weight=-0.1)
        # Minimize joint velocity (be still)
        joint_vel = RewardTermCfg(func=mdp.joint_vel_l2, weight=-0.01)
        # Alive bonus
        alive = RewardTermCfg(func=mdp.is_alive, weight=1.0)
        # Action smoothness
        action_rate = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01)
    rewards = RewardsCfg()

    # Terminations
    @configclass
    class TerminationsCfg:
        time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)
        base_contact = TerminationTermCfg(
            func=mdp.illegal_contact,
            params={
                "sensor_cfg": SceneEntityCfg("contact_sensor", body_names="base_link"),
                "threshold": 1.0,
            },
        )
    terminations = TerminationsCfg()

    # Events: push the robot to train balance recovery
    @configclass
    class EventsCfg:
        reset_base = EventTermCfg(
            func=mdp.reset_root_state_uniform,
            mode="reset",
            params={"pose_range": {"yaw": (-3.14, 3.14)}},
        )
        reset_joints = EventTermCfg(
            func=mdp.reset_joints_by_scale,
            mode="reset",
            params={"position_range": (0.95, 1.05), "velocity_range": (-0.1, 0.1)},
        )
        # Gentle pushes every 5-8 seconds
        push_robot = EventTermCfg(
            func=mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=(5.0, 8.0),
            params={
                "velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)},
            },
        )
    events = EventsCfg()
```

This standing task should converge in ~200-500 iterations with 4096 environments. Once it
works, move to the walking task (see `isaac-lab-envs.md`).
