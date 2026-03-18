# Sim-to-Real Transfer

## The Sim-to-Real Gap

The sim-to-real gap is the difference between simulated and real-world conditions that causes
a policy trained in simulation to fail on the real robot. This is the central challenge of
simulation-based RL for robotics.

Think of it like developing locally vs deploying to production. Your app works perfectly on
localhost, but in production there's latency, different hardware, edge cases, and real users.
The same principle applies: the simulator is localhost, the real robot is production.

---

## Sources of Sim-to-Real Gap

### Physics Modeling Errors

| Source | What's Wrong | Impact |
|---|---|---|
| **Friction** | Simulated friction coefficients don't match real surfaces | Robot slips or sticks unexpectedly |
| **Contact dynamics** | Simulated contacts are softer/stiffer than real | Foot impacts feel different; balance affected |
| **Mass and inertia** | URDF/USD model has approximate values | Center of mass is wrong; robot leans unexpectedly |
| **Joint damping** | Simulated damping is idealized | Joints move differently; oscillation or sluggishness |
| **Deformable surfaces** | Sim assumes rigid ground | Carpet, grass, soft flooring absorb energy differently |

### Actuator Modeling Gaps

This is often the **largest source** of sim-to-real gap.

| Source | What's Wrong | Impact |
|---|---|---|
| **Motor response delay** | Real motors take 5-20ms to respond; sim is instant | Actions arrive late; timing is off |
| **Torque-speed curve** | Real motors produce less torque at high speed; sim ignores this | Policy expects torque that isn't available |
| **Backlash** | Gears have play; sim has zero backlash | Small motions don't produce expected movement |
| **Motor saturation** | Real motors have hard torque limits; sim may not enforce correctly | Policy commands impossible torques |
| **Temperature effects** | Hot motors lose torque; sim motors don't heat up | Performance degrades during long runs |

### Sensor Differences

| Source | What's Wrong | Impact |
|---|---|---|
| **IMU noise** | Real IMU has drift, bias, and noise; sim IMU is perfect | Orientation estimate is noisier |
| **Joint encoder noise** | Real encoders have quantization and noise | Joint positions are less precise |
| **Sensor latency** | Real sensors have measurement delay | Observations are stale by 1-5ms |
| **Sensor calibration** | Real sensor offset/scale differs from nominal | Systematic bias in observations |

### Timing and Latency

| Source | What's Wrong | Impact |
|---|---|---|
| **Control loop jitter** | Real control loops don't execute at perfectly fixed intervals | Variable dt between actions |
| **Communication delay** | Network latency between compute and motors | Additional 1-5ms delay |
| **Computation time** | Policy inference takes time; sim assumes instant | Actions are computed on stale observations |

---

## Domain Randomization as Primary Mitigation

Domain randomization (see `domain-randomization.md`) is the primary technique for closing
the sim-to-real gap. The idea: if you randomize simulation parameters across a wide enough
range, the real world becomes "just another sample" from the distribution.

### Required Randomization for Sim-to-Real

At minimum, randomize these parameters:

```python
# Minimum randomization for sim-to-real transfer

@configclass
class SimToRealEventsCfg:
    # Physics
    friction = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "static_friction_range": (0.4, 2.5),
            "dynamic_friction_range": (0.4, 2.5),
        },
    )
    mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
        },
    )
    com = EventTermCfg(
        func=mdp.randomize_rigid_body_com,
        mode="reset",
        params={
            "com_distribution_params": {
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.03, 0.03),
            },
        },
    )

    # Actuators
    motor_strength = EventTermCfg(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "stiffness_distribution_params": (0.7, 1.3),
            "damping_distribution_params": (0.7, 1.3),
        },
    )

    # Perturbations
    push = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(8.0, 12.0),
        params={"velocity_range": {"x": (-0.8, 0.8), "y": (-0.8, 0.8)}},
    )

# And add noise to observations (see domain-randomization.md)
```

---

## System Identification

System identification ("sys-id") measures the real robot's physical parameters so you can
make the simulation more accurate. This reduces the gap that randomization needs to cover.

### What to Measure

| Parameter | How to Measure | Tool |
|---|---|---|
| **Robot mass** | Weigh the robot on a scale | Scale |
| **Link masses** | Weigh individual components (if disassembled) or estimate from CAD | Scale, CAD |
| **Center of mass** | Suspend from two points, find intersection of plumb lines | String, plumb bob |
| **Motor PD response** | Command step inputs, record position response | unitree_sdk2 logging |
| **Joint friction** | Move joints slowly, measure required torque | Torque sensor or current measurement |
| **Floor friction** | Drag a weighted block, measure force | Spring scale |
| **IMU noise** | Record IMU data while robot is stationary | unitree_sdk2 logging + numpy |

### Motor Characterization Example

```python
"""Characterize motor response for system identification."""

import time
import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_

class MotorCharacterizer:
    """Records motor response to step commands."""

    def __init__(self):
        ChannelFactoryInitialize(0, "enp3s0")
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._callback)
        self.log = []
        self.latest_state = None

    def _callback(self, msg):
        self.latest_state = msg

    def step_response_test(self, joint_idx: int, target_offset: float = 0.1):
        """Send a step command and record the response.

        Args:
            joint_idx: Which joint to test (0-11 for legs).
            target_offset: Step size in radians.
        """
        # Record initial position
        time.sleep(0.5)
        initial_pos = self.latest_state.motor_state[joint_idx].q

        # Send step command
        target = initial_pos + target_offset
        start_time = time.time()

        for _ in range(500):  # 10 seconds at 50 Hz
            t = time.time() - start_time
            state = self.latest_state

            self.log.append({
                "time": t,
                "commanded_pos": target,
                "actual_pos": state.motor_state[joint_idx].q,
                "actual_vel": state.motor_state[joint_idx].dq,
                "actual_torque": state.motor_state[joint_idx].tau_est,
            })

            self._send_position(joint_idx, target)
            time.sleep(0.02)

        return self.log

    def analyze(self):
        """Analyze step response to extract motor parameters."""
        import matplotlib.pyplot as plt

        times = [d["time"] for d in self.log]
        commanded = [d["commanded_pos"] for d in self.log]
        actual = [d["actual_pos"] for d in self.log]

        plt.figure(figsize=(10, 5))
        plt.plot(times, commanded, "r--", label="Commanded")
        plt.plot(times, actual, "b-", label="Actual")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.legend()
        plt.title("Motor Step Response")
        plt.savefig("motor_step_response.png")

        # Extract key metrics
        # Rise time: time to reach 90% of target
        # Overshoot: max position beyond target
        # Settling time: time to stay within 2% of target
        # These inform your actuator model in simulation
```

---

## Actuator Network Training

For the most accurate sim-to-real transfer, train a neural network to model the actuator
dynamics from real data.

### The Idea

Instead of using an idealized PD controller in simulation, use a learned model that captures
the real motor's behavior:

```
Idealized PD:     torque = Kp * (target - actual) + Kd * (0 - velocity)
Actuator Network:  torque = network(target, actual, velocity, history)
```

### Data Collection

```python
"""Collect motor data for actuator network training."""

def collect_motor_data(duration: float = 300.0) -> dict:
    """Run the robot with random commands and record motor data.

    Collect at least 5 minutes of diverse motion data.
    """
    data = {
        "timestamps": [],
        "commanded_positions": [],    # What we asked for
        "actual_positions": [],       # What we got
        "actual_velocities": [],      # How fast joints moved
        "actual_torques": [],         # Estimated torques
    }

    # Send random sinusoidal commands to explore the motor's operating range
    freq = np.random.uniform(0.5, 5.0, size=12)  # Different freq per joint
    amp = np.random.uniform(0.05, 0.3, size=12)  # Different amplitude per joint

    start = time.time()
    while time.time() - start < duration:
        t = time.time() - start
        targets = default_pos + amp * np.sin(2 * np.pi * freq * t)

        send_position_command(targets)
        state = get_robot_state()

        data["timestamps"].append(t)
        data["commanded_positions"].append(targets.copy())
        data["actual_positions"].append(state.joint_positions.copy())
        data["actual_velocities"].append(state.joint_velocities.copy())
        data["actual_torques"].append(state.joint_torques.copy())

        time.sleep(0.02)  # 50 Hz

    # Convert to numpy arrays
    for key in data:
        data[key] = np.array(data[key])

    np.savez("motor_data.npz", **data)
    return data
```

### Training the Actuator Network

```python
"""Train an actuator network from real motor data."""

import torch
import torch.nn as nn
import numpy as np

class ActuatorNetwork(nn.Module):
    """Learns mapping: (command_history, state_history) -> torque."""

    def __init__(self, history_len: int = 6):
        super().__init__()
        # Input: history_len steps of (commanded_pos, actual_pos, actual_vel)
        input_dim = history_len * 3
        self.net = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.ELU(),
            nn.Linear(64, 32),
            nn.ELU(),
            nn.Linear(32, 1),  # Output: torque for one joint
        )

    def forward(self, x):
        return self.net(x)


def train_actuator_network():
    data = np.load("motor_data.npz")
    history_len = 6

    # Prepare training data
    X, Y = [], []
    for t in range(history_len, len(data["timestamps"])):
        for joint in range(12):
            history = []
            for h in range(history_len):
                idx = t - history_len + h
                history.extend([
                    data["commanded_positions"][idx, joint],
                    data["actual_positions"][idx, joint],
                    data["actual_velocities"][idx, joint],
                ])
            X.append(history)
            Y.append(data["actual_torques"][t, joint])

    X = torch.tensor(np.array(X), dtype=torch.float32)
    Y = torch.tensor(np.array(Y), dtype=torch.float32).unsqueeze(-1)

    model = ActuatorNetwork(history_len)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

    for epoch in range(100):
        pred = model(X)
        loss = nn.MSELoss()(pred, Y)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if epoch % 10 == 0:
            print(f"Epoch {epoch}: Loss = {loss.item():.6f}")

    torch.save(model.state_dict(), "actuator_network.pt")
    print("Actuator network saved.")
```

---

## Policy Deployment Steps

### Step 1: Export Policy

```bash
# Export trained policy to TorchScript or ONNX
python export_policy.py --checkpoint logs/g1_walking/model_1500.pt --output policy.pt
```

### Step 2: Verify on Jetson (if deploying on-robot)

```python
# Test that policy runs on Jetson Orin
import torch
policy = torch.jit.load("policy.pt")
dummy_obs = torch.randn(1, 81)  # Match observation dimension
action = policy(dummy_obs)
print(f"Policy output shape: {action.shape}")
print(f"Inference time: {measure_inference_time(policy, dummy_obs):.2f} ms")
# Should be < 1ms on Jetson Orin
```

### Step 3: Test Communication

```python
# Verify you can read state and send commands
# Don't move any joints yet -- just test the communication pipeline
runner = G1PolicyRunner("policy.pt")
obs = runner.get_observations()
print(f"Observation: {obs}")
print(f"Observation dimension: {len(obs)}")
# Verify observation dimension matches training
```

### Step 4: Stand Test

```python
# First test: stand in place with the trained policy
# The policy should maintain balance without walking commands
runner = G1PolicyRunner("policy.pt")
runner.set_command(vx=0.0, vy=0.0, vyaw=0.0)  # Stand still
runner.run(duration=10.0)
```

### Step 5: Slow Walk Test

```python
# Second test: walk very slowly
# Have a spotter ready to catch the robot
runner = G1PolicyRunner("policy.pt")
runner.set_command(vx=0.2, vy=0.0, vyaw=0.0)  # 0.2 m/s forward
runner.run(duration=10.0)
```

### Step 6: Gradual Speed Increase

```python
# Gradually increase speed over multiple test sessions
speeds = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5]
for speed in speeds:
    print(f"Testing at {speed} m/s -- press Enter when ready")
    input()
    runner.set_command(vx=speed, vy=0.0, vyaw=0.0)
    runner.run(duration=10.0)
    print(f"Speed {speed} m/s: Success? (y/n)")
    result = input()
    if result.lower() != 'y':
        print(f"Failed at {speed} m/s. Go back to simulation.")
        break
```

---

## Gradual Deployment Strategy

```
Day 1: Communication test (no motion)
  - Verify observation pipeline
  - Verify action pipeline (send zero actions)
  - Check timing (50 Hz loop maintained?)

Day 2: Standing test
  - Deploy standing policy
  - Test balance with gentle manual pushes
  - Record observations for offline comparison with sim

Day 3: Slow walking (0.2 m/s)
  - Forward only
  - With spotter
  - On flat, clear ground
  - Record video and data

Day 4: Medium walking (0.5 m/s)
  - If Day 3 was successful
  - Add turning commands
  - Test on different floor surfaces

Day 5+: Progressive testing
  - Higher speeds
  - Lateral walking
  - Rough surfaces
  - Push recovery (gentle pushes)
  - Longer duration tests
```

---

## Gazebo Cross-Validation

Before deploying to real hardware, test in a second simulator (Gazebo) as a sanity check.
If the policy fails in both Isaac Sim and Gazebo, there may be a fundamental issue. If it
works in Isaac Sim but fails in Gazebo, the sim-to-real gap analysis should focus on physics
differences.

```python
# Export policy for Gazebo testing
# Gazebo uses ROS 2, so wrap the policy in a ROS 2 node

"""ROS 2 node for deploying RL policy in Gazebo."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import torch
import numpy as np

class PolicyNode(Node):
    def __init__(self):
        super().__init__("rl_policy")
        self.policy = torch.jit.load("policy.pt")
        self.policy.eval()

        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/joint_position_controller/commands", 10
        )
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

    def joint_callback(self, msg):
        self.latest_joint_state = msg

    def control_loop(self):
        obs = self.build_observation()
        with torch.no_grad():
            action = self.policy(torch.tensor(obs).unsqueeze(0))
        action = action.squeeze(0).numpy()
        targets = self.default_pos + action * 0.25

        msg = Float64MultiArray()
        msg.data = targets.tolist()
        self.cmd_pub.publish(msg)
```

---

## Real-World Testing Safety Protocols

### Speed Limits

Always enforce speed limits in the deployment code, even if the policy doesn't command them:

```python
# Enforce safety limits on policy output
def safe_action(action: np.ndarray, prev_action: np.ndarray) -> np.ndarray:
    """Apply safety limits to policy actions."""

    # Limit action magnitude (joint position offset)
    MAX_ACTION = 0.3  # radians
    action = np.clip(action, -MAX_ACTION, MAX_ACTION)

    # Limit action rate (change between steps)
    MAX_RATE = 0.1  # radians per step (at 50 Hz = 5 rad/s)
    delta = action - prev_action
    delta = np.clip(delta, -MAX_RATE, MAX_RATE)
    action = prev_action + delta

    # Clip final joint targets to hardware limits
    targets = default_pos + action * 0.25
    targets = np.clip(targets, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

    return action
```

### Fall Detection

```python
def check_safety(state) -> bool:
    """Check if robot is in a safe state. Returns False if emergency stop needed."""

    # Check orientation (robot too tilted)
    roll, pitch, yaw = quaternion_to_euler(state.imu_state.quaternion)
    if abs(roll) > 0.8 or abs(pitch) > 0.8:  # ~45 degrees
        print("SAFETY: Excessive tilt detected!")
        return False

    # Check base height (estimated from leg kinematics)
    base_height = estimate_base_height(state)
    if base_height < 0.3:  # Robot is falling or has fallen
        print("SAFETY: Low base height!")
        return False

    # Check joint velocities (too fast = something is wrong)
    for i in range(12):
        if abs(state.motor_state[i].dq) > 25.0:  # rad/s
            print(f"SAFETY: Joint {i} velocity too high!")
            return False

    return True
```

---

## Monitoring Deployed Policies

### Logging

Always log data during real-world tests for post-hoc analysis:

```python
import csv
import time

class DeploymentLogger:
    """Log observations, actions, and metadata during deployment."""

    def __init__(self, filename: str):
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "timestamp", "obs_0", "obs_1", "...",  # All observations
            "act_0", "act_1", "...",                # All actions
            "reward_estimate",                       # Compute reward using same function as training
            "base_height", "roll", "pitch",
        ])

    def log(self, obs, action, state):
        row = [time.time()]
        row.extend(obs.tolist())
        row.extend(action.tolist())
        row.append(self.estimate_reward(obs))
        row.extend([
            estimate_base_height(state),
            *quaternion_to_euler(state.imu_state.quaternion)[:2],
        ])
        self.writer.writerow(row)

    def close(self):
        self.file.close()
```

### Anomaly Detection

```python
def detect_anomalies(obs_history: list, window: int = 50) -> list:
    """Detect unusual patterns in observation history.

    Returns list of anomaly descriptions.
    """
    anomalies = []
    recent = np.array(obs_history[-window:])

    # Check for observation spikes (sensor malfunction?)
    for dim in range(recent.shape[1]):
        if np.std(recent[:, dim]) > 5.0:  # Unusually high variance
            anomalies.append(f"High variance in obs dim {dim}")

    # Check for constant observations (sensor frozen?)
    for dim in range(recent.shape[1]):
        if np.std(recent[:, dim]) < 1e-6:
            anomalies.append(f"Frozen obs dim {dim}")

    return anomalies
```

---

## When to Go Back to Simulation

Go back to simulation when:

| Failure | What to Do in Simulation |
|---|---|
| Robot falls within 2 seconds | Increase domain randomization ranges; check actuator model |
| Robot walks but drifts sideways | Add lateral velocity penalty; check IMU calibration |
| Robot walks but too jerky | Increase action smoothness penalty; reduce action scale |
| Robot falls on specific terrain | Add that terrain type to training; increase terrain randomization |
| Robot can't handle pushes | Increase push force range; train longer with perturbations |
| Policy works at 0.3 m/s but not 0.6 m/s | Train with higher speed commands; add speed curriculum |
| Joints overheat on real robot | Add energy penalty; reduce action magnitude |

### Failure Analysis Workflow

```
1. Record failure video and data
    |
    v
2. Reproduce in simulation
    - Set sim parameters to match observed conditions
    - Does it fail in sim too?
    |
    |--- YES: Fix reward/training/randomization
    |
    |--- NO: Sim-to-real gap is the issue
            |
            v
3. Identify gap source
    - Compare sim vs real observations
    - Compare sim vs real motor responses
    - Check timing (is 50 Hz maintained?)
    |
    v
4. Close the gap
    - Widen domain randomization for that parameter
    - Improve actuator model (collect more real data)
    - Add the failure condition to training
    |
    v
5. Retrain and retest
```

---

## Complete Example: Deployment Checklist

### Pre-Deployment

- [ ] Policy trained for 1500+ iterations with domain randomization
- [ ] Policy evaluated in simulation with 16+ envs (visual inspection)
- [ ] Policy handles push perturbations in simulation
- [ ] Policy handles varied terrain in simulation (if applicable)
- [ ] Policy exported to TorchScript (.pt) or ONNX
- [ ] Policy inference time < 2ms on target hardware
- [ ] Observation pipeline tested (dims match training)
- [ ] Action pipeline tested (joint names/order match training)
- [ ] Safety limits implemented (speed, torque, fall detection)
- [ ] Emergency stop tested
- [ ] Logging enabled

### Deployment Day

- [ ] Battery fully charged
- [ ] Testing area cleared
- [ ] Spotter present
- [ ] E-stop within reach
- [ ] Kill script ready on separate terminal
- [ ] Video camera recording
- [ ] Data logger running
- [ ] Communication test passed (no motion)
- [ ] Standing test passed (10 seconds)
- [ ] Slow walk test passed (0.2 m/s, 10 seconds)
- [ ] Progressive speed testing

### Post-Deployment

- [ ] Data downloaded and backed up
- [ ] Video reviewed
- [ ] Observation comparison: sim vs real (are distributions similar?)
- [ ] Action comparison: sim vs real (are motor responses similar?)
- [ ] Failures documented with timestamps and conditions
- [ ] Sim-to-real gap sources identified
- [ ] Next iteration plan written
