# RL Training

## PPO Explained for Developers

PPO (Proximal Policy Optimization) is the workhorse algorithm for robot locomotion RL. Here
is how it works, explained without heavy math.

### The Intuition

Imagine you are training a new employee (the policy) to handle customer support:

1. The employee handles 100 tickets (rollout / data collection)
2. You review the outcomes and score each response (compute rewards and advantages)
3. You give feedback: "do more of what worked, less of what didn't" (policy update)
4. But you limit how much the employee changes at once (clipping -- the "Proximal" part)
5. Repeat

The "proximal" constraint is key: if the employee changes their approach too drastically
after one round of feedback, they might overfit to that batch and get worse overall. PPO
prevents this by limiting how much the policy can change per update.

### PPO Training Loop

```
for iteration in range(max_iterations):
    # 1. COLLECT DATA
    #    Run policy in all parallel environments for N steps
    #    Store: observations, actions, rewards, dones
    rollout_data = collect_rollout(policy, env, num_steps=24)

    # 2. COMPUTE ADVANTAGES
    #    "How much better was this action than average?"
    #    Uses GAE (Generalized Advantage Estimation)
    advantages = compute_gae(rollout_data, value_network)

    # 3. UPDATE POLICY (multiple mini-batch passes)
    for epoch in range(num_epochs):
        for mini_batch in rollout_data.shuffle_and_split():
            # Update actor (policy) network
            # With PPO clipping: don't change too much
            actor_loss = ppo_clip_loss(mini_batch, advantages, clip_range=0.2)

            # Update critic (value) network
            # "How good is this state?" -- used for advantage computation
            critic_loss = value_loss(mini_batch)

            # Entropy bonus: encourage exploration
            entropy = entropy_bonus(mini_batch)

            total_loss = actor_loss + value_coeff * critic_loss - entropy_coeff * entropy
            optimizer.step(total_loss)

    # 4. LOG METRICS
    log("mean_reward", rollout_data.mean_reward)
    log("mean_episode_length", rollout_data.mean_episode_length)
```

### Key PPO Components

| Component | What It Is |
|---|---|
| **Actor (Policy) network** | Neural network that maps observations to actions |
| **Critic (Value) network** | Neural network that estimates "how good is this state" |
| **Rollout buffer** | Stores trajectories from environment interaction |
| **Advantage** | "Was this action better than expected?" -- measures how much better an action was compared to the average |
| **Clipping** | Limits policy change per update to prevent destructive large updates |

---

## Hyperparameters That Matter

These are the knobs you'll tune. Listed in order of importance for locomotion tasks.

### Critical Parameters

| Parameter | Default | Range to Try | What It Does |
|---|---|---|---|
| **learning_rate** | 1e-3 | 1e-4 to 3e-3 | How fast the policy updates. Too high = unstable, too low = slow. |
| **num_envs** | 4096 | 512-16384 | Number of parallel environments. More = faster, more stable training. |
| **clip_range** | 0.2 | 0.1-0.3 | How much the policy can change per update. Lower = more conservative. |
| **num_epochs** | 5 | 3-8 | Number of passes over the rollout data per update. |
| **mini_batch_size** | num_envs * num_steps / 4 | Varies | Size of each mini-batch. Larger = more stable gradients. |
| **gamma** (discount) | 0.99 | 0.95-0.999 | How much future rewards matter. Higher = more far-sighted. |
| **GAE lambda** | 0.95 | 0.9-0.99 | Bias-variance tradeoff in advantage estimation. |
| **entropy_coeff** | 0.01 | 0.0-0.05 | Encourages exploration. Reduce over training for convergence. |

### Learning Rate Schedule

A constant learning rate often works, but a schedule can help:

```python
# Linear decay: start high, reduce to zero
# Helps convergence in later stages
learning_rate_schedule = "adaptive"  # or "fixed"

# Adaptive: if KL divergence is too high, reduce LR
# If KL divergence is too low, increase LR
# This automatically adjusts to training dynamics
desired_kl = 0.01
```

### Network Architecture

For locomotion, simple MLPs (Multi-Layer Perceptrons) work well:

```python
# Actor and Critic networks
# Typical architecture for locomotion:
policy_network = {
    "type": "MLP",
    "hidden_dims": [256, 128, 64],    # 3 hidden layers
    "activation": "elu",               # ELU activation (works better than ReLU for RL)
    "init_noise_std": 1.0,             # Initial exploration noise
}

value_network = {
    "type": "MLP",
    "hidden_dims": [256, 128, 64],
    "activation": "elu",
}
```

**Don't over-complicate the network.** For proprioceptive locomotion (no cameras),
3-layer MLPs with 128-512 neurons per layer are sufficient. Larger networks don't help
and train slower.

---

## Training Loop Architecture

### With rsl_rl (Recommended for Isaac Lab)

`rsl_rl` is the RL library from Robotic Systems Lab (ETH Zurich). It's the default for
Isaac Lab locomotion tasks.

```python
"""Training script using rsl_rl with Isaac Lab."""

from omni.isaac.lab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import gymnasium as gym
from omni.isaac.lab_tasks.utils import get_checkpoint_path, parse_env_cfg
from rsl_rl.runners import OnPolicyRunner

# Create environment
env_cfg = parse_env_cfg("MyLab-G1-Walking-v0", num_envs=4096)
env = gym.make("MyLab-G1-Walking-v0", cfg=env_cfg)

# Training configuration
train_cfg = {
    "runner_type": "OnPolicyRunner",
    "policy": {
        "class_name": "ActorCritic",
        "init_noise_std": 1.0,
        "actor_hidden_dims": [256, 128, 64],
        "critic_hidden_dims": [256, 128, 64],
        "activation": "elu",
    },
    "algorithm": {
        "class_name": "PPO",
        "value_loss_coef": 1.0,
        "use_clipped_value_loss": True,
        "clip_param": 0.2,
        "entropy_coef": 0.01,
        "num_learning_epochs": 5,
        "num_mini_batches": 4,
        "learning_rate": 1e-3,
        "schedule": "adaptive",
        "desired_kl": 0.01,
        "gamma": 0.99,
        "lam": 0.95,
        "max_grad_norm": 1.0,
    },
    "num_steps_per_env": 24,       # Steps per rollout
    "max_iterations": 1500,         # Total training iterations
    "save_interval": 100,           # Save checkpoint every 100 iterations
    "log_interval": 10,             # Log metrics every 10 iterations
}

# Create runner and train
runner = OnPolicyRunner(env, train_cfg, log_dir="logs/g1_walking")
runner.learn(num_learning_iterations=train_cfg["max_iterations"])

env.close()
simulation_app.close()
```

### With rl_games (Alternative)

`rl_games` is another RL library with good Isaac Lab integration. It's faster for some tasks
but has more complex configuration:

```python
"""Training with rl_games."""

from omni.isaac.lab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

from omni.isaac.lab_tasks.utils.wrappers.rl_games import RlGamesGpuEnv, RlGamesVecEnvWrapper
from rl_games.torch_runner import Runner

# rl_games uses YAML config files
# See Isaac Lab examples for full config format
runner = Runner()
runner.load({
    "params": {
        "algo": {"name": "a2c_continuous"},
        "network": {
            "name": "actor_critic",
            "mlp": {"units": [256, 128, 64], "activation": "elu"},
        },
        "config": {
            "learning_rate": 1e-3,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_actors": 4096,
        },
    }
})
runner.run({"train": True})
```

### rsl_rl vs rl_games

| Feature | rsl_rl | rl_games |
|---|---|---|
| Simplicity | Simpler, easier to modify | More complex, more features |
| Speed | Good | Often faster (optimized CUDA kernels) |
| Algorithms | PPO | PPO, SAC, A2C, and more |
| Community | ETH Zurich robotics labs | NVIDIA, broader RL community |
| Debugging | Easier to step through | Harder to debug |
| **Recommendation** | Start here | Switch if you need speed or SAC |

---

## Monitoring Training

### TensorBoard

```bash
# Start TensorBoard
tensorboard --logdir logs/ --bind_all

# Open in browser: http://localhost:6006
```

### Key Metrics to Watch

| Metric | What It Means | Healthy Sign | Problem Sign |
|---|---|---|---|
| **Mean reward** | Average episode reward | Increasing over time | Flat, decreasing, or oscillating wildly |
| **Mean episode length** | How long before episode ends | Increasing (robot stays alive longer) | Stuck at minimum (falls immediately) |
| **Policy loss** | Actor network loss | Small, stable | Large spikes, NaN |
| **Value loss** | Critic network loss | Decreasing | Increasing or not decreasing |
| **KL divergence** | How much policy changed | ~0.01 (with adaptive LR) | >>0.05 (updates too aggressive) |
| **Entropy** | Policy randomness | Gradually decreasing | Zero (collapsed, no exploration) or flat |
| **Learning rate** | Current LR (if adaptive) | Stable or slowly decreasing | Oscillating rapidly |

### Reading Training Curves

```
GOOD training curve:            BAD training curve:

Reward                          Reward
^                               ^
|         ___________           |
|        /                      |     /\  /\
|       /                       |    /  \/  \____
|      /                        |   /
|     /                         |  /
|    /                          | /
|___/                           |/
+-------------------> Steps     +-------------------> Steps
  Steady increase,                Oscillating, never
  then plateau                    converges
```

---

## Curriculum Learning

Curriculum learning starts with easy tasks and progressively increases difficulty. This helps
the agent learn basic skills before tackling harder challenges.

### Example: Terrain Curriculum

```python
# Start on flat ground, gradually add obstacles

@configclass
class CurriculumCfg:
    """Curriculum configuration."""

    terrain_levels = CurriculumTermCfg(
        func=mdp.terrain_levels_vel,
        params={
            "min_level": 0,      # Start: flat ground
            "max_level": 9,      # End: rough terrain with stairs
            # Move to next level when mean reward > threshold
            "reward_threshold": 0.7,
        },
    )
```

### Example: Command Curriculum

```python
# Start with slow walking commands, increase speed as agent improves

@configclass
class CommandCfg:
    """Command curriculum."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        # Start slow, increase over training
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.5, 1.0),   # Start: up to 1.0 m/s
            lin_vel_y=(-0.3, 0.3),
            ang_vel_z=(-0.5, 0.5),
        ),
    )

# Later in training, increase to:
# lin_vel_x=(-1.0, 2.0)  # Faster walking
```

### Curriculum Tips

1. **Start flat, add terrain.** Get the robot walking on flat ground first. Then add slopes,
   then stairs, then rough terrain.

2. **Start slow, increase speed.** Command 0.5 m/s first. Once that works, push to 1.0, 1.5.

3. **Start with no perturbations, add pushes.** Once walking is stable, add random external
   forces to train push recovery.

4. **Automate curriculum progression.** Isaac Lab supports automatic level advancement based
   on reward thresholds.

---

## Checkpointing and Resuming

```python
# Checkpoints are saved automatically based on save_interval
# Default location: logs/{experiment_name}/model_{iteration}.pt

# Resume from checkpoint
runner = OnPolicyRunner(env, train_cfg, log_dir="logs/g1_walking")
runner.load("logs/g1_walking/model_500.pt")
runner.learn(num_learning_iterations=1000)  # Continue for 1000 more iterations

# Best practice: always save the final model
# runner automatically saves on KeyboardInterrupt (Ctrl+C)
```

### Checkpoint Contents

A checkpoint contains:
- Actor network weights
- Critic network weights
- Optimizer state
- Training iteration number
- Mean reward at checkpoint time

---

## When Training Is "Done"

Training is done when:

1. **Reward has plateaued** for 200+ iterations (no more improvement)
2. **Episode length is near maximum** (robot survives full episodes)
3. **Visual inspection looks good** (run play.py and watch the robot)

```bash
# Evaluate trained policy visually
python play.py --task MyLab-G1-Walking-v0 --num_envs 16 --checkpoint logs/best_model.pt
```

**Don't stop too early.** Locomotion policies often plateau, then suddenly improve again.
A typical training run for robust humanoid walking is **1000-3000 iterations** with 4096 envs.

---

## Common Training Failures

### Reward Doesn't Increase

| Possible Cause | Diagnostic | Fix |
|---|---|---|
| Reward function is broken | Print raw reward components; check for NaN/inf | Fix reward computation; add clipping |
| Observations are wrong | Print observation tensor; verify values are reasonable | Check normalization; ensure all obs are being populated |
| Actions don't affect the robot | Print actions; verify joints move in sim | Check action space config; verify joint names match |
| Learning rate too low | Training progresses but extremely slowly | Increase LR by 2-5x |
| Reward is too sparse | Agent never accidentally gets reward | Add dense shaping rewards |

### Reward Spikes Then Crashes

| Possible Cause | Diagnostic | Fix |
|---|---|---|
| Learning rate too high | KL divergence spikes during crash | Reduce LR; use adaptive schedule |
| Clip range too high | Policy changes too much per update | Reduce clip_param from 0.2 to 0.1 |
| Bad advantage estimation | Value loss is very high | Increase num_epochs; check gamma/lambda |

### Agent Exploits Reward (Reward Hacking)

The agent WILL find loopholes. Common exploits:

| Exploit | What Happens | Fix |
|---|---|---|
| Vibrating | Joints oscillate rapidly to maximize velocity | Add action rate penalty; add joint acceleration penalty |
| Knee walking | Falls to knees, scoots forward | Add knee contact penalty; add base height reward |
| Spinning | Rotates to game velocity reward | Use directional velocity (project velocity onto heading) |
| Leg flailing | Waves legs for angular momentum | Add joint limit penalty; add symmetry reward |

---

## Complete Example: PPO Training Config

```python
"""PPO config for Unitree G1 walking with rsl_rl."""

from rsl_rl.algorithms import PPO
from rsl_rl.modules import ActorCritic

@configclass
class G1WalkingPPOCfg:
    """Training config."""

    seed: int = 42
    runner_type: str = "OnPolicyRunner"
    max_iterations: int = 1500

    # Save and log intervals
    save_interval: int = 100
    log_interval: int = 10
    experiment_name: str = "g1_walking"

    # Policy network
    policy = {
        "class_name": "ActorCritic",
        "init_noise_std": 1.0,
        "actor_hidden_dims": [256, 128, 64],
        "critic_hidden_dims": [256, 128, 64],
        "activation": "elu",
    }

    # PPO algorithm
    algorithm = {
        "class_name": "PPO",
        # Core PPO parameters
        "value_loss_coef": 1.0,
        "use_clipped_value_loss": True,
        "clip_param": 0.2,
        "entropy_coef": 0.01,

        # Optimization
        "num_learning_epochs": 5,
        "num_mini_batches": 4,
        "learning_rate": 1e-3,
        "schedule": "adaptive",
        "desired_kl": 0.01,

        # Advantage estimation
        "gamma": 0.99,
        "lam": 0.95,

        # Gradient clipping (prevents exploding gradients)
        "max_grad_norm": 1.0,
    }

    # Rollout
    num_steps_per_env: int = 24  # Steps collected per env per iteration
    # Total samples per iteration = num_envs * num_steps_per_env
    # With 4096 envs: 4096 * 24 = 98,304 samples per iteration

    # Normalization
    empirical_normalization: bool = False  # Normalize observations by running mean/std
    # Set to True if observation magnitudes vary widely
```

### Training Command

```bash
# Train with rsl_rl
cd IsaacLab
python source/standalone/workflows/rsl_rl/train.py \
    --task MyLab-G1-Walking-v0 \
    --num_envs 4096 \
    --headless \
    --max_iterations 1500

# Monitor with TensorBoard
tensorboard --logdir logs/rsl_rl/g1_walking/
```

### Evaluation Command

```bash
# Play back trained policy (with GUI)
python source/standalone/workflows/rsl_rl/play.py \
    --task MyLab-G1-Walking-v0 \
    --num_envs 16 \
    --checkpoint logs/rsl_rl/g1_walking/model_1500.pt
```
