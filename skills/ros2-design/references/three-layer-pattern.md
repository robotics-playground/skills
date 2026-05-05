# Three-Layer Architecture Pattern (Spinal Cord / Cerebellum / Cortex)

## When to apply

Adopt this pattern **only** if the project's architecture document already names the layers (or uses the biology metaphor). It maps cleanly onto robotics projects that have:

- A real-time safety tier (must respond in <200 ms, must work without WiFi/network)
- A sensor-fusion + path-execution tier (Nav2, EKF, SLAM)
- A high-level reasoning tier (mission planning, perception, optional LLM)

If the architecture doesn't structure things this way, **don't impose it**. Use whatever vocabulary the architecture uses ("safety", "navigation", "mission", "perception" — fine as flat layers).

## The three layers

| Layer | Biology analogy | What it does | Latency budget | Survives WiFi loss? |
|---|---|---|---|---|
| **Spinal Cord** (Reactive) | Spinal reflexes — automatic, hardware-grade safety | Atomic e-stop, motor PID at 50 Hz, battery emergency cutoff, motor stall watchdog. PC-side: battery-policy node, threshold registry, sensor-liveness watchdog. | < 50 ms firmware, < 500 ms PC | **Yes** (firmware atomic + hardware BMS) |
| **Cerebellum** (Sequencing) | Cerebellum — coordinates learned movement | Sensor fusion (EKF), localisation (SLAM/AMCL), path planning + execution (Nav2), mission orchestration (BT). | 100 ms tick | No (depends on agent connection) |
| **Cortex** (Deliberative) | Prefrontal cortex — planning, reasoning | Mission YAML loading, defect aggregation, future LLM. | Eventual (seconds) | No |

## Concrete mappings

### Spinal Cord lives in two homes

The Spinal Cord layer is the only one that spans hardware — *atomic* safety must run in firmware (where it can't be killed by an OS), but *policy* (battery return thresholds, sensor timeouts, override audit logs) can live on the PC side because the firmware atomic layer is the last-line defence.

- **Firmware Spinal Cord:** atomic flag checked every motor-PID tick. Hard checks: cmd_vel timeout, battery emergency, motor stall, e-stop input.
- **PC-side Spinal Cord:** typically a `<project>_safety` package with `power_manager_node`, `threshold_manager_node`, `health_node`. Subscribes to firmware-published topics, applies soft-threshold policy, emits `/abort_mission` and `/emergency_stop` mirrors, exposes `/safety/override` service.
- **Hardware Spinal Cord:** BMS at the battery, hardware e-stop button. Independent of all software.

### Cerebellum is mostly off-the-shelf

- `robot_localization::ekf_node` — sensor fusion
- `slam_toolbox` — mapping
- `nav2_*` stack — planning + execution + collision monitor
- `<project>_bt_nodes` — the only custom Cerebellum package, hosting BT.CPP custom nodes + the BT executor

### Cortex is project-specific

- `<project>_mission` — YAML loader, mission queue, mission services
- `<project>_perception` (often Phase 2) — camera + ML inference
- `<project>_llm_behavior` (often Phase 3) — outbound LLM calls

## Override mechanism (only if the architecture defines it)

If the architecture has classified thresholds as Soft vs Hard:

- **Hard thresholds** can NEVER be overridden by software. Examples: emergency stop, battery emergency cutoff, BMS hardware cutoff. Hardcoded in firmware or hardware.
- **Soft thresholds** can be tuned via a service call (typically `/safety/override`) within bounded ranges. Examples: battery return level, max speed, sensor timeout, obstacle stop distance, cmd_vel timeout, motor stall window.

Every override request — accepted, rejected, or auto-reverted on duration expiry — must be **audit-logged** (JSONL file, SQLite, or structured ROS log). The audit trail is non-negotiable.

If the architecture doesn't classify thresholds this way, don't fabricate the mechanism. Ask the user whether they want to add it before writing the spec.

## Process boundaries (the why)

Each layer should run as its **own OS process** (own systemd unit or its own Compose service) so a crash in one layer doesn't take the others down:

- Spinal Cord process must outlive Cerebellum (BT) and Cortex (mission) crashes.
- Cerebellum process must outlive Cortex crashes.

Within a process, multiple nodes can be composed into the same executor (zero-copy intra-process). Use this for nodes that share high-frequency data (e.g., camera_driver + perception in the same process).

## When to push back on the user

- They ask for a single `<project>_main` package with everything in it. Push back: process isolation matters for safety.
- They ask to put the firmware atomic check on the PC. Push back: WiFi can fail; safety can't depend on the network.
- They ask to make `emergency_stop` a soft threshold. Push back hard: that's the whole point of Hard.
