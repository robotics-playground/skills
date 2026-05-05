# Node Spec Authoring Rules

## File structure (every node)

```
# <node_name>

## Identity
| Package · Module · Process Group · Layer · Node Type · Logic Class |

---

## Published Topics
(table + per-type schema + per-type example)

---

## Subscribed Topics
(table + reference earlier schema if already shown)

---

## Services Served
(per-service: type, request, response, rejection reasons, examples)

## Services Called
(brief table → which provider)

## Actions Served / Called
(same shape as services if applicable)

---

## Parameters
(table — type, default, soft/hard, override range, description)

---

## Lifecycle Behavior
(per-transition table for LifecycleNode)

---

## Dependencies
(Depends On + Consumed By tables)

---

## Related Stories
(Story X.Y → what it delivers — only if epics.md exists)
```

## Schema inlining rule

A custom message or standard ROS message schema is shown **once**, the first time it appears in a node file. Subsequent uses just reference the earlier file. Example:

> ### nav_msgs/Odometry schema
> See [`ekf_node.md`](ekf_node.md) §Published Topics for the full schema. EKF-only `/odom` populates `pose.position.x/y` … (this node's specifics).

This keeps individual files readable without exploding the total page count when 8 nodes all subscribe to `/odom`.

## Examples — at least one per type

Every published topic gets at least one YAML-style example (proper indentation, real values, not `<value>` placeholders). Use realistic numbers — e.g., for an IMU at rest, `quat ≈ (0, 0, 0.087, 0.996)` not `(0, 0, 0, 0)`.

Two examples are good for state-machine messages (one in a "normal" state, one in a "triggered" state) — see how `power_manager_node.md` shows both NORMAL and LOW battery states.

## Off-the-shelf nodes — full spec, not just a stub

`nav2_*`, `slam_toolbox`, `robot_localization`, `foxglove_bridge`, `joy_node` are not shipped as part of the project's package, but they belong in `nodes/` because:

- They publish topics other nodes subscribe to.
- The project's launch files configure them (parameter YAML).
- An implementer needs to know which version, which plugin, which costmap layer.

For each, document:

- **Identity** — `Package` is the upstream package name (e.g., `nav2_planner`), `Process Group` is the Compose profile or systemd unit, `Node Type` is `LifecycleNode` if applicable.
- **Topics + services** — only the ones this project uses (don't enumerate every Nav2 topic).
- **Parameters (key subset)** — show the YAML stanza the project actually mounts. Don't paste the whole `nav2_params.yaml` from the upstream example.
- **Dependencies** — what publishes `/scan` / `/odom` / `/map` for this node.

## Firmware tier — single conceptual node

Any microcontroller (ESP32, STM32, Pico W, Arduino) that connects via micro-ROS shows up in the ROS 2 graph as one XRCE-DDS participant. Document it as one node file (e.g., `<project>_firmware.md`) with these specifics:

- **Build flags** — `RMW_UXRCE_MAX_PUBLISHERS`, `RMW_UXRCE_MAX_SUBSCRIPTIONS`, `RMW_UXRCE_MAX_NODES` exact values
- **All publishers** — cover the full rate table
- **All subscribers** — `/cmd_vel`, `/servo/*`, etc.
- **FreeRTOS task layout** — table with task name, core pin, priority, stack, rate, responsibility (matches the architecture's Firmware Architecture section)
- **Layer 1 safety table** — every check, threshold, soft/hard, override path
- **Reconnect FSM** — explicit reconnect strategy (recreate publishers from scratch on each successful reconnect, working around `rmw_microxrcedds#241`)
- **Parameters** — compile-time `#define`s in `config.h` and `secrets.h`. Note these are **not** ROS 2 parameters (no parameter server inside micro-ROS).

## Phase 2 / Phase 3 placeholders

Nodes that won't ship until a later phase still get a node file, but with an **opening note** that flags placeholder status:

```markdown
# tank_perception_node

> **Phase 2 placeholder.** This file describes the *intended* node for Phase 2 (target 2026-06-30, …). FRs are authored at Phase 2 entry. Phase 1 ships the system **without** this node.

## Identity
| (best-effort fields based on the architecture) |

...
```

The placeholder file should still have meaningful content — best-known FRs, expected topic shapes, parameter sketch — so the implementer at Phase 2 entry has something to extend rather than starting blank.

## Process group field — what to put

The Process Group field names the **OS process / systemd unit / Compose service** the node lives in. It is **not** a layer abstraction. Match the architecture's process orchestration decision exactly.

**If the architecture adopts the biology metaphor**, you may use these layer names as process-group identifiers:

- **Firmware** — for ESP32/STM32 etc. tiers
- **Safety / Spinal Cord** — fault-isolated PC process for safety nodes
- **Cerebellum** — sensor fusion + Nav2 + BT
- **Cortex** — mission, perception, LLM
- **Nav2** — when the node is a Nav2 stack member (separate Compose service)
- **Visualisation / Infrastructure** — foxglove_bridge, robot_state_publisher
- **Teleop** — joy + teleop_twist_joy (Compose `gamepad` profile)

**If the architecture rejects the biology metaphor** (flat package layout, decision like "no biology metaphor / no Spinal Cord-Cerebellum-Cortex"), **never** put `Spinal Cord`, `Cerebellum`, or `Cortex` in the Process Group or Layer fields. Use the project's actual systemd unit / Compose service names — usually identical to the package name minus `_node`. Examples:

- `<project>_mission` (systemd service) — for `mission_orchestrator_node`
- `<project>_drivers` (systemd service) — for the sensor / actuator driver nodes
- `<project>_perception` (Compose service) — for the camera + ML inference node
- `nav2` — for any Nav2 stack member (this name is fine in either vocabulary because it names a stack, not a metaphor)
- `infrastructure` — for `foxglove_bridge`, `robot_state_publisher` etc.

The same rule applies to the Layer field. Use functional layer names from the architecture's package-structure §1.1 (Drivers / Safety / Navigation / Mission / Perception / Operator I/O / Infrastructure) — never the rejected metaphor.

**Quick self-check:** after writing each node file, glance at the Identity table. If the project rejects the metaphor and you see `Spinal Cord`, `Cerebellum`, or `Cortex` anywhere, replace them now. Catching it at one node saves grep-and-replace across the tree later.

## Logic class field

For every custom Python/C++ node, name the *pure logic class* in the Identity table:

- **Logic Class:** `PowerManager` in `power_manager.py` (zero ROS imports)

This enforces the pure-logic separation rule (see `package-structure-rules.md` §1.2) at the design stage. If the node has no separable logic (it's a trivial bridge), say so:

- **Logic Class:** None — this node is a thin XRCE-DDS bridge with no business logic.

## Lifecycle behavior table — be specific

```markdown
## Lifecycle Behavior

| Transition | Behavior |
|-----------|---------|
| `on_configure` | Load parameters; instantiate `PowerManager` logic; validate threshold classifications |
| `on_activate` | Start 1 Hz publisher timer; subscribe to `/odom`; expose `/power/range_check` service |
| `on_deactivate` | Stop publisher timer; destroy subscriptions and service |
| `on_cleanup` | Release all resources |
| `on_shutdown` | Flush pending data |
```

Each row says what concrete things happen. Don't write "standard lifecycle" — that doesn't tell the implementer anything.

## Dependencies — split into two tables

```markdown
| Depends On | Why |
|---|---|
| `<project>_interfaces` | Custom message types |
| `<project>_firmware` | Source of `/battery_voltage` |

| Consumed By | Why |
|---|---|
| `bt_executor_node` | `CheckBattery` BT condition reads `/battery/state` |
| `mission_orchestrator_node` | `/abort_mission` triggers ReturnToEntrance |
| `foxglove_bridge` | Battery gauge panel |
```

This bidirectional view is invaluable for review — the implementer of `power_manager_node` immediately sees who breaks if the topic schema changes.

## When in doubt — pattern-match against an existing file

If you've already authored several node files in this project, find the closest match in `designs/nodes/` and use it as a structural template. Match the table column order, the schema-inlining style, and the dependency-table shape. Consistency across files is more valuable than per-file optimisation.
