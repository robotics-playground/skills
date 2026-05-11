# <node_name>

> **Template instructions for the skill (delete this blockquote in the actual file):**
>
> Fill in every field. If a section doesn't apply (e.g., a sensor driver with no services), keep the heading and write "None" — don't drop the heading silently. For schemas that have already been shown in an earlier node file, use a "See [`<other_node>.md`](<other_node>.md) §Published Topics for the full schema" reference instead of pasting the same block twice.
>
> **Layer + Process Group naming — pick the right vocabulary for THIS project:**
>
> - **If** the project's architecture adopts the **biology metaphor** (Spinal Cord / Cerebellum / Cortex), use those names.
> - **If** the architecture **rejects** the biology metaphor (look for a decision like "no biology metaphor" or a flat-package layout), **never** use Spinal Cord / Cerebellum / Cortex anywhere in the node files. Instead use the project's actual vocabulary — typically functional names like `safety`, `navigation`, `perception`, `mission`, `drivers`, or systemd unit / Compose service names like `<project>_mission`, `<project>_drivers`. The Process Group field should name the systemd unit or Compose service, not a layer abstraction.
>
> This is the single most common drift point — the templates show biology-metaphor names as one common pattern, but a project that rejected them must not see those words in its design tree. Grep your output for `Spinal Cord`, `Cerebellum`, `Cortex` after authoring; if the architecture rejected them and a hit appears, that's a leak.

## Identity

| Field | Value |
|-------|-------|
| **Package** | `<package_name>` |
| **Module** | `<package_name>.<node_module>` |
| **Process Group** | <name the actual systemd unit or Compose service — e.g. `<project>_mission`, `<project>_drivers`. If the project adopts the biology metaphor, use Spinal Cord / Cerebellum / Cortex / Nav2 / Visualisation / Teleop / Firmware> |
| **Layer** | <Use the project's vocabulary. Biology metaphor: Reactive (Spinal Cord) / Sequencing (Cerebellum) / Deliberative (Cortex) / Senses / Infrastructure. Flat layout: Drivers / Safety / Navigation / Mission / Perception / Operator I/O / Infrastructure — whatever the architecture's `package-structure.md` §1.1 names.> |
| **Node Type** | `LifecycleNode` / Standalone / micro-ROS XRCE-DDS client |
| **Logic Class** | `<ClassName>` in `<role>.py` (zero ROS imports) — or "None" if trivial bridge |

---

## Role

Two-to-four sentences. What does this node do, why does it exist, what makes it non-trivial.

---

## Published Topics

| Topic | Type | QoS | Hz | Description |
|-------|------|-----|----|-------------|
| `/topic/name` | `pkg/MsgType` | Reliable, Keep Last 1 | 10 | Description |

### `pkg/MsgType` schema
\`\`\`
pkg/MsgType
field1
field2
...
\`\`\`

**Example `/topic/name`:**
\`\`\`yaml
field1: value
field2: value
\`\`\`

(Repeat the schema + example block for each new type. For types already shown in another file, reference that file. For standard ROS 2 types like `sensor_msgs/Imu` or `std_msgs/Bool`, use the type as-is — do not paste the schema; just link to `references/standard-ros2-types.md`.)

---

## Subscribed Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/incoming/topic` | `pkg/MsgType` | QoS | What this node does with it |

---

## Services Served

### `/service/name`
One-line description.

**Type:** `<package>/srv/<TypeName>`

**Request:**
\`\`\`
<schema>
\`\`\`

**Response:**
\`\`\`
<schema>
\`\`\`

**Rejection reasons** (if applicable):
- `"reason_a"` — when this fires
- `"reason_b"` — when this fires

(Repeat per service.)

## Services Called

| Service | Provider | Why |
|---------|----------|-----|
| `/other/service` | `other_node` | What this call accomplishes |

---

## Actions Served / Called

(Same shape as services. If none, write "None.")

---

## Parameters

All in `<config_path>.yaml` under `<node_name>.ros__parameters`.

| Parameter | Type | Default | Soft/Hard | Override Range | Description |
|-----------|------|---------|-----------|----------------|-------------|
| `param_name` | float | 0.0 | Soft | 0.0–1.0 | What this controls |

(Drop the Soft/Hard and Override Range columns if the project doesn't use the override mechanism.)

---

## Lifecycle Behavior (LifecycleNode only)

| Transition | Behavior |
|-----------|---------|
| `on_configure` | Load parameters; instantiate `<LogicClass>` logic |
| `on_activate` | Subscribe to topics; create publishers; expose services; start timers |
| `on_deactivate` | Stop timers; destroy publishers, subscriptions, services |
| `on_cleanup` | Release all resources |
| `on_shutdown` | Final flush (audit logs, files) |

---

## Dependencies

| Depends On | Why |
|------------|-----|
| `<package>` | Custom message types |
| `<other_node>` | Source of subscribed topic |

| Consumed By | Why |
|-------------|-----|
| `<consumer_node>` | What it does with this node's output |

---

## Related Stories (only if epics.md exists)

| Story | What it delivers |
|-------|------------------|
| Story X.Y | This node skeleton + initial implementation |
| Story X.Z | Tuning / Phase 1.5 promotion |
