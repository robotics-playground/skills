# PlantUML Diagram Style Guide

Seven diagrams. Each has one job. Together they answer all the visual questions an implementer asks.

## The seven diagrams

| File | Purpose | Arrow budget |
|---|---|---|
| `node-map.puml` | Layer-grouped catalogue. Trunk arrows only. | ≤12 |
| `data-flow-overview.puml` | Layer-to-layer flows. No node detail. | ≤10 |
| `data-flow-sensing.puml` | Sensors → Spinal Cord → Cerebellum chain | ≤10 |
| `data-flow-control.puml` | Cortex → Cerebellum → Spinal Cord → Actuators chain | ≤10 |
| `data-flow-safety.puml` | Inside Spinal Cord — battery, override, faults | ≤10 |
| `data-flow-operator.puml` | Operator I/O + persistence (Infrastructure detail) | ≤10 |
| `data-flow-tf.puml` | TF tree — URDF, robot_state_publisher, EKF/SLAM transforms | ≤10 |

If you find yourself adding an 11th arrow to a diagram, *split* it — don't compress.

## Mandatory rules

### No `title` line

Markdown headings in `index.md` provide the title. The PlantUML `title` line just duplicates that and makes the rendered SVG taller for no value. **Remove it from every `.puml`.**

### Topic labels include frequency

Every arrow that carries a ROS 2 topic should label the topic name **and** the frequency:

```
imu --> agent : "/imu/data_raw\n<size:9>100 Hz, BestEffort</size>"
```

Or for the overview (where node-grain detail is suppressed):

```
SPINAL --> CEREBELLUM : "<size:10>micro-ROS topics\n/imu · /odom_unfiltered · /scan · /sonar · /tof</size>"
```

The frequency is what the implementer needs to size buffers, set publisher rates, and reason about latency. Including it on the arrow saves them a lookup. The `<size:9>` smaller text keeps the label compact.

For arrows that don't carry a single topic (e.g., a service call, a logical relationship), label them with the *contract* instead:

```
bte --> mo : "GetNextWaypoint\n<size:9>service</size>"
pm --> bte : "/safety/override\n<size:9>service</size>"
```

### Declare components before use

PlantUML rejects inline `--> "label" as alias` shorthand. Every component must be declared first:

```plantuml
' WRONG — fails to parse
slam --> "downstream\n(Nav2, BT)" as out

' RIGHT
component "downstream\n(Nav2, BT)" as out #DBEAFE
slam --> out : "/map · /odom"
```

For topics shown as separate nodes (TF diagram), use `queue`:

```plantuml
queue "/tf_static" as tfs #F3F4F6
queue "/tf" as tf #F3F4F6
rsp --> tfs : "<size:9>fixed frames</size>"
rsp --> tf  : "<size:9>turret_joint</size>"
```

### Colour coding (consistent across all diagrams)

| Layer | Group fill | Component fill |
|---|---|---|
| **SENSORS** | `#FEF3C7` | `#FDE68A` |
| **ACTUATORS** | `#FED7AA` | `#FB923C` |
| **SPINAL CORD** | `#FEE2E2` outer / `#FECACA` sub-groups | `#FCA5A5` |
| **CEREBELLUM** | `#DBEAFE` outer / `#BFDBFE` sub-groups | `#93C5FD` |
| **CORTEX** | `#E0E7FF` outer / `#C7D2FE` sub-groups | `#A5B4FC` |
| **INFRASTRUCTURE** | `#F3F4F6` | `#D1D5DB` |
| **OPERATOR I/O** | `#F3E8FF` | `#DDD6FE` |

Consistency across diagrams matters — readers learn that "red = safety" once, then it pays off in every subsequent diagram.

### Edge styles — keep small palette

| Style | Meaning | When |
|---|---|---|
| `-->` plain | Normal data/control flow | Default |
| `-[#DC2626,bold]->` | Hard-safety path (never overridable) | `/emergency_stop`, BMS cutoff, atomic flags |
| `-[#999,dotted]->` | Phase 2/3 future, or VOIDED requirement | A future-phase perception node feeding a BT; excluded sensors |
| `-[#666,dashed]-` | Lifecycle / TF (no data, just relationship) | `nav2_lifecycle_manager` ↔ Nav2 nodes |

Don't introduce new colours/styles. Keep the legend small.

### Header skin params (use this exact stanza)

Every `.puml` starts with:

```plantuml
@startuml <name>
!theme plain
skinparam backgroundColor #FEFEFE
skinparam shadowing false
skinparam defaultFontName Arial
skinparam defaultFontSize 11
skinparam roundCorner 8
```

For diagrams with rectangle groups (overview, node-map):

```plantuml
skinparam padding 6
skinparam component {
  BorderThickness 1
  FontSize 11
}
skinparam rectangle {
  BorderThickness 2
  FontStyle bold
}
```

### Layout direction

Use `left to right direction` for most diagrams — they read more naturally for cause-and-effect chains.

For the node-map (which is dense), use the default top-to-bottom and add hidden layout hints:

```plantuml
SENSORS    -[hidden]down- ACTUATORS
SPINAL     -[hidden]right- CEREBELLUM
CEREBELLUM -[hidden]right- CORTEX
INFRA      -[hidden]down- OPIO
```

These tell PlantUML how to position the layer rectangles relative to each other without drawing real arrows.

## Per-diagram specifics

### node-map.puml

Contains every node grouped by layer. Sub-group inside Spinal Cord (Firmware / PC-side / Hardware), inside Cerebellum (Sensor Fusion / SLAM / Nav2 / BT), inside Cortex (Mission / Perception / LLM).

Trunk arrows only — one per inter-layer transition (sensing trunk + action trunk + safety + operator + future = ~12 arrows). Detail arrows go in the data-flow files.

End with a `legend bottom` that explains the colour code.

### data-flow-overview.puml

Six layer rectangles. Eight arrows:
- 3 sensing-direction (Sensors→Spinal→Cerebellum→Cortex)
- 3 control-direction (Cortex→Cerebellum→Spinal→Actuators)
- 1 bidirectional safety (Spinal⇆Cerebellum, red)
- 1 infra cross-cuts (clustered: `INFRA <--> CORTEX`, `INFRA <-- CEREBELLUM`, `INFRA <-- SPINAL`)

Add 2 `note` blocks: one anchored to SPINAL explaining "Two physical homes…", one anchored to INFRA explaining "Carries every cross-layer message…".

### data-flow-sensing.puml

Show: sensor components on the left, micro-ROS agent in the middle, EKF + SLAM on the right, with a `downstream (Nav2, BT)` component as the visual sink so the chain has a clear endpoint.

### data-flow-control.puml

Show: mission orchestrator on the left, BT executor, Nav2 stack, collision monitor, agent, firmware, motors on the right. Strict left-to-right.

### data-flow-safety.puml

The most arrow-heavy of the details (~9 arrows is normal). Includes:
- Battery ADC → BMS (red, hard cutoff)
- Battery ADC → agent → power_manager
- power_manager → bt_executor (red, /emergency_stop)
- power_manager → nav2_controller (red, /cmd_vel zero-vel)
- power_manager ↔ threshold_manager (registry)
- agent → health_node (liveness)
- health_node → bt_executor (sensor faults)

Add a `note right of pm` block enumerating Hard vs Soft thresholds.

### data-flow-operator.puml

Foxglove + gamepad on the left, mission/BT/safety nodes in the middle, persistence (mission events, override audit, saved maps) on the right.

### data-flow-tf.puml

URDF + robot_state_publisher + dynamic-joint sources flow to `/tf_static` and `/tf` queues. EKF and SLAM contribute additional transforms.

End with a `note bottom` showing the frame chain in REP-103/105 form:

```
note bottom
**Frame chain (REP-103/105):**
map → odom → base_link →
  imu_link · lidar_link · sonar_*_link
  turret_base_link → turret_link →
    tof_turret_link · sonar_turret_link
end note
```

## Render and embed

After authoring all `.puml` files:

```bash
cd <designs>/diagrams
plantuml -tsvg *.puml
```

This produces one `.svg` per `.puml` file. Commit both — the SVGs are the rendered preview, the PUML files are the editable source.

In `index.md`, embed each SVG with `<img>` (not markdown `![]()`) because:
- GitHub sometimes blocks inline SVG in markdown image syntax for security reasons.
- The diagrams are wide (some > 3000 px); `width="100%"` lets them scale to the preview pane.

```html
<img src="data-flow-overview.svg" alt="Data-flow overview" width="100%">
```

Do this under each diagram's heading in `index.md`.

## Common mistakes to avoid

- **Inline `"label" as alias`** in arrow targets — declare components first.
- **Redundant `title` lines** — the markdown heading is the title.
- **Missing frequencies** on topic arrows — implementer needs them.
- **Mixing colours by node** rather than by layer — pick layer-based palette and stick.
- **Cramming everything into one diagram** — split. Each canvas should tell one story.
- **Using `![]()` markdown image syntax** in `index.md` — use `<img width="100%">`.
- **Forgetting `legend bottom`** in `node-map.puml` — readers need the colour key.
