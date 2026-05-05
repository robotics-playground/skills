---
type: package-structure
project: <project>
created: <YYYY-MM-DD>
status: active
related:
  - <path-to-architecture.md>
  - <path-to-designs>/nodes/index.md
  - <path-to-designs>/interfaces/index.md
  - <path-to-designs>/diagrams/data-flow-overview.puml
---

# ROS 2 Package Structure — <Project>

Authoritative package design for the **`<workspace_path>`** colcon workspace. Every package in this document maps to a specific layer of the architecture and to a specific subset of the FRs from the [PRD](../prd.md).

> **Scope.** This document covers the **Linux/PC-side ROS 2 workspace** only. The microcontroller firmware is a <PlatformIO/IDF> project, *not* a colcon package, and is documented separately in [`architecture.md`](../architecture.md) §Firmware Architecture.

---

## 1. Design Principles

### 1.1 One layer ↔ one package

Each architectural layer corresponds to exactly one (or two) ROS 2 packages so process boundaries match layer boundaries.

| Layer | Package(s) | Process group |
|-------|-----------|---------------|
| <Layer 1> | `<project>_<pkg>` | `<process_group>` |
| <Layer 2> | `<project>_<pkg>` | `<process_group>` |
| ... |

### 1.2 Separate ROS code from pure logic — non-negotiable

Every node has a corresponding *logic class* with **zero ROS imports**. The node wraps the logic; the logic is testable on the host with `pytest`, no `rclpy` involved.

\`\`\`python
# GOOD — node thin, logic pure
class <ClassName>Node(LifecycleNode):
    def __init__(self):
        self.logic = <ClassName>(...)   # zero ROS imports
    def callback(self, msg):
        action = self.logic.evaluate(msg.data)   # pure
        if action.kind == 'do_thing':
            self._publisher.publish(...)         # ROS

# BAD — logic mixed with ROS, untestable on host
class <ClassName>Node(LifecycleNode):
    def callback(self, msg):
        if msg.data < 11.4:                # hardcoded
            self.get_logger().warn("low")  # ROS noise in pure logic
\`\`\`

### 1.3 Single source of truth for configuration

All ROS 2 node parameters live in **`<config_path>/*.yaml`** (mounted read-only into containers). No hardcoded thresholds in code.

### 1.4 Build types

| Package category | Build type | Why |
|---|---|---|
| `<project>_interfaces` | `ament_cmake` | rosidl_generate_interfaces requires CMake |
| `<project>_bt_nodes` | `ament_cmake` (C++) | BT.CPP is C++ native (only if BT pattern adopted) |
| Everything else | `ament_python` | Faster iteration |

### 1.5 Naming convention

- Package: `<project>_<purpose>`
- Module: same as package
- Node executable: `<role>_node`
- Logic class file: `<role>.py` (no `_node` suffix)

### 1.6 Phase tagging

| Phase | What ships |
|-------|-----------|
| Phase 1 | (full list) |
| Phase 2 | + `<project>_perception` |
| Phase 3 | + `<project>_llm_behavior` |

Phase 2/3 packages have skeleton `package.xml` + `setup.py` ship at Phase 1 (so the workspace builds clean) but no implementation until phase entry.

---

## 2. Package Catalogue

| # | Package | Layer / Role | Build | Phase | Maps to nodes |
|---|---------|-------------|-------|-------|---------------|
| 1 | `<project>_bringup` | Infrastructure | ament_python | P1 | (no nodes; launch only) |
| 2 | `<project>_description` | Infrastructure — URDF | ament_python | P1 | `robot_state_publisher` |
| 3 | `<project>_interfaces` | Cross-cutting | ament_cmake | P1 | (interface package) |
| ... |

---

## 3. Per-Package Detail

### 2.X `<project>_<name>`

**Role:** "<short metaphor — Birth / Body / Voice / Brain etc>" — what this package does.

**FRs / NFRs supported:** FR{X-Y}.

**Process isolation:** <runs as own systemd service / Compose service / shared with X>.

**Layout:**

\`\`\`
<project>_<name>/
├── <project>_<name>/
│   ├── __init__.py
│   ├── <role>_node.py           # ROS LifecycleNode wrapper
│   ├── <role>.py                # PURE — zero rclpy imports
│   └── ...
├── test/
│   ├── test_<role>.py
│   └── test_<role>_integration.py
├── config/
│   └── <name>.yaml
├── launch/
│   └── <name>.launch.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
\`\`\`

**Logic class skeletons:** (Python or C++ snippet)

**Notes:** (per-package gotchas)

(Repeat per package.)

---

## 4. Package Internal Structure (Template)

Every Python `<project>_*` package follows this template — deviations require justification:

\`\`\`
<project>_<name>/
├── <project>_<name>/                   # Python module, same name as package
│   ├── __init__.py
│   ├── <role>_node.py                  # ROS 2 LifecycleNode wrapper(s)
│   └── <role>.py                       # Pure logic (zero rclpy imports)
├── test/
│   ├── test_<role>.py                  # unit tests on pure logic
│   └── test_<role>_integration.py      # launch_testing on the node
├── config/
│   └── <name>.yaml                     # default parameters
├── launch/
│   └── <name>.launch.py
├── fixtures/                           # test fixtures (only if needed)
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
\`\`\`

---

## 5. Dependency Graph

\`\`\`
<project>_interfaces  (interfaces only — no deps on other <project>_*)
       ▲
       │
       ├── <project>_safety
       ├── <project>_mission
       ├── <project>_bt_nodes ──────► <project>_safety  (calls /safety/override)
       │                  └────► <project>_mission (calls /mission/next_waypoint)
       ├── <project>_ldlidar
       └── <project>_diagnostics

<project>_description  (no <project>_* deps)
<project>_bringup     ──► all of the above (launch composition)
\`\`\`

**Critical invariant:** `<project>_interfaces` depends on **nothing** — IDL-only sink. Verified by `colcon graph` in CI.

---

## 6. CMakeLists.txt / setup.py Skeletons

### 6.1 Python package (`ament_python`)

(Show full `setup.py` + `package.xml` example for one representative package.)

### 6.2 Interface package (`ament_cmake`)

See [`interfaces/<project>_interfaces.md`](interfaces/<project>_interfaces.md) §CMakeLists.txt skeleton.

### 6.3 C++ package (`ament_cmake`) (only if BT pattern adopted)

(Show CMakeLists.txt for the BT nodes package.)

---

## 7. Workspace Layout

\`\`\`
<workspace_path>/
├── src/
│   ├── <project>_bringup/
│   ├── <project>_description/
│   ├── <project>_interfaces/
│   ├── <project>_safety/
│   ├── <project>_ldlidar/
│   ├── <project>_bt_nodes/
│   ├── <project>_mission/
│   ├── <project>_diagnostics/
│   ├── <project>_perception/        # Phase 2 — skeleton in P1
│   └── <project>_llm_behavior/      # Phase 3 — skeleton in P2
├── build/                            # gitignored
├── install/                          # gitignored
├── log/                              # gitignored
└── colcon.meta
\`\`\`

External configs (single source of truth, mounted into containers):

\`\`\`
<config_path>/
├── cyclonedds.xml
├── ekf.yaml
├── slam.yaml
├── nav2.yaml
├── safety_params.yaml
├── mission_orchestrator.yaml
├── bt_executor.yaml
├── ldlidar.yaml
└── foxglove_bridge.yaml
\`\`\`

---

## 8. Build Commands

\`\`\`bash
# Full build
colcon build --symlink-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Single package
colcon build --symlink-install --packages-select <project>_safety

# Single package + dependents (after changing <project>_interfaces)
colcon build --symlink-install --packages-up-to <project>_interfaces \
  --packages-above-and-dependencies <project>_interfaces

# Tests
colcon test --packages-select <project>_safety --event-handlers console_direct+
colcon test-result --verbose
\`\`\`

---

## 9. FR-to-Package Traceability

> **Sanity check.** Every functional requirement must map to exactly one package or to the firmware. No orphans, no duplicates.

### Firmware (not a colcon package)

| FR | Layer | Implementation site |
|----|-------|---------------------|
| FR{X-Y} | Spinal Cord | `firmware/.../src/...` |

### Linux/PC-side packages

| FR | Package | Node / file |
|----|---------|-------------|
| FR{X} | `<project>_<pkg>` | `<role>_node.py` |

**Phase 1 sanity:** N active FRs (FR1–FRN minus voided), all mapped. No orphans, no duplicates.

---

## 10. Anti-Patterns (Forbidden in This Workspace)

- ❌ Creating `<project>_navigation` or `<project>_slam` to "wrap" Nav2/SLAM — pure churn.
- ❌ Putting business logic inside `*_node.py` — every node has a paired pure logic file.
- ❌ Hardcoded thresholds in code — all configurable via YAML.
- ❌ Custom messages where a `std_msgs` / `sensor_msgs` / `nav_msgs` type already exists.
- ❌ ONNX model weights committed to git — fetched on first build.
- ❌ Secrets in package config files — env-mounted only.
- ❌ Circular `<project>_*` dependencies — `colcon graph` catches.
- ❌ Mixing C++ and Python in one package.

---

## 11. Implementation Sequence

| Phase | Sub-phase | Packages standing up |
|-------|-----------|---------------------|
| Phase 0 | bootstrap | none — runtime + Compose stack |
| Phase 1a | UDP gate | (firmware only) |
| Phase 1b | URDF + Foxglove | `<project>_description` |
| Phase 1c | VS Code ergonomics | `<project>_diagnostics` |
| Phase 1d | SLAM/Nav2 sim | `<project>_bringup` |
| Phase 1e | Real robot | `<project>_ldlidar` |
| Phase 1f | Mission system | `<project>_interfaces`, `<project>_safety`, `<project>_mission`, `<project>_bt_nodes` |
| Phase 1g | Polish | (no new packages) |
| ... |

---

## 12. Architecture Validation

| Dimension | Status | Notes |
|-----------|--------|-------|
| One layer ↔ one package | ✅ | |
| Pure logic separated | ✅ | Every `*_node.py` paired with a logic file |
| YAML config single-source | ✅ | All in `<config_path>/` |
| FR coverage | ✅ | N/N active FRs mapped |
| No circular deps | ✅ | `<project>_interfaces` is sink |
| Build types correct | ✅ | CMake for IDL + C++; Python for the rest |
| Phase tagging | ✅ | P1/P2/P3 packages enumerated |
| Test scaffolding | ✅ | Every package has `test/` |
| Anti-patterns enumerated | ✅ | §10 |
| Workspace mounts cleanly | ✅ | Container path matches the architecture |

---

## 13. Related Documentation

| Document | Path |
|----------|------|
| Architecture | [`../architecture.md`](../architecture.md) |
| PRD | [`../prd.md`](../prd.md) |
| Epics | [`../epics.md`](../epics.md) |
| Per-node specs | [`nodes/index.md`](nodes/index.md) |
| Custom interfaces | [`interfaces/<project>_interfaces.md`](interfaces/<project>_interfaces.md) |
| Diagrams | [`diagrams/`](diagrams/) |
