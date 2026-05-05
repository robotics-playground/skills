# Package Structure Authoring Rules

## Section order (mandatory)

1. **Design principles** — the "why" behind the layout choices
2. **Package catalogue** — table of every package with layer / build type / phase / nodes hosted
3. **Per-package detail** — one section per package with directory layout + key code skeletons
4. **Internal structure template** — the standard layout every Python package uses
5. **Dependency graph** — visualised as ASCII or text; verify no circular deps
6. **CMakeLists.txt / setup.py skeletons** — examples for Python, CMake-IDL, C++
7. **Workspace layout** — `<root>/ros2_ws/src/` tree + external configs path
8. **Build commands** — `colcon build`, `--packages-select`, etc.
9. **FR-to-package traceability** — every active FR mapped to exactly one site
10. **Anti-patterns** — what's forbidden in this workspace
11. **Implementation sequence** — phase-by-phase package roll-out
12. **Validation checklist** — every dimension checked

## Design principles — load-bearing

Always include these as numbered subsections under §1:

- **§1.1 One layer ↔ one package.** Each architectural layer = one (sometimes two) ROS 2 packages so process boundaries match layer boundaries.
- **§1.2 Separate ROS code from pure logic — non-negotiable.** Show the GOOD/BAD example with rclpy imports inside vs outside. Reference: every node has a `<role>_node.py` (ROS wrapper) and a `<role>.py` (pure logic, zero ROS imports).
- **§1.3 Single source of truth for configuration.** All YAML params live in one external `<root>/docker/config/` (or equivalent) directory, mounted read-only. No hardcoded thresholds in code.
- **§1.4 Build types.** `ament_cmake` for `<project>_interfaces` (IDL gen) and any C++ package (BT.CPP nodes). `ament_python` for everything else — faster iteration.
- **§1.5 Naming convention.** Package = `<project>_<purpose>`. Module = same as package. Node executable = `<role>_node`. Logic class file = `<role>.py` (no `_node` suffix — emphasises the no-ROS-import rule).
- **§1.6 Phase tagging.** Phase 2 / Phase 3 packages ship as **skeletons** in P1 so the workspace builds clean. No implementation until phase entry.

## Package catalogue table

```markdown
| # | Package | Layer / Role | Build | Phase | Maps to nodes |
|---|---------|-------------|-------|-------|---------------|
| 1 | `<project>_bringup` | Infrastructure — system startup | ament_python | P1 | (no nodes; launch only) |
| 2 | `<project>_description` | Infrastructure — URDF / robot model | ament_python | P1 | `robot_state_publisher` |
| 3 | `<project>_interfaces` | Cross-cutting — message / service definitions | ament_cmake | P1 | (interface package) |
| ... |
```

Total package count typically lands at:
- **~9 P1 packages** for a full robot project (firmware + safety + nav + BT + mission + bringup + description + interfaces + diagnostics)
- **+1 P2** (perception or similar)
- **+1 P3** (LLM or remote API integration)

If the project is simpler (no BT, no mission system), 5–6 packages is normal. Don't pad.

## Per-package detail — what every section needs

```markdown
### 2.X `<project>_<name>`

**Role:** "Voice / Body / Brain / etc" — short metaphor.

**FRs / NFRs supported:** FR{X-Y}.

**Process isolation:** dedicated process group / systemd / Compose service.

**Layout:**
\`\`\`
<project>_<name>/
├── <project>_<name>/
│   ├── __init__.py
│   ├── <role>_node.py           # ROS LifecycleNode wrapper
│   ├── <role>.py                # PURE — zero rclpy imports
│   └── <other_pure_logic>.py
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

**Logic class skeletons:** (Python or C++ snippet showing the pure logic class signature)

**Notes:** (per-package gotchas — e.g., "production mission YAMLs live at `<root>/missions/`, not inside this package")
```

Include code skeletons for the load-bearing logic classes (PowerManager, ThresholdManager, MissionOrchestrator, BT custom nodes). They're the design-stage commitment to the pure-logic-vs-node-class split.

## Dependency graph — text or ASCII

Show the package dependency graph. The `<project>_interfaces` package must be the sink (no outbound deps, only inbound). Verify no circular deps.

```
<project>_interfaces  (IDL only — no deps on other <project>_*)
       ▲
       │
       ├── <project>_safety
       ├── <project>_mission
       ├── <project>_bt_nodes ──────► <project>_safety  (calls /safety/override)
       │                  └────► <project>_mission (calls /mission/next_waypoint)
       ├── <project>_ldlidar
       ├── <project>_diagnostics
       ├── <project>_perception        (Phase 2)
       └── <project>_llm_behavior      (Phase 3)

<project>_description  (no <project>_* deps)
<project>_bringup     ──► all of the above
```

Add a one-liner at the bottom: "Verified by `colcon graph` in CI" so the implementer knows there's a check that should run.

## FR-to-package traceability — mandatory

Single table, one row per FR. Mark voided FRs explicitly. Mark Phase 2/3 FRs (often "TBD at phase entry") explicitly. **Every active FR must map to exactly one site.** No orphans. No duplicates.

Two sub-tables work better than one if there's a firmware tier:

```markdown
### Firmware (not a colcon package; included for completeness)

| FR | Layer | Implementation site |
|---|---|---|
| FR1–FR9 | Spinal Cord | `firmware/.../src/motor_control.{h,cpp}` |
| FR42 | **VOIDED** (AR22) | n/a |
| ... |

### Linux/PC-side packages

| FR | Package | Node / file |
|---|---|---|
| FR23 | `<project>_ldlidar` | `lds02rr_decoder_node.py` |
| ... |
```

End with a **sanity check sentence**: "Phase 1 sanity: 67 active FRs (FR1–FR68 minus FR42), all mapped. No orphans, no duplicates."

If you can't write that sentence honestly, the design isn't done.

## Anti-patterns — project-specific list

Build on the architecture's general anti-patterns. Add ones specific to the workspace:

- ❌ Creating a `<project>_navigation` or `<project>_slam` to "wrap" Nav2/SLAM — pure churn, the YAML configs are the wrapper.
- ❌ Putting business logic inside `*_node.py` — every node has a paired pure logic file.
- ❌ Hardcoded thresholds in code — all configurable via YAML.
- ❌ Custom messages where a `std_msgs` / `sensor_msgs` / `nav_msgs` type already exists.
- ❌ Production mission YAMLs in `<project>_mission/fixtures/` — those live at `<root>/missions/`.
- ❌ ONNX model weights committed to git — fetched on first build, gitignored.
- ❌ Secrets in package config files — environment-mounted only.
- ❌ Circular `<project>_*` dependencies — `colcon graph` catches.
- ❌ Mixing C++ and Python in one package — keep `ament_cmake` and `ament_python` distinct.

## Implementation sequence — match the roadmap

The implementation sequence table maps roadmap phases to packages standing up:

```markdown
| Phase | Sub-phase | Packages standing up |
|-------|-----------|---------------------|
| Phase 0 | (bootstrap) | none — runtime + Compose stack |
| Phase 1a | UDP gate | (firmware only) |
| Phase 1b | URDF + Foxglove | `<project>_description` |
| ... |
```

This gives the implementer a sequenced plan: which package to build first, second, etc., aligned to the roadmap's gates.

## Validation checklist (§12)

End the document with a one-row-per-dimension checklist:

```markdown
| Dimension | Status | Notes |
|---|---|---|
| One layer ↔ one package | ✅ | Spinal/Cerebellum/Cortex map clean |
| Pure logic separated | ✅ | Every `*_node.py` paired with a logic file |
| YAML config single-source | ✅ | All in `<root>/docker/config/` |
| FR coverage | ✅ | N/N active FRs mapped (table §9) |
| No circular deps | ✅ | `<project>_interfaces` is sink |
| Build types correct | ✅ | CMake for IDL + C++; Python for the rest |
| Phase tagging | ✅ | P1/P2/P3 packages enumerated |
| Test scaffolding | ✅ | Every package has `test/` with unit + integration |
| Anti-patterns enumerated | ✅ | §10 |
| Workspace mounts cleanly | ✅ | Container path matches D17 |
```

Every row says ✅ — if any can't, the design isn't done.
