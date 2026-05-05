# <project>_interfaces — Custom ROS 2 Interface Package

Custom message, service, and action definitions for the <project> project. Lives at `<workspace>/<project>_interfaces/`. Build type: `ament_cmake`.

\`\`\`
<project>_interfaces/
├── package.xml
├── CMakeLists.txt
├── msg/
│   ├── <Name>.msg
│   └── ...
├── srv/
│   ├── <Name>.srv
│   └── ...
└── action/
    └── (none, if Nav2 actions reused unchanged)
\`\`\`

Phase 1 ships <N> messages + <M> services. (Phase 2 adds <…>.)

---

## Messages

### `<Name>.msg`

One-line description. Published as `/topic/name` with `<QoS profile>`.

\`\`\`
# <project>_interfaces/<Name>
builtin_interfaces/Time timestamp

string field1
uint8 STATUS_A = 0
uint8 STATUS_B = 1
uint8 status

(...other fields with units/comments where helpful...)
\`\`\`

**Example — <state-name>:**
\`\`\`yaml
timestamp: {sec: 1718458400, nanosec: 0}
field1: "value"
status: 1     # STATUS_B
...
\`\`\`

(Repeat for each .msg type. Show 2 examples when the message has distinct states — e.g., normal vs degraded.)

---

## Services

### `<Name>.srv`

One-line description. Called by `<caller_node>` to do `<thing>`.

\`\`\`
# <project>_interfaces/srv/<Name>
<request fields>
---
<response fields>
\`\`\`

**Request example — <scenario>:**
\`\`\`yaml
<request example>
\`\`\`

**Response — accepted:**
\`\`\`yaml
<response example>
\`\`\`

**Response — rejected (<reason>):**
\`\`\`yaml
<response example showing rejection>
\`\`\`

**Rejection reasons:**
- `"reason_a"` — when this fires
- `"reason_b"` — when this fires
- `"reason_c"` — when this fires

(Repeat per service. Always enumerate rejection reasons explicitly.)

---

## Threshold / Configuration Registry (only if the project uses the override mechanism)

The authoritative list for the override broker (called via `OverrideThreshold.srv` or equivalent). Lives in `<config_path>/safety_params.yaml`.

| Threshold name | Default | Soft/Hard | Override range |
|---|---|---|---|
| `battery_return_voltage` | 11.4 V | Soft | 10.5–11.7 |
| `battery_emergency_voltage` | 9.6 V | **HARD** | NEVER |
| ... |

(Cross-reference the matching node spec — usually the threshold-manager or power-manager node — so the registry has one source of truth.)

---

## CMakeLists.txt skeleton

\`\`\`cmake
cmake_minimum_required(VERSION 3.8)
project(<project>_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
# Add: vision_msgs (Phase 2), etc.

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/<Name1>.msg
  msg/<Name2>.msg
  srv/<Name1>.srv
  srv/<Name2>.srv
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
\`\`\`

---

## Related stories

| Story | What it delivers |
|-------|------------------|
| Story X.Y | This package skeleton + Phase 1 message/service definitions |
