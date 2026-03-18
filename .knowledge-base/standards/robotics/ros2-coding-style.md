---
title: "ROS 2 Coding Style Standards"
category: "robotics"
tags: ["ros2", "python", "cpp", "naming-conventions", "rep-103", "rep-105", "coding-style"]
description: "Coding style guidelines for Python and C++ in ROS 2, including naming conventions and REP compliance"
last_updated: "2026-03-18"
---

# ROS 2 Coding Style Standards

> **Philosophy:** Follow ROS 2 community conventions. Consistency across packages matters more than personal preference. When in doubt, match what `rclpy` and `rclcpp` examples do.

## Python Naming Conventions (PEP 8 + ROS 2)

| Type | Pattern | Example |
|------|---------|---------|
| Packages | `snake_case` | `inspection`, `motor_driver` |
| Modules/Files | `snake_case.py` | `lidar_processor.py`, `nav_utils.py` |
| Classes | `PascalCase` | `ScanInspectorNode`, `MotorController` |
| Functions | `snake_case` | `process_scan()`, `calculate_odometry()` |
| Variables | `snake_case` | `scan_data`, `max_velocity` |
| Constants | `UPPER_SNAKE_CASE` | `MAX_LINEAR_SPEED`, `DEFAULT_TIMEOUT` |
| Private members | `_leading_underscore` | `self._timer`, `self._publisher` |
| ROS parameters | `snake_case` | `wheel_radius`, `max_speed` |

```python
# GOOD
class ScanInspectorNode(Node):
    MAX_SCAN_RANGE = 30.0

    def __init__(self):
        super().__init__('scan_inspector')
        self._scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self._scan_callback, 10
        )
        self._max_speed = 0.5

    def _scan_callback(self, msg: LaserScan) -> None:
        filtered_ranges = self._filter_invalid_ranges(msg.ranges)
        self._process_filtered_scan(filtered_ranges)

# BAD
class scanInspector(Node):          # camelCase class
    def ScanCallback(self, msg):      # PascalCase function
        scanData = msg.ranges          # camelCase variable
```

## C++ Naming Conventions

| Type | Pattern | Example |
|------|---------|---------|
| Classes/Structs | `PascalCase` | `MotorController`, `ScanFilter` |
| Functions/Methods | `snake_case` | `process_scan()`, `get_velocity()` |
| Variables | `snake_case` | `scan_data`, `wheel_radius` |
| Member variables | `snake_case_` (trailing) | `timer_`, `publisher_` |
| Constants | `UPPER_SNAKE_CASE` | `MAX_SPEED`, `BUFFER_SIZE` |
| Namespaces | `snake_case` | `inspection`, `nav_utils` |
| Header guards | `PACKAGE__FILE_HPP_` | `MOTOR_DRIVER__MOTOR_CONTROLLER_HPP_` |
| Files | `snake_case.cpp` / `.hpp` | `motor_controller.cpp` |

```cpp
// GOOD
namespace inspection
{
class ScanProcessor : public rclcpp::Node
{
public:
  static constexpr double MAX_RANGE = 30.0;

  explicit ScanProcessor(const rclcpp::NodeOptions & options);
  void process_scan(const sensor_msgs::msg::LaserScan & msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  double max_speed_;
};
}  // namespace inspection
```

## ROS 2 Name Conventions

### Node Names

- **Format:** `snake_case`, descriptive of function
- **Examples:** `lidar_processor`, `motor_controller`, `camera_streamer`
- **Anti-pattern:** `node1`, `my_node`, `test` (non-descriptive)

### Topic Names

- **Format:** `snake_case`, hierarchical with `/`
- Use relative names (`scan`) not absolute (`/scan`) inside nodes
- Private topics use `~/` prefix (expands to `/<node_name>/`)

| Purpose | Topic Name | Message Type |
|---------|-----------|-------------|
| Sensor data | `scan`, `imu/data`, `camera/image_raw` | Standard sensor_msgs |
| Commands | `cmd_vel`, `joint_commands` | geometry_msgs/Twist |
| Status | `~/status`, `battery_state` | Custom or std_msgs |
| Diagnostics | `/diagnostics` | diagnostic_msgs |

### Service Names

- **Format:** `snake_case`, verb or action-based
- **Examples:** `trigger_inspection`, `set_mode`, `get_map`

### Package Names

- **Format:** lowercase with underscores, no hyphens
- **Prefix** with project name for custom packages: `my_robot_inspection`, `my_robot_motor_driver`
- **GOOD:** `inspection`, `motor_driver`
- **BAD:** `ScanInspection`, `motor-driver`, `motorDriver`

## REP Compliance (MANDATORY)

### REP 103 — Standard Units and Coordinate Conventions

| Measurement | Unit | Notes |
|-------------|------|-------|
| Distance | meters | All linear measurements |
| Angle | radians | All angular measurements |
| Time | seconds | Durations and timestamps |
| Mass | kilograms | |
| Force | newtons | |
| Frequency | hertz | |

**Coordinate frame orientation:**
- **x** = forward, **y** = left, **z** = up (right-hand rule)
- This applies to ALL frames: robot body, sensors, map


### REP 105 — Standard Frame Names

| Frame | Purpose | Attached To |
|-------|---------|-------------|
| `map` | Global fixed frame (corrected by SLAM/localization) | World origin |
| `odom` | Continuous but drifting odometry frame | World origin |
| `base_link` | Robot body frame (rigidly attached to robot) | Robot center |
| `base_footprint` | Projection of base_link on ground plane | Ground below robot |

**Transform chain:** `map → odom → base_link`

### Parameter Naming Conventions

- Parameters use `snake_case`
- Nested parameters use `.` separator: `motor.max_speed`, `lidar.range_min`
- Boolean parameters use `enable_` or `use_` prefix: `use_sim_time`, `enable_logging`

## Import Ordering (Python)

Imports MUST follow this order, separated by blank lines:

```python
# 1. Standard library
import os
import math
from typing import List, Optional

# 2. Third-party
import numpy as np
import cv2

# 3. ROS 2 core
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# 4. ROS 2 message types
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# 5. Local package
from .scan_filter import ScanFilter
from .utils import clamp_velocity
```

## Type Hints (Python)

Type hints are REQUIRED for all public API functions and recommended for private methods.

```python
# GOOD — typed public API
def calculate_distance(point_a: Point, point_b: Point) -> float:
    """Calculate Euclidean distance between two points."""
    ...

def filter_ranges(ranges: List[float], min_range: float, max_range: float) -> List[float]:
    ...

# Private methods — recommended but not mandatory
def _process_internal(self, data: LaserScan) -> None:
    ...
```

## Docstrings (Google Style)

```python
def navigate_to_waypoint(
    self,
    waypoint: Pose,
    timeout: float = 30.0,
) -> bool:
    """Navigate the robot to a specified waypoint.

    Uses the Nav2 action server to plan and execute a path
    to the given waypoint pose.

    Args:
        waypoint: Target pose in the map frame.
        timeout: Maximum time in seconds to reach the waypoint.

    Returns:
        True if the waypoint was reached within the timeout.

    Raises:
        NavigationError: If path planning fails.
    """
```

## Line Length

| Language | Max Line Length |
|----------|---------------|
| Python | 99 characters |
| C++ | 100 characters |
| XML (launch/URDF) | 120 characters |
| YAML | 120 characters |

## File Organization

### Python Node File

```python
"""Module docstring: one-line description of the node."""

# Imports (ordered as above)

# Constants
MAX_SPEED = 1.0

class MyNode(Node):
    """Node docstring."""

    def __init__(self):
        ...

    # Public methods

    # Callback methods (prefixed with _)

    # Private helper methods

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quick Checklist

- [ ] Python: snake_case everywhere except class names (PascalCase)
- [ ] C++: snake_case functions/variables, PascalCase classes, trailing `_` for members
- [ ] All units in meters, radians, seconds (REP 103)
- [ ] Standard frame names used: map, odom, base_link (REP 105)
- [ ] Parameters are snake_case with `.` nesting
- [ ] Node names are descriptive snake_case
- [ ] Topic names are hierarchical snake_case
- [ ] Package names are lowercase with underscores
- [ ] Python imports ordered: stdlib → third-party → ROS 2 → messages → local
- [ ] Type hints on all public Python APIs
- [ ] Google-style docstrings on public functions
- [ ] Line length within limits (99 Python, 100 C++)
