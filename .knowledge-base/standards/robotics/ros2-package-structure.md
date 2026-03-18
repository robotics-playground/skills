---
title: "ROS 2 Package Structure Standards"
category: "robotics"
tags: ["ros2", "package-structure", "ament", "workspace", "cmake", "python"]
description: "Standard package layout conventions for ROS 2 workspaces including Python, C++, and mixed packages"
last_updated: "2026-03-18"
---

# ROS 2 Package Structure Standards

> **Philosophy:** Consistent package layout enables any team member to navigate unfamiliar packages immediately. Follow ament conventions — do not invent custom structures.

## Workspace Layout

```
ros2_ws/                          # Workspace root (like project root)
├── src/                          # All packages live here (like /packages in a monorepo)
│   ├── my_robot_inspection/     # Application package
│   ├── my_robot_motor_driver/         # Hardware driver package
│   ├── my_robot_description/          # URDF/robot description package
│   ├── my_robot_bringup/              # Launch + config package
│   └── my_robot_interfaces/           # Custom messages, services, actions
├── build/                        # Build artifacts (gitignored)
├── install/                      # Installed packages (gitignored)
└── log/                          # Build logs (gitignored)
```

**Rules:**
- Source packages go in `src/` only
- NEVER commit `build/`, `install/`, or `log/` directories
- Prefix with `my_robot_` for project identification
- One `.gitignore` at workspace root ignoring `build/`, `install/`, `log/`

## Python Package Structure (ament_python)

```
my_robot_inspection/
├── package.xml                   # Package manifest (like package.json)
├── setup.py                      # Python build config (like tsconfig + build)
├── setup.cfg                     # Tool configuration
├── resource/
│   └── my_robot_inspection      # Resource index marker (empty file, REQUIRED)
├── my_robot_inspection/          # Python module (same name as package)
│   ├── __init__.py
│   ├── inspector_node.py          # Main node implementation
│   ├── scan_processor.py          # Processing logic
│   └── utils/
│       ├── __init__.py
│       └── geometry_helpers.py
├── launch/
│   ├── inspection.launch.py       # Launch files
│   └── inspection_sim.launch.py
├── config/
│   ├── params.yaml                # Default parameters
│   └── rviz_config.rviz           # RViz configuration
├── test/
│   ├── test_scan_processor.py     # Unit tests
│   ├── test_inspector_node.py     # Node integration tests
│   └── test_copyright.py          # License compliance (ament default)
└── README.md                      # Package documentation
```

### package.xml (Format 3)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_inspection</name>
  <version>0.1.0</version>
  <description>Inspection node for the robot platform</description>
  <maintainer email="team@example.com">Your Team Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build type -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>launch_testing</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Dependency types:**
| Tag | Purpose |
|-----|---------|
| `<buildtool_depend>` | Build system (e.g., ament_python, ament_cmake) |
| `<build_depend>` | Compile-time only |
| `<exec_depend>` | Runtime |
| `<depend>` | Both build + runtime |
| `<test_depend>` | Test only |

### setup.py

```python
from setuptools import find_packages, setup

package_name = 'my_robot_inspection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/inspection.launch.py']),
        ('share/' + package_name + '/config',
            ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Team Name',
    maintainer_email='team@example.com',
    description='Inspection node for the robot platform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspector_node = my_robot_inspection.inspector_node:main',
            'scan_processor = my_robot_inspection.scan_processor:main',
        ],
    },
)
```

### setup.cfg

```ini
[develop]
script_dir=$base/lib/my_robot_inspection

[install]
install_scripts=$base/lib/my_robot_inspection
```

### Resource Index Marker

The file `resource/my_robot_inspection` MUST exist and can be empty. This is how `ament_index` discovers installed packages.

```bash
# Create the marker file
touch resource/my_robot_inspection
```

## C++ Package Structure (ament_cmake)

```
my_robot_motor_driver/
├── package.xml
├── CMakeLists.txt
├── include/
│   └── my_robot_motor_driver/          # Public headers (same name as package)
│       ├── motor_controller.hpp
│       └── pid_controller.hpp
├── src/
│   ├── motor_controller.cpp
│   ├── pid_controller.cpp
│   └── motor_driver_node.cpp      # Node with main()
├── launch/
│   └── motor_driver.launch.py
├── config/
│   └── motor_params.yaml
├── test/
│   ├── test_pid_controller.cpp
│   └── test_motor_controller.cpp
└── README.md
```

### CMakeLists.txt Pattern

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_motor_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Library
add_library(motor_driver_lib
  src/motor_controller.cpp
  src/pid_controller.cpp
)
target_include_directories(motor_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_driver_lib rclcpp geometry_msgs)

# Executable
add_executable(motor_driver_node src/motor_driver_node.cpp)
target_link_libraries(motor_driver_node motor_driver_lib)
ament_target_dependencies(motor_driver_node rclcpp)

# Install
install(TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS motor_driver_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid src/test/test_pid_controller.cpp)
  target_link_libraries(test_pid motor_driver_lib)
endif()

ament_package()
```

## Custom Interfaces Package

Custom message, service, and action definitions go in a **dedicated package**.

```
my_robot_interfaces/
├── package.xml
├── CMakeLists.txt
├── msg/
│   ├── InspectionStatus.msg
│   └── SensorHealth.msg
├── srv/
│   ├── TriggerInspection.srv
│   └── SetMode.srv
└── action/
    └── NavigateToWaypoint.action
```

**Rules:**
- Interface packages are ALWAYS `ament_cmake` (even if rest of project is Python)
- One interfaces package per project (not per feature)
- Message names are `PascalCase` (ROS 2 convention)
- See [ros2-coding-style.md](ros2-coding-style.md) for naming details

## Launch File Conventions

- Launch files go in `launch/` directory of the **bringup** package or the owning package
- Use Python launch files (`.launch.py`) — they allow conditionals and composition
- Name format: `<purpose>.launch.py` or `<purpose>_<variant>.launch.py`

```
launch/
├── robot.launch.py           # Full robot bringup
├── robot_sim.launch.py       # Simulation variant
├── inspection.launch.py      # Inspection subsystem only
└── teleop.launch.py          # Teleoperation mode
```

## Config File Conventions

- Config files go in `config/` directory
- Use YAML for ROS parameters
- Name format: `<purpose>_params.yaml` or `<purpose>.yaml`

```yaml
# config/inspection_params.yaml
inspector_node:
  ros__parameters:
    scan_topic: "scan"
    max_range: 30.0
    min_range: 0.1
    update_rate: 10.0
    enable_visualization: true
```

## When to Create a New Package

| Situation | Decision |
|-----------|----------|
| New sensor driver | New package (`my_robot_<sensor>_driver`) |
| New algorithm / processing pipeline | New package if reusable, otherwise add to existing |
| Custom messages needed | Add to `my_robot_interfaces` (single shared package) |
| Launch + config only (bringup) | Dedicated `my_robot_bringup` package |
| Robot URDF/description | Dedicated `my_robot_description` package |
| Shared utility functions | `my_robot_common` package |
| Tightly coupled feature addition | Add to existing package |

**Rule of thumb:** If two packages always change together, they should probably be one package.

## Build and Source Commands

```bash
# Build all packages
cd ~/ros2_ws && colcon build --symlink-install

# Build specific package
colcon build --packages-select my_robot_inspection

# Source the workspace (add to .bashrc)
source ~/ros2_ws/install/setup.bash

# Clean build (when things go wrong)
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Quick Checklist

- [ ] Package name is `snake_case`, prefixed with your project name (e.g., `my_robot_`) for custom packages
- [ ] `package.xml` uses format 3 with correct dependency tags
- [ ] Python module directory matches package name exactly
- [ ] `resource/<package_name>` marker file exists (Python packages)
- [ ] Entry points defined in `setup.py` `console_scripts`
- [ ] Launch files in `launch/` directory, Python format
- [ ] Config files in `config/` directory, YAML format
- [ ] Tests in `test/` directory
- [ ] `build/`, `install/`, `log/` are gitignored
- [ ] Custom interfaces in dedicated `my_robot_interfaces` package
