---
title: "Hardware Integration Standards"
category: "robotics"
tags: ["ros2", "hardware", "drivers", "udev", "serial", "sensors", "safety"]
description: "Standards for hardware integration including device management, sensor configuration, motor control, and pre-flight checklists"
last_updated: "2026-03-18"
---

# Hardware Integration Standards

> **Philosophy:** Hardware is unpredictable. Assume every connection will fail, every sensor will glitch, and every motor will stall. Design for graceful degradation, not perfect operation.

## Supported Hardware Platforms

### Example Platform: Tracked Chassis

| Component | Model | Interface | ROS 2 Package |
|-----------|-------|-----------|---------------|
| Chassis | Your robot platform (e.g., tracked chassis) | Serial/CAN | `my_robot_motor_driver` (custom) |
| LiDAR | RPLidar A1/A2 or equivalent | USB Serial | `rplidar_ros` |
| IMU | BNO055 or MPU6050 | I2C/USB | `bno055` or custom |
| Camera | OAK-D / RealSense D435 | USB 3.0 | `depthai_ros` / `realsense2_camera` |
| Compute | Jetson Orin Nano / RPi 5 | — | — |
| Motor Driver | Motor HAT / custom STM32 | Serial/I2C | `my_robot_motor_driver` (custom) |

## udev Rules for Consistent Device Naming

USB devices get different `/dev/ttyUSBx` names depending on plug order. Use udev rules to assign persistent names.

### Creating udev Rules

```bash
# Step 1: Find device attributes
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep -E 'idVendor|idProduct|serial'

# Step 2: Create rule file
sudo nano /etc/udev/rules.d/99-robot.rules
```

### Rule File

```bash
# /etc/udev/rules.d/99-robot.rules
# Robot USB Device Rules

# RPLidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="ttyLidar", MODE="0666"

# Motor controller (STM32)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  SYMLINK+="ttyMotor", MODE="0666"

# IMU (if USB)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", \
  SYMLINK+="ttyIMU", MODE="0666"
```

```bash
# Step 3: Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Step 4: Verify
ls -la /dev/ttyLidar /dev/ttyMotor
```

**Rules:**
- ALL USB devices MUST have udev rules — never use raw `/dev/ttyUSBx` in launch files
- Store the rules file in the repo: `my_robot_bringup/udev/99-robot.rules`
- Include setup instructions in the package README

## Serial Port Permissions

```bash
# Add user to dialout group (required for serial port access)
sudo usermod -a -G dialout $USER

# Verify membership (requires logout/login)
groups $USER | grep dialout

# Temporary fix (does not survive reboot)
sudo chmod 666 /dev/ttyUSB0
```

**MANDATORY:** All team members MUST be in the `dialout` group. The udev rules above set `MODE="0666"` as a fallback, but group membership is the proper solution.

## USB Device Enumeration and Management

### Checking Connected Devices

```bash
# List all USB devices
lsusb

# Detailed info for a specific device
lsusb -v -d 10c4:ea60

# List serial ports
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Monitor USB events in real-time
udevadm monitor --subsystem-match=usb
```

### Handling USB Disconnects

USB devices can disconnect unexpectedly (vibration, loose cable, power glitch). Nodes MUST handle this:

```python
class MotorDriverNode(LifecycleNode):
    """Motor driver with USB disconnect recovery."""

    RECONNECT_INTERVAL = 2.0  # seconds

    def _send_command(self, cmd: bytes) -> bool:
        """Send command with disconnect handling."""
        try:
            self._serial.write(cmd)
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self._attempt_reconnect()
            return False

    def _attempt_reconnect(self) -> None:
        """Try to reconnect to the serial device."""
        if self._reconnect_timer is None:
            self.get_logger().warn('Starting reconnect attempts...')
            self._reconnect_timer = self.create_timer(
                self.RECONNECT_INTERVAL, self._reconnect_callback
            )

    def _reconnect_callback(self) -> None:
        """Periodic reconnect attempt."""
        try:
            self._serial = serial.Serial(self._port_name, self._baud_rate, timeout=1.0)
            self._reconnect_timer.cancel()
            self._reconnect_timer = None
            self.get_logger().info('Serial reconnected successfully')
        except serial.SerialException:
            self.get_logger().warn('Reconnect failed, retrying...', throttle_duration_sec=10.0)
```

## Network Configuration for Robot Communication

### Single-Machine Development

```bash
# Restrict ROS 2 traffic to localhost (faster discovery)
export ROS_LOCALHOST_ONLY=1
```

### Multi-Machine Setup (Dev PC + Robot)

```bash
# On BOTH machines — use same domain ID
export ROS_DOMAIN_ID=42

# Verify connectivity
ping <robot_ip>
ros2 topic list  # Should show topics from both machines
```

| Machine | Purpose | IP Convention |
|---------|---------|--------------|
| Robot (onboard) | Runs drivers, low-level control | `192.168.1.10` |
| Dev PC | Runs RViz2, high-level planning | `192.168.1.20` |
| Simulation PC | Runs Gazebo (if separate) | `192.168.1.30` |

**Firewall rules:** DDS uses UDP multicast for discovery (ports 7400-7500). Ensure these ports are open on all machines.

## Sensor Driver Configuration

### Launch File Pattern for Sensors

```python
"""Sensor bringup with consistent parameter pattern."""

lidar_node = Node(
    package='rplidar_ros',
    executable='rplidar_node',
    name='rplidar',
    parameters=[{
        'serial_port': '/dev/ttyLidar',       # Use udev symlink
        'serial_baudrate': 256000,
        'frame_id': 'laser_frame',             # Must match URDF
        'angle_compensate': True,
        'scan_mode': 'Standard',
    }],
    remappings=[
        ('scan', '/scan'),                      # Remap to standard topic
    ],
)
```

### Sensor Health Monitoring

Every sensor driver MUST publish diagnostics:

```python
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SensorDriver(Node):
    def __init__(self):
        super().__init__('sensor_driver')
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)

    def _publish_diagnostics(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'Sensor: LiDAR'
        status.hardware_id = 'rplidar_a2'
        status.level = DiagnosticStatus.OK  # OK, WARN, ERROR, STALE
        status.message = 'Operating normally'
        status.values = [
            KeyValue(key='scan_rate_hz', value=str(self._current_rate)),
            KeyValue(key='temperature_c', value=str(self._temperature)),
        ]
        msg.status.append(status)
        self._diag_pub.publish(msg)
```

## Motor Controller Integration Patterns

### Command Interface

```python
"""Standard velocity command interface for motor controllers."""

class MotorControllerInterface:
    """Abstract interface for motor controllers.

    All motor controllers MUST implement this interface regardless
    of the underlying hardware (motor HAT, STM32, etc.).
    """

    def set_velocity(self, linear: float, angular: float) -> None:
        """Set robot velocity.

        Args:
            linear: Forward velocity in m/s (REP 103).
            angular: Rotational velocity in rad/s (REP 103).
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Immediately stop all motors."""
        raise NotImplementedError

    def get_encoder_ticks(self) -> tuple[int, int]:
        """Read encoder tick counts (left, right)."""
        raise NotImplementedError
```

### Watchdog Timer (MANDATORY for Motor Controllers)

```python
class MotorControllerNode(Node):
    """Motor controller with command timeout watchdog."""

    CMD_TIMEOUT = 0.5  # Stop motors if no command received for 500ms

    def __init__(self):
        super().__init__('motor_controller')
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_check)
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_callback, 10
        )

    def _cmd_callback(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._controller.set_velocity(msg.linear.x, msg.angular.z)

    def _watchdog_check(self) -> None:
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.CMD_TIMEOUT:
            self._controller.stop()
```

## Power Management Considerations

| Component | Typical Draw | Notes |
|-----------|-------------|-------|
| Jetson Orin Nano | 7-15W | Use power mode settings |
| RPi 5 | 5-12W | Depends on peripherals |
| RPLidar A2 | 2.5W | 5V USB powered |
| RealSense D435 | 3.5W | USB 3.0 required |
| Motor drivers | 2-20W | Depends on load, use spikes |
| Total robot | 20-50W | Size battery accordingly |

**Rules:**
- Monitor battery voltage and publish to `/battery_state`
- Implement low-battery warnings (software-defined threshold)
- Graceful shutdown at critical battery level
- See [safety-protocols.md](safety-protocols.md) for LiPo handling

## Hardware Abstraction Layer Pattern

```
Application Layer     →  Uses MotorControllerInterface
                          │
HAL Interface         →  MotorControllerInterface (abstract)
                          │
Driver Implementations → PlatformMotorDriver | STM32MotorDriver | SimMotorDriver
                          │
Hardware / Simulator  →  Serial port / Gazebo plugin
```

This pattern allows swapping hardware without changing application code.

## Pre-Flight Checklist

Run through this checklist BEFORE every hardware test session.

### Hardware Checks

- [ ] Battery charged above 80% (check voltage with multimeter)
- [ ] All USB cables securely connected
- [ ] udev symlinks verified: `ls /dev/ttyLidar /dev/ttyMotor`
- [ ] Wheels/tracks move freely (no obstructions)
- [ ] E-stop button accessible and tested
- [ ] Camera lens clean
- [ ] LiDAR unobstructed (360-degree clear view)

### Software Checks

- [ ] Workspace built cleanly: `colcon build`
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] All driver nodes launch without errors
- [ ] `ros2 topic list` shows expected topics
- [ ] `ros2 topic hz /scan` shows expected rate
- [ ] TF tree is complete: `ros2 run tf2_tools view_frames`
- [ ] Teleoperation works: move robot with keyboard/joystick
- [ ] Emergency stop tested (software kill switch)

### Environment Checks

- [ ] Test area clear of obstacles and people
- [ ] Safety zone marked (see [safety-protocols.md](safety-protocols.md))
- [ ] Network connectivity verified (if multi-machine)
- [ ] ros2 bag recording started

## Quick Checklist

- [ ] All USB devices have udev rules (no raw `/dev/ttyUSBx`)
- [ ] All team members in `dialout` group
- [ ] Driver nodes handle USB disconnect/reconnect
- [ ] Motor controller has watchdog timer (stops on command timeout)
- [ ] Sensors publish to `/diagnostics`
- [ ] Battery monitoring implemented
- [ ] Hardware abstraction layer separates drivers from application logic
- [ ] Pre-flight checklist completed before every test session
