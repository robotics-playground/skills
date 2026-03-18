---
title: "ROS 2 Node Implementation Patterns"
category: "robotics"
tags: ["ros2", "nodes", "patterns", "lifecycle", "callbacks", "python", "best-practices"]
description: "Standard patterns for ROS 2 node implementation including class-based, lifecycle, and component patterns with error handling and anti-patterns"
last_updated: "2026-03-18"
---

# ROS 2 Node Implementation Patterns

> **Philosophy:** Nodes are the fundamental unit of computation in ROS 2. Use the simplest pattern that meets your requirements. Start with class-based nodes; upgrade to lifecycle or components only when needed.

## Pattern Selection Guide

| Pattern | When to Use | Complexity |
|---------|-------------|------------|
| Class-based node | Default for all new nodes | Low |
| Lifecycle node | Nodes with hardware, managed state transitions | Medium |
| Component node | Performance-critical, zero-copy IPC | Medium |
| Minimal node | Quick prototyping, scripts | Minimal |

## Minimal Node Template

Use only for quick prototyping or one-off scripts. Not for production nodes.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = Node('minimal_node')
    pub = node.create_publisher(String, 'output', 10)
    timer = node.create_timer(1.0, lambda: pub.publish(String(data='hello')))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Class-Based Node Pattern (Recommended Default)

This is the standard pattern for all nodes. Use this unless you have a specific reason for lifecycle or components.

```python
"""Scan processor node for filtering and publishing LiDAR data."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ScanProcessorNode(Node):
    """Processes incoming LiDAR scans and publishes filtered results.

    Subscribes to raw laser scan data, applies range filtering,
    and publishes the minimum detected distance.
    """

    DEFAULT_MIN_RANGE = 0.1
    DEFAULT_MAX_RANGE = 30.0

    def __init__(self):
        super().__init__('scan_processor')

        # Declare parameters with defaults
        self.declare_parameter('min_range', self.DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range', self.DEFAULT_MAX_RANGE)
        self.declare_parameter('scan_topic', 'scan')

        # Read parameters
        self._min_range = self.get_parameter('min_range').value
        self._max_range = self.get_parameter('max_range').value
        scan_topic = self.get_parameter('scan_topic').value

        # Create publishers and subscribers
        self._min_dist_pub = self.create_publisher(Float32, '~/min_distance', 10)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10
        )

        self.get_logger().info(
            f'Scan processor started: range=[{self._min_range}, {self._max_range}]'
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        """Process incoming scan and publish minimum distance."""
        valid_ranges = [
            r for r in msg.ranges
            if self._min_range < r < self._max_range
        ]
        if not valid_ranges:
            self.get_logger().warn('No valid ranges in scan', throttle_duration_sec=5.0)
            return

        min_dist = min(valid_ranges)
        self._min_dist_pub.publish(Float32(data=min_dist))


def main(args=None):
    rclpy.init(args=args)
    node = ScanProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Lifecycle Node Pattern

Use for nodes that manage hardware resources or need explicit state transitions (configure → activate → deactivate → cleanup).

```python
"""Motor controller with managed lifecycle."""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from geometry_msgs.msg import Twist


class MotorControllerNode(LifecycleNode):
    """Lifecycle-managed motor controller.

    States: unconfigured → inactive → active → finalized
    """

    def __init__(self):
        super().__init__('motor_controller')
        self._cmd_sub = None
        self._serial_port = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Load parameters and prepare hardware connection."""
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear_speed', 0.5)

        self._port_name = self.get_parameter('serial_port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._max_speed = self.get_parameter('max_linear_speed').value

        self.get_logger().info(f'Configured: port={self._port_name}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Open hardware connection and start accepting commands."""
        try:
            self._serial_port = self._open_serial(self._port_name, self._baud_rate)
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return TransitionCallbackReturn.FAILURE

        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_callback, 10
        )
        self.get_logger().info('Motor controller activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop motors and stop accepting commands."""
        self._send_stop_command()
        self.destroy_subscription(self._cmd_sub)
        self.get_logger().info('Motor controller deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Close hardware connection and release resources."""
        if self._serial_port:
            self._serial_port.close()
            self._serial_port = None
        self.get_logger().info('Motor controller cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def _cmd_callback(self, msg: Twist) -> None:
        """Forward velocity commands to hardware."""
        clamped_linear = max(-self._max_speed, min(self._max_speed, msg.linear.x))
        self._send_velocity(clamped_linear, msg.angular.z)
```

**When to use lifecycle nodes:**
- Hardware drivers (serial ports, GPIO, cameras)
- Nodes that must initialize in a specific order
- Nodes that need graceful startup/shutdown sequences
- Nodes managed by a launch system or orchestrator

## Component Node Pattern

Use for performance-critical nodes that benefit from zero-copy IPC when composed in the same process.

```python
"""Component node for intra-process composition."""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Image
from std_msgs.msg import String


class ImageDetectorNode(Node):
    """Detect objects in images. Designed for component composition."""

    def __init__(self, **kwargs):
        super().__init__('image_detector', **kwargs)
        self._sub = self.create_subscription(
            Image, 'camera/image_raw', self._detect_callback, 10
        )
        self._pub = self.create_publisher(String, '~/detections', 10)

    def _detect_callback(self, msg: Image) -> None:
        result = self._run_detection(msg)
        self._pub.publish(String(data=result))
```

## Callback Group Patterns

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class MultiCallbackNode(Node):
    def __init__(self):
        super().__init__('multi_callback_node')

        # Group 1: sensor callbacks can run in parallel
        sensor_group = ReentrantCallbackGroup()

        # Group 2: hardware commands must be sequential
        hardware_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            LaserScan, 'scan', self._scan_cb, 10,
            callback_group=sensor_group
        )
        self.create_subscription(
            Imu, 'imu/data', self._imu_cb, 10,
            callback_group=sensor_group
        )
        self.create_subscription(
            Twist, 'cmd_vel', self._cmd_cb, 10,
            callback_group=hardware_group
        )
```

**Use `MultiThreadedExecutor` when using callback groups:**

```python
def main(args=None):
    rclpy.init(args=args)
    node = MultiCallbackNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Parameter Declaration and Validation

```python
def __init__(self):
    super().__init__('parameterized_node')

    # Declare with type and description
    self.declare_parameter('max_speed', 1.0,
        ParameterDescriptor(description='Maximum linear speed in m/s'))
    self.declare_parameter('mode', 'auto',
        ParameterDescriptor(description='Operating mode: auto, manual, or standby'))

    # Validate after reading
    max_speed = self.get_parameter('max_speed').value
    if max_speed <= 0.0 or max_speed > 5.0:
        raise ValueError(f'max_speed must be in (0, 5], got {max_speed}')

    mode = self.get_parameter('mode').value
    valid_modes = {'auto', 'manual', 'standby'}
    if mode not in valid_modes:
        raise ValueError(f'mode must be one of {valid_modes}, got {mode}')

    # Dynamic parameter callback
    self.add_on_set_parameters_callback(self._on_param_change)

def _on_param_change(self, params):
    for param in params:
        if param.name == 'max_speed' and param.value <= 0.0:
            return SetParametersResult(successful=False, reason='max_speed must be positive')
    return SetParametersResult(successful=True)
```

## Error Handling in Callbacks

**CRITICAL:** Never let exceptions escape from callbacks. Unhandled exceptions in callbacks kill the node silently.

```python
# GOOD — exceptions caught and logged
def _scan_callback(self, msg: LaserScan) -> None:
    try:
        self._process_scan(msg)
    except ValueError as e:
        self.get_logger().error(f'Invalid scan data: {e}')
    except Exception as e:
        self.get_logger().fatal(f'Unexpected error in scan callback: {e}')

# BAD — exception kills the node
def _scan_callback(self, msg: LaserScan) -> None:
    self._process_scan(msg)  # If this throws, node dies silently
```

## Logging Best Practices

```python
# Use appropriate severity levels
self.get_logger().debug('Processing scan frame 1234')       # Verbose debugging
self.get_logger().info('Scan processor initialized')         # Normal operation
self.get_logger().warn('Scan data contains NaN values')      # Unexpected but handled
self.get_logger().error('Failed to process scan: timeout')   # Error, node continues
self.get_logger().fatal('Cannot open serial port, shutting down')  # Unrecoverable

# Throttled logging (prevent log spam from high-frequency callbacks)
self.get_logger().warn('Low battery', throttle_duration_sec=30.0)

# Log with context
self.get_logger().info(f'Waypoint reached: [{x:.2f}, {y:.2f}], took {elapsed:.1f}s')
```

## Timer Patterns

```python
# Periodic timer (like setInterval)
self._control_timer = self.create_timer(0.1, self._control_loop)  # 10 Hz

# One-shot timer (like setTimeout)
self._startup_timer = self.create_timer(2.0, self._delayed_init)

def _delayed_init(self):
    """Run once after 2 seconds, then cancel."""
    self._startup_timer.cancel()
    self.get_logger().info('Delayed initialization complete')
```

## Graceful Shutdown

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down via keyboard interrupt')
    finally:
        node.on_shutdown()      # Custom cleanup
        node.destroy_node()
        rclpy.shutdown()
```

## Anti-Patterns

| Anti-Pattern | Problem | Correct Approach |
|-------------|---------|-----------------|
| Global state / module-level variables | Breaks node isolation, untestable | Keep all state in the node class |
| Blocking calls in callbacks | Starves the executor, blocks all other callbacks | Use async services or separate threads |
| Too many timers | High CPU usage, scheduling jitter | Combine related logic into fewer timers |
| Hardcoded topic names | Cannot remap, breaks composition | Use parameters or relative names |
| Ignoring QoS mismatches | Silent communication failure | Match publisher and subscriber QoS profiles |
| `time.sleep()` in callbacks | Blocks the entire executor thread | Use timers or async patterns |
| Publishing in `__init__` | Subscribers may not be ready yet | Publish from callbacks or after a short timer |
| Catching all exceptions silently | Hides bugs | Log the error, re-raise if unrecoverable |

## Quick Checklist

- [ ] Node uses class-based pattern (default) or lifecycle (if hardware)
- [ ] All parameters declared with defaults and descriptions
- [ ] Parameter values validated after reading
- [ ] All callbacks have try/except error handling
- [ ] Logging uses `get_logger()` with appropriate severity
- [ ] High-frequency warnings use `throttle_duration_sec`
- [ ] No blocking calls in callbacks
- [ ] No global state — all state lives in the node instance
- [ ] Topic names are relative (remappable), not hardcoded absolute paths
- [ ] Graceful shutdown handled in `main()`
- [ ] QoS profiles match between publishers and subscribers
