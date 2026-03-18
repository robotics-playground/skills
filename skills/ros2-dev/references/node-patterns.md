# Node Patterns Reference

> ROS 2 Jazzy | Python (rclpy) primary, C++ (rclcpp) for comparison

---

## 1. Python Node Structures

### Minimal Node (Quick Prototyping)

The simplest possible ROS 2 node. Good for one-off scripts and quick tests.

```python
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('minimal_node')
    node.get_logger().info('Hello from minimal node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**When to use:** Throwaway scripts, quick data inspection, learning.
**When NOT to use:** Anything with state, multiple callbacks, or that will grow.

### Class-Based Node (Standard Pattern)

The workhorse pattern for 90% of nodes. Subclass `Node` and set up everything
in `__init__`.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    """Subscribes to LiDAR, publishes velocity commands to avoid obstacles."""

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Declare parameters with defaults and descriptions
        self.declare_parameter('min_distance', 0.5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)

        # Read parameters
        self.min_distance = self.get_parameter('min_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Create subscriber -- calls scan_callback when data arrives
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10  # QoS depth
        )

        # Create publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a periodic timer (optional -- for heartbeat, status, etc.)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('Obstacle avoidance node started')

    def scan_callback(self, msg: LaserScan):
        """Process incoming LiDAR scan."""
        # Find minimum distance in front of robot
        min_range = min(msg.ranges) if msg.ranges else float('inf')

        cmd = Twist()
        if min_range < self.min_distance:
            # Obstacle detected -- turn
            cmd.angular.z = self.angular_speed
            self.get_logger().warn(f'Obstacle at {min_range:.2f}m -- turning')
        else:
            # Clear path -- go forward
            cmd.linear.x = self.linear_speed

        self.cmd_pub.publish(cmd)

    def status_callback(self):
        """Periodic status log."""
        self.get_logger().debug('Obstacle avoidance running')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
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

**Key points:**
- `__init__` is the constructor where all setup happens
- `create_subscription` registers a callback -- called when data arrives
- `rclpy.spin()` is the event loop -- it dispatches callbacks forever
- The node is the unit of composition -- one class, one responsibility

### Lifecycle Node (Managed Startup/Shutdown)

Lifecycle nodes add a state machine to control startup and shutdown order. This is
critical for hardware drivers -- you want to configure the camera before starting
the image processing pipeline.

Lifecycle nodes use a state machine (unconfigured -> inactive -> active -> finalized)
with transition callbacks for each step.

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import Image

class CameraDriverNode(LifecycleNode):
    """Lifecycle-managed camera driver."""

    def __init__(self):
        super().__init__('camera_driver')
        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 30.0)
        self._camera = None
        self._publisher = None
        self._timer = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Transition: unconfigured -> inactive. Allocate resources."""
        self.get_logger().info('Configuring camera driver...')
        device_id = self.get_parameter('device_id').value

        try:
            # Import here -- only needed when actually configuring
            import cv2
            self._camera = cv2.VideoCapture(device_id)
            if not self._camera.isOpened():
                self.get_logger().error(f'Cannot open camera {device_id}')
                return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Camera init failed: {e}')
            return TransitionCallbackReturn.FAILURE

        # Create publisher in configure (but don't publish yet)
        self._publisher = self.create_lifecycle_publisher(Image, 'image_raw', 10)
        self.get_logger().info('Camera configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Transition: inactive -> active. Start publishing."""
        fps = self.get_parameter('fps').value
        self._timer = self.create_timer(1.0 / fps, self._capture_callback)
        self.get_logger().info(f'Camera activated at {fps} FPS')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Transition: active -> inactive. Stop publishing."""
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info('Camera deactivated')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Transition: inactive -> unconfigured. Release resources."""
        if self._camera:
            self._camera.release()
            self._camera = None
        self._publisher = None
        self.get_logger().info('Camera cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Transition: any -> finalized. Final cleanup."""
        if self._camera:
            self._camera.release()
        self.get_logger().info('Camera shut down')
        return TransitionCallbackReturn.SUCCESS

    def _capture_callback(self):
        """Timer callback: capture and publish frame."""
        if self._camera is None:
            return
        ret, frame = self._camera.read()
        if not ret:
            self.get_logger().warn('Frame capture failed')
            return
        # Convert to ROS Image message and publish
        # (using cv_bridge in real code)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Lifecycle State Transitions

```
                  +------------------+
                  |  Unconfigured    |
                  +--------+---------+
                           |
                    on_configure()
                           |
                  +--------v---------+
             +--->|    Inactive      |<---+
             |    +--------+---------+    |
             |             |              |
       on_cleanup()  on_activate()  on_deactivate()
             |             |              |
             |    +--------v---------+    |
             +----+     Active       +----+
                  +--------+---------+
                           |
                    on_shutdown() (from any state)
                           |
                  +--------v---------+
                  |    Finalized     |
                  +------------------+
```

**Trigger transitions from CLI:**
```bash
ros2 lifecycle set /camera_driver configure
ros2 lifecycle set /camera_driver activate
ros2 lifecycle set /camera_driver deactivate
ros2 lifecycle set /camera_driver cleanup
ros2 lifecycle set /camera_driver shutdown
```

---

## 2. C++ Node Structure (For Comparison)

The same obstacle avoidance node in C++. Notice the structural similarity.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance") {
        declare_parameter("min_distance", 0.5);
        declare_parameter("linear_speed", 0.3);
        min_distance_ = get_parameter("min_distance").as_double();
        linear_speed_ = get_parameter("linear_speed").as_double();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&ObstacleAvoidanceNode::scan_callback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        auto cmd = geometry_msgs::msg::Twist();
        if (min_range < min_distance_) {
            cmd.angular.z = 1.0;
        } else {
            cmd.linear.x = linear_speed_;
        }
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    double min_distance_;
    double linear_speed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 3. Callback Groups

**This is the most important concept for multi-threaded nodes.** Without callback
groups, a `MultiThreadedExecutor` will run ALL callbacks in parallel -- which causes
race conditions if callbacks share state.

### Types of Callback Groups

| Type | Behavior | Use When |
|---|---|---|
| `MutuallyExclusiveCallbackGroup` | Only one callback in the group runs at a time | Callbacks that share state (default if no group specified) |
| `ReentrantCallbackGroup` | All callbacks in the group can run in parallel | Callbacks that are independently stateless |

### Example: Protecting Shared State

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class MultiSensorNode(Node):
    def __init__(self):
        super().__init__('multi_sensor')

        # Group 1: These callbacks share self._latest_scan -- must not run in parallel
        self.scan_group = MutuallyExclusiveCallbackGroup()

        # Group 2: These callbacks are independent -- can run in parallel
        self.independent_group = ReentrantCallbackGroup()

        # LiDAR scan updates shared state
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10,
            callback_group=self.scan_group
        )

        # Timer reads shared state
        self.process_timer = self.create_timer(
            0.1, self.process_callback,
            callback_group=self.scan_group  # Same group -- mutual exclusion
        )

        # IMU is independent
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10,
            callback_group=self.independent_group
        )

        self._latest_scan = None

    def scan_callback(self, msg):
        self._latest_scan = msg  # Safe -- mutually exclusive with process_callback

    def process_callback(self):
        if self._latest_scan is not None:
            # Process scan -- safe because scan_callback can't run simultaneously
            pass

    def imu_callback(self, msg):
        # Independent -- can run in parallel with anything
        pass
```

**Rule of thumb:** If two callbacks read/write the same instance variable, put them
in the same `MutuallyExclusiveCallbackGroup`. If they're independent, use
`ReentrantCallbackGroup` for parallelism.

---

## 4. Executors

### SingleThreadedExecutor (Default)

```python
# This is what rclpy.spin() uses internally
rclpy.spin(node)

# Equivalent explicit version:
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

All callbacks run sequentially on one thread. Simple and safe. No need for
callback groups. **Start here.**

### MultiThreadedExecutor

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(sensor_node)
executor.add_node(processing_node)
executor.spin()
```

Callbacks can run in parallel across threads. **You MUST use callback groups**
to prevent race conditions. Use when:
- You have multiple independent subscribers that need parallel processing
- A slow callback would block other callbacks in single-threaded mode
- You need to service multiple nodes in one process

### Multiple Nodes in One Executor

```python
def main():
    rclpy.init()
    node_a = SensorNode()
    node_b = ProcessingNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node_a)
    executor.add_node(node_b)

    try:
        executor.spin()
    finally:
        node_a.destroy_node()
        node_b.destroy_node()
        rclpy.shutdown()
```

---

## 5. Parameter Handling

Parameters are the ROS 2 way to configure nodes at runtime. Think of them as
environment variables or configuration files, but with type safety and
dynamic update support.

### Declaring and Reading Parameters

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare with default value -- type is inferred
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('sensor_topic', '/sensors/lidar')
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('thresholds', [0.5, 1.0, 2.0])

        # Read values
        rate = self.get_parameter('update_rate').value
        topic = self.get_parameter('sensor_topic').value

        # Use ParameterDescriptor for documentation and constraints
        from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
        self.declare_parameter(
            'max_speed',
            1.0,
            ParameterDescriptor(
                description='Maximum linear speed in m/s',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0,
                    to_value=3.0,
                    step=0.1
                )]
            )
        )
```

### Dynamic Parameter Updates

```python
class DynamicNode(Node):
    def __init__(self):
        super().__init__('dynamic_node')
        self.declare_parameter('gain', 1.0)
        self._gain = self.get_parameter('gain').value

        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self._param_callback)

    def _param_callback(self, params):
        """Called when parameters are changed externally."""
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'gain':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='Gain must be non-negative'
                    )
                self._gain = param.value
                self.get_logger().info(f'Gain updated to {self._gain}')
        return SetParametersResult(successful=True)
```

**Set parameters from CLI:**
```bash
ros2 param set /dynamic_node gain 2.5
ros2 param get /dynamic_node gain
ros2 param list /dynamic_node
ros2 param dump /dynamic_node  # dump all params to YAML
```

---

## 6. Timer Patterns

### Periodic Timer (Most Common)

```python
# Fires every 0.1 seconds (10 Hz)
self.timer = self.create_timer(0.1, self.timer_callback)
```

### One-Shot Timer

```python
# Fire once after 5 seconds, then cancel
def delayed_action(self):
    self.one_shot = self.create_timer(5.0, self._one_shot_callback)

def _one_shot_callback(self):
    self.get_logger().info('One-shot fired!')
    self.one_shot.cancel()  # Cancel so it doesn't fire again
```

### Wall Timer (Ignores Simulation Time)

```python
# Use for diagnostics that should run at real-world rate regardless of sim time
self.wall_timer = self.create_timer(1.0, self.diagnostics_callback)
# Note: In rclpy, create_timer uses ROS time by default which follows sim time.
# For wall time, use: self.create_timer(1.0, callback, clock=rclpy.clock.Clock())
```

---

## 7. Error Handling in Nodes

**An unhandled exception in a callback will crash the node.** Always protect callbacks.

```python
def scan_callback(self, msg):
    try:
        # Processing logic
        result = self._process_scan(msg)
        self.result_pub.publish(result)
    except ValueError as e:
        self.get_logger().warn(f'Invalid scan data: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error in scan_callback: {e}')
        # Don't re-raise -- let the node continue running
```

**For lifecycle nodes**, return `TransitionCallbackReturn.FAILURE` from transition
callbacks to signal errors cleanly instead of raising exceptions.

---

## 8. Complete Example: Sensor Processing Node

A production-ready IMU processing node demonstrating all patterns together.

```python
#!/usr/bin/env python3
"""IMU data processor with filtering and republishing.

Subscribes to raw IMU data, applies a low-pass filter, and publishes
the filtered result. Demonstrates: class-based node, parameters,
callbacks, timers, error handling, and logging.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math


class ImuFilterNode(Node):
    """Low-pass filter for IMU data."""

    def __init__(self):
        super().__init__('imu_filter')

        # Parameters
        self.declare_parameter('alpha', 0.1)  # Filter coefficient (0-1)
        self.declare_parameter('input_topic', 'imu/data_raw')
        self.declare_parameter('output_topic', 'imu/data_filtered')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        self._alpha = self.get_parameter('alpha').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate = self.get_parameter('publish_rate').value

        # Dynamic parameter updates
        self.add_on_set_parameters_callback(self._on_param_change)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Communications
        self.imu_sub = self.create_subscription(
            Imu, input_topic, self._imu_callback, sensor_qos
        )
        self.filtered_pub = self.create_publisher(Imu, output_topic, 10)
        self.tilt_pub = self.create_publisher(Float64, 'imu/tilt_angle', 10)

        # Periodic publish timer
        self.publish_timer = self.create_timer(1.0 / rate, self._publish_callback)

        # Diagnostics timer
        self.diag_timer = self.create_timer(5.0, self._diagnostics_callback)

        # State
        self._filtered_msg = None
        self._raw_count = 0
        self._publish_count = 0

        self.get_logger().info(
            f'IMU filter started: alpha={self._alpha}, '
            f'input={input_topic}, output={output_topic}, rate={rate}Hz'
        )

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'alpha':
                if not 0.0 < param.value <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason='alpha must be in (0, 1]'
                    )
                self._alpha = param.value
                self.get_logger().info(f'Filter alpha updated to {self._alpha}')
        return SetParametersResult(successful=True)

    def _imu_callback(self, msg: Imu):
        try:
            self._raw_count += 1
            if self._filtered_msg is None:
                self._filtered_msg = msg
                return

            # Low-pass filter on linear acceleration
            a = self._alpha
            f = self._filtered_msg
            f.linear_acceleration.x = a * msg.linear_acceleration.x + (1 - a) * f.linear_acceleration.x
            f.linear_acceleration.y = a * msg.linear_acceleration.y + (1 - a) * f.linear_acceleration.y
            f.linear_acceleration.z = a * msg.linear_acceleration.z + (1 - a) * f.linear_acceleration.z

            # Update timestamp
            f.header.stamp = msg.header.stamp
            f.header.frame_id = msg.header.frame_id

        except Exception as e:
            self.get_logger().error(f'Error filtering IMU data: {e}')

    def _publish_callback(self):
        if self._filtered_msg is None:
            return

        self.filtered_pub.publish(self._filtered_msg)
        self._publish_count += 1

        # Compute tilt angle from accelerometer
        try:
            ax = self._filtered_msg.linear_acceleration.x
            az = self._filtered_msg.linear_acceleration.z
            tilt = math.atan2(ax, az)
            tilt_msg = Float64()
            tilt_msg.data = math.degrees(tilt)
            self.tilt_pub.publish(tilt_msg)
        except Exception as e:
            self.get_logger().warn(f'Tilt computation failed: {e}')

    def _diagnostics_callback(self):
        self.get_logger().info(
            f'IMU filter stats: raw={self._raw_count}, '
            f'published={self._publish_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```
