# Communication Patterns Reference

> ROS 2 Jazzy | Topics, Services, Actions, QoS, Custom Messages

---

## Overview: Three Communication Patterns

ROS 2 provides three communication patterns. Choosing the right one is critical.

| Pattern | Direction | Use When |
|---|---|---|
| **Topic** | Many-to-many, async | Continuous data streams, events, sensor data |
| **Service** | One-to-one, sync | Occasional request/reply, configuration queries |
| **Action** | One-to-one, async with feedback | Long-running tasks with progress updates |

**Decision flowchart:**
```
Is data flowing continuously?
├── YES --> Topic
└── NO --> Is the operation instant (< 1 second)?
           ├── YES --> Service
           └── NO --> Action (with feedback)
```

---

## 1. Topics: Publishers and Subscribers

Topics are the primary communication mechanism. Data flows from publishers to
subscribers through named channels. Publishers don't know who subscribes.
Subscribers don't know who publishes. This is the key decoupling.

### Publisher

```python
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(
            Twist,           # Message type
            'cmd_vel',       # Topic name
            10               # QoS depth (keep last 10 messages)
        )
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5   # 0.5 m/s forward
        msg.angular.z = 0.0   # no rotation
        self.publisher.publish(msg)
```

### Subscriber

```python
from sensor_msgs.msg import LaserScan

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,          # Message type
            'scan',             # Topic name
            self.scan_callback, # Callback function
            10                  # QoS depth
        )

    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')
```

### Common Message Types

| Package | Message | Contains | Used For |
|---|---|---|---|
| `std_msgs` | `String`, `Int32`, `Float64`, `Bool` | Single primitive value | Simple data, debugging |
| `std_msgs` | `Header` | stamp + frame_id | Timestamp and coordinate frame |
| `geometry_msgs` | `Twist` | linear + angular velocity | Robot velocity commands |
| `geometry_msgs` | `Pose` | position + orientation | Robot/object position |
| `geometry_msgs` | `PoseStamped` | Header + Pose | Timestamped position |
| `geometry_msgs` | `TransformStamped` | Header + child_frame + Transform | tf2 transforms |
| `sensor_msgs` | `LaserScan` | ranges, angles, intensities | 2D LiDAR data |
| `sensor_msgs` | `PointCloud2` | 3D point cloud | 3D LiDAR, depth camera |
| `sensor_msgs` | `Image` | pixel data, encoding | Camera images |
| `sensor_msgs` | `Imu` | orientation, angular vel, linear accel | IMU sensor data |
| `sensor_msgs` | `JointState` | joint positions, velocities, efforts | Robot joint state |
| `nav_msgs` | `Odometry` | Pose + Twist with covariance | Robot odometry |
| `nav_msgs` | `OccupancyGrid` | 2D grid map | Mapping |
| `nav_msgs` | `Path` | array of PoseStamped | Planned path |

**Rule:** Always use existing message types before creating custom ones. Check
[sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/index.html),
[geometry_msgs](https://docs.ros2.org/latest/api/geometry_msgs/index.html), and
[nav_msgs](https://docs.ros2.org/latest/api/nav_msgs/index.html) first.

---

## 2. Services: Request/Reply

Services are for synchronous, one-shot operations. The client sends a request,
the server processes it, and returns a response. **Do NOT use services for
continuous data** -- use topics instead.

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client (Async -- Preferred)

```python
from example_interfaces.srv import AddTwoInts

class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for addition service...')

    def send_request(self, a: int, b: int):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
```

### When to Use Services vs Topics

| Use a Service | Use a Topic |
|---|---|
| Querying robot configuration | Streaming sensor data |
| Triggering a one-shot action (save map, calibrate) | Publishing velocity commands |
| Requesting a computation result | Broadcasting state (battery, temperature) |
| Setting a mode (manual/auto) | Sending periodic heartbeats |
| Getting a parameter from another node | Distributing processed data |

**Warning:** Never call a service from inside a subscriber callback without
using async -- it will deadlock the executor.

---

## 3. Actions: Long-Running Tasks with Feedback

Actions handle tasks that take time and need progress reporting. They combine
the request/response of services with the streaming of topics.

Actions work like submitting a job to a task queue -- you get a handle, receive
progress updates, and eventually get the result. This is built into the framework.

### Action Server

```python
import asyncio
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import time

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """Accept or reject a goal."""
        self.get_logger().info('Received navigation goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancellation."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the navigation goal with feedback."""
        self.get_logger().info('Executing navigation...')
        feedback = NavigateToPose.Feedback()

        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation canceled')
                return NavigateToPose.Result()

            # Simulate progress
            feedback.distance_remaining = float(10 - i)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}m')
            await asyncio.sleep(1.0)  # Non-blocking sleep

        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result
```

### Action Client

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def send_goal(self, x: float, y: float):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist:.1f}m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete')
```

---

## 4. QoS (Quality of Service) Profiles

QoS controls the reliability and behavior of message delivery. **Getting QoS
wrong is the #1 cause of "my subscriber isn't receiving anything."**

### QoS Parameters

| Parameter | Options | Default | Effect |
|---|---|---|---|
| **Reliability** | `RELIABLE` / `BEST_EFFORT` | `RELIABLE` | Reliable retransmits lost messages; Best Effort drops them |
| **Durability** | `VOLATILE` / `TRANSIENT_LOCAL` | `VOLATILE` | Transient Local sends last message to late-joining subscribers |
| **History** | `KEEP_LAST` / `KEEP_ALL` | `KEEP_LAST` | Keep Last discards old messages when depth is exceeded |
| **Depth** | Integer | 10 | How many messages to keep in the queue |
| **Deadline** | Duration | Infinite | Max time between messages before deadline missed event |
| **Liveliness** | `AUTOMATIC` / `MANUAL` | `AUTOMATIC` | How to detect if a publisher is alive |
| **Lifespan** | Duration | Infinite | How long a message is valid |

### QoS Compatibility Rules

**CRITICAL:** Publisher and subscriber QoS must be compatible or communication
silently fails. No error. No warning. Just silence.

| Publisher | Subscriber | Result |
|---|---|---|
| Reliable | Reliable | Works |
| Reliable | Best Effort | Works (subscriber gets less guarantees than offered) |
| Best Effort | Best Effort | Works |
| Best Effort | Reliable | **FAILS SILENTLY** -- subscriber demands more than publisher offers |
| Transient Local | Transient Local | Works (late joiners get last message) |
| Transient Local | Volatile | Works |
| Volatile | Volatile | Works |
| Volatile | Transient Local | **FAILS SILENTLY** |

**Rule:** The subscriber can request EQUAL or LESS strict QoS than the publisher.
A subscriber asking for MORE guarantees than the publisher offers = no data.

### Predefined QoS Profiles

```python
from rclpy.qos import (
    qos_profile_sensor_data,    # Best Effort, Volatile, depth=5
    qos_profile_system_default, # Defers all settings to RMW implementation
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

# For sensor data (camera, LiDAR, IMU) -- drop old data rather than block
sensor_sub = self.create_subscription(
    LaserScan, 'scan', self.callback,
    qos_profile_sensor_data
)

# For commands and important data -- ensure delivery
cmd_pub = self.create_publisher(
    Twist, 'cmd_vel',
    qos_profile_system_default
)

# For latched data (like map) -- late subscribers get the last message
map_pub = self.create_publisher(
    OccupancyGrid, 'map',
    QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1
    )
)
```

### Debugging QoS Mismatches

```bash
# Check QoS of a topic
ros2 topic info /scan -v
# Look at the "QoS profile" section for publisher and subscriber

# Force echo with specific QoS
ros2 topic echo /scan --qos-reliability best_effort
ros2 topic echo /map --qos-durability transient_local
```

---

## 5. Custom Message Definitions

When standard messages don't fit, create custom ones. Custom messages live in
a dedicated package.

### Package Structure

```
my_robot_msgs/
  msg/
    Detection.msg
    Report.msg
  srv/
    AnalyzeArea.srv
  action/
    InspectArea.action
  CMakeLists.txt
  package.xml
```

### Message Definition (.msg)

```
# my_robot_msgs/msg/Detection.msg
# Represents a detected target object during inspection

std_msgs/Header header

# Detection classification
uint8 CLASS_A=0
uint8 CLASS_B=1
uint8 CLASS_C=2
uint8 CLASS_D=3
uint8 CLASS_E=4

uint8 detection_type
float32 confidence      # 0.0 to 1.0
string description

# Location relative to robot path
float64 distance_from_start  # meters along path
float32 angular_position     # radians
float32 extent_length        # meters

# Bounding box in image (if from camera)
uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height
```

### Service Definition (.srv)

```
# my_robot_msgs/srv/AnalyzeArea.srv
# Request analysis of a section

# Request
string zone_id
float64 start_distance
float64 end_distance
---
# Response
bool success
string message
my_robot_msgs/Detection[] detections
```

### Action Definition (.action)

```
# my_robot_msgs/action/InspectArea.action
# Long-running inspection of an area

# Goal
string area_id
float64 target_distance
float32 max_speed
---
# Result
bool success
my_robot_msgs/Detection[] detections_found
float64 total_distance_covered
---
# Feedback
float64 current_distance
float32 current_speed
float32 battery_percent
uint32 detections_found_so_far
```

### Building Custom Messages

**package.xml** must include:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<!-- Dependencies on other message packages used in your definitions -->
<depend>std_msgs</depend>
```

**CMakeLists.txt:**
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "srv/AnalyzeArea.srv"
  "action/InspectArea.action"
  DEPENDENCIES std_msgs
)
```

**After building**, use in Python:
```python
from my_robot_msgs.msg import Detection
from my_robot_msgs.srv import AnalyzeArea
from my_robot_msgs.action import InspectArea
```

### Message Design Best Practices

1. **Always include a Header** for stamped data -- it provides timestamp and frame_id
2. **Use constants for enums** (uint8 with named constants in .msg file)
3. **Use SI units** (meters, radians, seconds) per REP 103
4. **Keep messages focused** -- one message per concept
5. **Document fields with comments** in the .msg file
6. **Prefer existing types** -- use `geometry_msgs/Pose` instead of reinventing x/y/z/quat
7. **Version your message package** independently from node packages

---

## 6. Serialization Considerations

ROS 2 uses CDR (Common Data Representation) serialization by default (part of the
DDS standard). You generally don't need to think about serialization, but know these:

- **Fixed-size messages** (no strings, no variable-length arrays) are fastest to serialize
- **Large messages** (images, point clouds) benefit from intra-process communication
  (zero-copy) via composable nodes -- see `references/launch-files.md`
- **Cross-language compatibility** is automatic -- a Python publisher and C++ subscriber
  just work, because both serialize to CDR
- **Avoid deeply nested messages** -- flat structures serialize faster
- For **very high throughput**, consider using `sensor_msgs/PointCloud2` with its
  built-in binary data field rather than arrays of individual points

---

## 7. Complete Example: Custom Service with Client and Server

A complete working example of defining and using a custom service.

### Service Definition

```
# my_robot_msgs/srv/GetZoneStatus.srv
string zone_id
---
bool exists
float32 health_score      # 0.0 to 1.0
float32 coverage_rate     # meters per second
string condition           # "good", "warning", "critical"
uint32 detection_count
```

### Service Server Node

```python
#!/usr/bin/env python3
"""Zone status service server."""
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import GetZoneStatus


class ZoneStatusServer(Node):
    def __init__(self):
        super().__init__('zone_status_server')
        self.srv = self.create_service(
            GetZoneStatus,
            'get_zone_status',
            self.handle_request
        )
        # Simulated zone database
        self._zones = {
            'zone_001': {'health': 0.9, 'coverage': 1.5, 'condition': 'good', 'detections': 0},
            'zone_002': {'health': 0.5, 'coverage': 0.8, 'condition': 'warning', 'detections': 2},
            'zone_003': {'health': 0.2, 'coverage': 0.3, 'condition': 'critical', 'detections': 7},
        }
        self.get_logger().info('Zone status server ready')

    def handle_request(self, request, response):
        zone_id = request.zone_id
        self.get_logger().info(f'Status request for: {zone_id}')

        if zone_id in self._zones:
            data = self._zones[zone_id]
            response.exists = True
            response.health_score = data['health']
            response.coverage_rate = data['coverage']
            response.condition = data['condition']
            response.detection_count = data['detections']
        else:
            response.exists = False
            response.health_score = 0.0
            response.coverage_rate = 0.0
            response.condition = 'unknown'
            response.detection_count = 0

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ZoneStatusServer()
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

### Service Client Node

```python
#!/usr/bin/env python3
"""Zone status service client."""
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import GetZoneStatus


class ZoneStatusClient(Node):
    def __init__(self):
        super().__init__('zone_status_client')
        self.client = self.create_client(GetZoneStatus, 'get_zone_status')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for zone_status_server...')
        self.get_logger().info('Connected to zone status server')

    def query_zone(self, zone_id: str):
        request = GetZoneStatus.Request()
        request.zone_id = zone_id
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.exists:
            self.get_logger().info(
                f'Zone {zone_id}: health={result.health_score:.0%}, '
                f'coverage={result.coverage_rate:.1f} m/s, '
                f'condition={result.condition}, '
                f'detections={result.detection_count}'
            )
        else:
            self.get_logger().warn(f'Zone {zone_id} not found')
        return result


def main(args=None):
    rclpy.init(args=args)
    client = ZoneStatusClient()
    for zone in ['zone_001', 'zone_002', 'zone_003', 'zone_999']:
        client.query_zone(zone)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
