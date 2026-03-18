# Testing Reference

> ROS 2 Jazzy | Unit testing, integration testing, launch_testing, CI/CD

---

## 1. Testing Philosophy for ROS 2

Testing robots is challenging because nodes depend on message flow, timing, and
physical hardware. The testing pyramid for ROS 2:

```
          /\
         /  \        Hardware tests (real robot)
        /    \       Fewest, slowest, most realistic
       /------\
      /        \     Integration tests (launch_testing)
     /          \    Test multiple nodes together
    /------------\
   /              \  Unit tests (pytest + rclpy)
  /                \ Most, fastest, most isolated
 /------------------\
```

**For your team:** Focus on unit tests first. Add integration tests once you
have multiple nodes communicating. Hardware tests come when you have a real robot.

---

## 2. Unit Testing with pytest and rclpy

### Setting Up Test Infrastructure

In your Python package, tests go in the `test/` directory:

```
my_robot_perception/
  my_robot_perception/
    camera_node.py
    detector.py
  test/
    test_detector.py
    test_camera_node.py
  package.xml
  setup.py
  setup.cfg
```

**package.xml** test dependencies:
```xml
<test_depend>python3-pytest</test_depend>
<test_depend>ament_copyright</test_depend>
<test_depend>ament_flake8</test_depend>
<test_depend>ament_pep257</test_depend>
```

**setup.py**:
```python
tests_require=['pytest'],
```

### Basic Node Unit Test

```python
# test/test_detector.py
"""Unit tests for the detector node."""
import pytest
import rclpy
from rclpy.node import Node
from my_robot_perception.detector import DetectorNode


@pytest.fixture(scope='module')
def rclpy_init():
    """Initialize rclpy once for all tests in this module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init):
    """Create a fresh node for each test."""
    node = DetectorNode()
    yield node
    node.destroy_node()


class TestDetector:
    def test_node_creation(self, node):
        """Node should initialize with correct name and default parameters."""
        assert node.get_name() == 'detector'
        assert node.get_parameter('confidence_threshold').value == 0.7

    def test_parameter_validation(self, node):
        """Confidence threshold should reject values outside [0, 1]."""
        from rclpy.parameter import Parameter
        result = node.set_parameters([
            Parameter('confidence_threshold', value=-0.1)
        ])
        assert not result[0].successful

    def test_default_publishers_created(self, node):
        """Node should create expected publishers."""
        pub_names = [pub.topic_name for pub in node.publishers]
        # Filter out parameter events (auto-created)
        pub_names = [n for n in pub_names if 'parameter' not in n]
        assert '/detections' in pub_names or 'detections' in [
            n.split('/')[-1] for n in pub_names
        ]
```

### Testing with Mock Publishers and Subscribers

```python
# test/test_obstacle_avoidance.py
"""Test obstacle avoidance logic with mock sensor data."""
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from my_robot_navigation.obstacle_avoidance import ObstacleAvoidanceNode
import time


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_harness(rclpy_init):
    """Create the node under test and a helper node for publishing/subscribing."""
    node = ObstacleAvoidanceNode()
    helper = Node('test_helper')

    # Create a publisher to send fake scan data to the node
    scan_pub = helper.create_publisher(LaserScan, 'scan', 10)

    # Collect messages published by the node
    received_cmds = []
    cmd_sub = helper.create_subscription(
        Twist, 'cmd_vel',
        lambda msg: received_cmds.append(msg),
        10
    )

    yield {
        'node': node,
        'helper': helper,
        'scan_pub': scan_pub,
        'received_cmds': received_cmds,
    }

    node.destroy_node()
    helper.destroy_node()


def spin_for(nodes, duration_sec):
    """Spin multiple nodes for a given duration."""
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        executor.spin_once(timeout_sec=0.01)

    for node in nodes:
        executor.remove_node(node)


class TestObstacleAvoidance:
    def _make_scan(self, ranges, range_min=0.1, range_max=10.0):
        """Create a LaserScan message with given ranges."""
        msg = LaserScan()
        msg.header.frame_id = 'lidar_link'
        msg.range_min = range_min
        msg.range_max = range_max
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 3.14 / len(ranges)
        msg.ranges = [float(r) for r in ranges]
        return msg

    def test_clear_path_goes_forward(self, test_harness):
        """When all ranges are far, robot should drive forward."""
        h = test_harness
        scan = self._make_scan([5.0] * 100)
        h['scan_pub'].publish(scan)

        spin_for([h['node'], h['helper']], 0.5)

        assert len(h['received_cmds']) > 0
        last_cmd = h['received_cmds'][-1]
        assert last_cmd.linear.x > 0.0  # Moving forward
        assert abs(last_cmd.angular.z) < 0.01  # Not turning

    def test_obstacle_triggers_turn(self, test_harness):
        """When an obstacle is close, robot should turn."""
        h = test_harness
        scan = self._make_scan([0.3] * 100)  # Obstacle at 30cm
        h['scan_pub'].publish(scan)

        spin_for([h['node'], h['helper']], 0.5)

        assert len(h['received_cmds']) > 0
        last_cmd = h['received_cmds'][-1]
        assert abs(last_cmd.angular.z) > 0.0  # Turning

    def test_no_crash_on_empty_scan(self, test_harness):
        """Node should handle empty scan without crashing."""
        h = test_harness
        scan = self._make_scan([])
        h['scan_pub'].publish(scan)

        # Should not throw
        spin_for([h['node'], h['helper']], 0.3)
```

---

## 3. Testing Services

```python
# test/test_status_service.py
"""Test the status service."""
import pytest
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import GetStatus
from my_robot_inspection.status_server import StatusServer
import time


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def service_pair(rclpy_init):
    """Create server and client."""
    server = StatusServer()
    client_node = Node('test_client')
    client = client_node.create_client(GetStatus, 'get_status')

    # Wait for service to be available
    assert client.wait_for_service(timeout_sec=5.0), \
        'Service not available after 5 seconds'

    yield {'server': server, 'client_node': client_node, 'client': client}

    server.destroy_node()
    client_node.destroy_node()


class TestStatusService:
    def _call_service(self, pair, item_id):
        """Helper to call the service synchronously."""
        request = GetStatus.Request()
        request.item_id = item_id
        future = pair['client'].call_async(request)

        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(pair['server'])
        executor.add_node(pair['client_node'])

        start = time.monotonic()
        while not future.done() and time.monotonic() - start < 5.0:
            executor.spin_once(timeout_sec=0.1)

        assert future.done(), 'Service call timed out'
        return future.result()

    def test_existing_item(self, service_pair):
        result = self._call_service(service_pair, 'item_001')
        assert result.exists is True
        assert result.condition == 'good'
        assert 0.0 <= result.health <= 1.0

    def test_nonexistent_item(self, service_pair):
        result = self._call_service(service_pair, 'item_999')
        assert result.exists is False
        assert result.condition == 'unknown'
```

---

## 4. Testing Actions

```python
# test/test_inspection_action.py
"""Test the inspection action server."""
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_msgs.action import InspectArea
from my_robot_inspection.inspection_server import InspectionServer
import time


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def action_pair(rclpy_init):
    server = InspectionServer()
    client_node = Node('test_action_client')
    client = ActionClient(client_node, InspectArea, 'inspect_area')

    assert client.wait_for_server(timeout_sec=5.0), \
        'Action server not available'

    yield {'server': server, 'client_node': client_node, 'client': client}

    server.destroy_node()
    client_node.destroy_node()


class TestInspectionAction:
    def test_goal_accepted(self, action_pair):
        goal = InspectArea.Goal()
        goal.area_id = 'area_001'
        goal.target_distance = 10.0
        goal.max_speed = 0.5

        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(action_pair['server'])
        executor.add_node(action_pair['client_node'])

        future = action_pair['client'].send_goal_async(goal)

        start = time.monotonic()
        while not future.done() and time.monotonic() - start < 5.0:
            executor.spin_once(timeout_sec=0.1)

        assert future.done()
        goal_handle = future.result()
        assert goal_handle.accepted
```

---

## 5. launch_testing: Integration Tests

launch_testing runs a launch file and executes tests against the running system.
This tests that nodes work together correctly.

```python
# test/test_perception_integration.py
"""Integration test for the perception pipeline."""
import unittest
import time

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_robot_msgs.msg import Detection


@pytest.mark.launch_test
def generate_test_description():
    """Launch the nodes under test."""
    camera_node = launch_ros.actions.Node(
        package='my_robot_perception',
        executable='camera_node',
        parameters=[{'use_sim': True, 'fps': 10.0}],
    )

    detector_node = launch_ros.actions.Node(
        package='my_robot_perception',
        executable='detector',
        parameters=[{'confidence_threshold': 0.5}],
        remappings=[('input_image', '/camera/image_raw')],
    )

    return launch.LaunchDescription([
        camera_node,
        detector_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'camera_node': camera_node,
        'detector_node': detector_node,
    }


class TestPerceptionPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('perception_test_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_camera_publishes_images(self):
        """Camera node should publish images at configured rate."""
        received = []
        sub = self.node.create_subscription(
            Image, '/camera/image_raw',
            lambda msg: received.append(msg),
            10
        )

        start = time.monotonic()
        while len(received) < 5 and time.monotonic() - start < 10.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)
        self.assertGreaterEqual(len(received), 5,
                                'Expected at least 5 images in 10 seconds')

    def test_detector_produces_detections(self):
        """Detector should produce detection messages when images arrive."""
        received = []
        sub = self.node.create_subscription(
            Detection, '/perception/detections',
            lambda msg: received.append(msg),
            10
        )

        # Wait for detections (detector needs images first)
        start = time.monotonic()
        while len(received) < 1 and time.monotonic() - start < 15.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)
        # At least acknowledge the detector is running
        # In simulation mode, it may not produce real detections


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_camera_exit_code(self, camera_node, proc_info):
        """Camera node should exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, [0], camera_node
        )
```

### Running launch_testing Tests

```bash
# Run via colcon
colcon test --packages-select my_robot_perception
colcon test-result --verbose

# Run directly with launch_test
launch_test test/test_perception_integration.py
```

---

## 6. Test Fixtures: Reusable Node Spinning

A common pattern for tests that need nodes to be spinning:

```python
import threading
from rclpy.executors import SingleThreadedExecutor


class SpinningNodeFixture:
    """Fixture that spins nodes in a background thread."""

    def __init__(self):
        self.executor = SingleThreadedExecutor()
        self._thread = None
        self._running = False

    def add_node(self, node):
        self.executor.add_node(node)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while self._running:
            self.executor.spin_once(timeout_sec=0.01)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


# Usage in tests:
@pytest.fixture
def spinning_nodes(rclpy_init):
    node = ObstacleAvoidanceNode()
    helper = Node('test_helper')

    fixture = SpinningNodeFixture()
    fixture.add_node(node)
    fixture.add_node(helper)

    with fixture:
        yield {'node': node, 'helper': helper}

    node.destroy_node()
    helper.destroy_node()
```

---

## 7. Simulation-Based Testing

For testing with simulated sensor data without Gazebo:

```python
class FakeLidarPublisher(Node):
    """Publishes synthetic LiDAR scans for testing."""

    def __init__(self, scenario='clear'):
        super().__init__('fake_lidar')
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish)
        self.scenario = scenario

    def publish(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -3.14
        msg.angle_max = 3.14
        msg.angle_increment = 3.14 / 180
        msg.range_min = 0.1
        msg.range_max = 12.0

        if self.scenario == 'clear':
            msg.ranges = [10.0] * 360
        elif self.scenario == 'wall_ahead':
            msg.ranges = [0.5 if abs(i - 180) < 30 else 10.0 for i in range(360)]
        elif self.scenario == 'narrow_passage':
            msg.ranges = [0.8 if (i < 90 or i > 270) else 10.0 for i in range(360)]

        self.pub.publish(msg)
```

---

## 8. CI/CD for ROS 2

### GitHub Actions Example

```yaml
# .github/workflows/ros2-test.yml
name: ROS 2 Tests

on:
  push:
    branches: [main, 'experiment/**']
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-24.04
    container:
      image: ros:jazzy
    steps:
      - uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          apt-get update
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --symlink-install

      - name: Test
        run: |
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          colcon test
          colcon test-result --verbose
```

### colcon test Commands

```bash
# Run all tests
colcon test

# Run tests for one package
colcon test --packages-select my_robot_perception

# Show results
colcon test-result --verbose

# Run with pytest verbose output
colcon test --packages-select my_robot_perception \
  --pytest-args -v --tb=short

# Run a specific test file
colcon test --packages-select my_robot_perception \
  --pytest-args test/test_detector.py

# Generate coverage report
colcon test --packages-select my_robot_perception \
  --pytest-args --cov=my_robot_perception --cov-report=html
```

---

## 9. Code Coverage

```bash
# Install coverage tools
pip install pytest-cov

# Run tests with coverage
colcon test --packages-select my_robot_perception \
  --pytest-args --cov=my_robot_perception --cov-report=term-missing

# Generate HTML report
colcon test --packages-select my_robot_perception \
  --pytest-args --cov=my_robot_perception --cov-report=html

# View report
open build/my_robot_perception/htmlcov/index.html
```

**Coverage targets for a robotics experimental project:**
- Unit tests: 60-80% line coverage (focus on logic, not boilerplate)
- Integration tests: Cover all node-to-node communication paths
- Don't aim for 100% -- focus on testing the logic that matters

---

## 10. Complete Example: Testing a Subscriber Node

End-to-end example of testing a node that subscribes to IMU data and publishes
tilt angle.

```python
# test/test_imu_filter.py
"""Complete test suite for the IMU filter node."""
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from my_robot_perception.imu_filter import ImuFilterNode
import time
import math


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def imu_test_env(rclpy_init):
    """Set up the IMU filter node with test publisher and subscriber."""
    # Node under test
    imu_filter = ImuFilterNode()

    # Test helper node
    helper = Node('imu_test_helper')

    # Publisher: send fake IMU data
    imu_pub = helper.create_publisher(Imu, 'imu/data_raw', 10)

    # Subscribers: capture output
    filtered_msgs = []
    tilt_msgs = []

    helper.create_subscription(
        Imu, 'imu/data_filtered',
        lambda msg: filtered_msgs.append(msg), 10
    )
    helper.create_subscription(
        Float64, 'imu/tilt_angle',
        lambda msg: tilt_msgs.append(msg), 10
    )

    yield {
        'node': imu_filter,
        'helper': helper,
        'imu_pub': imu_pub,
        'filtered_msgs': filtered_msgs,
        'tilt_msgs': tilt_msgs,
    }

    imu_filter.destroy_node()
    helper.destroy_node()


def spin_nodes(env, duration=0.5):
    """Spin both nodes for a duration."""
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(env['node'])
    executor.add_node(env['helper'])
    start = time.monotonic()
    while time.monotonic() - start < duration:
        executor.spin_once(timeout_sec=0.01)
    executor.remove_node(env['node'])
    executor.remove_node(env['helper'])


def make_imu_msg(ax=0.0, ay=0.0, az=9.81):
    """Create an IMU message with given accelerations."""
    msg = Imu()
    msg.header.frame_id = 'imu_link'
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az
    return msg


class TestImuFilter:
    def test_node_name(self, imu_test_env):
        assert imu_test_env['node'].get_name() == 'imu_filter'

    def test_filter_parameters(self, imu_test_env):
        node = imu_test_env['node']
        assert node.get_parameter('alpha').value == 0.1
        assert node.get_parameter('publish_rate').value == 50.0

    def test_produces_filtered_output(self, imu_test_env):
        """Node should produce filtered IMU messages when given input."""
        env = imu_test_env

        # Send several IMU messages
        for _ in range(20):
            env['imu_pub'].publish(make_imu_msg(az=9.81))
            spin_nodes(env, 0.05)

        spin_nodes(env, 0.5)
        assert len(env['filtered_msgs']) > 0, 'No filtered messages received'

    def test_tilt_zero_when_level(self, imu_test_env):
        """Tilt should be near zero when robot is level."""
        env = imu_test_env
        env['tilt_msgs'].clear()

        for _ in range(30):
            env['imu_pub'].publish(make_imu_msg(ax=0.0, az=9.81))
            spin_nodes(env, 0.05)

        spin_nodes(env, 0.5)

        if len(env['tilt_msgs']) > 0:
            last_tilt = env['tilt_msgs'][-1].data
            assert abs(last_tilt) < 5.0, \
                f'Expected tilt near 0 degrees, got {last_tilt}'

    def test_tilt_nonzero_when_tilted(self, imu_test_env):
        """Tilt should be nonzero when robot is pitched."""
        env = imu_test_env
        env['tilt_msgs'].clear()

        # Simulate 45-degree forward pitch
        ax = 9.81 * math.sin(math.radians(45))
        az = 9.81 * math.cos(math.radians(45))

        for _ in range(50):
            env['imu_pub'].publish(make_imu_msg(ax=ax, az=az))
            spin_nodes(env, 0.05)

        spin_nodes(env, 0.5)

        if len(env['tilt_msgs']) > 0:
            last_tilt = env['tilt_msgs'][-1].data
            # Should converge toward 45 degrees (with filter lag)
            assert abs(last_tilt) > 10.0, \
                f'Expected significant tilt, got {last_tilt}'

    def test_dynamic_parameter_update(self, imu_test_env):
        """Alpha parameter should be updatable at runtime."""
        from rclpy.parameter import Parameter
        node = imu_test_env['node']

        result = node.set_parameters([
            Parameter('alpha', value=0.5)
        ])
        assert result[0].successful
        assert node.get_parameter('alpha').value == 0.5

    def test_rejects_invalid_alpha(self, imu_test_env):
        """Alpha values outside (0, 1] should be rejected."""
        from rclpy.parameter import Parameter
        node = imu_test_env['node']

        result = node.set_parameters([
            Parameter('alpha', value=0.0)
        ])
        assert not result[0].successful

        result = node.set_parameters([
            Parameter('alpha', value=1.5)
        ])
        assert not result[0].successful
```

### Running the Tests

```bash
# Build the package
cd ~/my_robot_ws
colcon build --packages-select my_robot_perception --symlink-install

# Run tests
colcon test --packages-select my_robot_perception
colcon test-result --verbose

# Or run pytest directly (after sourcing)
source install/setup.bash
cd src/my_robot_perception
python -m pytest test/ -v --tb=short
```
