---
title: "ROS 2 Testing Standards"
category: "robotics"
tags: ["ros2", "testing", "pytest", "launch-testing", "integration", "ci-cd", "colcon"]
description: "Testing standards for ROS 2 packages including unit tests, node integration tests, simulation-based testing, and CI/CD patterns"
last_updated: "2026-03-18"
---

# ROS 2 Testing Standards

> **Philosophy:** Test the logic, not the framework. Pure computation gets unit tests. Node integration gets launch tests. Hardware gets simulation tests. Never ship untested nodes.

## Testing Pyramid for ROS 2

| Level | Tool | What to Test | Speed |
|-------|------|-------------|-------|
| Unit tests | `pytest` | Pure logic, algorithms, calculations | Fast (ms) |
| Node tests | `launch_testing` | Node behavior, pub/sub, services | Medium (s) |
| Integration tests | `launch_testing` + sim | Multi-node systems, TF chains | Slow (10s+) |
| Simulation tests | Gazebo + launch_testing | Full system with simulated hardware | Very slow (min) |

## Coverage Requirements

| Package Type | Minimum Coverage | Target |
|-------------|-----------------|--------|
| Algorithm/processing packages | 80% | 90%+ |
| Node packages (callbacks) | 60% | 80% |
| Driver packages | 40% (logic only) | 60% |
| Launch/config packages | Not required | — |

## Unit Testing with pytest

Unit tests cover pure logic extracted from nodes. No ROS 2 dependencies needed.

### Test File Organization

```
my_robot_inspection/
├── my_robot_inspection/
│   ├── scan_filter.py           # Pure logic
│   └── geometry_helpers.py      # Pure logic
└── test/
    ├── test_scan_filter.py      # Unit test
    ├── test_geometry_helpers.py  # Unit test
    ├── test_inspector_node.py   # Node integration test
    └── conftest.py              # Shared fixtures
```

**Naming rules:**
- Test files: `test_<module_name>.py`
- Test functions: `test_<behavior_description>()`
- Test classes: `Test<ClassName>`

### Unit Test Example

```python
"""Tests for scan_filter module."""

import math
import pytest
from my_robot_inspection.scan_filter import ScanFilter


class TestScanFilter:
    """Tests for the ScanFilter class."""

    def setup_method(self):
        """Create a default ScanFilter for each test."""
        self.target = ScanFilter(min_range=0.1, max_range=30.0)

    def test_filter_removes_values_below_min_range(self):
        """Valid ranges below min_range are excluded."""
        # Arrange
        raw_ranges = [0.05, 0.5, 1.0, 2.0]

        # Act
        result = self.target.filter(raw_ranges)

        # Assert
        assert result == [0.5, 1.0, 2.0]

    def test_filter_removes_inf_values(self):
        """Infinite values (no return) are excluded."""
        # Arrange
        raw_ranges = [1.0, float('inf'), 2.0, float('inf')]

        # Act
        result = self.target.filter(raw_ranges)

        # Assert
        assert result == [1.0, 2.0]

    def test_filter_removes_nan_values(self):
        """NaN values are excluded."""
        # Arrange
        raw_ranges = [1.0, float('nan'), 2.0]

        # Act
        result = self.target.filter(raw_ranges)

        # Assert
        assert result == [1.0, 2.0]

    def test_filter_returns_empty_for_all_invalid(self):
        """Empty list returned when all values are invalid."""
        # Arrange
        raw_ranges = [0.0, float('inf'), float('nan')]

        # Act
        result = self.target.filter(raw_ranges)

        # Assert
        assert result == []

    def test_filter_handles_empty_input(self):
        """Empty input returns empty output."""
        assert self.target.filter([]) == []

    def test_constructor_rejects_negative_min_range(self):
        """Negative min_range raises ValueError."""
        with pytest.raises(ValueError, match='min_range must be non-negative'):
            ScanFilter(min_range=-1.0, max_range=30.0)
```

### Fixtures (conftest.py)

```python
"""Shared test fixtures for my_robot_inspection tests."""

import pytest
from sensor_msgs.msg import LaserScan


@pytest.fixture
def sample_laser_scan() -> LaserScan:
    """Create a sample LaserScan message for testing."""
    msg = LaserScan()
    msg.angle_min = -1.57
    msg.angle_max = 1.57
    msg.angle_increment = 0.01
    msg.range_min = 0.1
    msg.range_max = 30.0
    msg.ranges = [1.0, 2.0, 3.0, float('inf'), 0.05]
    return msg


@pytest.fixture
def scan_filter():
    """Create a default ScanFilter instance."""
    from my_robot_inspection.scan_filter import ScanFilter
    return ScanFilter(min_range=0.1, max_range=30.0)
```

## Node Testing with launch_testing

Node tests verify that a ROS 2 node behaves correctly: publishes expected topics, responds to services, handles parameters.

```python
"""Integration test for ScanProcessorNode."""

import unittest
import time

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


@launch_testing.parametrize('test_args', [
    ({'min_range': '0.1', 'max_range': '30.0'}),
])
def generate_test_description(test_args):
    """Launch the node under test."""
    node_under_test = launch_ros.actions.Node(
        package='my_robot_inspection',
        executable='scan_processor',
        name='scan_processor',
        parameters=[{
            'min_range': float(test_args['min_range']),
            'max_range': float(test_args['max_range']),
        }],
    )
    return launch.LaunchDescription([
        node_under_test,
        launch_testing.actions.ReadyToTest(),
    ]), {'node_under_test': node_under_test}


class TestScanProcessor(unittest.TestCase):
    """Test the ScanProcessorNode via ROS 2 communication."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = Node('test_scan_processor')
        self.received_msgs = []
        self.sub = self.test_node.create_subscription(
            Float32, '/scan_processor/min_distance',
            lambda msg: self.received_msgs.append(msg), 10
        )
        self.pub = self.test_node.create_publisher(LaserScan, 'scan', 10)

    def tearDown(self):
        self.test_node.destroy_node()

    def test_publishes_min_distance(self):
        """Node publishes minimum distance from valid scan ranges."""
        # Arrange
        scan_msg = LaserScan()
        scan_msg.ranges = [1.0, 0.5, 2.0, 3.0]

        # Act — publish and spin to allow processing
        end_time = time.time() + 5.0
        while time.time() < end_time and len(self.received_msgs) == 0:
            self.pub.publish(scan_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Assert
        assert len(self.received_msgs) > 0, 'No min_distance messages received'
        assert self.received_msgs[-1].data == pytest.approx(0.5, abs=0.01)
```

## Mock Publishers and Subscribers

For unit testing node logic without full ROS 2 infrastructure:

```python
"""Testing node logic with mock ROS 2 primitives."""

from unittest.mock import MagicMock, patch
import pytest


class TestNodeLogic:
    """Test node methods by mocking ROS 2 primitives."""

    def test_scan_callback_publishes_min_distance(self):
        """Callback computes and publishes minimum distance."""
        # Arrange
        node = ScanProcessorNode.__new__(ScanProcessorNode)
        node._min_range = 0.1
        node._max_range = 30.0
        node._min_dist_pub = MagicMock()
        node.get_logger = MagicMock(return_value=MagicMock())

        scan = MagicMock()
        scan.ranges = [1.0, 0.5, 2.0]

        # Act
        node._scan_callback(scan)

        # Assert
        node._min_dist_pub.publish.assert_called_once()
        published_msg = node._min_dist_pub.publish.call_args[0][0]
        assert published_msg.data == pytest.approx(0.5)
```

## Testing Services and Actions

```python
def test_trigger_service_returns_success(self):
    """Service returns success when inspection can be triggered."""
    # Arrange
    client = self.test_node.create_client(Trigger, '/trigger_inspection')
    assert client.wait_for_service(timeout_sec=5.0), 'Service not available'

    # Act
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

    # Assert
    response = future.result()
    assert response.success is True
    assert 'started' in response.message.lower()
```

## colcon test Integration

### Running Tests

```bash
# Run all tests
cd ~/ros2_ws && colcon test

# Run tests for specific package
colcon test --packages-select my_robot_inspection

# Show test results
colcon test-result --verbose

# Run with pytest verbose output
colcon test --packages-select my_robot_inspection --pytest-args '-v'

# Run specific test file
colcon test --packages-select my_robot_inspection \
  --pytest-args 'test/test_scan_filter.py -v'
```

### Package Configuration for Tests

In `setup.py`, tests are discovered automatically from the `test/` directory. Ensure `test_depend` entries are in `package.xml`:

```xml
<test_depend>python3-pytest</test_depend>
<test_depend>launch_testing</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
```

## CI/CD Patterns for ROS 2

### GitHub Actions Workflow

```yaml
name: ROS 2 CI
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    container:
      image: ros:humble
    steps:
      - uses: actions/checkout@v4
        with:
          path: ros2_ws/src/my_robot

      - name: Install dependencies
        run: |
          apt-get update
          cd ros2_ws
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build
        run: |
          source /opt/ros/humble/setup.bash
          cd ros2_ws
          colcon build --symlink-install

      - name: Test
        run: |
          source /opt/ros/humble/setup.bash
          source ros2_ws/install/setup.bash
          cd ros2_ws
          colcon test --return-code-on-test-failure
          colcon test-result --verbose
```

## Performance Testing

```python
"""Performance test for scan processing latency."""

import time
import statistics


class TestScanProcessorPerformance:
    """Verify scan processing meets timing requirements."""

    def test_scan_processing_under_10ms(self, scan_filter, sample_laser_scan):
        """Single scan processing must complete within 10ms."""
        durations = []
        for _ in range(1000):
            start = time.perf_counter()
            scan_filter.filter(sample_laser_scan.ranges)
            durations.append(time.perf_counter() - start)

        mean_ms = statistics.mean(durations) * 1000
        p99_ms = sorted(durations)[int(0.99 * len(durations))] * 1000

        assert mean_ms < 5.0, f'Mean processing time {mean_ms:.2f}ms exceeds 5ms'
        assert p99_ms < 10.0, f'P99 processing time {p99_ms:.2f}ms exceeds 10ms'
```

## Simulation-Based Testing

For tests that require a full simulated environment, see [simulation-workflow.md](simulation-workflow.md).

```python
def generate_test_description():
    """Launch Gazebo + robot for integration testing."""
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r -s empty.sdf'}.items(),  # Headless
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'), 'launch', 'robot_sim.launch.py'
            ])
        ]),
    )
    return LaunchDescription([gazebo, robot, ReadyToTest()])
```

## Anti-Patterns

| Anti-Pattern | Problem | Correct Approach |
|-------------|---------|-----------------|
| Testing ROS 2 framework behavior | Wastes time, already tested by maintainers | Test YOUR logic only |
| `time.sleep()` for synchronization | Flaky, slow | Use `spin_until_future_complete` or event-based waits |
| No test isolation | Tests affect each other | Fresh node per test in `setUp` |
| Testing only happy path | Misses real failures | Test error cases, edge cases, timeouts |
| Skipping unit tests for "ROS stuff" | Logic bugs ship untested | Extract logic into pure functions, unit test those |
| Hardcoded timeouts | Fails on slow CI | Use generous timeouts, parameterize |

## Quick Checklist

- [ ] Pure logic extracted into testable functions/classes
- [ ] Unit tests with pytest for all algorithm code (80%+ coverage)
- [ ] Node integration tests with launch_testing for pub/sub behavior
- [ ] Test files named `test_<module>.py` in `test/` directory
- [ ] Arrange-Act-Assert pattern with comments in every test
- [ ] Fixtures in `conftest.py` for shared test data
- [ ] `test_depend` entries in `package.xml` for all test dependencies
- [ ] Tests pass with `colcon test --packages-select <pkg>`
- [ ] No `time.sleep()` — use event-based synchronization
- [ ] Performance tests for latency-critical callbacks
- [ ] CI pipeline runs tests on every push
