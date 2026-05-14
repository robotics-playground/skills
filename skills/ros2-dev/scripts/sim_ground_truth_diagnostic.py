#!/usr/bin/env python3
"""Ground-truth diagnostic for ROS 2 + Gazebo simulations.

Compares the navigation stack's BELIEF of where the robot is (``/odom`` from a
diff-drive / mecanum / ackermann plugin, or an EKF) against the SIMULATOR'S
ACTUAL PHYSICS POSE. When these two diverge, every downstream symptom you might
chase -- "the laser scan drifts", "the map is corrupted", "the robot won't
navigate" -- is actually caused by this one upstream fault. Catching the
divergence directly, with a number, saves hours of guessing.

WHY THIS IS THE HIGHEST-LEVERAGE SIM DIAGNOSTIC
-----------------------------------------------
A drive plugin computes odometry by integrating wheel joint velocities. If
physics constraints push the chassis around independently of the wheels
(collision drag from a belly plate, a free-floating link, a stuck wheel, an
unstable footprint that tips), the plugin keeps integrating a clean number
while the real body goes somewhere else. ``/odom`` lies. TF built on ``/odom``
lies. SLAM, fed a lying TF, produces a distorted ``map -> odom`` correction.
Foxglove / RViz render all of it faithfully -- faithfully wrong.

The fix is always upstream (the physics setup), but you cannot SEE that without
a side-by-side comparison. This node provides exactly that comparison, runs
continuously, and logs a one-shot WARN the instant divergence appears.

HOW TO GET THE SIMULATOR'S GROUND-TRUTH POSE
--------------------------------------------
Gazebo's SceneBroadcaster publishes every dynamic entity's true world pose on
``/world/<world_name>/dynamic_pose/info`` (gz.msgs.Pose_V). Bridge it to ROS 2.
Caveat worth knowing: the standard ros_gz_bridge Pose_V -> PoseArray / TFMessage
conversions DROP the per-entity name. So you cannot address the robot by name
through the plain bridge -- you address it by INDEX. When the robot is the only
dynamic entity (or the first one spawned), index 0 is reliably the robot's
chassis/base link. If you have several dynamic entities, either (a) make the
robot the first spawned, (b) write a tiny custom Gazebo system plugin that
publishes the named pose on its own topic, or (c) add a model-scoped
pose-publisher plugin to the robot so it emits its own ``/model/<name>/pose``.

Bridge line (ros_gz_bridge YAML):

    - ros_topic_name: "/sim/ground_truth_pose_array"
      gz_topic_name:  "/world/<WORLD_NAME>/dynamic_pose/info"
      ros_type_name:  "geometry_msgs/msg/PoseArray"
      gz_type_name:   "gz.msgs.Pose_V"
      direction:      GZ_TO_ROS

USAGE
-----
Run alongside the sim (e.g. from the sim container's entrypoint), with
use_sim_time enabled so timestamps line up:

    ros2 run <your_pkg> sim_ground_truth_diagnostic --ros-args \
        -p use_sim_time:=true \
        -p odom_topic:=/odom \
        -p ground_truth_topic:=/sim/ground_truth_pose_array \
        -p robot_pose_index:=0 \
        -p pos_threshold_m:=0.10 \
        -p yaw_threshold_deg:=5.0

Then watch the result:

    ros2 topic echo /diagnostic/odom_vs_physics

Output strings look like:

    ok: odom=(+0.010,+0.000,+1.1deg) truth=(+0.009,+0.000,+1.1deg) d_pos=0.001m d_yaw=0.0deg
    WARN: odom=(-0.464,+0.989,-130.4deg) truth=(-0.511,+0.526,-91.6deg) d_pos=0.465m d_yaw=38.9deg

A clean run holds near zero. A WARN that grows while driving is the signature
of physics/odometry decoupling -- go fix the robot's physical footprint
(wheel friction, collision shapes, free-floating links), not the sensors.

This script is intentionally dependency-light (rclpy + standard messages only)
and self-contained so it can be copied into any ROS 2 package's scripts/ or
exposed as a console_scripts entry point.
"""
import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_from_quaternion(q) -> float:
    """Extract the Z-axis (yaw) rotation from a quaternion, in radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _normalize_deg(d: float) -> float:
    """Wrap an angle in degrees into [-180, 180]."""
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class SimGroundTruthDiagnostic(Node):
    """Publishes a continuous comparison of /odom vs the simulator's true pose."""

    def __init__(self) -> None:
        super().__init__("sim_ground_truth_diagnostic")

        # All behaviour is parameter-driven so this node drops into any project.
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("ground_truth_topic", "/sim/ground_truth_pose_array")
        self.declare_parameter("diagnostic_topic", "/diagnostic/odom_vs_physics")
        self.declare_parameter("robot_pose_index", 0)
        self.declare_parameter("pos_threshold_m", 0.10)
        self.declare_parameter("yaw_threshold_deg", 5.0)
        self.declare_parameter("publish_period_s", 1.0)

        self._odom_topic = self.get_parameter("odom_topic").value
        self._truth_topic = self.get_parameter("ground_truth_topic").value
        self._diag_topic = self.get_parameter("diagnostic_topic").value
        self._index = int(self.get_parameter("robot_pose_index").value)
        self._pos_threshold = float(self.get_parameter("pos_threshold_m").value)
        self._yaw_threshold = float(self.get_parameter("yaw_threshold_deg").value)
        period = float(self.get_parameter("publish_period_s").value)

        self._diag_pub = self.create_publisher(String, self._diag_topic, 10)
        self._last_odom: Odometry | None = None
        self._last_truth = None  # geometry_msgs/Pose
        self._divergent = False  # debounce so the WARN logs only on transition

        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.create_subscription(PoseArray, self._truth_topic, self._truth_cb, 10)
        self.create_timer(period, self._publish_diagnostic)

        self.get_logger().info(
            f"sim_ground_truth_diagnostic: comparing '{self._odom_topic}' against "
            f"'{self._truth_topic}'[{self._index}]; thresholds "
            f"{self._pos_threshold:.2f} m / {self._yaw_threshold:.1f} deg"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _truth_cb(self, msg: PoseArray) -> None:
        if len(msg.poses) > self._index:
            self._last_truth = msg.poses[self._index]

    def _publish_diagnostic(self) -> None:
        if self._last_odom is None or self._last_truth is None:
            self._diag_pub.publish(String(
                data="waiting: odom or ground-truth pose not yet received"))
            return

        ox = self._last_odom.pose.pose.position.x
        oy = self._last_odom.pose.pose.position.y
        oyaw = math.degrees(_yaw_from_quaternion(self._last_odom.pose.pose.orientation))

        tx = self._last_truth.position.x
        ty = self._last_truth.position.y
        tyaw = math.degrees(_yaw_from_quaternion(self._last_truth.orientation))

        d_pos = math.hypot(ox - tx, oy - ty)
        d_yaw = abs(_normalize_deg(oyaw - tyaw))

        diverged_now = d_pos > self._pos_threshold or d_yaw > self._yaw_threshold
        status = "WARN" if diverged_now else "ok"

        # Log only on state transitions -- no spam during a sustained fault,
        # and a clear "recovered" message when it clears.
        if diverged_now and not self._divergent:
            self.get_logger().warn(
                f"odom/physics divergence began: d_pos={d_pos:.3f} m "
                f"d_yaw={d_yaw:.1f} deg | odom=({ox:+.3f},{oy:+.3f},{oyaw:+.1f}deg) "
                f"truth=({tx:+.3f},{ty:+.3f},{tyaw:+.1f}deg). "
                f"Root cause is in the robot's physics setup (wheel friction, "
                f"collision shapes, footprint stability) -- not the sensors."
            )
        elif not diverged_now and self._divergent:
            self.get_logger().info(
                f"odom/physics back in agreement: d_pos={d_pos:.3f} m d_yaw={d_yaw:.1f} deg")
        self._divergent = diverged_now

        self._diag_pub.publish(String(data=(
            f"{status}: odom=({ox:+.3f},{oy:+.3f},{oyaw:+.1f}deg) "
            f"truth=({tx:+.3f},{ty:+.3f},{tyaw:+.1f}deg) "
            f"d_pos={d_pos:.3f}m d_yaw={d_yaw:.1f}deg"
        )))


def main() -> None:
    rclpy.init()
    node = SimGroundTruthDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
