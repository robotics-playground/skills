#!/usr/bin/env python3
"""One-shot sim motion sanity check: command a motion, verify physics agrees.

This is the FIRST diagnostic to run when anything seems wrong with a simulated
robot's motion, mapping, or sensor data. It answers a single question with hard
numbers: "When I command the robot to move, does the simulator actually move it
the way the odometry claims?"

It is deliberately a short, run-it-and-read-it script rather than a long-lived
node -- use it interactively while debugging, then move on.

WHAT IT DOES
------------
1. Records the starting /odom pose AND the simulator's ground-truth pose.
2. Commands a fixed /cmd_vel for a few seconds (forward, then rotate).
3. Records the ending poses.
4. Prints a side-by-side comparison and a verdict.

If odom and ground-truth agree, motion is sound -- look elsewhere for the bug
(Foxglove display frame, QoS, TF timing). If they disagree, the robot's physics
setup is the problem and no amount of sensor/TF tweaking will fix it.

PREREQUISITES
-------------
- Ground-truth pose bridged to a PoseArray topic (see
  sim_ground_truth_diagnostic.py docstring for the bridge line).
- use_sim_time:=true so timestamps align with the simulator clock.

USAGE
-----
    ros2 run <your_pkg> sim_motion_check --ros-args \
        -p use_sim_time:=true \
        -p ground_truth_topic:=/sim/ground_truth_pose_array \
        -p robot_pose_index:=0

INTERPRETING THE OUTPUT
-----------------------
- "MOTION SOUND"      odom tracks physics within tolerance -> bug is downstream
                      (rendering, QoS, TF timing, SLAM tuning).
- "MOTION DECOUPLED"  odom and physics diverged -> fix the robot model's physics
                      (wheel friction/radius/separation, collision shapes,
                      footprint stability, free-floating links).
- "ROBOT DID NOT MOVE" physics pose barely changed despite a drive command ->
                      the robot is stuck (collision with world, jammed joint,
                      zero wheel friction) -- odom may still be integrating a
                      phantom motion.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _norm_deg(d: float) -> float:
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class SimMotionCheck(Node):
    def __init__(self) -> None:
        super().__init__("sim_motion_check")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("ground_truth_topic", "/sim/ground_truth_pose_array")
        self.declare_parameter("robot_pose_index", 0)
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("segment_seconds", 2.0)
        self.declare_parameter("pos_tolerance_m", 0.10)
        self.declare_parameter("yaw_tolerance_deg", 10.0)

        self._index = int(self.get_parameter("robot_pose_index").value)
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self._odom = None
        self._truth = None
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._odom_cb, 10)
        self.create_subscription(
            PoseArray, self.get_parameter("ground_truth_topic").value,
            self._truth_cb, 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _truth_cb(self, msg: PoseArray) -> None:
        if len(msg.poses) > self._index:
            self._truth = msg.poses[self._index]

    def _spin_for(self, seconds: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _drive(self, lin: float, ang: float, seconds: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self._cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Stop and let physics settle.
        for _ in range(20):
            self._cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        self._spin_for(0.5)

    def _snapshot(self):
        ox = self._odom.pose.pose.position.x
        oy = self._odom.pose.pose.position.y
        oyaw = math.degrees(_yaw(self._odom.pose.pose.orientation))
        tx = self._truth.position.x
        ty = self._truth.position.y
        tyaw = math.degrees(_yaw(self._truth.orientation))
        return (ox, oy, oyaw), (tx, ty, tyaw)

    def run(self) -> None:
        lin = float(self.get_parameter("linear_speed").value)
        ang = float(self.get_parameter("angular_speed").value)
        seg = float(self.get_parameter("segment_seconds").value)
        pos_tol = float(self.get_parameter("pos_tolerance_m").value)
        yaw_tol = float(self.get_parameter("yaw_tolerance_deg").value)

        print("Waiting for /odom and ground-truth pose ...")
        t0 = time.monotonic()
        while (self._odom is None or self._truth is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > 10.0:
                print("ERROR: never received both /odom and the ground-truth "
                      "pose. Check the topics exist and the bridge is running.")
                return

        odom_start, truth_start = self._snapshot()
        print(f"START  odom=({odom_start[0]:+.3f},{odom_start[1]:+.3f},"
              f"{odom_start[2]:+.1f}deg)  truth=({truth_start[0]:+.3f},"
              f"{truth_start[1]:+.3f},{truth_start[2]:+.1f}deg)")

        print(f"Driving forward {lin} m/s for {seg}s ...")
        self._drive(lin, 0.0, seg)
        print(f"Rotating {ang} rad/s for {seg}s ...")
        self._drive(0.0, ang, seg)

        odom_end, truth_end = self._snapshot()
        print(f"END    odom=({odom_end[0]:+.3f},{odom_end[1]:+.3f},"
              f"{odom_end[2]:+.1f}deg)  truth=({truth_end[0]:+.3f},"
              f"{truth_end[1]:+.3f},{truth_end[2]:+.1f}deg)")

        # Deltas
        odom_dpos = math.hypot(odom_end[0] - odom_start[0],
                               odom_end[1] - odom_start[1])
        truth_dpos = math.hypot(truth_end[0] - truth_start[0],
                                truth_end[1] - truth_start[1])
        odom_dyaw = abs(_norm_deg(odom_end[2] - odom_start[2]))
        truth_dyaw = abs(_norm_deg(truth_end[2] - truth_start[2]))

        # How far apart are odom and physics at the end?
        final_pos_gap = math.hypot(odom_end[0] - truth_end[0],
                                   odom_end[1] - truth_end[1])
        final_yaw_gap = abs(_norm_deg(odom_end[2] - truth_end[2]))

        print()
        print(f"odom moved : d_pos={odom_dpos:.3f} m  d_yaw={odom_dyaw:.1f} deg")
        print(f"physics moved: d_pos={truth_dpos:.3f} m  d_yaw={truth_dyaw:.1f} deg")
        print(f"final gap   : d_pos={final_pos_gap:.3f} m  d_yaw={final_yaw_gap:.1f} deg")
        print()

        if truth_dpos < 0.02 and truth_dyaw < 2.0:
            print("VERDICT: ROBOT DID NOT MOVE -- physics pose barely changed "
                  "despite a drive command. The robot is stuck (world collision, "
                  "jammed joint, or zero wheel friction). odom may still be "
                  "integrating a phantom motion.")
        elif final_pos_gap > pos_tol or final_yaw_gap > yaw_tol:
            print("VERDICT: MOTION DECOUPLED -- odom and physics disagree beyond "
                  "tolerance. The robot's physics setup is the bug: check wheel "
                  "friction / radius / separation, collision shapes, footprint "
                  "stability, and free-floating links. Fixing sensors or TF will "
                  "NOT help until this is resolved.")
        else:
            print("VERDICT: MOTION SOUND -- odom tracks physics within tolerance. "
                  "If you still see drift in a viewer, the bug is downstream: "
                  "display frame setting, QoS, TF buffer timing, or SLAM tuning.")


def main() -> None:
    rclpy.init()
    node = SimMotionCheck()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
