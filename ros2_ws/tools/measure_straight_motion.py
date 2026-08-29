#!/usr/bin/env python3
"""Measure straight-line Nav2 motion quality without relying on RViz visuals."""

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=0.8)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=100.0)
    parser.add_argument("--home-first", action="store_true")
    parser.add_argument("--return-home", action="store_true")
    parser.add_argument("--csv", default="")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--max-lateral", type=float, default=0.08)
    parser.add_argument("--max-path-ratio", type=float, default=1.12)
    parser.add_argument("--max-lag", type=float, default=0.40)
    parser.add_argument("--max-angular-reversals", type=int, default=4)
    return parser.parse_args()


def quaternion_yaw(orientation):
    return 2.0 * math.atan2(orientation.z, orientation.w)


def yaw_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def path_metrics(points):
    if len(points) < 2:
        return 0.0, 0.0, 0.0
    start_x, start_y = points[0]
    end_x, end_y = points[-1]
    chord = math.hypot(end_x - start_x, end_y - start_y)
    length = sum(
        math.hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(points, points[1:])
    )
    if chord < 1e-6:
        return length, 0.0, 0.0
    dx = end_x - start_x
    dy = end_y - start_y
    max_lateral = max(
        abs(dy * (x - start_x) - dx * (y - start_y)) / chord
        for x, y in points
    )
    return length, length / chord, max_lateral


class StraightMotionProbe(Node):
    def __init__(self):
        super().__init__("measure_straight_motion")
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        self.latest_odom = None
        self.latest_amcl = None
        self.latest_raw = Twist()
        self.latest_safe = Twist()
        self.recording = False
        self.started_at = 0.0
        self.samples = []
        self.plans = []

        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            lambda message: setattr(self, "latest_amcl", message),
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            lambda message: setattr(self, "latest_raw", message),
            20,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            lambda message: setattr(self, "latest_safe", message),
            20,
        )
        self.create_subscription(NavPath, "/plan", self._plan_callback, 10)

    def _odom_callback(self, message):
        self.latest_odom = message
        if not self.recording:
            return
        pose = message.pose.pose
        twist = message.twist.twist
        self.samples.append(
            {
                "time": time.monotonic() - self.started_at,
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": quaternion_yaw(pose.orientation),
                "cmd_v": self.latest_raw.linear.x,
                "cmd_w": self.latest_raw.angular.z,
                "safe_v": self.latest_safe.linear.x,
                "safe_w": self.latest_safe.angular.z,
                "odom_v": twist.linear.x,
                "odom_w": twist.angular.z,
            }
        )

    def _plan_callback(self, message):
        if self.recording and message.poses:
            self.plans.append(
                [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
            )

    def wait_ready(self):
        if not self.navigation_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/navigate_to_pose action is unavailable.")
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and self.latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_odom is None:
            raise RuntimeError("Timed out waiting for /odom.")

    def navigate(self, x, y, yaw, timeout, record=False):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_quaternion(yaw)

        future = self.navigation_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Timed out sending navigation goal.")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("Goal Guard rejected the navigation goal.")

        if record:
            self.samples.clear()
            self.plans.clear()
            self.started_at = time.monotonic()
            self.recording = True

        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not result_future.done():
            if time.monotonic() >= deadline:
                handle.cancel_goal_async()
                raise RuntimeError("Navigation timed out.")
            rclpy.spin_once(self, timeout_sec=0.02)

        self.recording = False
        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = result.status if result is not None else "none"
            raise RuntimeError(f"Navigation result was {status}, not SUCCEEDED.")
        return time.monotonic() - self.started_at if record else 0.0


def first_threshold_time(samples, key, threshold):
    for sample in samples:
        if abs(sample[key]) >= threshold:
            return sample["time"]
    return None


def angular_reversals(samples):
    reversals = 0
    previous = 0
    for sample in samples:
        if abs(sample["safe_v"]) < 0.03 or abs(sample["safe_w"]) < 0.03:
            continue
        sign = 1 if sample["safe_w"] > 0.0 else -1
        if previous and sign != previous:
            reversals += 1
        previous = sign
    return reversals


def main():
    args = parse_args()
    rclpy.init()
    node = StraightMotionProbe()
    try:
        node.wait_ready()
        if args.home_first:
            node.navigate(0.0, 0.0, 0.0, args.timeout)
            settle_until = time.monotonic() + 1.0
            while time.monotonic() < settle_until:
                rclpy.spin_once(node, timeout_sec=0.05)

        duration = node.navigate(
            args.goal_x,
            args.goal_y,
            0.0,
            args.timeout,
            record=True,
        )
        samples = node.samples
        if len(samples) < 2:
            raise RuntimeError("Too few odometry samples were recorded.")

        actual_points = [(sample["x"], sample["y"]) for sample in samples]
        actual_length, actual_ratio, actual_lateral = path_metrics(actual_points)
        plan = max(node.plans, key=len) if node.plans else []
        plan_length, plan_ratio, plan_lateral = path_metrics(plan)
        command_time = first_threshold_time(samples, "safe_v", 0.03)
        response_time = first_threshold_time(samples, "odom_v", 0.03)
        lag = (
            max(0.0, response_time - command_time)
            if command_time is not None and response_time is not None
            else float("inf")
        )
        moving = [sample for sample in samples if abs(sample["safe_v"]) >= 0.03]
        linear_rmse = math.sqrt(
            sum((sample["safe_v"] - sample["odom_v"]) ** 2 for sample in moving)
            / max(1, len(moving))
        )
        angular_rmse = math.sqrt(
            sum((sample["safe_w"] - sample["odom_w"]) ** 2 for sample in moving)
            / max(1, len(moving))
        )
        max_abs_angular = max(abs(sample["safe_w"]) for sample in samples)
        reversals = angular_reversals(samples)

        final_amcl = node.latest_amcl.pose.pose if node.latest_amcl else None
        final_error = (
            math.hypot(
                final_amcl.position.x - args.goal_x,
                final_amcl.position.y - args.goal_y,
            )
            if final_amcl is not None
            else float("inf")
        )
        final_yaw_error = (
            abs(wrap_angle(quaternion_yaw(final_amcl.orientation)))
            if final_amcl is not None
            else float("inf")
        )

        metrics = {
            "STRAIGHT_DURATION_SEC": duration,
            "STRAIGHT_ACTUAL_LENGTH": actual_length,
            "STRAIGHT_ACTUAL_PATH_RATIO": actual_ratio,
            "STRAIGHT_ACTUAL_MAX_LATERAL": actual_lateral,
            "STRAIGHT_PLAN_LENGTH": plan_length,
            "STRAIGHT_PLAN_PATH_RATIO": plan_ratio,
            "STRAIGHT_PLAN_MAX_LATERAL": plan_lateral,
            "STRAIGHT_RESPONSE_LAG_SEC": lag,
            "STRAIGHT_LINEAR_TRACKING_RMSE": linear_rmse,
            "STRAIGHT_ANGULAR_TRACKING_RMSE": angular_rmse,
            "STRAIGHT_MAX_ABS_ANGULAR_CMD": max_abs_angular,
            "STRAIGHT_ANGULAR_REVERSALS": float(reversals),
            "STRAIGHT_FINAL_ERROR": final_error,
            "STRAIGHT_FINAL_YAW_ERROR": final_yaw_error,
        }
        for name, value in metrics.items():
            print(f"{name}={value:.4f}")

        if args.csv:
            output_path = Path(args.csv)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with output_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
                writer.writeheader()
                writer.writerows(samples)
            print(f"STRAIGHT_CSV={output_path}")

        failures = []
        if actual_lateral > args.max_lateral:
            failures.append("actual lateral deviation")
        if actual_ratio > args.max_path_ratio:
            failures.append("actual path ratio")
        if lag > args.max_lag:
            failures.append("response lag")
        if reversals > args.max_angular_reversals:
            failures.append("angular reversals")
        if final_error > 0.30:
            failures.append("final position error")
        if args.strict and failures:
            raise RuntimeError("Straight-motion limits failed: " + ", ".join(failures))
        print("STRAIGHT_MOTION_MEASUREMENT_OK")

        if args.return_home:
            node.navigate(0.0, 0.0, 0.0, args.timeout)
            print("STRAIGHT_RETURN_HOME_OK")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
