#!/usr/bin/env python3
"""Measure odometry and AMCL stability during an in-place Gazebo turn."""

import argparse
import math
import statistics
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--angular-speed", type=float, default=0.40)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--min-turn-yaw", type=float, default=0.80)
    parser.add_argument("--max-position-drift", type=float, default=0.15)
    parser.add_argument(
        "--max-odom-position-error",
        type=float,
        default=0.04,
    )
    parser.add_argument(
        "--max-amcl-correction-step",
        type=float,
        default=0.03,
    )
    parser.add_argument("--max-amcl-yaw-rms", type=float, default=0.08)
    return parser.parse_args()


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def angle_delta(current, previous):
    return math.atan2(
        math.sin(current - previous),
        math.cos(current - previous),
    )


class UnwrappedPose:
    def __init__(self):
        self.initial = None
        self.previous_yaw = None
        self.unwrapped_yaw = 0.0

    def update(self, pose):
        yaw = yaw_from_quaternion(pose.orientation)
        if self.initial is None:
            self.initial = (pose.position.x, pose.position.y, yaw)
            self.previous_yaw = yaw
            return 0.0, 0.0, 0.0

        self.unwrapped_yaw += angle_delta(yaw, self.previous_yaw)
        self.previous_yaw = yaw
        return (
            pose.position.x - self.initial[0],
            pose.position.y - self.initial[1],
            self.unwrapped_yaw,
        )


def rms(values):
    if not values:
        return float("nan")
    return math.sqrt(statistics.fmean(value * value for value in values))


def main():
    args = parse_args()
    rclpy.init()
    node = Node("validate_turn_localization")
    command_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {
        "ground_truth": None,
        "odom": None,
        "amcl": None,
        "safe": None,
    }

    def model_callback(message):
        try:
            index = message.name.index("zhirong_diffbot")
        except ValueError:
            return
        state["ground_truth"] = message.pose[index]

    node.create_subscription(ModelStates, "/model_states", model_callback, 10)
    node.create_subscription(
        Odometry,
        "/odom",
        lambda message: state.__setitem__("odom", message.pose.pose),
        10,
    )
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        lambda message: state.__setitem__("amcl", message.pose.pose),
        10,
    )
    node.create_subscription(
        Twist,
        "/cmd_vel_safe",
        lambda message: state.__setitem__("safe", message),
        10,
    )

    trackers = {
        name: UnwrappedPose()
        for name in ("ground_truth", "odom", "amcl")
    }
    samples = []

    def publish_command(angular_speed):
        message = Twist()
        message.angular.z = angular_speed
        command_publisher.publish(message)

    def collect_for(duration, angular_speed):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            publish_command(angular_speed)
            rclpy.spin_once(node, timeout_sec=0.04)
            if all(state[name] is not None for name in trackers):
                sample = {
                    name: trackers[name].update(state[name])
                    for name in trackers
                }
                safe = state["safe"]
                sample["safe_angular"] = (
                    safe.angular.z if safe is not None else float("nan")
                )
                samples.append(sample)
            time.sleep(0.01)

    try:
        ready_deadline = time.monotonic() + 25.0
        while (
            rclpy.ok()
            and not all(
                state[name] is not None
                for name in ("ground_truth", "odom")
            )
            and time.monotonic() < ready_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        missing = [
            name
            for name in ("ground_truth", "odom")
            if state[name] is None
        ]
        if missing:
            raise RuntimeError(
                "Timed out waiting for: " + ", ".join(missing)
            )

        collect_for(1.0, 0.0)
        collect_for(args.duration, args.angular_speed)
        collect_for(args.settle, 0.0)

        ground_truth = [sample["ground_truth"] for sample in samples]
        odom = [sample["odom"] for sample in samples]
        amcl = [sample["amcl"] for sample in samples]
        odom_position_error = [
            math.hypot(o[0] - g[0], o[1] - g[1])
            for o, g in zip(odom, ground_truth)
        ]
        amcl_position_error = [
            math.hypot(a[0] - g[0], a[1] - g[1])
            for a, g in zip(amcl, ground_truth)
        ]
        odom_yaw_error = [o[2] - g[2] for o, g in zip(odom, ground_truth)]
        amcl_yaw_error = [a[2] - g[2] for a, g in zip(amcl, ground_truth)]
        amcl_error_steps = [
            math.hypot(
                amcl_position_error_x - previous_error_x,
                amcl_position_error_y - previous_error_y,
            )
            for (amcl_position_error_x, amcl_position_error_y),
            (previous_error_x, previous_error_y) in zip(
                [
                    (a[0] - g[0], a[1] - g[1])
                    for a, g in zip(amcl[1:], ground_truth[1:])
                ],
                [
                    (a[0] - g[0], a[1] - g[1])
                    for a, g in zip(amcl[:-1], ground_truth[:-1])
                ],
            )
        ]

        final_ground_truth = ground_truth[-1]
        final_odom = odom[-1]
        final_amcl = amcl[-1]
        ground_truth_position_drift = math.hypot(
            final_ground_truth[0],
            final_ground_truth[1],
        )
        odom_position_error_max = max(odom_position_error)
        amcl_correction_step_max = max(amcl_error_steps, default=0.0)
        amcl_yaw_error_rms = rms(amcl_yaw_error)
        safe_angular_values = [
            abs(sample["safe_angular"])
            for sample in samples
            if math.isfinite(sample["safe_angular"])
        ]

        print(f"TURN_SAMPLE_COUNT={len(samples)}")
        print(f"TURN_GROUND_TRUTH_YAW_RAD={final_ground_truth[2]:.4f}")
        print(
            "TURN_GROUND_TRUTH_POSITION_DRIFT_M="
            f"{ground_truth_position_drift:.4f}"
        )
        print(
            "TURN_ODOM_POSITION_ERROR_RMS_M="
            f"{rms(odom_position_error):.4f}"
        )
        print(
            "TURN_ODOM_POSITION_ERROR_MAX_M="
            f"{odom_position_error_max:.4f}"
        )
        print(f"TURN_ODOM_YAW_ERROR_RMS_RAD={rms(odom_yaw_error):.4f}")
        print(
            "TURN_ODOM_YAW_ERROR_FINAL_RAD="
            f"{final_odom[2] - final_ground_truth[2]:.4f}"
        )
        print(
            "TURN_AMCL_POSITION_ERROR_RMS_M="
            f"{rms(amcl_position_error):.4f}"
        )
        print(
            "TURN_AMCL_POSITION_ERROR_MAX_M="
            f"{max(amcl_position_error):.4f}"
        )
        print(
            "TURN_AMCL_CORRECTION_STEP_MAX_M="
            f"{amcl_correction_step_max:.4f}"
        )
        print(f"TURN_AMCL_YAW_ERROR_RMS_RAD={amcl_yaw_error_rms:.4f}")
        print(
            "TURN_AMCL_YAW_ERROR_FINAL_RAD="
            f"{final_amcl[2] - final_ground_truth[2]:.4f}"
        )
        print(
            "TURN_SAFE_ANGULAR_MAX_RAD_S="
            f"{max(safe_angular_values, default=0.0):.4f}"
        )

        failures = []
        if abs(final_ground_truth[2]) < args.min_turn_yaw:
            failures.append(
                "turn yaw below minimum: "
                f"{abs(final_ground_truth[2]):.4f} < {args.min_turn_yaw:.4f}"
            )
        if ground_truth_position_drift > args.max_position_drift:
            failures.append(
                "physical position drift too large: "
                f"{ground_truth_position_drift:.4f} > "
                f"{args.max_position_drift:.4f}"
            )
        if odom_position_error_max > args.max_odom_position_error:
            failures.append(
                "odometry position error too large: "
                f"{odom_position_error_max:.4f} > "
                f"{args.max_odom_position_error:.4f}"
            )
        if amcl_correction_step_max > args.max_amcl_correction_step:
            failures.append(
                "AMCL correction step too large: "
                f"{amcl_correction_step_max:.4f} > "
                f"{args.max_amcl_correction_step:.4f}"
            )
        if amcl_yaw_error_rms > args.max_amcl_yaw_rms:
            failures.append(
                "AMCL yaw RMS error too large: "
                f"{amcl_yaw_error_rms:.4f} > "
                f"{args.max_amcl_yaw_rms:.4f}"
            )
        if failures:
            raise RuntimeError("; ".join(failures))
        print("TURN_LOCALIZATION_MEASUREMENT_OK")
    finally:
        for _ in range(10):
            publish_command(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
