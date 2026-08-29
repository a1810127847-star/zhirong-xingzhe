#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=0.8)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    return parser.parse_args()


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main():
    args = parse_args()
    rclpy.init()
    navigator = Node("auto_navigate_test")
    navigation_client = ActionClient(
        navigator,
        NavigateToPose,
        "/navigate_to_pose",
    )
    latest_odom = {"message": None}

    navigator.create_subscription(
        Odometry,
        "/odom",
        lambda message: latest_odom.__setitem__("message", message),
        10,
    )

    try:
        clock_deadline = time.monotonic() + 15.0
        while (
            rclpy.ok()
            and navigator.get_clock().now().nanoseconds == 0
            and time.monotonic() < clock_deadline
        ):
            rclpy.spin_once(navigator, timeout_sec=0.1)
        if navigator.get_clock().now().nanoseconds == 0:
            raise RuntimeError("Simulation clock did not start.")

        odom_deadline = time.monotonic() + 15.0
        while (
            rclpy.ok()
            and latest_odom["message"] is None
            and time.monotonic() < odom_deadline
        ):
            rclpy.spin_once(navigator, timeout_sec=0.1)
        if latest_odom["message"] is None:
            raise RuntimeError("Timed out waiting for /odom.")

        initial_odom = latest_odom["message"].pose.pose.position

        if not navigation_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/navigate_to_pose action is unavailable.")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = args.goal_x
        goal_pose.pose.position.y = args.goal_y
        goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = (
            quaternion_from_yaw(args.goal_yaw)
        )

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_future = navigation_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            navigator,
            send_future,
            timeout_sec=8.0,
        )
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError("Timed out sending navigation goal.")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal Guard rejected the navigation test goal.")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + args.timeout

        while rclpy.ok() and not result_future.done():
            now = time.monotonic()
            if now >= deadline:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    navigator,
                    cancel_future,
                    timeout_sec=5.0,
                )
                raise RuntimeError(
                    f"Navigation did not finish within {args.timeout:.1f} seconds."
                )
            rclpy.spin_once(navigator, timeout_sec=0.1)

        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = result.status if result is not None else "none"
            raise RuntimeError(
                f"Navigation result status was {status}, not SUCCEEDED."
            )

        settle_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)

        final_odom = latest_odom["message"].pose.pose.position
        displacement = math.hypot(
            final_odom.x - initial_odom.x,
            final_odom.y - initial_odom.y,
        )
        goal_error = math.hypot(final_odom.x - args.goal_x, final_odom.y - args.goal_y)

        print(f"NAV_INITIAL_ODOM={initial_odom.x:.3f},{initial_odom.y:.3f}")
        print(f"NAV_GOAL_MAP={args.goal_x:.3f},{args.goal_y:.3f}")
        print(f"NAV_FINAL_ODOM={final_odom.x:.3f},{final_odom.y:.3f}")
        print(f"NAV_DISPLACEMENT={displacement:.3f}")
        print(f"NAV_GOAL_ERROR_APPROX={goal_error:.3f}")
        print("NAV_RESULT=SUCCEEDED")

        if displacement < 0.45:
            raise RuntimeError("Robot did not travel far enough toward the goal.")
        if goal_error > 0.30:
            raise RuntimeError("Robot stopped too far from the requested goal.")

        print("NAVIGATION_VALIDATION_OK")
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
