#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=1.6)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--obstacle-hold", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument(
        "--home-first",
        action="store_true",
        help="Navigate to map origin before placing the dynamic obstacle.",
    )
    return parser.parse_args()


def pose_stamped(node, x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def send_navigation_goal(node, action_client, pose):
    goal = NavigateToPose.Goal()
    goal.pose = pose
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("Timed out sending navigation goal.")
    goal_handle = future.result()
    if not goal_handle.accepted:
        raise RuntimeError("Goal Guard rejected the navigation test goal.")
    return goal_handle, goal_handle.get_result_async()


def cancel_navigation_goal(node, goal_handle):
    future = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)


def set_obstacle(node, client, x, y):
    request = SetEntityState.Request()
    request.state.name = "dynamic_obstacle"
    request.state.pose.position.x = x
    request.state.pose.position.y = y
    request.state.pose.position.z = 0.36
    request.state.pose.orientation.w = 1.0
    request.state.reference_frame = "world"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None or not future.result().success:
        raise RuntimeError("Failed to move dynamic_obstacle.")


def wait_for_message(node, state, timeout=15.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and state["value"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["value"] is None:
        raise RuntimeError("Timed out waiting for required robot state.")


def main():
    args = parse_args()
    rclpy.init()
    navigator = Node("validate_dynamic_avoidance")
    navigation_client = ActionClient(
        navigator,
        NavigateToPose,
        "/navigate_to_pose",
    )
    latest_odom = {"value": None}
    latest_amcl = {"value": None}
    latest_raw = {"value": None, "time": 0.0}
    metrics = {
        "max_lateral_deviation": 0.0,
        "min_front_scan": float("inf"),
        "min_center_distance": float("inf"),
        "speed_reduction_samples": 0,
        "plan_messages": 0,
    }

    navigator.create_subscription(
        Odometry,
        "/odom",
        lambda message: latest_odom.__setitem__("value", message),
        10,
    )
    navigator.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        lambda message: latest_amcl.__setitem__("value", message),
        10,
    )

    def raw_callback(message):
        latest_raw["value"] = message
        latest_raw["time"] = time.monotonic()

    def safe_callback(message):
        raw = latest_raw["value"]
        if raw is None or time.monotonic() - latest_raw["time"] > 0.3:
            return
        raw_speed = abs(raw.linear.x)
        safe_speed = abs(message.linear.x)
        if raw_speed > 0.04 and safe_speed < raw_speed * 0.60:
            metrics["speed_reduction_samples"] += 1

    def scan_callback(message):
        half_window = max(
            1,
            int(math.radians(15.0) / max(message.angle_increment, 1e-6)),
        )
        center = int((0.0 - message.angle_min) / message.angle_increment)
        ranges = message.ranges[
            max(0, center - half_window) : min(
                len(message.ranges),
                center + half_window + 1,
            )
        ]
        finite = [value for value in ranges if math.isfinite(value)]
        if finite:
            metrics["min_front_scan"] = min(
                metrics["min_front_scan"],
                min(finite),
            )

    def model_states_callback(message):
        try:
            robot_index = message.name.index("zhirong_diffbot")
            obstacle_index = message.name.index("dynamic_obstacle")
        except ValueError:
            return
        robot = message.pose[robot_index].position
        obstacle = message.pose[obstacle_index].position
        distance = math.hypot(robot.x - obstacle.x, robot.y - obstacle.y)
        metrics["min_center_distance"] = min(
            metrics["min_center_distance"],
            distance,
        )

    navigator.create_subscription(Twist, "/cmd_vel", raw_callback, 10)
    navigator.create_subscription(Twist, "/cmd_vel_safe", safe_callback, 10)
    navigator.create_subscription(LaserScan, "/scan", scan_callback, 10)
    navigator.create_subscription(
        ModelStates,
        "/model_states",
        model_states_callback,
        10,
    )
    navigator.create_subscription(
        Path,
        "/plan",
        lambda message: metrics.__setitem__(
            "plan_messages",
            metrics["plan_messages"] + (1 if message.poses else 0),
        ),
        10,
    )
    state_client = navigator.create_client(
        SetEntityState,
        "/set_entity_state",
    )

    try:
        if not state_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/set_entity_state service is unavailable.")
        if not navigation_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/navigate_to_pose action is unavailable.")
        wait_for_message(navigator, latest_odom)

        if args.home_first:
            home_handle, home_result_future = send_navigation_goal(
                navigator,
                navigation_client,
                pose_stamped(navigator, 0.0, 0.0),
            )
            home_deadline = time.monotonic() + 100.0
            while rclpy.ok() and not home_result_future.done():
                rclpy.spin_once(navigator, timeout_sec=0.1)
                if time.monotonic() >= home_deadline:
                    cancel_navigation_goal(navigator, home_handle)
                    raise RuntimeError(
                        "Return home timed out before dynamic test."
                    )
            home_result = home_result_future.result()
            if (
                home_result is None
                or home_result.status != GoalStatus.STATUS_SUCCEEDED
            ):
                raise RuntimeError("Return home failed before dynamic test.")
            settle_deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < settle_deadline:
                rclpy.spin_once(navigator, timeout_sec=0.1)
            print("DYNAMIC_HOME_READY=0.000,0.000")

        start_position = latest_odom["value"].pose.pose.position
        obstacle_x = start_position.x + 0.90
        obstacle_y = start_position.y
        set_obstacle(navigator, state_client, obstacle_x, obstacle_y)
        print(f"DYNAMIC_OBSTACLE_INSERTED={obstacle_x:.3f},{obstacle_y:.3f}")

        mark_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < mark_deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)

        goal_handle, result_future = send_navigation_goal(
            navigator,
            navigation_client,
            pose_stamped(navigator, args.goal_x, args.goal_y),
        )
        started = time.monotonic()
        obstacle_removed = False
        deadline = started + args.timeout

        while rclpy.ok() and not result_future.done():
            now = time.monotonic()
            if now >= deadline:
                cancel_navigation_goal(navigator, goal_handle)
                raise RuntimeError("Dynamic avoidance navigation timed out.")

            rclpy.spin_once(navigator, timeout_sec=0.05)
            odom = latest_odom["value"]
            if odom is not None:
                position = odom.pose.pose.position
                metrics["max_lateral_deviation"] = max(
                    metrics["max_lateral_deviation"],
                    abs(position.y - start_position.y),
                )

            if not obstacle_removed and now - started >= args.obstacle_hold:
                set_obstacle(navigator, state_client, -2.35, -2.35)
                obstacle_removed = True
                print("DYNAMIC_OBSTACLE_CLEARED=-2.350,-2.350")

            time.sleep(0.05)

        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            result_status = result.status if result is not None else "none"
            raise RuntimeError(
                "Dynamic avoidance result was "
                f"{result_status}, not SUCCEEDED."
            )
        if not obstacle_removed:
            set_obstacle(navigator, state_client, -2.35, -2.35)

        settle_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)

        final_pose = latest_amcl["value"]
        if final_pose is None:
            raise RuntimeError("No AMCL pose received.")
        final_position = final_pose.pose.pose.position
        goal_error = math.hypot(
            final_position.x - args.goal_x,
            final_position.y - args.goal_y,
        )
        duration = time.monotonic() - started

        print(f"DYNAMIC_DURATION_SEC={duration:.3f}")
        print(f"DYNAMIC_GOAL_ERROR={goal_error:.3f}")
        print(
            "DYNAMIC_MAX_LATERAL_DEVIATION="
            f"{metrics['max_lateral_deviation']:.3f}"
        )
        print(f"DYNAMIC_MIN_FRONT_SCAN={metrics['min_front_scan']:.3f}")
        print(
            "DYNAMIC_MIN_CENTER_DISTANCE="
            f"{metrics['min_center_distance']:.3f}"
        )
        print(
            "DYNAMIC_SPEED_REDUCTION_SAMPLES="
            f"{metrics['speed_reduction_samples']}"
        )
        print(f"DYNAMIC_PLAN_MESSAGES={metrics['plan_messages']}")

        if goal_error > 0.30:
            raise RuntimeError("Robot did not reach the dynamic-avoidance goal.")
        if metrics["min_center_distance"] < 0.40:
            raise RuntimeError("Robot came too close to the moving obstacle.")
        if (
            metrics["max_lateral_deviation"] < 0.12
            and metrics["speed_reduction_samples"] < 3
        ):
            raise RuntimeError(
                "No detour or safety speed intervention was observed."
            )
        if metrics["plan_messages"] < 1:
            raise RuntimeError("No Nav2 global plan was observed.")

        home_handle, home_result_future = send_navigation_goal(
            navigator,
            navigation_client,
            pose_stamped(navigator, 0.0, 0.0),
        )
        home_deadline = time.monotonic() + 100.0
        while rclpy.ok() and not home_result_future.done():
            rclpy.spin_once(navigator, timeout_sec=0.1)
            if time.monotonic() >= home_deadline:
                cancel_navigation_goal(navigator, home_handle)
                raise RuntimeError("Return home timed out after dynamic test.")
        home_result = home_result_future.result()
        if (
            home_result is None
            or home_result.status != GoalStatus.STATUS_SUCCEEDED
        ):
            raise RuntimeError("Return home failed after dynamic test.")

        print("DYNAMIC_AVOIDANCE_VALIDATION_OK")
    finally:
        try:
            if state_client.service_is_ready():
                set_obstacle(navigator, state_client, -2.35, -2.35)
        except Exception:
            pass
        navigation_client.destroy()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
