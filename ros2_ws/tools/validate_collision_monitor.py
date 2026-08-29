#!/usr/bin/env python3
import math
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


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


def yaw_from_orientation(orientation):
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def angular_distance(first, second):
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


def publish_for(node, publisher, command, duration, sample_callback):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)
        sample_callback()
        time.sleep(0.05)


def main():
    rclpy.init()
    node = Node("validate_collision_monitor")
    raw_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    state_client = node.create_client(SetEntityState, "/set_entity_state")
    latest_odom = {"value": None}
    latest_safe = {"value": Twist()}
    model_positions = {"robot": None}

    node.create_subscription(
        Odometry,
        "/odom",
        lambda message: latest_odom.__setitem__("value", message),
        10,
    )
    node.create_subscription(
        Twist,
        "/cmd_vel_safe",
        lambda message: latest_safe.__setitem__("value", message),
        10,
    )

    def model_callback(message):
        try:
            index = message.name.index("zhirong_diffbot")
        except ValueError:
            return
        model_positions["robot"] = message.pose[index].position

    node.create_subscription(ModelStates, "/model_states", model_callback, 10)

    try:
        if not state_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/set_entity_state service is unavailable.")
        deadline = time.monotonic() + 15.0
        while (
            rclpy.ok()
            and (latest_odom["value"] is None or model_positions["robot"] is None)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest_odom["value"] is None or model_positions["robot"] is None:
            raise RuntimeError("Robot state was not available.")

        robot = model_positions["robot"]
        orientation = latest_odom["value"].pose.pose.orientation
        robot_yaw = yaw_from_orientation(orientation)
        set_obstacle(
            node,
            state_client,
            robot.x + 0.52 * math.cos(robot_yaw),
            robot.y + 0.52 * math.sin(robot_yaw),
        )
        time.sleep(0.5)

        start_odom = latest_odom["value"].pose.pose.position
        blocked_max_safe = 0.0

        def sample_blocked():
            nonlocal blocked_max_safe
            blocked_max_safe = max(
                blocked_max_safe,
                abs(latest_safe["value"].linear.x),
            )

        command = Twist()
        command.linear.x = 0.20
        publish_for(node, raw_publisher, command, 2.0, sample_blocked)

        blocked_end = latest_odom["value"].pose.pose.position
        blocked_displacement = math.hypot(
            blocked_end.x - start_odom.x,
            blocked_end.y - start_odom.y,
        )

        rotation_start = yaw_from_orientation(
            latest_odom["value"].pose.pose.orientation
        )
        rotation_max_safe = 0.0

        def sample_rotation():
            nonlocal rotation_max_safe
            rotation_max_safe = max(
                rotation_max_safe,
                abs(latest_safe["value"].angular.z),
            )

        rotate_command = Twist()
        rotate_command.angular.z = 0.50
        publish_for(
            node,
            raw_publisher,
            rotate_command,
            2.0,
            sample_rotation,
        )
        rotation_end = yaw_from_orientation(
            latest_odom["value"].pose.pose.orientation
        )
        rotation_delta = angular_distance(rotation_start, rotation_end)

        reverse_start = latest_odom["value"].pose.pose.position
        reverse_max_safe = 0.0

        def sample_reverse():
            nonlocal reverse_max_safe
            reverse_max_safe = max(
                reverse_max_safe,
                abs(latest_safe["value"].linear.x),
            )

        reverse_command = Twist()
        reverse_command.linear.x = -0.12
        publish_for(
            node,
            raw_publisher,
            reverse_command,
            1.5,
            sample_reverse,
        )
        reverse_end = latest_odom["value"].pose.pose.position
        reverse_displacement = math.hypot(
            reverse_end.x - reverse_start.x,
            reverse_end.y - reverse_start.y,
        )

        set_obstacle(node, state_client, -2.35, -2.35)
        release_deadline = time.monotonic() + 0.8
        while rclpy.ok() and time.monotonic() < release_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        released_max_safe = 0.0

        def sample_released():
            nonlocal released_max_safe
            released_max_safe = max(
                released_max_safe,
                abs(latest_safe["value"].linear.x),
            )

        release_start = latest_odom["value"].pose.pose.position
        publish_for(node, raw_publisher, command, 1.6, sample_released)
        raw_publisher.publish(Twist())
        release_end = latest_odom["value"].pose.pose.position
        released_displacement = math.hypot(
            release_end.x - release_start.x,
            release_end.y - release_start.y,
        )

        print(f"SAFETY_BLOCKED_MAX_SAFE_SPEED={blocked_max_safe:.3f}")
        print(f"SAFETY_BLOCKED_DISPLACEMENT={blocked_displacement:.3f}")
        print(f"SAFETY_ROTATION_MAX_SAFE_SPEED={rotation_max_safe:.3f}")
        print(f"SAFETY_ROTATION_DELTA_RAD={rotation_delta:.3f}")
        print(f"SAFETY_REVERSE_MAX_SAFE_SPEED={reverse_max_safe:.3f}")
        print(f"SAFETY_REVERSE_DISPLACEMENT={reverse_displacement:.3f}")
        print(f"SAFETY_RELEASED_MAX_SAFE_SPEED={released_max_safe:.3f}")
        print(f"SAFETY_RELEASED_DISPLACEMENT={released_displacement:.3f}")

        if blocked_max_safe > 0.03:
            raise RuntimeError(
                "Predictive safety layer allowed unsafe forward velocity."
            )
        if blocked_displacement > 0.06:
            raise RuntimeError(
                "Robot moved too far toward the occupied forward path."
            )
        if rotation_max_safe < 0.10 or rotation_delta < 0.15:
            raise RuntimeError(
                "Safe in-place rotation was incorrectly suppressed."
            )
        if reverse_max_safe < 0.08 or reverse_displacement < 0.08:
            raise RuntimeError(
                "Safe rearward escape was incorrectly suppressed."
            )
        if released_max_safe < 0.15:
            raise RuntimeError("Safe velocity did not resume after obstacle removal.")
        if released_displacement < 0.12:
            raise RuntimeError("Robot did not resume movement after obstacle removal.")

        print("COLLISION_MONITOR_VALIDATION_OK")
    finally:
        raw_publisher.publish(Twist())
        try:
            if state_client.service_is_ready():
                set_obstacle(node, state_client, -2.35, -2.35)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
