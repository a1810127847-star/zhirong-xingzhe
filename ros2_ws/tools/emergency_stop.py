#!/usr/bin/env python3
"""Cancel active navigation tasks and hold a zero velocity command briefly."""

import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node("zhirong_emergency_stop")
    velocity_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    task_publisher = node.create_publisher(String, "/tasks/command", 10)
    task_cancel_client = node.create_client(Trigger, "/tasks/cancel")
    cancel_client = node.create_client(
        CancelGoal,
        "/navigate_to_pose/_action/cancel_goal",
    )

    try:
        cancel_sent = False
        if cancel_client.wait_for_service(timeout_sec=2.0):
            request = CancelGoal.Request()
            future = cancel_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            cancel_sent = future.done() and future.result() is not None

        task_command = String()
        task_command.data = "cancel"
        discovery_deadline = time.monotonic() + 1.0
        while (
            task_publisher.get_subscription_count() == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        task_publisher.publish(task_command)

        task_cancel_sent = False
        if task_cancel_client.wait_for_service(timeout_sec=1.0):
            future = task_cancel_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            task_cancel_sent = future.done() and future.result() is not None

        zero = Twist()
        deadline = time.monotonic() + 1.5
        while rclpy.ok() and time.monotonic() < deadline:
            velocity_publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        print(f"NAVIGATION_CANCEL_SENT={str(cancel_sent).lower()}")
        print(f"TASK_CANCEL_SENT={str(task_cancel_sent).lower()}")
        print("ZERO_VELOCITY_HELD_SEC=1.5")
        print("EMERGENCY_STOP_OK")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
