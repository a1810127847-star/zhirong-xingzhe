#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_args(arguments):
    parser = argparse.ArgumentParser(
        description="Send a text command to the Zhirong task manager."
    )
    parser.add_argument(
        "command",
        nargs="+",
        help="Examples: patrol demo | goto east | return_home | vision on",
    )
    return parser.parse_args(arguments)


def main(args=None):
    cli_args = parse_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=[])
    node = Node("task_cli")
    publisher = node.create_publisher(String, "/tasks/command", 10)
    message = String()
    message.data = " ".join(cli_args.command)

    deadline = time.monotonic() + 2.0
    while rclpy.ok() and publisher.get_subscription_count() == 0:
        if time.monotonic() >= deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    publisher.publish(message)
    node.get_logger().info(f"Sent task command: {message.data}")
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
