#!/usr/bin/env python3
"""Suppress wrong-way steering only on proven one-curvature patrol paths."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int8


class PatrolAngularGuard(Node):
    def __init__(self):
        super().__init__("patrol_angular_guard")
        self.turn_sign = 0
        self.last_log_time = 0.0
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 20)
        self.create_subscription(
            Twist,
            "/cmd_vel_nav_smoothed",
            self._velocity_callback,
            20,
        )
        self.create_subscription(
            Int8,
            "/tasks/route_turn_sign",
            self._turn_sign_callback,
            10,
        )
        self.get_logger().info(
            "Patrol angular guard ready: smoothed=/cmd_vel_nav_smoothed, "
            "filtered=/cmd_vel"
        )

    def _turn_sign_callback(self, message):
        new_sign = 1 if message.data > 0 else -1 if message.data < 0 else 0
        self.turn_sign = new_sign

    def _velocity_callback(self, message):
        output = Twist()
        output.linear.x = message.linear.x
        output.linear.y = message.linear.y
        output.linear.z = message.linear.z
        output.angular.x = message.angular.x
        output.angular.y = message.angular.y
        output.angular.z = message.angular.z
        if (
            self.turn_sign
            and message.angular.z * self.turn_sign < 0.0
        ):
            output.angular.z = 0.0
            now = time.monotonic()
            if now - self.last_log_time >= 1.0:
                self.get_logger().info(
                    "Suppressed wrong-way patrol steering correction "
                    f"{message.angular.z:.3f} rad/s."
                )
                self.last_log_time = now
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = PatrolAngularGuard()
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
