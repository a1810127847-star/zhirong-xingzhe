#!/usr/bin/env python3

import json
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


PACKET_MAGIC = "zhirong_xinput_v1"


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class GamepadUdpTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_udp_teleop")
        self.declare_parameter("port", 15150)
        self.declare_parameter("watchdog_seconds", 0.25)
        self.declare_parameter("max_linear", 0.80)
        self.declare_parameter("max_angular", 2.00)

        self.port = int(self.get_parameter("port").value)
        self.watchdog_seconds = float(
            self.get_parameter("watchdog_seconds").value
        )
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setblocking(False)
        self.socket.bind(("0.0.0.0", self.port))

        self.last_packet_time = 0.0
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.enabled = False
        self.last_active = None

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(
            f"Listening for Windows XInput packets on UDP port {self.port}; "
            f"watchdog={self.watchdog_seconds:.2f}s"
        )

    def receive_packets(self):
        while True:
            try:
                payload, _sender = self.socket.recvfrom(4096)
            except BlockingIOError:
                break

            try:
                packet = json.loads(payload.decode("utf-8"))
                if packet.get("magic") != PACKET_MAGIC:
                    continue

                self.requested_linear = clamp(
                    float(packet.get("linear", 0.0)),
                    -self.max_linear,
                    self.max_linear,
                )
                self.requested_angular = clamp(
                    float(packet.get("angular", 0.0)),
                    -self.max_angular,
                    self.max_angular,
                )
                self.enabled = bool(packet.get("enabled", False))
                self.last_packet_time = time.monotonic()
            except (UnicodeDecodeError, ValueError, TypeError, json.JSONDecodeError):
                continue

    def publish(self, linear, angular):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def update(self):
        self.receive_packets()
        packet_fresh = (
            time.monotonic() - self.last_packet_time
        ) <= self.watchdog_seconds
        active = packet_fresh and self.enabled

        if active:
            linear = self.requested_linear
            angular = self.requested_angular
        else:
            linear = 0.0
            angular = 0.0

        self.publish(linear, angular)

        if active != self.last_active:
            if active:
                self.get_logger().info("Gamepad enabled.")
            else:
                self.get_logger().info("Gamepad stopped or watchdog active.")
            self.last_active = active

    def destroy_node(self):
        for _ in range(3):
            self.publish(0.0, 0.0)
        self.socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = GamepadUdpTeleop()
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
