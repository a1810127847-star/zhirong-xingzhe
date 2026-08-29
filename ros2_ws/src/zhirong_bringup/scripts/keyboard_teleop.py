#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
四轮机器人键盘控制
------------------
W：增加前进油门
S：减小油门 / 继续按可倒车
A：方向向左
D：方向向右
X：方向回正（保留当前油门）
空格：立即停车并回正
Q：停车并退出
"""


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear = 0.0
        self.angular = 0.0

    def publish_command(self):
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        for _ in range(3):
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.02)

    def print_state(self):
        sys.stdout.write(
            f"\r油门 linear.x={self.linear:+.2f} m/s | "
            f"方向 angular.z={self.angular:+.2f} rad/s    "
        )
        sys.stdout.flush()


def main():
    if not sys.stdin.isatty():
        raise SystemExit("必须在可交互的 Terminal 中运行键盘控制器。")

    original_terminal = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleop()

    print(HELP)
    node.print_state()

    try:
        tty.setcbreak(sys.stdin.fileno())
        last_publish = 0.0

        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                key = sys.stdin.read(1).lower()

                if key == "w":
                    node.linear = clamp(node.linear + 0.05, -0.50, 0.80)
                elif key == "s":
                    node.linear = clamp(node.linear - 0.05, -0.50, 0.80)
                elif key == "a":
                    node.angular = clamp(node.angular + 0.15, -2.00, 2.00)
                elif key == "d":
                    node.angular = clamp(node.angular - 0.15, -2.00, 2.00)
                elif key == "x":
                    node.angular = 0.0
                elif key == " ":
                    node.linear = 0.0
                    node.angular = 0.0
                elif key == "q":
                    break
                elif key == "\x03":
                    raise KeyboardInterrupt

                node.publish_command()
                node.print_state()

            now = time.monotonic()
            if now - last_publish >= 0.1:
                node.publish_command()
                last_publish = now

            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal)
        print("\n机器人已停止。")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
