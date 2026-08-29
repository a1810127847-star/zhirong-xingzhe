#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=15.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("capture_ros_image")
    bridge = CvBridge()
    received = {"message": None}
    node.create_subscription(
        Image,
        args.topic,
        lambda message: received.__setitem__("message", message),
        qos_profile_sensor_data,
    )

    try:
        deadline = time.monotonic() + args.timeout
        while (
            rclpy.ok()
            and received["message"] is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if received["message"] is None:
            raise RuntimeError(f"No image received from {args.topic}.")

        image = bridge.imgmsg_to_cv2(
            received["message"],
            desired_encoding="bgr8",
        )
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output), image):
            raise RuntimeError(f"OpenCV could not write {output}.")
        print(f"CAPTURE_TOPIC={args.topic}")
        print(f"CAPTURE_SIZE={image.shape[1]}x{image.shape[0]}")
        print(f"CAPTURE_OUTPUT={output}")
        print("CAPTURE_OK")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
