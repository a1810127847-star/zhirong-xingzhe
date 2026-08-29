#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--fps", type=float, default=15.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("record_demo_video")
    bridge = CvBridge()
    state = {
        "frame": None,
        "task": {},
        "velocity": Twist(),
        "pose": None,
    }

    def image_callback(message):
        state["frame"] = bridge.imgmsg_to_cv2(
            message,
            desired_encoding="bgr8",
        )

    def task_callback(message):
        try:
            state["task"] = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    node.create_subscription(
        Image,
        "/vision/debug_image",
        image_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(String, "/tasks/status", task_callback, 10)
    node.create_subscription(
        Twist,
        "/cmd_vel_safe",
        lambda message: state.__setitem__("velocity", message),
        10,
    )
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        lambda message: state.__setitem__("pose", message),
        10,
    )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (320, 240),
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the MP4 video writer.")

    try:
        frame_period = 1.0 / args.fps
        started = time.monotonic()
        next_frame = started
        written = 0
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now < next_frame or state["frame"] is None:
                continue

            frame = state["frame"].copy()
            task_status = state["task"]
            active = task_status.get("active_task") or {}
            velocity = state["velocity"]
            pose_message = state["pose"]
            pose_text = "pose: waiting"
            if pose_message is not None:
                position = pose_message.pose.pose.position
                pose_text = f"pose: {position.x:+.2f}, {position.y:+.2f}"

            overlay = [
                "Zhirong Complete-System Demo",
                f"state: {task_status.get('state', 'WAITING')}",
                f"task: {active.get('name', 'none')}",
                (
                    f"safe cmd: v={velocity.linear.x:+.2f} "
                    f"w={velocity.angular.z:+.2f}"
                ),
                pose_text,
            ]
            for index, text in enumerate(overlay):
                y = 18 + index * 19
                cv2.putText(
                    frame,
                    text,
                    (7, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 0, 0),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    text,
                    (7, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            writer.write(frame)
            written += 1
            next_frame += frame_period

        if written < int(args.fps * 3):
            raise RuntimeError("Too few frames were recorded.")
        print(f"DEMO_VIDEO_FRAMES={written}")
        print(f"DEMO_VIDEO_OUTPUT={output}")
        print("DEMO_VIDEO_OK")
    finally:
        writer.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
