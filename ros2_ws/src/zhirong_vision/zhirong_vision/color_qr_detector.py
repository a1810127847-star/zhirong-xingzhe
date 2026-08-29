#!/usr/bin/env python3
import json
from typing import Dict, Optional

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from zhirong_vision.processor import DetectionResult, VisionProcessor


class StableValue:
    def __init__(self, required_frames: int):
        self.required_frames = max(1, required_frames)
        self.candidate = ""
        self.candidate_count = 0
        self.stable = ""

    def update(self, value: str) -> Optional[str]:
        if value == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate = value
            self.candidate_count = 1

        if self.candidate_count < self.required_frames or value == self.stable:
            return None

        previous = self.stable
        self.stable = value
        if value and value != previous:
            return value
        return None


class ColorQrDetector(Node):
    def __init__(self):
        super().__init__("color_qr_detector")
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("debug_topic", "/vision/debug_image")
        self.declare_parameter("detection_topic", "/vision/detections")
        self.declare_parameter("event_topic", "/vision/events")
        self.declare_parameter("color_topic", "/vision/color")
        self.declare_parameter("qr_topic", "/vision/qr")
        self.declare_parameter("min_color_area_ratio", 0.025)
        self.declare_parameter("required_consecutive_frames", 3)
        self.declare_parameter("process_every_n_frames", 2)
        self.declare_parameter("event_cooldown_sec", 8.0)

        self.bridge = CvBridge()
        self.processor = VisionProcessor(
            min_color_area_ratio=float(
                self.get_parameter("min_color_area_ratio").value
            )
        )
        required_frames = int(
            self.get_parameter("required_consecutive_frames").value
        )
        self.color_stability = StableValue(required_frames)
        self.qr_stability = StableValue(required_frames)
        self.process_every_n_frames = max(
            1,
            int(self.get_parameter("process_every_n_frames").value),
        )
        self.event_cooldown_sec = float(
            self.get_parameter("event_cooldown_sec").value
        )
        self.frame_count = 0
        self.last_event_time: Dict[str, float] = {}

        self.debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("debug_topic").value),
            qos_profile_sensor_data,
        )
        self.detection_publisher = self.create_publisher(
            String,
            str(self.get_parameter("detection_topic").value),
            10,
        )
        self.event_publisher = self.create_publisher(
            String,
            str(self.get_parameter("event_topic").value),
            10,
        )
        self.color_publisher = self.create_publisher(
            String,
            str(self.get_parameter("color_topic").value),
            10,
        )
        self.qr_publisher = self.create_publisher(
            String,
            str(self.get_parameter("qr_topic").value),
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter("input_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Vision detector ready: color + QR -> /vision/events"
        )

    def _image_callback(self, message: Image):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = self.processor.process(image)
        except Exception as exc:
            self.get_logger().error(f"Image processing failed: {exc}")
            return

        self._publish_detection(message, result)
        self._update_stable_outputs(result)

        annotated = self.processor.annotate(image, result)
        debug_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)

    def _publish_detection(self, image_message: Image, result: DetectionResult):
        payload = result.to_dict()
        payload["stamp"] = {
            "sec": image_message.header.stamp.sec,
            "nanosec": image_message.header.stamp.nanosec,
        }
        payload["frame_id"] = image_message.header.frame_id
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.detection_publisher.publish(message)

    def _update_stable_outputs(self, result: DetectionResult):
        color_value = result.color.label if result.color else ""
        qr_value = result.qr.data if result.qr else ""

        color_message = String()
        color_message.data = color_value or "none"
        self.color_publisher.publish(color_message)

        qr_message = String()
        qr_message.data = qr_value or "none"
        self.qr_publisher.publish(qr_message)

        new_color = self.color_stability.update(color_value)
        if new_color:
            confidence = result.color.confidence if result.color else 0.0
            self._publish_event("color", new_color, confidence)

        new_qr = self.qr_stability.update(qr_value)
        if new_qr:
            self._publish_event("qr", new_qr, 1.0)

    def _publish_event(self, event_type: str, value: str, confidence: float):
        event_key = f"{event_type}:{value}"
        now_sec = self.get_clock().now().nanoseconds / 1e9
        last_time = self.last_event_time.get(event_key, float("-inf"))
        if now_sec - last_time < self.event_cooldown_sec:
            return

        self.last_event_time[event_key] = now_sec
        payload = {
            "type": event_type,
            "value": value,
            "confidence": round(float(confidence), 4),
            "stamp": now_sec,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.event_publisher.publish(message)
        self.get_logger().info(f"Vision event: {message.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorQrDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
