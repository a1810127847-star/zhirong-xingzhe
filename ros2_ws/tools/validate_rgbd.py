#!/usr/bin/env python3
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


EXPECTED_FRAME = "camera_optical_link"
EXPECTED_WIDTH = 320
EXPECTED_HEIGHT = 240


class RgbdValidator(Node):
    def __init__(self):
        super().__init__("zhirong_rgbd_validator")
        self.color = None
        self.depth = None
        self.color_info = None
        self.depth_info = None
        self.cloud = None
        self.color_count = 0
        self.depth_count = 0
        self.cloud_count = 0

        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.on_color_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/depth/camera_info",
            self.on_depth_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/camera/depth/points",
            self.on_cloud,
            qos_profile_sensor_data,
        )

    def on_color(self, message):
        self.color = message
        self.color_count += 1

    def on_depth(self, message):
        self.depth = message
        self.depth_count += 1

    def on_color_info(self, message):
        self.color_info = message

    def on_depth_info(self, message):
        self.depth_info = message

    def on_cloud(self, message):
        self.cloud = message
        self.cloud_count += 1

    def ready(self):
        return (
            self.color is not None
            and self.depth is not None
            and self.color_info is not None
            and self.depth_info is not None
            and self.cloud is not None
            and self.color_count >= 5
            and self.depth_count >= 5
            and self.cloud_count >= 3
        )


def check_frame(message, label):
    frame = message.header.frame_id.lstrip("/")
    if frame != EXPECTED_FRAME:
        raise RuntimeError(
            f"{label} frame is {message.header.frame_id!r}, expected {EXPECTED_FRAME!r}."
        )


def check_dimensions(message, label):
    if message.width != EXPECTED_WIDTH or message.height != EXPECTED_HEIGHT:
        raise RuntimeError(
            f"{label} is {message.width}x{message.height}, "
            f"expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}."
        )


def validate_color(message):
    check_frame(message, "Color image")
    check_dimensions(message, "Color image")
    if message.encoding not in {"rgb8", "bgr8"}:
        raise RuntimeError(f"Unexpected color encoding: {message.encoding!r}.")
    if message.step < message.width * 3:
        raise RuntimeError("Color image row step is too small.")

    stride = max(1, len(message.data) // 1000)
    sampled_values = set(message.data[::stride])
    if len(sampled_values) < 3:
        raise RuntimeError("Color image does not contain enough visual variation.")


def validate_depth(message):
    check_frame(message, "Depth image")
    check_dimensions(message, "Depth image")
    if message.encoding != "32FC1":
        raise RuntimeError(f"Unexpected depth encoding: {message.encoding!r}.")
    if message.step < message.width * 4:
        raise RuntimeError("Depth image row step is too small.")

    byte_order = ">" if message.is_bigendian else "<"
    sample_stride = max(1, (message.width * message.height) // 1200)
    finite_depths = []
    for flat_index in range(0, message.width * message.height, sample_stride):
        row, column = divmod(flat_index, message.width)
        offset = row * message.step + column * 4
        value = struct.unpack_from(f"{byte_order}f", message.data, offset)[0]
        if math.isfinite(value) and value > 0.0:
            finite_depths.append(value)

    if len(finite_depths) < 10:
        raise RuntimeError("Depth image contains too few finite measurements.")
    if min(finite_depths) < 0.10 or min(finite_depths) > 8.5:
        raise RuntimeError("Depth measurements are outside the configured range.")
    return min(finite_depths), len(finite_depths)


def validate_camera_info(message, label):
    check_frame(message, label)
    check_dimensions(message, label)
    if len(message.k) != 9 or message.k[0] <= 0.0 or message.k[4] <= 0.0:
        raise RuntimeError(f"{label} does not contain a valid camera matrix.")


def validate_cloud(message):
    check_frame(message, "Point cloud")
    if message.point_step <= 0 or not message.data:
        raise RuntimeError("Point cloud is empty.")

    fields = {field.name: field for field in message.fields}
    for name in ("x", "y", "z"):
        if name not in fields:
            raise RuntimeError(f"Point cloud is missing the {name!r} field.")
        if fields[name].datatype != PointField.FLOAT32:
            raise RuntimeError(f"Point cloud field {name!r} is not FLOAT32.")

    byte_order = ">" if message.is_bigendian else "<"
    total_points = message.width * message.height
    sample_stride = max(1, total_points // 1200)
    ranges = []

    for flat_index in range(0, total_points, sample_stride):
        row, column = divmod(flat_index, message.width)
        base = row * message.row_step + column * message.point_step
        xyz = [
            struct.unpack_from(
                f"{byte_order}f",
                message.data,
                base + fields[axis].offset,
            )[0]
            for axis in ("x", "y", "z")
        ]
        if all(math.isfinite(value) for value in xyz):
            distance = math.sqrt(sum(value * value for value in xyz))
            if distance > 0.0:
                ranges.append(distance)

    if len(ranges) < 10:
        raise RuntimeError("Point cloud contains too few finite points.")
    if min(ranges) < 0.10 or min(ranges) > 8.5:
        raise RuntimeError("Point cloud measurements are outside the configured range.")
    return min(ranges), len(ranges), sorted(fields)


def main():
    rclpy.init()
    validator = RgbdValidator()
    deadline = time.monotonic() + 30.0

    try:
        while rclpy.ok() and not validator.ready() and time.monotonic() < deadline:
            rclpy.spin_once(validator, timeout_sec=0.2)

        if not validator.ready():
            raise RuntimeError(
                "Timed out waiting for color, depth, camera-info, and point-cloud data."
            )

        validate_color(validator.color)
        min_depth, finite_depth_count = validate_depth(validator.depth)
        validate_camera_info(validator.color_info, "Color camera info")
        validate_camera_info(validator.depth_info, "Depth camera info")
        min_cloud_range, finite_cloud_count, cloud_fields = validate_cloud(
            validator.cloud
        )

        print(f"RGBD_FRAME={validator.color.header.frame_id}")
        print(
            f"RGB_IMAGE={validator.color.width}x{validator.color.height}"
            f":{validator.color.encoding}"
        )
        print(
            f"DEPTH_IMAGE={validator.depth.width}x{validator.depth.height}"
            f":{validator.depth.encoding}"
        )
        print(f"DEPTH_FINITE_SAMPLE_COUNT={finite_depth_count}")
        print(f"DEPTH_NEAREST_SAMPLE={min_depth:.3f}")
        print(
            f"POINT_CLOUD={validator.cloud.width}x{validator.cloud.height}"
            f":{validator.cloud.point_step}_bytes"
        )
        print(f"POINT_CLOUD_FIELDS={','.join(cloud_fields)}")
        print(f"POINT_CLOUD_FINITE_SAMPLE_COUNT={finite_cloud_count}")
        print(f"POINT_CLOUD_NEAREST_SAMPLE={min_cloud_range:.3f}")
        print(
            "RGBD_MESSAGE_COUNTS="
            f"color:{validator.color_count},"
            f"depth:{validator.depth_count},"
            f"cloud:{validator.cloud_count}"
        )
        print("RGBD_VALIDATION_OK")
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
