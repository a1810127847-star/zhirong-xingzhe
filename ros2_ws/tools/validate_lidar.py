#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarValidator(Node):
    def __init__(self):
        super().__init__("lidar_validator")
        self.scan = None
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.receive_scan,
            qos_profile_sensor_data,
        )

    def receive_scan(self, message):
        self.scan = message


rclpy.init()
node = LidarValidator()
deadline = time.monotonic() + 8.0

while node.scan is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

if node.scan is None:
    raise SystemExit("ERROR: No /scan message received within 8 seconds.")

scan = node.scan
finite_ranges = [
    value
    for value in scan.ranges
    if math.isfinite(value) and scan.range_min <= value <= scan.range_max
]

print(f"SCAN_FRAME={scan.header.frame_id}")
print(f"SCAN_SAMPLES={len(scan.ranges)}")
print(f"SCAN_ANGLE_MIN={scan.angle_min:.4f}")
print(f"SCAN_ANGLE_MAX={scan.angle_max:.4f}")
print(f"SCAN_RANGE_MIN={scan.range_min:.2f}")
print(f"SCAN_RANGE_MAX={scan.range_max:.2f}")
print(f"SCAN_FINITE_COUNT={len(finite_ranges)}")

if scan.header.frame_id != "lidar_link":
    raise SystemExit(
        f"ERROR: Expected lidar_link frame, got {scan.header.frame_id!r}."
    )

if not 700 <= len(scan.ranges) <= 740:
    raise SystemExit(
        f"ERROR: Expected about 720 scan samples, got {len(scan.ranges)}."
    )

if scan.angle_min > -3.0 or scan.angle_max < 3.0:
    raise SystemExit("ERROR: Laser scan does not cover approximately 360 degrees.")

if not finite_ranges:
    raise SystemExit("ERROR: Laser scan contains no finite obstacle ranges.")

nearest = min(finite_ranges)
print(f"SCAN_NEAREST_OBSTACLE={nearest:.3f}")
if not 1.70 <= nearest <= 2.00:
    raise SystemExit(
        "ERROR: Nearest obstacle is outside the expected 1.70-2.00 m range."
    )

node.destroy_node()
rclpy.shutdown()
print("LIDAR_VALIDATION_OK")
