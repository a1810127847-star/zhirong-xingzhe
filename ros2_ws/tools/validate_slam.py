#!/usr/bin/env python3
import argparse
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SlamValidator(Node):
    def __init__(self):
        super().__init__("zhirong_slam_validator")
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_message = None
        self.map_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            map_qos,
        )

    def on_map(self, message):
        self.map_message = message
        self.map_count += 1


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--minimum-known-cells", type=int, default=1000)
    parser.add_argument("--minimum-map-messages", type=int, default=2)
    parser.add_argument("--skip-tf", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    validator = SlamValidator()
    deadline = time.monotonic() + args.timeout
    map_to_odom = None

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(validator, timeout_sec=0.2)
            if not args.skip_tf:
                try:
                    map_to_odom = validator.tf_buffer.lookup_transform(
                        "map",
                        "odom",
                        Time(),
                        timeout=Duration(seconds=0.1),
                    )
                except TransformException:
                    map_to_odom = None

            if (
                validator.map_message is not None
                and validator.map_count >= args.minimum_map_messages
                and (args.skip_tf or map_to_odom is not None)
            ):
                break

        if validator.map_message is None:
            raise RuntimeError("Timed out waiting for /map.")
        if validator.map_count < args.minimum_map_messages:
            raise RuntimeError(
                f"Only received {validator.map_count} /map messages; "
                f"expected at least {args.minimum_map_messages}."
            )
        if not args.skip_tf and map_to_odom is None:
            raise RuntimeError("Timed out waiting for map -> odom TF.")

        grid = validator.map_message
        if grid.header.frame_id.lstrip("/") != "map":
            raise RuntimeError(f"Unexpected map frame: {grid.header.frame_id!r}.")
        if grid.info.width <= 0 or grid.info.height <= 0:
            raise RuntimeError("Map dimensions are empty.")
        if len(grid.data) != grid.info.width * grid.info.height:
            raise RuntimeError("Map data length does not match its dimensions.")
        if abs(grid.info.resolution - 0.05) > 1e-6:
            raise RuntimeError(
                f"Map resolution is {grid.info.resolution}, expected 0.05 m/cell."
            )

        known_cells = sum(value >= 0 for value in grid.data)
        free_cells = sum(value == 0 for value in grid.data)
        occupied_cells = sum(value >= 50 for value in grid.data)
        unknown_cells = len(grid.data) - known_cells

        if known_cells < args.minimum_known_cells:
            raise RuntimeError(
                f"Map has only {known_cells} known cells; "
                f"expected at least {args.minimum_known_cells}."
            )
        if free_cells < 100:
            raise RuntimeError("Map contains too few free cells.")
        if occupied_cells < 20:
            raise RuntimeError("Map contains too few occupied cells.")

        print(f"MAP_FRAME={grid.header.frame_id}")
        print(
            f"MAP_SIZE={grid.info.width}x{grid.info.height}"
            f":{grid.info.resolution:.3f}_m_per_cell"
        )
        print(
            f"MAP_ORIGIN={grid.info.origin.position.x:.3f},"
            f"{grid.info.origin.position.y:.3f}"
        )
        print(f"MAP_KNOWN_CELLS={known_cells}")
        print(f"MAP_FREE_CELLS={free_cells}")
        print(f"MAP_OCCUPIED_CELLS={occupied_cells}")
        print(f"MAP_UNKNOWN_CELLS={unknown_cells}")
        print(f"MAP_MESSAGE_COUNT={validator.map_count}")
        if args.skip_tf:
            print("MAP_TO_ODOM_CHECK=SKIPPED")
        else:
            translation = map_to_odom.transform.translation
            rotation = map_to_odom.transform.rotation
            print(
                "MAP_TO_ODOM_TRANSLATION="
                f"{translation.x:.4f},{translation.y:.4f},{translation.z:.4f}"
            )
            print(
                "MAP_TO_ODOM_ROTATION="
                f"{rotation.x:.4f},{rotation.y:.4f},{rotation.z:.4f},{rotation.w:.4f}"
            )
        print("SLAM_VALIDATION_OK")
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
