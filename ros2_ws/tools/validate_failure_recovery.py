#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String


def publish_command(publisher, text):
    message = String()
    message.data = text
    publisher.publish(message)
    print(f"RECOVERY_COMMAND={text}")


def main():
    rclpy.init()
    node = Node("validate_failure_recovery")
    publisher = node.create_publisher(String, "/tasks/command", 10)
    latest_status = {"value": None}
    latest_pose = {"value": None}

    node.create_subscription(
        String,
        "/tasks/status",
        lambda message: latest_status.__setitem__(
            "value",
            json.loads(message.data),
        ),
        10,
    )
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        lambda message: latest_pose.__setitem__("value", message),
        10,
    )

    try:
        discovery_deadline = time.monotonic() + 15.0
        while (
            rclpy.ok()
            and (
                publisher.get_subscription_count() == 0
                or latest_status["value"] is None
            )
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("Task manager command subscriber is unavailable.")
        if latest_status["value"] is None:
            raise RuntimeError("Task status is unavailable.")

        baseline_completed = latest_status["value"]["completed_count"]
        baseline_failed = latest_status["value"]["failed_count"]

        publish_command(publisher, "goto 4.5 4.5 0")
        failure_started = time.monotonic()
        failure_deadline = failure_started + 150.0
        last_progress = None

        while rclpy.ok() and time.monotonic() < failure_deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            status = latest_status["value"]
            active = status.get("active_task") or {}
            progress = (
                status["state"],
                active.get("attempt", 0),
                status["failed_count"],
            )
            if progress != last_progress:
                print(
                    "RECOVERY_FAILURE_PROGRESS="
                    f"state:{progress[0]},attempt:{progress[1]},"
                    f"failed:{progress[2]}"
                )
                last_progress = progress

            if status["failed_count"] > baseline_failed:
                break
        else:
            raise RuntimeError("Unreachable goal did not enter FAILED state.")

        failure_status = latest_status["value"]
        failure_record = failure_status["failed"][-1]
        failure_duration = time.monotonic() - failure_started
        if failure_record["attempt"] != 2:
            raise RuntimeError(
                f"Expected two attempts, got {failure_record['attempt']}."
            )
        print(f"RECOVERY_FAILURE_ATTEMPTS={failure_record['attempt']}")
        print(f"RECOVERY_FAILURE_ERROR={failure_record['error']}")
        print(f"RECOVERY_FAILURE_DURATION_SEC={failure_duration:.3f}")

        publish_command(publisher, "goto east")
        recovery_started = time.monotonic()
        recovery_deadline = recovery_started + 100.0
        while rclpy.ok() and time.monotonic() < recovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            status = latest_status["value"]
            if status["completed_count"] >= baseline_completed + 1:
                if status["completed"][-1]["name"] == "east":
                    break
        else:
            raise RuntimeError("Valid task did not recover after failure.")

        print(
            "RECOVERY_VALID_TASK_DURATION_SEC="
            f"{time.monotonic() - recovery_started:.3f}"
        )

        publish_command(publisher, "return_home")
        home_deadline = time.monotonic() + 100.0
        while rclpy.ok() and time.monotonic() < home_deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            status = latest_status["value"]
            if (
                status["completed_count"] >= baseline_completed + 2
                and status["completed"][-1]["name"] == "home"
                and status["active_task"] is None
                and not status["queue"]
            ):
                break
        else:
            raise RuntimeError("Return home did not complete after recovery.")

        pose = latest_pose["value"]
        if pose is None:
            raise RuntimeError("No AMCL pose received.")
        position = pose.pose.pose.position
        home_error = math.hypot(position.x, position.y)
        print(f"RECOVERY_HOME_ERROR={home_error:.3f}")
        if home_error > 0.30:
            raise RuntimeError("Recovered task sequence ended too far from home.")

        print("FAILURE_RECOVERY_VALIDATION_OK")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
