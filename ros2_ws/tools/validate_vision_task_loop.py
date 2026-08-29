#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


def call_arm(node, client, enabled):
    request = SetBool.Request()
    request.data = enabled
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None or not future.result().success:
        raise RuntimeError("Failed to change vision trigger arm state.")


def main():
    rclpy.init()
    node = Node("validate_vision_task_loop")
    command_publisher = node.create_publisher(String, "/tasks/command", 10)
    arm_client = node.create_client(SetBool, "/tasks/arm_vision")
    latest_status = {"value": None}
    latest_pose = {"value": None}
    observed_qr_values = set()
    observed_qr_messages = []
    observed_events = []

    node.create_subscription(
        String,
        "/tasks/status",
        lambda message: latest_status.__setitem__(
            "value",
            json.loads(message.data),
        ),
        10,
    )
    def qr_callback(message):
        observed_qr_values.add(message.data)
        observed_qr_messages.append((time.monotonic(), message.data))

    node.create_subscription(String, "/vision/qr", qr_callback, 10)

    def event_callback(message):
        try:
            observed_events.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass

    node.create_subscription(String, "/vision/events", event_callback, 10)
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        lambda message: latest_pose.__setitem__("value", message),
        10,
    )

    try:
        if not arm_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/tasks/arm_vision service is unavailable.")

        discovery_deadline = time.monotonic() + 15.0
        while (
            rclpy.ok()
            and command_publisher.get_subscription_count() == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if command_publisher.get_subscription_count() == 0:
            raise RuntimeError("Task manager did not subscribe to commands.")

        # Keep task triggering disarmed during the approach. Otherwise a
        # mapped color seen along the route can consume the one-shot permit
        # before the QR marker enters the camera view.
        call_arm(node, arm_client, False)
        wait_status_deadline = time.monotonic() + 10.0
        while latest_status["value"] is None and time.monotonic() < wait_status_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest_status["value"] is None:
            raise RuntimeError("No task status received before the vision test.")

        baseline_completed = int(latest_status["value"]["completed_count"])
        baseline_failed = int(latest_status["value"]["failed_count"])
        command = String()
        command.data = "goto qr_station"
        command_publisher.publish(command)
        print("VISION_LOOP_COMMAND=goto qr_station")
        print("VISION_LOOP_ARMED_DURING_APPROACH=false")

        started = time.monotonic()
        deadline = started + 150.0
        saw_qr_station = False
        saw_home = False
        armed_at_qr_station = False
        arm_event_index = 0
        arm_time = None
        last_progress = None

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            status = latest_status["value"]
            if status is None:
                continue

            active = status.get("active_task") or {}
            active_name = active.get("name", "")
            if active_name == "qr_station":
                saw_qr_station = True
            if active_name == "home":
                saw_home = True

            progress = (
                status["state"],
                active_name,
                status["completed_count"],
                len(status["queue"]),
            )
            if progress != last_progress:
                print(
                    "VISION_LOOP_PROGRESS="
                    f"state:{progress[0]},active:{active_name or 'none'},"
                    f"completed:{progress[2]},queued:{progress[3]}"
                )
                last_progress = progress

            if status["failed_count"] > baseline_failed or status["state"] == "FAILED":
                raise RuntimeError(
                    f"Vision task loop failed: {status['last_error']}"
                )

            completed_names = [item["name"] for item in status["completed"]]
            new_completed_names = completed_names[baseline_completed:]
            qr_station_finished = (
                saw_qr_station
                and "qr_station" in new_completed_names
                and status["active_task"] is None
                and not status["queue"]
            )
            if not armed_at_qr_station and qr_station_finished:
                arm_event_index = len(observed_events)
                arm_time = time.monotonic()
                call_arm(node, arm_client, True)
                armed_at_qr_station = True
                print("VISION_LOOP_ARMED_AT_QR_STATION=true")

            qr_event_seen = any(
                event.get("type") == "qr"
                and event.get("value") == "NAV:HOME"
                for event in observed_events[arm_event_index:]
            )
            if (
                armed_at_qr_station
                and saw_home
                and qr_event_seen
                and "home" in new_completed_names
                and status["active_task"] is None
                and not status["queue"]
            ):
                break
        else:
            raise RuntimeError("Vision-triggered return-home loop timed out.")

        pose = latest_pose["value"]
        if pose is None:
            raise RuntimeError("No AMCL pose received.")
        position = pose.pose.pose.position
        home_error = math.hypot(position.x, position.y)
        duration = time.monotonic() - started
        qr_values_after_arm = {
            value
            for observed_time, value in observed_qr_messages
            if arm_time is not None and observed_time >= arm_time
        }

        print(f"VISION_LOOP_QR_VALUES={','.join(sorted(observed_qr_values))}")
        print(
            "VISION_LOOP_QR_VALUES_AFTER_ARM="
            + ",".join(sorted(qr_values_after_arm))
        )
        print(
            "VISION_LOOP_EVENTS="
            + json.dumps(observed_events, ensure_ascii=False)
        )
        print(f"VISION_LOOP_DURATION_SEC={duration:.3f}")
        print(f"VISION_LOOP_HOME_ERROR={home_error:.3f}")
        if "NAV:HOME" not in qr_values_after_arm:
            raise RuntimeError("Live QR output never reported NAV:HOME after arming.")
        if home_error > 0.30:
            raise RuntimeError("Vision-triggered return home ended too far away.")

        print("VISION_TASK_LOOP_VALIDATION_OK")
    finally:
        try:
            if arm_client.service_is_ready():
                call_arm(node, arm_client, False)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
