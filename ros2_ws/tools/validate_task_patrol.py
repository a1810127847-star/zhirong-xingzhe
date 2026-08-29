#!/usr/bin/env python3
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Int8, String


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", default="patrol demo")
    parser.add_argument("--expected-count", type=int, default=4)
    parser.add_argument(
        "--expected-names",
        default="east,northeast,north,home",
    )
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--final-x", type=float, default=0.0)
    parser.add_argument("--final-y", type=float, default=0.0)
    parser.add_argument("--final-tolerance", type=float, default=0.30)
    parser.add_argument("--arrival-window", type=float, default=5.0)
    parser.add_argument("--arrival-distance", type=float, default=0.30)
    parser.add_argument("--arrival-linear-limit", type=float, default=0.10)
    parser.add_argument("--angular-deadband", type=float, default=0.04)
    parser.add_argument("--min-reversal-angle", type=float, default=0.04)
    parser.add_argument("--max-arrival-reversals", type=int, default=0)
    parser.add_argument("--max-arrival-wobble", type=float, default=0.12)
    parser.add_argument("--min-arrival-samples", type=int, default=5)
    return parser.parse_args()


def arrival_motion_metrics(
    samples,
    window_sec,
    linear_limit,
    angular_deadband,
    min_reversal_angle,
    max_distance,
    require_low_speed,
):
    if not samples:
        return 0, 0.0, 0, []

    cutoff = samples[-1]["time"] - window_sec
    arrival = [
        sample
        for sample in samples
        if sample["time"] >= cutoff
        and sample["distance"] <= max_distance
        and (
            not require_low_speed
            or abs(sample["linear"]) <= linear_limit
        )
    ]

    absolute_rotation = 0.0
    net_rotation = 0.0
    signed_runs = []
    for previous, current in zip(arrival, arrival[1:]):
        delta_time = max(0.0, current["time"] - previous["time"])
        angular = 0.5 * (previous["angular"] + current["angular"])
        absolute_rotation += abs(angular) * delta_time
        net_rotation += angular * delta_time
        if abs(angular) < angular_deadband:
            continue
        sign = 1 if angular > 0.0 else -1
        impulse = abs(angular) * delta_time
        if signed_runs and signed_runs[-1][0] == sign:
            signed_runs[-1][1] += impulse
        else:
            signed_runs.append([sign, impulse])
    significant_runs = [
        (sign, impulse)
        for sign, impulse in signed_runs
        if impulse >= min_reversal_angle
    ]
    reversals = max(0, len(significant_runs) - 1)
    wobble = max(0.0, absolute_rotation - abs(net_rotation))
    return reversals, wobble, len(arrival), significant_runs


def path_turn_metrics(path):
    points = []
    for pose in path.poses:
        point = (float(pose.pose.position.x), float(pose.pose.position.y))
        if not points or math.hypot(
            point[0] - points[-1][0], point[1] - points[-1][1]
        ) >= 0.02:
            points.append(point)
    headings = [
        math.atan2(current[1] - previous[1], current[0] - previous[0])
        for previous, current in zip(points, points[1:])
    ]
    turns = []
    for previous, current in zip(headings, headings[1:]):
        delta = math.atan2(
            math.sin(current - previous),
            math.cos(current - previous),
        )
        if abs(delta) >= 0.01:
            turns.append(delta)
    signs = [1 if turn > 0.0 else -1 for turn in turns]
    reversals = sum(
        previous != current for previous, current in zip(signs, signs[1:])
    )
    absolute_turn = sum(abs(turn) for turn in turns)
    net_turn = sum(turns)
    wobble = max(0.0, absolute_turn - abs(net_turn))
    return reversals, wobble, len(points)


def main():
    args = parse_args()
    rclpy.init()
    node = Node("validate_task_patrol")
    latest_status = {"value": None}
    latest_pose = {"value": None}
    latest_route_turn_sign = {"value": 0}
    pose_history = []
    command_samples = {}
    diagnostic_command_samples = {
        "controller": {},
        "smoothed": {},
        "guarded": {},
    }
    watched_tasks = {}
    completed_task_ids = set()
    smoothed_paths = []
    command_publisher = node.create_publisher(String, "/tasks/command", 10)

    def status_callback(message):
        status = json.loads(message.data)
        latest_status["value"] = status
        candidates = []
        if status.get("active_task"):
            candidates.append(status["active_task"])
        candidates.extend(status.get("queue", []))
        for task in candidates:
            task_id = task.get("task_id")
            if task_id:
                watched_tasks[task_id] = task
        for task in status.get("completed", []):
            task_id = task.get("task_id")
            if task_id:
                completed_task_ids.add(task_id)

    def record_velocity(message, sample_store):
        status = latest_status["value"]
        if status is None or status.get("state") != "NAVIGATING":
            return
        pose = latest_pose["value"]
        if pose is None:
            return
        position = pose.pose.pose.position
        for task_id, task in watched_tasks.items():
            # Seal a waypoint's arrival window as soon as the task manager
            # completes it.  Commands for the following route leg must not be
            # misclassified as head-wag at the previous point.
            if task_id in completed_task_ids:
                continue
            distance = math.hypot(
                position.x - float(task["x"]),
                position.y - float(task["y"]),
            )
            if distance > args.arrival_distance:
                continue
            sample_store.setdefault(
                task_id,
                {"name": task.get("name", "unknown"), "samples": []},
            )["samples"].append(
                {
                    "time": time.monotonic(),
                    "linear": float(message.linear.x),
                    "angular": float(message.angular.z),
                    "distance": distance,
                    "route_turn_sign": latest_route_turn_sign["value"],
                }
            )

    def pose_callback(message):
        latest_pose["value"] = message
        position = message.pose.pose.position
        pose_history.append((float(position.x), float(position.y)))

    def path_callback(message):
        smoothed_paths.append(message)

    node.create_subscription(
        String,
        "/tasks/status",
        status_callback,
        10,
    )
    node.create_subscription(
        Int8,
        "/tasks/route_turn_sign",
        lambda message: latest_route_turn_sign.update(value=int(message.data)),
        10,
    )
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        pose_callback,
        10,
    )
    node.create_subscription(
        Twist,
        "/cmd_vel_safe",
        lambda message: record_velocity(message, command_samples),
        50,
    )
    for topic, label in (
        ("/cmd_vel_nav", "controller"),
        ("/cmd_vel_nav_smoothed", "smoothed"),
        ("/cmd_vel", "guarded"),
    ):
        node.create_subscription(
            Twist,
            topic,
            lambda message, sample_store=diagnostic_command_samples[label]: (
                record_velocity(message, sample_store)
            ),
            50,
        )
    node.create_subscription(Path, "/plan_smoothed", path_callback, 10)

    try:
        discovery_deadline = time.monotonic() + 15.0
        while (
            rclpy.ok()
            and command_publisher.get_subscription_count() == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if command_publisher.get_subscription_count() == 0:
            raise RuntimeError("Task manager did not subscribe to /tasks/command.")

        status_deadline = time.monotonic() + 10.0
        while latest_status["value"] is None and time.monotonic() < status_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest_status["value"] is None:
            raise RuntimeError("No initial /tasks/status message received.")
        baseline_completed = latest_status["value"]["completed_count"]
        baseline_failed = latest_status["value"]["failed_count"]
        target_completed = baseline_completed + args.expected_count

        command = String()
        command.data = args.command
        command_publisher.publish(command)
        print(f"TASK_COMMAND={args.command}")

        started = time.monotonic()
        deadline = started + args.timeout
        last_state = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            status = latest_status["value"]
            if status is None:
                continue

            state = status["state"]
            active = status.get("active_task") or {}
            state_key = (
                state,
                active.get("name", ""),
                status["completed_count"],
            )
            if state_key != last_state:
                print(
                    "TASK_PROGRESS="
                    f"state:{state},active:{active.get('name', 'none')},"
                    f"completed:{status['completed_count']},"
                    f"queued:{len(status['queue'])}"
                )
                last_state = state_key

            if status["failed_count"] > baseline_failed or state == "FAILED":
                raise RuntimeError(
                    f"Task manager reported failure: {status['last_error']}"
                )

            if (
                status["completed_count"] >= target_completed
                and status["active_task"] is None
                and len(status["queue"]) == 0
            ):
                break
        else:
            raise RuntimeError(
                f"Patrol did not finish within {args.timeout:.1f} seconds."
            )

        status = latest_status["value"]
        completed_names = [
            item["name"] for item in status["completed"][-args.expected_count :]
        ]
        expected_names = [
            name.strip()
            for name in args.expected_names.split(",")
            if name.strip()
        ]
        if completed_names != expected_names:
            raise RuntimeError(
                f"Unexpected patrol order: {completed_names}, "
                f"expected {expected_names}."
            )

        pose = latest_pose["value"]
        if pose is None:
            raise RuntimeError("No /amcl_pose received during patrol.")
        position = pose.pose.pose.position
        final_error = math.hypot(
            position.x - args.final_x,
            position.y - args.final_y,
        )
        duration = time.monotonic() - started

        print(f"TASK_COMPLETED={','.join(completed_names)}")
        print(f"TASK_DURATION_SEC={duration:.3f}")
        print(f"TASK_FINAL_MAP={position.x:.3f},{position.y:.3f}")
        print(f"TASK_FINAL_ERROR={final_error:.3f}")
        for index, path in enumerate(smoothed_paths, start=1):
            path_reversals, path_wobble, path_points = path_turn_metrics(path)
            print(f"TASK_DIAGNOSTIC_PATH_{index}_POINTS={path_points}")
            print(
                f"TASK_DIAGNOSTIC_PATH_{index}_TURN_REVERSALS="
                f"{path_reversals}"
            )
            print(
                f"TASK_DIAGNOSTIC_PATH_{index}_TURN_WOBBLE_RAD="
                f"{path_wobble:.4f}"
            )
        completed_tasks = status["completed"][-args.expected_count :]
        for task in completed_tasks:
            minimum_distance = min(
                (
                    math.hypot(
                        x - float(task["x"]),
                        y - float(task["y"]),
                    )
                    for x, y in pose_history
                ),
                default=float("inf"),
            )
            print(
                f"TASK_MIN_DISTANCE_{task['name'].upper()}="
                f"{minimum_distance:.3f}"
            )
        if final_error > args.final_tolerance:
            raise RuntimeError(
                f"Final position error {final_error:.3f} m exceeds "
                f"{args.final_tolerance:.3f} m."
            )

        arrival_failures = []
        for task in completed_tasks:
            task_id = task["task_id"]
            record = command_samples.get(task_id, {"samples": []})
            pass_through = bool(task.get("pass_through", False))
            reversals, wobble, sample_count, significant_runs = arrival_motion_metrics(
                record["samples"],
                args.arrival_window,
                args.arrival_linear_limit,
                args.angular_deadband,
                args.min_reversal_angle,
                args.arrival_distance,
                require_low_speed=not pass_through,
            )
            name = task["name"]
            print(f"TASK_ARRIVAL_SAMPLES_{name.upper()}={sample_count}")
            print(f"TASK_ARRIVAL_REVERSALS_{name.upper()}={reversals}")
            print(f"TASK_ARRIVAL_WOBBLE_RAD_{name.upper()}={wobble:.4f}")
            print(
                f"TASK_ARRIVAL_SIGN_RUNS_{name.upper()}="
                + ",".join(
                    f"{'+' if sign > 0 else '-'}{impulse:.4f}"
                    for sign, impulse in significant_runs
                )
            )
            for label, sample_store in diagnostic_command_samples.items():
                diagnostic_record = sample_store.get(task_id, {"samples": []})
                (
                    diagnostic_reversals,
                    diagnostic_wobble,
                    diagnostic_count,
                    diagnostic_runs,
                ) = arrival_motion_metrics(
                    diagnostic_record["samples"],
                    args.arrival_window,
                    args.arrival_linear_limit,
                    args.angular_deadband,
                    args.min_reversal_angle,
                    args.arrival_distance,
                    require_low_speed=not pass_through,
                )
                print(
                    f"TASK_DIAGNOSTIC_{label.upper()}_{name.upper()}="
                    f"samples:{diagnostic_count},"
                    f"reversals:{diagnostic_reversals},"
                    f"wobble:{diagnostic_wobble:.4f},runs:"
                    + ",".join(
                        f"{'+' if sign > 0 else '-'}{impulse:.4f}"
                        for sign, impulse in diagnostic_runs
                    )
                )
                if label == "guarded":
                    wrong_way_impulse = 0.0
                    inactive_impulse = 0.0
                    for previous, current in zip(
                        diagnostic_record["samples"],
                        diagnostic_record["samples"][1:],
                    ):
                        delta_time = max(
                            0.0,
                            current["time"] - previous["time"],
                        )
                        angular = 0.5 * (
                            previous["angular"] + current["angular"]
                        )
                        sign = int(current.get("route_turn_sign", 0))
                        if sign and angular * sign < 0.0:
                            wrong_way_impulse += abs(angular) * delta_time
                        elif not sign:
                            inactive_impulse += abs(angular) * delta_time
                    print(
                        f"TASK_DIAGNOSTIC_GUARD_STATE_{name.upper()}="
                        f"wrong_way:{wrong_way_impulse:.4f},"
                        f"inactive:{inactive_impulse:.4f}"
                    )
            if sample_count < args.min_arrival_samples:
                arrival_failures.append(
                    f"{name} has only {sample_count} velocity samples "
                    f"(< {args.min_arrival_samples})"
                )
            if reversals > args.max_arrival_reversals:
                arrival_failures.append(
                    f"{name} reversals {reversals} > "
                    f"{args.max_arrival_reversals}"
                )
            if wobble > args.max_arrival_wobble:
                arrival_failures.append(
                    f"{name} wobble {wobble:.4f} rad > "
                    f"{args.max_arrival_wobble:.4f} rad"
                )
        if arrival_failures:
            raise RuntimeError(
                "Arrival head-wag exceeded limits: "
                + "; ".join(arrival_failures)
            )

        print("TASK_PATROL_VALIDATION_OK")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
