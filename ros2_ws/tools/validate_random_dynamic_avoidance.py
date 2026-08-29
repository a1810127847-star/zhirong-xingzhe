#!/usr/bin/env python3
"""Reproducible randomized dynamic-obstacle robustness acceptance test."""

from __future__ import annotations

import argparse
import math
import random
import time

import rclpy
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8


ROBOT_NAME = "zhirong_diffbot"
OBSTACLE_RADIUS = 0.22
PARKED_DYNAMIC_OBSTACLE = (-2.35, -2.35)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=3)
    parser.add_argument(
        "--only-case",
        type=int,
        default=0,
        help="Run one generated case while preserving the master seed order.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="0 creates a fresh seed; use a printed seed to replay a run.",
    )
    parser.add_argument("--case-timeout", type=float, default=150.0)
    parser.add_argument("--min-goal-distance", type=float, default=2.0)
    parser.add_argument("--min-obstacles", type=int, default=1)
    parser.add_argument("--max-obstacles", type=int, default=4)
    return parser.parse_args()


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def point_segment_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-9:
        return distance(point, start)
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return distance(point, projection)


def make_pose_stamped(node, x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def send_goal(node, client, target, behavior_tree=""):
    goal = NavigateToPose.Goal()
    goal.pose = target
    goal.behavior_tree = behavior_tree
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("Timed out sending navigation goal.")
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("Goal Guard rejected a randomized test goal.")
    return handle, handle.get_result_async()


def cancel_goal(node, handle):
    future = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def wait_for(node, predicate, description, timeout=20.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise RuntimeError(f"Timed out waiting for {description}.")


def map_cell_is_free(grid, x, y, clearance):
    info = grid.info
    resolution = info.resolution
    origin = info.origin.position
    center_x = int((x - origin.x) / resolution)
    center_y = int((y - origin.y) / resolution)
    radius_cells = max(1, int(math.ceil(clearance / resolution)))
    if (
        center_x - radius_cells < 0
        or center_y - radius_cells < 0
        or center_x + radius_cells >= info.width
        or center_y + radius_cells >= info.height
    ):
        return False
    radius_squared = radius_cells * radius_cells
    for offset_y in range(-radius_cells, radius_cells + 1):
        for offset_x in range(-radius_cells, radius_cells + 1):
            if offset_x * offset_x + offset_y * offset_y > radius_squared:
                continue
            index = (center_y + offset_y) * info.width + center_x + offset_x
            if grid.data[index] < 0 or grid.data[index] >= 50:
                return False
    return True


def choose_far_goal(grid, start, heading, rng, minimum_distance):
    info = grid.info
    min_x = info.origin.position.x + 0.55
    min_y = info.origin.position.y + 0.55
    max_x = info.origin.position.x + info.width * info.resolution - 0.55
    max_y = info.origin.position.y + info.height * info.resolution - 0.55
    candidates = []
    for _ in range(1200):
        point = (rng.uniform(min_x, max_x), rng.uniform(min_y, max_y))
        point_distance = distance(start, point)
        if point_distance < minimum_distance:
            continue
        point_heading = math.atan2(point[1] - start[1], point[0] - start[0])
        if abs(normalize_angle(point_heading - heading)) > math.radians(35.0):
            continue
        if distance(point, PARKED_DYNAMIC_OBSTACLE) < 0.70:
            continue
        if map_cell_is_free(grid, point[0], point[1], 0.48):
            candidates.append((point_distance, point))
    if not candidates:
        raise RuntimeError("No far, collision-free goal could be sampled.")
    candidates.sort(key=lambda item: item[0], reverse=True)
    # Randomness is retained, but only among the farthest valid candidates.
    far_pool = candidates[: min(16, len(candidates))]
    return rng.choice(far_pool)[1]


def path_points(path):
    return [(item.pose.position.x, item.pose.position.y) for item in path.poses]


def cumulative_path(points):
    cumulative = [0.0]
    for index in range(1, len(points)):
        cumulative.append(cumulative[-1] + distance(points[index - 1], points[index]))
    return cumulative


def interpolate_path(points, cumulative, target):
    for index in range(1, len(points)):
        if cumulative[index] < target:
            continue
        segment = cumulative[index] - cumulative[index - 1]
        if segment <= 1e-9:
            return points[index], (1.0, 0.0)
        ratio = (target - cumulative[index - 1]) / segment
        x = points[index - 1][0] + ratio * (points[index][0] - points[index - 1][0])
        y = points[index - 1][1] + ratio * (points[index][1] - points[index - 1][1])
        tangent = (
            (points[index][0] - points[index - 1][0]) / segment,
            (points[index][1] - points[index - 1][1]) / segment,
        )
        return (x, y), tangent
    return points[-1], (1.0, 0.0)


def remaining_path(path, robot):
    points = path_points(path)
    if len(points) < 2:
        raise RuntimeError("Nav2 path is too short for obstacle placement.")
    nearest = min(range(len(points)), key=lambda index: distance(points[index], robot))
    points = points[nearest:]
    if len(points) < 2:
        raise RuntimeError("No usable path remains ahead of the robot.")
    cumulative = cumulative_path(points)
    return points, cumulative


def choose_obstacle_positions(grid, path, robot, goal, count, rng):
    points, cumulative = remaining_path(path, robot)
    path_length = cumulative[-1]
    if path_length < 1.30:
        raise RuntimeError("Not enough remaining path to insert dynamic obstacles.")

    # One primary cylinder overlaps the active route after enough lidar/replan
    # reaction distance. Additional randomized cylinders populate the same-side
    # outer field; they add environmental variation without forming a solid wall
    # across the only available bypass corridor.
    primary_target = min(max(0.82, path_length * 0.42), path_length - 0.52)
    preferred_side = rng.choice((-1.0, 1.0))
    for corridor_side in (preferred_side, -preferred_side):
        positions = []
        for _ in range(60):
            target = max(
                0.76,
                min(path_length - 0.48, primary_target + rng.uniform(-0.08, 0.08)),
            )
            center, tangent = interpolate_path(points, cumulative, target)
            lateral = corridor_side * rng.uniform(0.10, 0.16)
            primary = (
                center[0] - tangent[1] * lateral,
                center[1] + tangent[0] * lateral,
            )
            if distance(primary, robot) < 0.72 or distance(primary, goal) < 0.45:
                continue
            if not map_cell_is_free(grid, primary[0], primary[1], 0.25):
                continue
            positions.append(primary)
            break
        if not positions:
            continue

        while len(positions) < count:
            placed = None
            for _ in range(220):
                target = rng.uniform(0.35, path_length - 0.35)
                center, tangent = interpolate_path(points, cumulative, target)
                lateral = corridor_side * rng.uniform(0.68, 1.18)
                candidate = (
                    center[0] - tangent[1] * lateral,
                    center[1] + tangent[0] * lateral,
                )
                if distance(candidate, robot) < 0.70 or distance(candidate, goal) < 0.55:
                    continue
                if any(distance(candidate, existing) < 0.50 for existing in positions):
                    continue
                if not map_cell_is_free(grid, candidate[0], candidate[1], 0.28):
                    continue
                placed = candidate
                break
            if placed is None:
                break
            positions.append(placed)
        if len(positions) == count:
            return positions
    raise RuntimeError(f"Could not place {count} randomized obstacles with an open bypass corridor.")


def obstacle_sdf(name, color):
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='body'>
      <collision name='collision'>
        <geometry><cylinder><radius>{OBSTACLE_RADIUS}</radius><length>0.72</length></cylinder></geometry>
      </collision>
      <visual name='visual'>
        <geometry><cylinder><radius>{OBSTACLE_RADIUS}</radius><length>0.72</length></cylinder></geometry>
        <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def spawn_obstacle(node, client, name, position, color):
    request = SpawnEntity.Request()
    request.name = name
    request.xml = obstacle_sdf(name, color)
    request.initial_pose = Pose()
    request.initial_pose.position.x = position[0]
    request.initial_pose.position.y = position[1]
    request.initial_pose.position.z = 0.36
    request.initial_pose.orientation.w = 1.0
    request.reference_frame = "world"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    result = future.result() if future.done() else None
    if result is None or not result.success:
        message = result.status_message if result is not None else "timeout"
        raise RuntimeError(f"Failed to spawn {name}: {message}")


def delete_obstacle(node, client, name):
    request = DeleteEntity.Request()
    request.name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)


def wait_navigation(node, handle, result_future, timeout, description):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() >= deadline:
            cancel_goal(node, handle)
            raise RuntimeError(f"{description} timed out.")
    result = result_future.result()
    if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
        status = result.status if result is not None else "none"
        raise RuntimeError(f"{description} failed with status {status}.")


def ensure_home(node, client, model_poses, timeout=110.0):
    pose = model_poses[ROBOT_NAME]
    home_error = math.hypot(pose[0], pose[1])
    if home_error <= 0.25:
        print(f"RANDOM_DYNAMIC_HOME_READY={home_error:.3f}", flush=True)
        return home_error

    # Home acceptance is positional. Point the requested final heading along
    # the approach direction and cancel cleanly once the 0.25 m region is met,
    # avoiding a pointless in-place heading correction at the origin.
    yaw = math.atan2(-pose[1], -pose[0])
    handle, result_future = send_goal(node, client, make_pose_stamped(node, 0.0, 0.0, yaw))
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        pose = model_poses[ROBOT_NAME]
        home_error = math.hypot(pose[0], pose[1])
        if home_error <= 0.25:
            cancel_goal(node, handle)
            spin_for(node, 0.5)
            print(f"RANDOM_DYNAMIC_HOME_READY={home_error:.3f}", flush=True)
            return home_error
        if time.monotonic() >= deadline:
            cancel_goal(node, handle)
            raise RuntimeError("Return home timed out.")
    result = result_future.result()
    if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
        status = result.status if result is not None else "none"
        raise RuntimeError(f"Return home failed with status {status}.")
    pose = model_poses[ROBOT_NAME]
    home_error = math.hypot(pose[0], pose[1])
    print(f"RANDOM_DYNAMIC_HOME_READY={home_error:.3f}", flush=True)
    return home_error


def reverse_home_along_trace(
    node,
    velocity_publisher,
    model_poses,
    model_yaws,
    outbound_trace,
    timeout=100.0,
):
    """Backtrack the verified outbound trace without a 180-degree spin."""
    if len(outbound_trace) < 3:
        raise RuntimeError("Outbound trace is too short for spin-free return.")
    reverse_trace = list(reversed(outbound_trace))
    target_index = 1
    started = time.monotonic()
    deadline = started + timeout
    best_target_distance = float("inf")
    last_target_index = target_index
    last_progress = started
    max_heading_error = 0.0

    def stop_robot():
        zero = Twist()
        stop_deadline = time.monotonic() + 0.8
        while rclpy.ok() and time.monotonic() < stop_deadline:
            velocity_publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.04)

    print("RANDOM_DYNAMIC_RETURN_MODE=REVERSE_TRACE", flush=True)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.03)
        current = model_poses[ROBOT_NAME]
        current_yaw = model_yaws[ROBOT_NAME]
        home_error = math.hypot(current[0], current[1])
        if home_error <= 0.25:
            break

        while (
            target_index < len(reverse_trace) - 1
            and distance(current, reverse_trace[target_index]) < 0.16
        ):
            target_index += 1
        if target_index != last_target_index:
            last_target_index = target_index
            best_target_distance = float("inf")
            last_progress = time.monotonic()
        lookahead_index = target_index
        while (
            lookahead_index < len(reverse_trace) - 1
            and distance(current, reverse_trace[lookahead_index]) < 0.28
        ):
            lookahead_index += 1
        target = reverse_trace[lookahead_index]
        target_distance = distance(current, target)
        if target_distance < best_target_distance - 0.03:
            best_target_distance = target_distance
            last_progress = time.monotonic()
        if time.monotonic() - last_progress > 20.0:
            stop_robot()
            raise RuntimeError("Reverse-trace return stopped making progress.")
        if time.monotonic() >= deadline:
            stop_robot()
            raise RuntimeError("Reverse-trace return timed out.")
        motion_heading = math.atan2(target[1] - current[1], target[0] - current[0])
        desired_body_yaw = normalize_angle(motion_heading + math.pi)
        heading_error = normalize_angle(desired_body_yaw - current_yaw)
        max_heading_error = max(max_heading_error, abs(heading_error))

        command = Twist()
        command.linear.x = -max(0.06, 0.16 * (1.0 - min(abs(heading_error), 0.75) / 1.2))
        if home_error < 0.55:
            command.linear.x = max(command.linear.x, -0.09)
        command.angular.z = max(-0.45, min(0.45, 1.35 * heading_error))
        velocity_publisher.publish(command)

    stop_robot()
    home = model_poses[ROBOT_NAME]
    home_error = math.hypot(home[0], home[1])
    print(
        f"RANDOM_DYNAMIC_REVERSE_RETURN_SEC={time.monotonic() - started:.3f}",
        flush=True,
    )
    print(
        f"RANDOM_DYNAMIC_REVERSE_MAX_HEADING_ERROR={max_heading_error:.3f}",
        flush=True,
    )
    return home_error


def main():
    args = parse_args()
    if not 1 <= args.cases <= 10:
        raise SystemExit("--cases must be between 1 and 10.")
    if args.only_case and not 1 <= args.only_case <= args.cases:
        raise SystemExit("--only-case must be within the generated case range.")
    if not 1 <= args.min_obstacles <= args.max_obstacles <= 6:
        raise SystemExit("Require 1 <= min obstacles <= max obstacles <= 6.")

    master_seed = args.seed or random.SystemRandom().randrange(1, 2**32)
    seed_rng = random.Random(master_seed)
    case_seeds = [seed_rng.randrange(1, 2**32) for _ in range(args.cases)]
    selected_cases = [args.only_case] if args.only_case else list(range(1, args.cases + 1))

    print(f"RANDOM_DYNAMIC_SEED={master_seed}", flush=True)
    print(f"RANDOM_DYNAMIC_CASES={args.cases}", flush=True)
    if args.only_case:
        print(f"RANDOM_DYNAMIC_ONLY_CASE={args.only_case}", flush=True)

    rclpy.init()
    node = Node("validate_random_dynamic_avoidance")
    navigator = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    spawner = node.create_client(SpawnEntity, "/spawn_entity")
    deleter = node.create_client(DeleteEntity, "/delete_entity")
    latest = {"map": None, "pose": None, "path": None, "raw": None, "raw_time": 0.0}
    model_poses = {}
    model_yaws = {}
    plan_counter = {"value": 0}
    current_metrics = {"value": None}
    route_turn_publisher = node.create_publisher(Int8, "/tasks/route_turn_sign", 10)
    return_velocity_publisher = node.create_publisher(Twist, "/cmd_vel_nav", 20)

    map_qos = QoSProfile(depth=1)
    map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    map_qos.reliability = ReliabilityPolicy.RELIABLE
    node.create_subscription(
        OccupancyGrid,
        "/map",
        lambda msg: latest.__setitem__("map", msg),
        map_qos,
    )
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        lambda msg: latest.__setitem__("pose", msg),
        map_qos,
    )

    def path_callback(message):
        if message.poses:
            latest["path"] = message
            plan_counter["value"] += 1

    def raw_callback(message):
        latest["raw"] = message
        latest["raw_time"] = time.monotonic()

    def safe_callback(message):
        metrics = current_metrics["value"]
        if metrics is None:
            return
        now = time.monotonic()
        previous = metrics["last_safe_time"]
        metrics["last_safe_time"] = now
        elapsed = min(0.20, max(0.0, now - previous)) if previous else 0.0
        in_place = abs(message.linear.x) < 0.03 and abs(message.angular.z) > 0.15
        if in_place:
            metrics["in_place_total_sec"] += elapsed
            metrics["in_place_total_rad"] += abs(message.angular.z) * elapsed
            metrics["in_place_episode_sec"] += elapsed
            metrics["max_in_place_episode_sec"] = max(
                metrics["max_in_place_episode_sec"],
                metrics["in_place_episode_sec"],
            )
        else:
            metrics["in_place_episode_sec"] = 0.0

        raw = latest["raw"]
        if raw is None or now - latest["raw_time"] > 0.3:
            return
        raw_speed = abs(raw.linear.x)
        if raw_speed > 0.04 and abs(message.linear.x) < raw_speed * 0.60:
            metrics["speed_reductions"] += 1

    def model_callback(message):
        model_poses.clear()
        model_yaws.clear()
        for name, pose in zip(message.name, message.pose):
            model_poses[name] = (pose.position.x, pose.position.y)
            model_yaws[name] = quaternion_yaw(pose.orientation)
        metrics = current_metrics["value"]
        robot = model_poses.get(ROBOT_NAME)
        if metrics is None or robot is None:
            return
        for name in metrics["active_names"]:
            obstacle = model_poses.get(name)
            if obstacle is not None:
                metrics["min_center_distance"] = min(
                    metrics["min_center_distance"], distance(robot, obstacle)
                )

    node.create_subscription(Path, "/plan", path_callback, 10)
    node.create_subscription(Twist, "/cmd_vel", raw_callback, 10)
    node.create_subscription(Twist, "/cmd_vel_safe", safe_callback, 10)
    node.create_subscription(ModelStates, "/model_states", model_callback, 10)

    active_names = []
    passed = 0
    try:
        if not navigator.wait_for_server(timeout_sec=25.0):
            raise RuntimeError("/navigate_to_pose action is unavailable.")
        if not spawner.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("/spawn_entity service is unavailable.")
        if not deleter.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("/delete_entity service is unavailable.")
        wait_for(node, lambda: latest["map"] is not None, "the occupancy map")
        wait_for(node, lambda: ROBOT_NAME in model_poses, "the Gazebo robot pose")

        neutral_turn = Int8()
        neutral_turn.data = 0
        for _ in range(3):
            route_turn_publisher.publish(neutral_turn)
            spin_for(node, 0.15)
        print("RANDOM_DYNAMIC_PATROL_GUARD_RESET=1", flush=True)

        for case_number in selected_cases:
            rng = random.Random(case_seeds[case_number - 1])
            obstacle_count = rng.randint(args.min_obstacles, args.max_obstacles)
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_SEED={case_seeds[case_number - 1]}", flush=True)

            ensure_home(node, navigator, model_poses, timeout=100.0)
            spin_for(node, 0.8)
            start = model_poses[ROBOT_NAME]
            start_heading = model_yaws[ROBOT_NAME]
            goal = choose_far_goal(
                latest["map"], start, start_heading, rng, args.min_goal_distance
            )
            goal_distance = distance(start, goal)
            yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
            heading_delta = abs(normalize_angle(yaw - start_heading))
            print(
                f"RANDOM_DYNAMIC_CASE_{case_number}_GOAL={goal[0]:.3f},{goal[1]:.3f};DIST={goal_distance:.3f}",
                flush=True,
            )
            print(
                f"RANDOM_DYNAMIC_CASE_{case_number}_START_HEADING_DELTA={heading_delta:.3f}",
                flush=True,
            )
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_OBSTACLE_COUNT={obstacle_count}", flush=True)

            metrics = {
                "active_names": [],
                "min_center_distance": float("inf"),
                "max_cross_track": 0.0,
                "speed_reductions": 0,
                "last_safe_time": 0.0,
                "in_place_total_sec": 0.0,
                "in_place_total_rad": 0.0,
                "in_place_episode_sec": 0.0,
                "max_in_place_episode_sec": 0.0,
            }
            current_metrics["value"] = metrics
            latest["path"] = None
            outbound_trace = [start]
            goal_handle, result_future = send_goal(
                node,
                navigator,
                make_pose_stamped(node, goal[0], goal[1], yaw),
                behavior_tree="dynamic_smooth",
            )
            navigation_started = time.monotonic()
            insert_after = rng.uniform(1.5, 3.0)
            insertion_deadline = navigation_started + 30.0
            travelled_before_insert = 0.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.08)
                if result_future.done():
                    raise RuntimeError("Robot reached the goal before runtime obstacles could be inserted.")
                robot = model_poses[ROBOT_NAME]
                if distance(robot, outbound_trace[-1]) >= 0.04:
                    outbound_trace.append(robot)
                travelled_before_insert = distance(start, robot)
                if (
                    travelled_before_insert >= 0.25
                    and time.monotonic() - navigation_started >= insert_after
                    and latest["path"] is not None
                ):
                    break
                if time.monotonic() >= insertion_deadline:
                    cancel_goal(node, goal_handle)
                    raise RuntimeError("Robot did not move far enough for runtime obstacle insertion.")

            insertion_robot = robot
            positions = choose_obstacle_positions(
                latest["map"], latest["path"], insertion_robot, goal, obstacle_count, rng
            )
            colors = ("0.95 0.08 0.08", "0.95 0.45 0.05", "0.75 0.10 0.90", "0.10 0.75 0.90")
            plan_count_at_insert = plan_counter["value"]
            for index, position in enumerate(positions, start=1):
                name = f"random_dynamic_c{case_number}_{index}"
                spawn_obstacle(node, spawner, name, position, colors[(index - 1) % len(colors)])
                active_names.append(name)
                metrics["active_names"].append(name)
                print(
                    f"RANDOM_DYNAMIC_CASE_{case_number}_SPAWN_{index}={position[0]:.3f},{position[1]:.3f}",
                    flush=True,
                )
            print(
                f"RANDOM_DYNAMIC_CASE_{case_number}_INSERTED_AFTER_M={travelled_before_insert:.3f}",
                flush=True,
            )

            hold_seconds = rng.uniform(11.0, 16.0)
            clear_deadline = time.monotonic() + hold_seconds
            navigation_deadline = navigation_started + args.case_timeout
            cleared = False
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.06)
                robot = model_poses[ROBOT_NAME]
                if distance(robot, outbound_trace[-1]) >= 0.04:
                    outbound_trace.append(robot)
                metrics["max_cross_track"] = max(
                    metrics["max_cross_track"], point_segment_distance(robot, start, goal)
                )
                if not cleared and time.monotonic() >= clear_deadline:
                    for name in list(active_names):
                        delete_obstacle(node, deleter, name)
                        active_names.remove(name)
                    metrics["active_names"].clear()
                    cleared = True
                    print(f"RANDOM_DYNAMIC_CASE_{case_number}_OBSTACLES_CLEARED=1", flush=True)
                if time.monotonic() >= navigation_deadline:
                    cancel_goal(node, goal_handle)
                    raise RuntimeError("Randomized dynamic-avoidance navigation timed out.")

            result = result_future.result()
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                status = result.status if result is not None else "none"
                raise RuntimeError(f"Randomized navigation failed with status {status}.")
            for name in list(active_names):
                delete_obstacle(node, deleter, name)
                active_names.remove(name)
            metrics["active_names"].clear()
            spin_for(node, 0.8)

            final_pose = model_poses[ROBOT_NAME]
            if distance(final_pose, outbound_trace[-1]) >= 0.02:
                outbound_trace.append(final_pose)
            goal_error = distance(final_pose, goal)
            replans_after_insert = plan_counter["value"] - plan_count_at_insert
            min_center = metrics["min_center_distance"]
            duration = time.monotonic() - navigation_started
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_DURATION_SEC={duration:.3f}", flush=True)
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_GOAL_ERROR={goal_error:.3f}", flush=True)
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_MIN_CENTER_DISTANCE={min_center:.3f}", flush=True)
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_MAX_CROSS_TRACK={metrics['max_cross_track']:.3f}", flush=True)
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_SPEED_REDUCTIONS={metrics['speed_reductions']}", flush=True)
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_REPLANS_AFTER_INSERT={replans_after_insert}", flush=True)
            print(
                f"RANDOM_DYNAMIC_CASE_{case_number}_IN_PLACE_TURN_SEC={metrics['in_place_total_sec']:.3f}",
                flush=True,
            )
            print(
                f"RANDOM_DYNAMIC_CASE_{case_number}_IN_PLACE_TURN_RAD={metrics['in_place_total_rad']:.3f}",
                flush=True,
            )
            print(
                f"RANDOM_DYNAMIC_CASE_{case_number}_MAX_IN_PLACE_EPISODE_SEC={metrics['max_in_place_episode_sec']:.3f}",
                flush=True,
            )

            if goal_distance < args.min_goal_distance:
                raise RuntimeError("Randomized goal was not far enough.")
            if travelled_before_insert < 0.25:
                raise RuntimeError("Obstacles were inserted before the robot was visibly moving.")
            if len(positions) != obstacle_count:
                raise RuntimeError("The requested randomized obstacle count was not spawned.")
            if not math.isfinite(min_center) or min_center < 0.40:
                raise RuntimeError("Robot did not preserve the 0.40 m center safety distance.")
            if replans_after_insert < 1:
                raise RuntimeError("No Nav2 replan was observed after obstacle insertion.")
            if metrics["max_cross_track"] < 0.12 and metrics["speed_reductions"] < 3:
                raise RuntimeError("No observable detour or safety slowdown occurred.")
            if metrics["max_in_place_episode_sec"] > 1.50:
                raise RuntimeError("A sustained in-place turn was observed during avoidance.")
            if metrics["in_place_total_rad"] > 0.70:
                raise RuntimeError("Too much cumulative in-place rotation was observed.")
            if goal_error > 0.30:
                raise RuntimeError("Robot did not reach the randomized far goal accurately.")

            current_metrics["value"] = None
            home_error = reverse_home_along_trace(
                node,
                return_velocity_publisher,
                model_poses,
                model_yaws,
                outbound_trace,
                timeout=110.0,
            )
            spin_for(node, 0.8)
            home_pose = model_poses[ROBOT_NAME]
            home_error = math.hypot(home_pose[0], home_pose[1])
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_HOME_ERROR={home_error:.3f}", flush=True)
            if home_error > 0.30:
                raise RuntimeError("Robot did not return home accurately after the randomized case.")
            passed += 1
            print(f"RANDOM_DYNAMIC_CASE_{case_number}_OK=1", flush=True)

        print(f"RANDOM_DYNAMIC_SUCCESS_RATE={passed}/{len(selected_cases)}", flush=True)
        print("RANDOM_DYNAMIC_AVOIDANCE_VALIDATION_OK", flush=True)
    except Exception as error:
        print(f"RANDOM_DYNAMIC_FAILED={error}", flush=True)
        print(f"RANDOM_DYNAMIC_REPLAY_SEED={master_seed}", flush=True)
        raise
    finally:
        current_metrics["value"] = None
        for name in list(active_names):
            try:
                if deleter.service_is_ready():
                    delete_obstacle(node, deleter, name)
            except Exception:
                pass
        navigator.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
