import math

import numpy as np


MAX_LINEAR_SPEED = 0.30
MAX_ANGULAR_SPEED = 0.60


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def scale_action(action):
    """Map a normalized PPO action to the robot's safe speed limits."""
    normalized = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
    linear = float((normalized[0] + 1.0) * 0.5 * MAX_LINEAR_SPEED)
    angular = float(normalized[1] * MAX_ANGULAR_SPEED)
    return linear, angular


def goal_in_robot_frame(robot_pose, goal):
    robot_x, robot_y, robot_yaw = robot_pose
    delta_x = goal[0] - robot_x
    delta_y = goal[1] - robot_y
    distance = math.hypot(delta_x, delta_y)
    bearing = wrap_angle(math.atan2(delta_y, delta_x) - robot_yaw)
    return distance, bearing


def normalize_bearing(bearing, version="v2_bearing_half_pi"):
    """Normalize bearing while preserving compatibility with saved models."""
    if version == "v1_bearing_pi":
        scale = math.pi
    elif version == "v2_bearing_half_pi":
        scale = math.pi / 2.0
    else:
        raise ValueError(f"Unknown observation version: {version}")
    return float(np.clip(bearing / scale, -1.0, 1.0))


def reward_terms(
    previous_distance,
    distance,
    bearing,
    min_scan,
    action,
    previous_action,
    *,
    success=False,
    collision=False,
    timed_out=False,
    safety_intervened=False,
    stalled=False,
):
    """Reward V4: favor progress while turning before driving when misaligned."""
    progress = 18.0 * (previous_distance - distance)
    step = -0.015
    heading = 0.08 * math.cos(bearing)
    normalized_bearing = float(
        np.clip(bearing / (math.pi / 2.0), -1.0, 1.0)
    )
    turn_alignment = 0.15 * normalized_bearing * float(action[1])
    linear_fraction = float(
        (np.clip(float(action[0]), -1.0, 1.0) + 1.0) * 0.5
    )
    forward_misalignment = (
        -0.12 * abs(normalized_bearing) * linear_fraction
    )
    near_obstacle = -0.20 * float(
        np.clip((0.50 - min_scan) / 0.30, 0.0, 1.0)
    )
    angular_motion = -0.015 * abs(float(action[1]))
    action_change = -0.01 * float(
        np.linalg.norm(
            np.asarray(action, dtype=np.float32)
            - np.asarray(previous_action, dtype=np.float32)
        )
    )
    safety = -0.10 if safety_intervened else 0.0
    stagnation = -0.12 if stalled else 0.0
    terminal = 25.0 if success else 0.0
    if collision:
        terminal -= 25.0
    if timed_out:
        terminal -= 5.0

    terms = {
        "progress": progress,
        "step": step,
        "heading": heading,
        "turn_alignment": turn_alignment,
        "forward_misalignment": forward_misalignment,
        "near_obstacle": near_obstacle,
        "angular_motion": angular_motion,
        "action_change": action_change,
        "safety": safety,
        "stagnation": stagnation,
        "terminal": terminal,
    }
    terms["total"] = float(sum(terms.values()))
    return terms
