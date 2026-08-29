import math
import time

import gymnasium as gym
import numpy as np
import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .core import (
    MAX_ANGULAR_SPEED,
    MAX_LINEAR_SPEED,
    goal_in_robot_frame,
    normalize_bearing,
    reward_terms,
    scale_action,
    yaw_from_quaternion,
)


class ZhirongGazeboEnv(gym.Env):
    """Single-process Gymnasium environment backed by the running ROS2 world."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        curriculum="empty",
        action_duration=0.12,
        max_steps=140,
        scan_samples=60,
        goal_tolerance=0.20,
        ready_timeout=35.0,
        observation_version="v2_bearing_half_pi",
    ):
        super().__init__()
        if curriculum not in {"straight", "fan", "empty", "single"}:
            raise ValueError(
                "curriculum must be 'straight', 'fan', 'empty', or 'single'"
            )

        self.curriculum = curriculum
        self.action_duration = float(action_duration)
        self.max_steps = int(max_steps)
        self.scan_samples = int(scan_samples)
        self.goal_tolerance = float(goal_tolerance)
        self.ready_timeout = float(ready_timeout)
        if observation_version not in {
            "v1_bearing_pi",
            "v2_bearing_half_pi",
        }:
            raise ValueError("Unsupported observation_version")
        self.observation_version = observation_version
        self.max_scan_range = 6.0
        self.collision_scan_range = 0.19
        self.dynamic_collision_distance = 0.45

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32,
        )
        observation_low = np.concatenate(
            [
                np.zeros(self.scan_samples, dtype=np.float32),
                np.asarray([0.0, -1.0, -1.0, -1.0, -1.0, -1.0]),
            ]
        ).astype(np.float32)
        observation_high = np.ones(
            self.scan_samples + 6,
            dtype=np.float32,
        )
        self.observation_space = gym.spaces.Box(
            low=observation_low,
            high=observation_high,
            dtype=np.float32,
        )

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)
        self.node = Node(
            f"zhirong_ppo_env_{int(time.time() * 1000) % 1000000}",
            parameter_overrides=[],
        )
        self.node.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=True)]
        )

        self.cmd_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_publisher = self.node.create_publisher(
            PoseStamped,
            "/ppo_goal",
            10,
        )
        self.state_client = self.node.create_client(
            SetEntityState,
            "/set_entity_state",
        )
        self.node.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            ModelStates,
            "/model_states",
            self._model_callback,
            10,
        )
        self.node.create_subscription(
            Twist,
            "/cmd_vel_safe",
            self._safe_callback,
            10,
        )

        self._scan = None
        self._robot_pose = None
        self._robot_twist = (0.0, 0.0)
        self._robot_z = 0.02
        self._obstacle_position = None
        self._safe_samples = []
        self._goal = (1.4, 0.0)
        self._previous_action = np.zeros(2, dtype=np.float32)
        self._previous_distance = 0.0
        self._last_position = None
        self._step_count = 0
        self._path_length = 0.0
        self._min_scan = self.max_scan_range
        self._safety_interventions = 0
        self._smoothness = 0.0
        self._no_progress_steps = 0
        self._max_stall_steps = 0
        self._episode_started = time.monotonic()
        self._episode_counter = 0
        self._closed = False
        self._ready = False

    def _scan_callback(self, message):
        self._scan = message

    def _model_callback(self, message):
        try:
            robot_index = message.name.index("zhirong_diffbot")
        except ValueError:
            return
        pose = message.pose[robot_index]
        twist = message.twist[robot_index]
        self._robot_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self._robot_z = float(pose.position.z)
        self._robot_twist = (
            float(twist.linear.x),
            float(twist.angular.z),
        )
        try:
            obstacle_index = message.name.index("dynamic_obstacle")
        except ValueError:
            self._obstacle_position = None
        else:
            obstacle = message.pose[obstacle_index].position
            self._obstacle_position = (
                float(obstacle.x),
                float(obstacle.y),
            )

    def _safe_callback(self, message):
        self._safe_samples.append(
            (float(message.linear.x), float(message.angular.z))
        )
        if len(self._safe_samples) > 100:
            del self._safe_samples[:-100]

    def _pump(self, duration, command=None):
        deadline = time.monotonic() + max(0.0, duration)
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if command is not None and now >= next_publish:
                self.cmd_publisher.publish(command)
                next_publish = now + 0.04
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))

    def _ensure_ready(self):
        if self._ready:
            return
        if not self.state_client.wait_for_service(timeout_sec=self.ready_timeout):
            raise RuntimeError("/set_entity_state is unavailable; start the PPO world first")
        deadline = time.monotonic() + self.ready_timeout
        while (
            rclpy.ok()
            and (self._scan is None or self._robot_pose is None)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self._scan is None or self._robot_pose is None:
            raise RuntimeError("Timed out waiting for /scan and /model_states")
        self._ready = True

    def _set_entity(self, name, x, y, z, yaw=0.0):
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose.position.x = float(x)
        request.state.pose.position.y = float(y)
        request.state.pose.position.z = float(z)
        request.state.pose.orientation.z = math.sin(yaw / 2.0)
        request.state.pose.orientation.w = math.cos(yaw / 2.0)
        request.state.reference_frame = "world"
        future = self.state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        if response is None or not response.success:
            raise RuntimeError(f"Failed to reset Gazebo entity {name}")

    def _sample_episode(self):
        if self.curriculum == "straight":
            return (0.0, 0.0, 0.0), (1.40, 0.0), (-2.35, -2.35)
        if self.curriculum == "fan":
            lateral_targets = (-0.35, 0.35, -0.20, 0.20, 0.0)
            goal_y = lateral_targets[
                self._episode_counter % len(lateral_targets)
            ]
            self._episode_counter += 1
            return (0.0, 0.0, 0.0), (1.40, goal_y), (-2.35, -2.35)
        start_yaw = float(self.np_random.uniform(-0.20, 0.20))
        goal_x = float(self.np_random.uniform(1.25, 1.65))
        goal_y = float(self.np_random.uniform(-0.45, 0.45))
        if self.curriculum == "single":
            obstacle_y = 0.30 if self.np_random.random() > 0.5 else -0.30
            obstacle = (float(self.np_random.uniform(0.62, 0.88)), obstacle_y)
        else:
            obstacle = (-2.35, -2.35)
        return (0.0, 0.0, start_yaw), (goal_x, goal_y), obstacle

    def _front_ranges(self):
        if self._scan is None or not self._scan.ranges:
            return np.full(
                self.scan_samples,
                self.max_scan_range,
                dtype=np.float32,
            )
        message = self._scan
        angles = np.linspace(-math.pi / 2.0, math.pi / 2.0, self.scan_samples)
        indices = np.rint(
            (angles - message.angle_min) / message.angle_increment
        ).astype(np.int64)
        indices = np.clip(indices, 0, len(message.ranges) - 1)
        ranges = np.asarray(message.ranges, dtype=np.float32)[indices]
        ranges = np.nan_to_num(
            ranges,
            nan=self.max_scan_range,
            posinf=self.max_scan_range,
            neginf=message.range_min,
        )
        return np.clip(ranges, message.range_min, self.max_scan_range)

    def _observation(self):
        front = self._front_ranges()
        scan_min = max(float(self._scan.range_min), 1e-3)
        normalized_scan = np.clip(
            (front - scan_min) / (self.max_scan_range - scan_min),
            0.0,
            1.0,
        )
        distance, bearing = goal_in_robot_frame(self._robot_pose, self._goal)
        linear, angular = self._robot_twist
        state = np.asarray(
            [
                np.clip(distance / 6.0, 0.0, 1.0),
                normalize_bearing(bearing, self.observation_version),
                np.clip(linear / MAX_LINEAR_SPEED, -1.0, 1.0),
                np.clip(angular / MAX_ANGULAR_SPEED, -1.0, 1.0),
                self._previous_action[0],
                self._previous_action[1],
            ],
            dtype=np.float32,
        )
        return np.concatenate([normalized_scan, state]).astype(np.float32)

    def _publish_goal(self):
        message = PoseStamped()
        message.header.frame_id = "odom"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = self._goal[0]
        message.pose.position.y = self._goal[1]
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)

    def _stop(self):
        command = Twist()
        for _ in range(3):
            self.cmd_publisher.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.03)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._ensure_ready()
        self._stop()
        start, self._goal, obstacle = self._sample_episode()
        self._set_entity("dynamic_obstacle", obstacle[0], obstacle[1], 0.36)
        self._set_entity(
            "zhirong_diffbot",
            start[0],
            start[1],
            max(self._robot_z, 0.02),
            start[2],
        )
        self._safe_samples.clear()
        self._pump(0.30, Twist())
        self._publish_goal()

        self._previous_action = np.zeros(2, dtype=np.float32)
        self._step_count = 0
        self._path_length = 0.0
        self._min_scan = self.max_scan_range
        self._safety_interventions = 0
        self._smoothness = 0.0
        self._no_progress_steps = 0
        self._max_stall_steps = 0
        self._episode_started = time.monotonic()
        self._last_position = self._robot_pose[:2]
        self._previous_distance, _ = goal_in_robot_frame(
            self._robot_pose,
            self._goal,
        )
        return self._observation(), self._episode_info()

    def _episode_info(self):
        distance, bearing = goal_in_robot_frame(self._robot_pose, self._goal)
        return {
            "distance_to_goal": float(distance),
            "bearing_to_goal": float(bearing),
            "path_length": float(self._path_length),
            "min_scan": float(self._min_scan),
            "safety_interventions": int(self._safety_interventions),
            "smoothness": float(self._smoothness),
            "max_stall_steps": int(self._max_stall_steps),
            "steps": int(self._step_count),
            "elapsed_wall_sec": float(time.monotonic() - self._episode_started),
            "goal": [float(self._goal[0]), float(self._goal[1])],
            "goal_y": float(self._goal[1]),
            "curriculum": self.curriculum,
            "observation_version": self.observation_version,
        }

    def step(self, action):
        normalized_action = np.clip(
            np.asarray(action, dtype=np.float32),
            -1.0,
            1.0,
        )
        linear, angular = scale_action(normalized_action)
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular

        self._safe_samples.clear()
        self._pump(self.action_duration, command)
        self._step_count += 1

        position = self._robot_pose[:2]
        segment = math.hypot(
            position[0] - self._last_position[0],
            position[1] - self._last_position[1],
        )
        if segment < 0.25:
            self._path_length += segment
        self._last_position = position

        front = self._front_ranges()
        min_scan = float(np.min(front))
        self._min_scan = min(self._min_scan, min_scan)
        distance, bearing = goal_in_robot_frame(self._robot_pose, self._goal)

        dynamic_distance = float("inf")
        if self._obstacle_position is not None:
            dynamic_distance = math.hypot(
                position[0] - self._obstacle_position[0],
                position[1] - self._obstacle_position[1],
            )
        collision = min_scan <= self.collision_scan_range
        if self.curriculum == "single":
            collision = (
                collision
                or dynamic_distance <= self.dynamic_collision_distance
            )
        success = distance <= self.goal_tolerance
        timed_out = self._step_count >= self.max_steps

        safety_intervened = any(
            linear > 0.03 and abs(safe_linear) < linear * 0.50
            for safe_linear, _ in self._safe_samples
        )
        if safety_intervened:
            self._safety_interventions += 1
        if self._previous_distance - distance < 0.002:
            self._no_progress_steps += 1
        else:
            self._no_progress_steps = 0
        self._max_stall_steps = max(
            self._max_stall_steps,
            self._no_progress_steps,
        )
        stalled = self._no_progress_steps >= 10
        self._smoothness += float(
            np.linalg.norm(normalized_action - self._previous_action)
        )
        terms = reward_terms(
            self._previous_distance,
            distance,
            bearing,
            min_scan,
            normalized_action,
            self._previous_action,
            success=success,
            collision=collision,
            timed_out=timed_out and not success and not collision,
            safety_intervened=safety_intervened,
            stalled=stalled,
        )
        self._previous_distance = distance
        self._previous_action = normalized_action.copy()

        terminated = bool(success or collision)
        truncated = bool(timed_out and not terminated)
        if terminated or truncated:
            self._stop()

        info = self._episode_info()
        info.update(
            {
                "success": bool(success),
                "collision": bool(collision),
                "timed_out": bool(truncated),
                "dynamic_obstacle_distance": float(dynamic_distance),
                "safety_intervened": bool(safety_intervened),
                "stalled": bool(stalled),
                "reward_terms": terms,
            }
        )
        return self._observation(), terms["total"], terminated, truncated, info

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._stop()
        finally:
            self.node.destroy_node()
            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()
