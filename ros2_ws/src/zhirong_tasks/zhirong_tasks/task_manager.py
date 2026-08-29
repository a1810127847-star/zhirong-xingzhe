#!/usr/bin/env python3
import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Deque, Dict, List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import (
    ComputePathThroughPoses,
    FollowPath,
    NavigateToPose,
    SmoothPath,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8, String
from std_srvs.srv import SetBool, Trigger

from zhirong_tasks.model import (
    NavigationTask,
    TaskCatalog,
    path_turn_direction,
    polyline_max_deviation,
    quaternion_from_yaw,
)


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.declare_parameter("task_config", "")
        self.declare_parameter("continue_on_failure", False)
        self.declare_parameter("vision_armed", False)
        self.declare_parameter("vision_one_shot", True)
        self.declare_parameter("action_wait_timeout_sec", 1.0)

        config_path = Path(str(self.get_parameter("task_config").value))
        if not config_path.is_file():
            raise RuntimeError(f"Task config does not exist: {config_path}")
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        settings = config.get("settings", {})
        self.declare_parameter(
            "task_timeout_sec",
            float(settings.get("task_timeout_sec", 90.0)),
        )
        self.declare_parameter(
            "patrol_pass_radius",
            float(settings.get("patrol_pass_radius", 0.12)),
        )
        self.declare_parameter(
            "patrol_terminal_radius",
            float(settings.get("patrol_terminal_radius", 0.08)),
        )
        self.declare_parameter(
            "patrol_handoff_distance",
            float(settings.get("patrol_handoff_distance", 0.45)),
        )
        self.catalog = TaskCatalog(
            stations=config.get("stations", {}),
            patrols=config.get("patrols", {}),
            vision_mappings=config.get("vision_mappings", {}),
            default_patrol=str(settings.get("default_patrol", "demo")),
            max_retries=int(settings.get("max_retries", 1)),
        )

        self.continue_on_failure = bool(
            self.get_parameter("continue_on_failure").value
        )
        self.vision_armed = bool(self.get_parameter("vision_armed").value)
        self.vision_one_shot = bool(
            self.get_parameter("vision_one_shot").value
        )
        self.action_wait_timeout_sec = float(
            self.get_parameter("action_wait_timeout_sec").value
        )
        self.task_timeout_sec = float(
            self.get_parameter("task_timeout_sec").value
        )
        self.patrol_pass_radius = float(
            self.get_parameter("patrol_pass_radius").value
        )
        self.patrol_terminal_radius = float(
            self.get_parameter("patrol_terminal_radius").value
        )
        self.patrol_handoff_distance = float(
            self.get_parameter("patrol_handoff_distance").value
        )

        self.queue: Deque[NavigationTask] = deque()
        self.active_task: Optional[NavigationTask] = None
        self.active_route: List[NavigationTask] = []
        self.active_route_index = 0
        self.active_navigation_mode = "single"
        self.active_waypoint_started_monotonic: Optional[float] = None
        self.active_goal_handle = None
        self.state = "IDLE"
        self.sequence_id = 0
        self.completed: List[Dict] = []
        self.failed: List[Dict] = []
        self.last_error = ""
        self.task_started_monotonic: Optional[float] = None
        self.pending_start = False
        self.replacement_pending = False
        self.timeout_pending = False
        self.pass_through_pending = False
        self.pass_through_armed = False
        self.route_terminal_pending = False
        self.latest_navigation_pose = None

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/tasks/status",
            latched_qos,
        )
        self.queue_publisher = self.create_publisher(
            String,
            "/tasks/queue",
            latched_qos,
        )
        self.route_turn_publisher = self.create_publisher(
            Int8,
            "/tasks/route_turn_sign",
            latched_qos,
        )
        self.route_turn_sign = 0
        self.planned_route_turn_sign = 0
        self.command_subscription = self.create_subscription(
            String,
            "/tasks/command",
            self._command_callback,
            10,
        )
        self.vision_subscription = self.create_subscription(
            String,
            "/vision/events",
            self._vision_callback,
            10,
        )
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._pose_callback,
            20,
        )

        self.callback_group = ReentrantCallbackGroup()
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.route_planner_client = ActionClient(
            self,
            ComputePathThroughPoses,
            "/compute_path_through_poses",
            callback_group=self.callback_group,
        )
        self.route_controller_client = ActionClient(
            self,
            FollowPath,
            "/follow_path",
            callback_group=self.callback_group,
        )
        self.route_smoother_client = ActionClient(
            self,
            SmoothPath,
            "/smooth_path",
            callback_group=self.callback_group,
        )

        self.create_service(
            Trigger,
            "/tasks/start_patrol",
            self._start_patrol_service,
        )
        self.create_service(
            Trigger,
            "/tasks/return_home",
            self._return_home_service,
        )
        self.create_service(
            Trigger,
            "/tasks/cancel",
            self._cancel_service,
        )
        self.create_service(
            Trigger,
            "/tasks/clear",
            self._clear_service,
        )
        self.create_service(
            SetBool,
            "/tasks/arm_vision",
            self._arm_vision_service,
        )

        self.create_timer(0.5, self._drive_state_machine)
        self.create_timer(1.0, self._publish_status)
        self._set_route_turn_sign(0, force=True)
        self._publish_status()
        self.get_logger().info(
            "Task manager ready. Use /tasks/command or task_cli."
        )

    def _command_callback(self, message: String):
        self._handle_command(message.data, source="command")

    def _vision_callback(self, message: String):
        if not self.vision_armed:
            return
        try:
            event = json.loads(message.data)
            event_type = str(event["type"])
            value = str(event["value"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid vision event: {exc}")
            return

        command = self.catalog.command_for_vision(event_type, value)
        if not command:
            self.get_logger().info(
                f"No task mapping for vision event {event_type}:{value}"
            )
            return
        self.get_logger().info(
            f"Vision mapping {event_type}:{value} -> {command}"
        )
        if self.vision_one_shot:
            self.vision_armed = False
            self.get_logger().info(
                "Vision task trigger automatically disarmed after one event."
            )
        self._handle_command(command, source=f"vision:{event_type}:{value}")

    def _pose_callback(self, message: PoseWithCovarianceStamped):
        position = message.pose.pose.position
        current_x = float(position.x)
        current_y = float(position.y)
        self.latest_navigation_pose = (current_x, current_y)
        if self.active_navigation_mode == "route" and self.state == "NAVIGATING":
            self._update_route_progress(current_x, current_y)

    def _handle_command(self, command_text: str, source: str) -> bool:
        try:
            parsed = self.catalog.parse(command_text, source=source)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.last_error = str(exc)
            self.get_logger().warning(self.last_error)
            self._publish_status()
            return False

        if parsed.action == "enqueue":
            self.queue.extend(parsed.tasks)
            self.sequence_id += 1
            self.last_error = ""
            self.get_logger().info(
                f"Queued {len(parsed.tasks)} task(s): "
                + ", ".join(task.name for task in parsed.tasks)
            )
            self._drive_state_machine()
        elif parsed.action == "replace":
            self.queue.clear()
            self.queue.extend(parsed.tasks)
            self.sequence_id += 1
            self.last_error = ""
            if self.active_goal_handle is not None:
                self.replacement_pending = True
                self.state = "CANCELING"
                self.active_goal_handle.cancel_goal_async()
            else:
                self.active_task = None
                self._drive_state_machine()
        elif parsed.action == "cancel":
            self._cancel_active(clear_queue=False)
        elif parsed.action == "clear":
            self.queue.clear()
            self._publish_status()
        elif parsed.action == "vision":
            self.vision_armed = parsed.argument == "on"
            self.get_logger().info(
                f"Vision task triggers {'armed' if self.vision_armed else 'disarmed'}."
            )
        elif parsed.action == "status":
            pass

        self._publish_status()
        return True

    def _drive_state_machine(self):
        if (
            self.active_task is not None
            and self.active_goal_handle is not None
            and self.state == "NAVIGATING"
            and self._active_duration() >= self.task_timeout_sec
        ):
            self.timeout_pending = True
            self.state = "CANCELING_TIMEOUT"
            self.last_error = (
                f"Task {self.active_task.name} exceeded "
                f"{self.task_timeout_sec:.1f} seconds."
            )
            self.get_logger().error(self.last_error)
            self.active_goal_handle.cancel_goal_async()
            self._publish_status()
            return

        if self.active_task is not None or self.pending_start:
            return
        if not self.queue:
            if self.state not in ("FAILED", "CANCELED"):
                self.state = "IDLE"
            return
        route_pending = bool(self.queue[0].route_id)
        navigation_client = (
            self.route_planner_client
            if route_pending
            else self.navigation_client
        )
        if not navigation_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            self.state = "WAITING_FOR_NAV2"
            return
        if route_pending and not self.route_controller_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            self.state = "WAITING_FOR_NAV2"
            return
        if route_pending and not self.route_smoother_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            self.state = "WAITING_FOR_NAV2"
            return

        self.active_task = self.queue.popleft()
        if self.active_task.route_id:
            route_id = self.active_task.route_id
            self.active_route = [self.active_task]
            while self.queue and self.queue[0].route_id == route_id:
                self.active_route.append(self.queue.popleft())
            self.active_route_index = 0
            self.active_navigation_mode = "route"
            for route_task in self.active_route:
                route_task.attempt += 1
        else:
            self.active_route = []
            self.active_route_index = 0
            self.active_navigation_mode = "single"
            self.active_task.attempt += 1
        self.pass_through_armed = False
        self.route_terminal_pending = False
        if (
            self.active_navigation_mode == "single"
            and
            self.active_task.patrol_terminal
            and self.latest_navigation_pose is not None
        ):
            current_x, current_y = self.latest_navigation_pose
            delta_x = self.active_task.x - current_x
            delta_y = self.active_task.y - current_y
            if math.hypot(delta_x, delta_y) > 1e-6:
                self.active_task.yaw = math.atan2(delta_y, delta_x)
        self.task_started_monotonic = time.monotonic()
        self.active_waypoint_started_monotonic = self.task_started_monotonic
        self.state = "SENDING_GOAL"
        self.pending_start = True
        self._last_distance_remaining = -1.0

        if self.active_navigation_mode == "route":
            goal = ComputePathThroughPoses.Goal()
            handoff_pose = self._route_handoff_pose()
            if handoff_pose is not None:
                # A single closed global path that returns to its start is
                # ambiguous to DWB's nearest-path-point pruning. End the
                # continuous multi-pose leg before home, then hand off to one
                # straight terminal goal with the same incoming heading.
                goal.goals = [
                    self._pose_for_task(task) for task in self.active_route[:-1]
                ]
                goal.goals.append(handoff_pose)
            else:
                goal.goals = [
                    self._pose_for_task(task) for task in self.active_route
                ]
            goal.planner_id = "PatrolGrid"
            goal.use_start = False
            self.get_logger().info(
                "Planning continuous route through: "
                + " -> ".join(
                    f"({pose.pose.position.x:.2f},{pose.pose.position.y:.2f})"
                    for pose in goal.goals
                )
            )
            future = self.route_planner_client.send_goal_async(goal)
            future.add_done_callback(self._route_plan_goal_response_callback)
        else:
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_for_task(self.active_task)
            future = self.navigation_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback,
            )
            future.add_done_callback(self._goal_response_callback)
        self._publish_status()

    def _pose_for_task(self, task: NavigationTask) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = task.x
        pose.pose.position.y = task.y
        pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_yaw(
            task.yaw
        )
        return pose

    def _route_handoff_pose(self) -> Optional[PoseStamped]:
        if len(self.active_route) < 2 or self.patrol_handoff_distance <= 0.0:
            return None
        previous = self.active_route[-2]
        terminal = self.active_route[-1]
        delta_x = terminal.x - previous.x
        delta_y = terminal.y - previous.y
        distance = math.hypot(delta_x, delta_y)
        if distance <= 1e-6:
            return None
        yaw = math.atan2(delta_y, delta_x)
        handoff_distance = min(self.patrol_handoff_distance, distance * 0.75)
        handoff = NavigationTask(
            task_id="route_handoff",
            name="route_handoff",
            x=terminal.x - handoff_distance * delta_x / distance,
            y=terminal.y - handoff_distance * delta_y / distance,
            yaw=yaw,
            max_retries=0,
        )
        return self._pose_for_task(handoff)

    def _goal_response_callback(self, future):
        self.pending_start = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_failure(f"Goal request failed: {exc}")
            return

        if not goal_handle.accepted:
            self._finish_failure("Nav2 rejected the goal.")
            return

        self.active_goal_handle = goal_handle
        self.state = "NAVIGATING"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        self._publish_status()

    def _route_plan_goal_response_callback(self, future):
        self.pending_start = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_route_failure(f"Route planner request failed: {exc}")
            return
        if not goal_handle.accepted:
            self._finish_route_failure("Route planner rejected the goal.")
            return

        self.active_goal_handle = goal_handle
        self.state = "PLANNING_ROUTE"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._route_plan_result_callback)
        self._publish_status()

    def _route_plan_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._finish_route_failure(f"Route planning result failed: {exc}")
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._route_result(wrapped_result.status, self._active_duration())
            return

        path = wrapped_result.result.path
        if not path.poses:
            self._finish_route_failure("Route planner returned an empty path.")
            return
        self.get_logger().info(
            f"Continuous route planned with {len(path.poses)} path poses."
        )
        self.active_goal_handle = None
        self.pending_start = True
        self.state = "SMOOTHING_ROUTE"
        goal = SmoothPath.Goal()
        goal.path = path
        goal.smoother_id = "simple_smoother"
        goal.max_smoothing_duration.sec = 1
        goal.check_for_collisions = True
        future = self.route_smoother_client.send_goal_async(goal)
        future.add_done_callback(self._route_smooth_goal_response_callback)
        self._publish_status()

    def _route_smooth_goal_response_callback(self, future):
        self.pending_start = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_route_failure(f"Route smoother request failed: {exc}")
            return
        if not goal_handle.accepted:
            self._finish_route_failure("Route smoother rejected the path.")
            return

        self.active_goal_handle = goal_handle
        self.state = "SMOOTHING_ROUTE"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._route_smooth_result_callback)
        self._publish_status()

    def _route_smooth_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._finish_route_failure(f"Route smoothing result failed: {exc}")
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._route_result(wrapped_result.status, self._active_duration())
            return

        path = wrapped_result.result.path
        if not path.poses:
            self._finish_route_failure("Route smoother returned an empty path.")
            return
        self.get_logger().info(
            f"Continuous route smoothed to {len(path.poses)} path poses."
        )
        path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path.poses
        ]
        # A terminal one-point leg may legitimately turn either way to settle on
        # the final pose.  The one-way guard is only appropriate for the main
        # multi-point patrol route.
        turn_sign = (
            path_turn_direction(path_points)
            if len(self.active_route) >= 3
            else 0
        )
        if (
            not turn_sign
            and len(self.active_route) >= 3
            and self.latest_navigation_pose is not None
        ):
            guide_points = [self.latest_navigation_pose] + [
                (task.x, task.y) for task in self.active_route
            ]
            guide_sign = path_turn_direction(guide_points)
            guide_deviation = polyline_max_deviation(
                path_points,
                guide_points,
            )
            if guide_sign and guide_deviation <= 0.20:
                turn_sign = guide_sign
                self.get_logger().info(
                    "Using waypoint turn direction for a near-guide route "
                    f"(max deviation {guide_deviation:.3f} m)."
                )
        self.planned_route_turn_sign = turn_sign
        self._set_route_turn_sign(0)
        if turn_sign:
            self.get_logger().info(
                "Prepared one-way patrol angular guard for "
                f"{'left' if turn_sign > 0 else 'right'} turns; it will "
                "activate after the first waypoint."
            )
        self.active_goal_handle = None
        self._send_route_path(path)

    def _send_route_path(self, path):
        self.pending_start = True
        self.state = "SENDING_ROUTE_PATH"
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "PatrolPath"
        goal.goal_checker_id = "patrol_goal_checker"
        future = self.route_controller_client.send_goal_async(goal)
        future.add_done_callback(self._route_follow_goal_response_callback)
        self._publish_status()

    def _route_follow_goal_response_callback(self, future):
        self.pending_start = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_route_failure(f"Route controller request failed: {exc}")
            return
        if not goal_handle.accepted:
            self._finish_route_failure("Route controller rejected the path.")
            return

        self.active_goal_handle = goal_handle
        self.state = "NAVIGATING"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._route_follow_result_callback)
        self._publish_status()

    def _route_follow_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._finish_route_failure(f"Route controller result failed: {exc}")
            return
        self._route_result(wrapped_result.status, self._active_duration())

    def _feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        current_pose = getattr(feedback, "current_pose", None)
        if current_pose is not None:
            self.latest_navigation_pose = (
                float(current_pose.pose.position.x),
                float(current_pose.pose.position.y),
            )
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is not None:
            self._last_distance_remaining = float(distance_remaining)
            pass_radius = (
                self.patrol_terminal_radius
                if self.active_task is not None
                and self.active_task.patrol_terminal
                else self.patrol_pass_radius
            )
            if self._last_distance_remaining > pass_radius + 0.05:
                self.pass_through_armed = True
            if (
                self.active_task is not None
                and self.active_task.pass_through
                and self.active_goal_handle is not None
                and self.state == "NAVIGATING"
                and not self.pass_through_pending
                and self.pass_through_armed
                and self._last_distance_remaining <= pass_radius
            ):
                self.pass_through_pending = True
                self.state = "ADVANCING"
                self.get_logger().info(
                    f"Passed patrol waypoint {self.active_task.name} "
                    f"within {self._last_distance_remaining:.3f} m; "
                    "advancing without final-yaw correction."
                )
                self.active_goal_handle.cancel_goal_async()
                self._publish_status()

    def _through_feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        current_pose = getattr(feedback, "current_pose", None)
        if current_pose is None:
            return

        current_x = float(current_pose.pose.position.x)
        current_y = float(current_pose.pose.position.y)
        self.latest_navigation_pose = (current_x, current_y)
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is not None:
            self._last_distance_remaining = float(distance_remaining)

        self._update_route_progress(current_x, current_y)

    def _update_route_progress(self, current_x: float, current_y: float):
        if self.active_navigation_mode != "route":
            return

        if self.active_route_index >= len(self.active_route):
            return
        task = self.active_route[self.active_route_index]
        radius = (
            self.patrol_terminal_radius
            if task.patrol_terminal
            else self.patrol_pass_radius
        )
        waypoint_distance = math.hypot(task.x - current_x, task.y - current_y)
        if waypoint_distance > radius:
            return

        record = task.to_dict()
        started = self.active_waypoint_started_monotonic
        record["duration_sec"] = round(
            max(0.0, time.monotonic() - started) if started is not None else 0.0,
            3,
        )
        record["completion"] = "continuous_route"
        self.completed.append(record)
        self.active_route_index += 1
        if self.active_route_index == 1 and self.planned_route_turn_sign:
            self._set_route_turn_sign(self.planned_route_turn_sign)
            self.get_logger().info(
                "Activated one-way patrol angular guard after the first "
                "waypoint."
            )
        self.active_waypoint_started_monotonic = time.monotonic()
        if self.active_route_index < len(self.active_route):
            self.active_task = self.active_route[self.active_route_index]
            self.get_logger().info(
                f"Passed route waypoint {task.name} within "
                f"{waypoint_distance:.3f} m; continuing to "
                f"{self.active_task.name} without restarting the controller."
            )
        else:
            if self.active_goal_handle is not None and not self.route_terminal_pending:
                self.route_terminal_pending = True
                self.state = "STOPPING_AT_TERMINAL"
                self.active_goal_handle.cancel_goal_async()
            self.get_logger().info(
                f"Reached terminal route waypoint {task.name} within "
                f"{waypoint_distance:.3f} m; canceling the planning-only "
                "extension for a controlled stop."
            )
        self._publish_status()

    def _result_callback(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self._finish_failure(f"Navigation result failed: {exc}")
            return

        duration = self._active_duration()
        if self.active_navigation_mode == "route":
            self._route_result(status, duration)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            task = self.active_task
            if task is not None:
                record = task.to_dict()
                record["duration_sec"] = round(duration, 3)
                self.completed.append(record)
            self.state = "TASK_SUCCEEDED"
            self.active_task = None
            self.active_goal_handle = None
            self.task_started_monotonic = None
            self.replacement_pending = False
            self.pass_through_pending = False
            self.pass_through_armed = False
            self._publish_status()
            self._drive_state_machine()
            return

        if status == GoalStatus.STATUS_CANCELED:
            if self.timeout_pending:
                self.timeout_pending = False
                self.active_goal_handle = None
                self._finish_failure(self.last_error)
                return
            if self.replacement_pending:
                self.pass_through_pending = False
                self.pass_through_armed = False
                self.active_task = None
                self.active_goal_handle = None
                self.task_started_monotonic = None
                self.replacement_pending = False
                self.state = "IDLE"
                self._drive_state_machine()
                return
            if self.pass_through_pending:
                task = self.active_task
                if task is not None:
                    record = task.to_dict()
                    record["duration_sec"] = round(duration, 3)
                    record["completion"] = "pass_through"
                    self.completed.append(record)
                self.pass_through_pending = False
                self.pass_through_armed = False
                self.active_task = None
                self.active_goal_handle = None
                self.task_started_monotonic = None
                self.state = "TASK_SUCCEEDED"
                self._publish_status()
                self._drive_state_machine()
                return
            self.active_task = None
            self.active_goal_handle = None
            self.task_started_monotonic = None
            self.state = "CANCELED"
            self._publish_status()
            return

        self._finish_failure(f"Navigation finished with status {status}.")

    def _route_result(self, status: int, duration: float):
        if status == GoalStatus.STATUS_SUCCEEDED:
            required_end_index = (
                len(self.active_route) - 1
                if self.active_route
                and self.active_route[-1].patrol_terminal
                else len(self.active_route)
            )
            if self.active_route_index < required_end_index:
                missed = ", ".join(
                    task.name
                    for task in self.active_route[
                        self.active_route_index : required_end_index
                    ]
                )
                self._finish_route_failure(
                    "Continuous route controller reached its endpoint before "
                    f"entering the required radius of waypoint(s): {missed}."
                )
                return
            if (
                len(self.active_route) > 1
                and self.active_route[-1].patrol_terminal
                and self.active_route_index < len(self.active_route)
            ):
                terminal_task = self.active_route[-1]
                terminal_task.attempt = 0
                self._clear_active_route()
                self.queue.appendleft(terminal_task)
                self.state = "HANDOFF_TO_TERMINAL"
                self.get_logger().info(
                    f"Continuous route leg complete; handing off straight "
                    f"to terminal {terminal_task.name}."
                )
                self._publish_status()
                self._drive_state_machine()
                return
            while self.active_route_index < len(self.active_route):
                task = self.active_route[self.active_route_index]
                record = task.to_dict()
                record["duration_sec"] = round(duration, 3)
                record["completion"] = "continuous_route"
                self.completed.append(record)
                self.active_route_index += 1
            self.state = "TASK_SUCCEEDED"
            self._clear_active_route()
            self.replacement_pending = False
            self._publish_status()
            self._drive_state_machine()
            return

        if status == GoalStatus.STATUS_CANCELED:
            if self.route_terminal_pending:
                self.state = "TASK_SUCCEEDED"
                self._clear_active_route()
                self.replacement_pending = False
                self._publish_status()
                self._drive_state_machine()
                return
            if self.timeout_pending:
                self.timeout_pending = False
                self.active_goal_handle = None
                self._finish_route_failure(self.last_error)
                return
            if self.replacement_pending:
                self.replacement_pending = False
                self._clear_active_route()
                self.state = "IDLE"
                self._drive_state_machine()
                return
            self._clear_active_route()
            self.state = "CANCELED"
            self._publish_status()
            return

        self._finish_route_failure(
            f"Continuous route navigation finished with status {status}."
        )

    def _finish_route_failure(self, error: str):
        remaining = self.active_route[self.active_route_index :]
        self.last_error = error
        can_retry = bool(remaining) and remaining[0].attempt <= remaining[0].max_retries
        self._clear_active_route()
        if can_retry:
            for task in reversed(remaining):
                self.queue.appendleft(task)
            self.state = "RETRYING"
            self.get_logger().warning(
                f"{error} Retrying the remaining continuous route."
            )
            self._publish_status()
            return

        if remaining:
            record = remaining[0].to_dict()
            record["error"] = error
            self.failed.append(record)
        self.state = "FAILED"
        self.get_logger().error(error)
        if not self.continue_on_failure:
            self.queue.clear()
        self._publish_status()
        if self.continue_on_failure:
            self._drive_state_machine()

    def _clear_active_route(self):
        self._set_route_turn_sign(0)
        self.planned_route_turn_sign = 0
        self.active_task = None
        self.active_goal_handle = None
        self.active_route = []
        self.active_route_index = 0
        self.active_navigation_mode = "single"
        self.active_waypoint_started_monotonic = None
        self.task_started_monotonic = None
        self.pending_start = False
        self.pass_through_pending = False
        self.pass_through_armed = False
        self.route_terminal_pending = False

    def _set_route_turn_sign(self, sign: int, force: bool = False):
        normalized = 1 if sign > 0 else -1 if sign < 0 else 0
        if not force and normalized == self.route_turn_sign:
            return
        self.route_turn_sign = normalized
        message = Int8()
        message.data = normalized
        self.route_turn_publisher.publish(message)

    def _finish_failure(self, error: str):
        if self.active_navigation_mode == "route" and self.active_route:
            self._finish_route_failure(error)
            return
        task = self.active_task
        self.last_error = error
        self.active_goal_handle = None
        self.active_task = None
        self.pending_start = False
        self.pass_through_pending = False
        self.pass_through_armed = False
        self.task_started_monotonic = None

        if task is not None and task.attempt <= task.max_retries:
            self.queue.appendleft(task)
            self.state = "RETRYING"
            self.get_logger().warning(
                f"{error} Retrying {task.name} "
                f"({task.attempt}/{task.max_retries + 1})."
            )
            self._publish_status()
            return

        if task is not None:
            record = task.to_dict()
            record["error"] = error
            self.failed.append(record)
        self.state = "FAILED"
        self.get_logger().error(error)
        if not self.continue_on_failure:
            self.queue.clear()
        self._publish_status()
        if self.continue_on_failure:
            self._drive_state_machine()

    def _cancel_active(self, clear_queue: bool):
        if clear_queue:
            self.queue.clear()
        self.pass_through_pending = False
        self.pass_through_armed = False
        if self.active_goal_handle is not None:
            self.state = "CANCELING"
            self.active_goal_handle.cancel_goal_async()
        else:
            self.active_task = None
            self.active_route = []
            self.active_route_index = 0
            self.active_navigation_mode = "single"
            self.active_waypoint_started_monotonic = None
            self.pending_start = False
            self.state = "CANCELED"
        self._publish_status()

    def _publish_status(self):
        queued_tasks = list(self.queue)
        if self.active_navigation_mode == "route" and self.active_route:
            queued_tasks = (
                self.active_route[self.active_route_index + 1 :] + queued_tasks
            )
        status = {
            "sequence_id": self.sequence_id,
            "state": self.state,
            "vision_armed": self.vision_armed,
            "vision_one_shot": self.vision_one_shot,
            "active_task": (
                self.active_task.to_dict() if self.active_task else None
            ),
            "active_duration_sec": round(self._active_duration(), 3),
            "task_timeout_sec": self.task_timeout_sec,
            "distance_remaining": round(
                float(getattr(self, "_last_distance_remaining", -1.0)),
                3,
            ),
            "navigation_mode": self.active_navigation_mode,
            "queue": [task.to_dict() for task in queued_tasks],
            "completed_count": len(self.completed),
            "failed_count": len(self.failed),
            "completed": self.completed[-10:],
            "failed": self.failed[-10:],
            "last_error": self.last_error,
            "stamp": self.get_clock().now().nanoseconds / 1e9,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_publisher.publish(message)

        queue_message = String()
        queue_message.data = json.dumps(
            [task.to_dict() for task in queued_tasks],
            ensure_ascii=False,
        )
        self.queue_publisher.publish(queue_message)

    def _active_duration(self) -> float:
        if self.task_started_monotonic is None:
            return 0.0
        return max(0.0, time.monotonic() - self.task_started_monotonic)

    def _start_patrol_service(self, request, response):
        del request
        response.success = self._handle_command(
            f"patrol {self.catalog.default_patrol}",
            source="service:start_patrol",
        )
        response.message = "Patrol queued." if response.success else self.last_error
        return response

    def _return_home_service(self, request, response):
        del request
        response.success = self._handle_command(
            "return_home",
            source="service:return_home",
        )
        response.message = (
            "Return-home task queued." if response.success else self.last_error
        )
        return response

    def _cancel_service(self, request, response):
        del request
        self._cancel_active(clear_queue=False)
        response.success = True
        response.message = "Active task canceled."
        return response

    def _clear_service(self, request, response):
        del request
        self.queue.clear()
        self._publish_status()
        response.success = True
        response.message = "Pending task queue cleared."
        return response

    def _arm_vision_service(self, request, response):
        self.vision_armed = bool(request.data)
        self._publish_status()
        response.success = True
        response.message = (
            "Vision task triggers armed."
            if self.vision_armed
            else "Vision task triggers disarmed."
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
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
