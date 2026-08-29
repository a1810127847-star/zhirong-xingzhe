#!/usr/bin/env python3
"""Validate public Nav2 goals and proxy accepted goals to the raw server."""

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from goal_safety import validate_goal_on_grid


class GoalGuard(Node):
    def __init__(self):
        super().__init__("goal_guard")
        self.declare_parameter("goal_clearance_m", 0.34)
        self.declare_parameter("costmap_timeout_sec", 2.0)
        self.declare_parameter("occupied_threshold", 100)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter(
            "raw_action_name",
            "/nav2_raw/navigate_to_pose",
        )

        self.goal_clearance_m = float(
            self.get_parameter("goal_clearance_m").value
        )
        self.costmap_timeout_sec = float(
            self.get_parameter("costmap_timeout_sec").value
        )
        self.occupied_threshold = int(
            self.get_parameter("occupied_threshold").value
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.raw_action_name = str(
            self.get_parameter("raw_action_name").value
        )

        behavior_tree_dir = (
            Path(get_package_share_directory("zhirong_bringup"))
            / "behavior_trees"
        )
        self.safe_behavior_tree = str(
            behavior_tree_dir / "navigate_to_pose_safe_recovery.xml"
        )
        self.dynamic_behavior_tree = str(
            behavior_tree_dir / "navigate_to_pose_dynamic_smooth.xml"
        )

        self.callback_group = ReentrantCallbackGroup()
        self.costmap = None
        self.costmap_received_monotonic = 0.0
        self.costmap_ready_published = False
        self.raw_goals = {}
        self.raw_goals_lock = threading.Lock()

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap_callback,
            costmap_qos,
            callback_group=self.callback_group,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/goal_guard/status",
            status_qos,
        )
        self.raw_client = ActionClient(
            self,
            NavigateToPose,
            self.raw_action_name,
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._publish_status("STARTING", "等待全局代价地图和 Nav2")
        self.get_logger().info(
            "Goal guard ready: /navigate_to_pose -> "
            f"{self.raw_action_name}, clearance={self.goal_clearance_m:.2f} m"
        )

    def _costmap_callback(self, message: OccupancyGrid):
        self.costmap = message
        self.costmap_received_monotonic = time.monotonic()
        if not self.costmap_ready_published:
            self.costmap_ready_published = True
            self._publish_status("READY", "全局代价地图已就绪")
            self.get_logger().info("Global costmap received; goal guard is ready.")

    def _goal_callback(self, goal_request):
        valid, reason = self._validate_request(goal_request)
        pose = goal_request.pose.pose.position
        if not valid:
            self.get_logger().warning(
                f"Rejected goal ({pose.x:.2f}, {pose.y:.2f}): {reason}"
            )
            self._publish_status(
                "REJECTED",
                reason,
                goal_x=pose.x,
                goal_y=pose.y,
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepted safe goal ({pose.x:.2f}, {pose.y:.2f}): {reason}"
        )
        self._publish_status(
            "ACCEPTED",
            reason,
            goal_x=pose.x,
            goal_y=pose.y,
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        goal_key = bytes(goal_handle.goal_id.uuid)
        with self.raw_goals_lock:
            raw_goal = self.raw_goals.get(goal_key)
        if raw_goal is not None:
            raw_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        valid, reason = self._validate_request(goal_handle.request)
        if not valid:
            goal_handle.abort()
            self._publish_status("ABORTED", f"执行前复检失败：{reason}")
            return NavigateToPose.Result()

        if not self.raw_client.wait_for_server(timeout_sec=15.0):
            goal_handle.abort()
            reason = "Nav2 原始导航服务在 15 秒内未就绪"
            self.get_logger().error(reason)
            self._publish_status("ABORTED", reason)
            return NavigateToPose.Result()

        raw_request = NavigateToPose.Goal()
        raw_request.pose = goal_handle.request.pose
        # The public action may request the one built-in dynamic profile using
        # a stable token. Never forward arbitrary filesystem paths to Nav2.
        if goal_handle.request.behavior_tree == "dynamic_smooth":
            raw_request.behavior_tree = self.dynamic_behavior_tree
            self.get_logger().info("Using arc-only DynamicPath controller.")
        else:
            raw_request.behavior_tree = self.safe_behavior_tree
        raw_future = self.raw_client.send_goal_async(
            raw_request,
            feedback_callback=lambda feedback: self._forward_feedback(
                goal_handle,
                feedback,
            ),
        )
        raw_goal_handle = await raw_future
        if not raw_goal_handle.accepted:
            goal_handle.abort()
            reason = "Nav2 原始导航服务拒绝了已通过安全检查的目标"
            self._publish_status("ABORTED", reason)
            return NavigateToPose.Result()

        goal_key = bytes(goal_handle.goal_id.uuid)
        with self.raw_goals_lock:
            self.raw_goals[goal_key] = raw_goal_handle
        if goal_handle.is_cancel_requested:
            await raw_goal_handle.cancel_goal_async()

        raw_result = await raw_goal_handle.get_result_async()
        with self.raw_goals_lock:
            self.raw_goals.pop(goal_key, None)

        if raw_result.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            self._publish_status("SUCCEEDED", "目标导航成功")
        elif (
            raw_result.status == GoalStatus.STATUS_CANCELED
            or goal_handle.is_cancel_requested
        ):
            goal_handle.canceled()
            self._publish_status("CANCELED", "目标导航已取消")
        else:
            goal_handle.abort()
            self._publish_status(
                "ABORTED",
                f"有限脱困后导航仍失败，Nav2 状态码 {raw_result.status}",
            )
        return raw_result.result

    def _forward_feedback(self, goal_handle, feedback_message):
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback_message.feedback)

    def _validate_request(self, goal_request):
        frame_id = goal_request.pose.header.frame_id.lstrip("/")
        if frame_id != self.global_frame.lstrip("/"):
            return (
                False,
                f"目标坐标系必须是 {self.global_frame}，收到 "
                f"{goal_request.pose.header.frame_id or '<empty>'}",
            )

        position = goal_request.pose.pose.position
        orientation = goal_request.pose.pose.orientation
        if not all(
            math.isfinite(value)
            for value in (
                position.x,
                position.y,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        ):
            return False, "目标位姿包含非有限数值"

        if self.costmap is None:
            return False, "全局代价地图尚未就绪，请稍后重试"
        age = time.monotonic() - self.costmap_received_monotonic
        if age > self.costmap_timeout_sec:
            return False, f"全局代价地图已超时（{age:.1f} 秒）"

        origin = self.costmap.info.origin
        yaw = 2.0 * math.atan2(
            origin.orientation.z,
            origin.orientation.w,
        )
        return validate_goal_on_grid(
            data=self.costmap.data,
            width=self.costmap.info.width,
            height=self.costmap.info.height,
            resolution=self.costmap.info.resolution,
            origin_x=origin.position.x,
            origin_y=origin.position.y,
            origin_yaw=yaw,
            goal_x=position.x,
            goal_y=position.y,
            clearance_m=self.goal_clearance_m,
            occupied_threshold=self.occupied_threshold,
            reject_unknown=True,
        )

    def _publish_status(
        self,
        state,
        reason,
        *,
        goal_x=None,
        goal_y=None,
    ):
        payload = {
            "state": state,
            "reason": reason,
            "goal_clearance_m": self.goal_clearance_m,
            "stamp": self.get_clock().now().nanoseconds / 1e9,
        }
        if goal_x is not None:
            payload["goal_x"] = round(float(goal_x), 3)
        if goal_y is not None:
            payload["goal_y"] = round(float(goal_y), 3)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)

    def destroy_node(self):
        self.server.destroy()
        self.raw_client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalGuard()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
