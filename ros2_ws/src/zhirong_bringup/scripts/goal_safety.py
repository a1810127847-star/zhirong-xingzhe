#!/usr/bin/env python3
"""Pure helpers for validating navigation goals against an occupancy grid."""

import math
from typing import Sequence, Tuple


def validate_goal_on_grid(
    *,
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    goal_x: float,
    goal_y: float,
    clearance_m: float,
    occupied_threshold: int = 100,
    reject_unknown: bool = True,
) -> Tuple[bool, str]:
    """Return whether a map-frame goal has enough obstacle clearance."""
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return False, "代价地图尺寸或分辨率无效"
    if len(data) != width * height:
        return False, "代价地图数据长度不完整"
    if not all(
        math.isfinite(value)
        for value in (goal_x, goal_y, clearance_m, origin_yaw)
    ):
        return False, "目标坐标包含非有限数值"
    if clearance_m < 0.0:
        return False, "目标安全距离不能为负数"

    delta_x = goal_x - origin_x
    delta_y = goal_y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    goal_col = math.floor(local_x / resolution)
    goal_row = math.floor(local_y / resolution)

    if not (0 <= goal_col < width and 0 <= goal_row < height):
        return False, "目标点位于全局代价地图之外"

    radius_cells = math.ceil(clearance_m / resolution)
    nearest_occupied = math.inf
    for row in range(goal_row - radius_cells, goal_row + radius_cells + 1):
        for col in range(goal_col - radius_cells, goal_col + radius_cells + 1):
            cell_x = (col + 0.5) * resolution
            cell_y = (row + 0.5) * resolution
            distance = math.hypot(cell_x - local_x, cell_y - local_y)
            if distance > clearance_m:
                continue

            if not (0 <= col < width and 0 <= row < height):
                return False, "目标安全范围超出已知地图"

            cost = int(data[row * width + col])
            if cost < 0 and reject_unknown:
                return False, "目标安全范围包含未知区域"
            if cost >= occupied_threshold:
                nearest_occupied = min(nearest_occupied, distance)

    if math.isfinite(nearest_occupied):
        return (
            False,
            f"目标距障碍仅 {nearest_occupied:.2f} m，"
            f"要求至少 {clearance_m:.2f} m",
        )

    return True, f"目标周围 {clearance_m:.2f} m 范围安全"
