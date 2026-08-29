import math
import sys
from pathlib import Path


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "scripts"),
)

from goal_safety import validate_goal_on_grid


def _validate(data, *, goal_x, goal_y, clearance=0.25, **kwargs):
    return validate_goal_on_grid(
        data=data,
        width=10,
        height=10,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        goal_x=goal_x,
        goal_y=goal_y,
        clearance_m=clearance,
        occupied_threshold=100,
        **kwargs,
    )


def test_accepts_goal_with_clear_disk():
    valid, reason = _validate([0] * 100, goal_x=0.5, goal_y=0.5)

    assert valid
    assert "安全" in reason


def test_rejects_goal_near_occupied_cell():
    grid = [0] * 100
    grid[5 * 10 + 6] = 100

    valid, reason = _validate(grid, goal_x=0.5, goal_y=0.5)

    assert not valid
    assert "距障碍" in reason


def test_allows_nonlethal_inflation_cost():
    grid = [0] * 100
    grid[5 * 10 + 6] = 99

    valid, _ = _validate(grid, goal_x=0.5, goal_y=0.5)

    assert valid


def test_rejects_unknown_inside_clearance():
    grid = [0] * 100
    grid[5 * 10 + 6] = -1

    valid, reason = _validate(grid, goal_x=0.5, goal_y=0.5)

    assert not valid
    assert "未知区域" in reason


def test_rejects_goal_whose_clearance_crosses_boundary():
    valid, reason = _validate(
        [0] * 100,
        goal_x=0.05,
        goal_y=0.05,
    )

    assert not valid
    assert "超出已知地图" in reason


def test_supports_rotated_map_origin():
    valid, _ = validate_goal_on_grid(
        data=[0] * 100,
        width=10,
        height=10,
        resolution=0.1,
        origin_x=1.0,
        origin_y=2.0,
        origin_yaw=math.pi / 2.0,
        goal_x=0.5,
        goal_y=2.5,
        clearance_m=0.1,
    )

    assert valid
