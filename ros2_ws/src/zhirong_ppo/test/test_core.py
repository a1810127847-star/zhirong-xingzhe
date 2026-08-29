import math

import numpy as np
import pytest

from zhirong_ppo.core import (
    goal_in_robot_frame,
    normalize_bearing,
    reward_terms,
    scale_action,
    wrap_angle,
)


def test_scale_action_respects_robot_limits():
    assert scale_action([-1.0, -1.0]) == pytest.approx((0.0, -0.6))
    assert scale_action([1.0, 1.0]) == pytest.approx((0.3, 0.6))
    assert scale_action([0.0, 0.0]) == pytest.approx((0.15, 0.0))


def test_goal_is_expressed_in_robot_frame():
    distance, bearing = goal_in_robot_frame((0.0, 0.0, math.pi / 2.0), (1.0, 0.0))
    assert distance == pytest.approx(1.0)
    assert bearing == pytest.approx(-math.pi / 2.0)


def test_progress_reward_is_positive():
    terms = reward_terms(
        1.0,
        0.8,
        0.0,
        2.0,
        np.zeros(2),
        np.zeros(2),
    )
    assert terms["progress"] > 0.0
    assert terms["total"] > 0.0


def test_terminal_rewards_have_expected_ordering():
    common = (1.0, 1.0, 0.0, 2.0, np.zeros(2), np.zeros(2))
    success = reward_terms(*common, success=True)["total"]
    collision = reward_terms(*common, collision=True)["total"]
    assert success > 0.0
    assert collision < 0.0
    assert success > collision


def test_stagnation_and_safety_are_penalized():
    common = (1.0, 1.0, 0.0, 2.0, np.zeros(2), np.zeros(2))
    normal = reward_terms(*common)["total"]
    blocked = reward_terms(
        *common,
        stalled=True,
        safety_intervened=True,
    )["total"]
    assert blocked < normal


def test_turn_alignment_rewards_correct_direction():
    common = (1.0, 1.0, 0.5, 2.0)
    left = reward_terms(*common, [0.0, 1.0], np.zeros(2))["total"]
    right = reward_terms(*common, [0.0, -1.0], np.zeros(2))["total"]
    assert left > right

    common = (1.0, 1.0, -0.5, 2.0)
    left = reward_terms(*common, [0.0, 1.0], np.zeros(2))["total"]
    right = reward_terms(*common, [0.0, -1.0], np.zeros(2))["total"]
    assert right > left


def test_turn_alignment_is_symmetric():
    previous_action = np.asarray([-1.0, 0.0])
    left = reward_terms(
        1.0,
        1.0,
        0.5,
        2.0,
        [-1.0, 1.0],
        previous_action,
    )["turn_alignment"]
    right = reward_terms(
        1.0,
        1.0,
        -0.5,
        2.0,
        [-1.0, -1.0],
        previous_action,
    )["turn_alignment"]
    assert left == pytest.approx(right)


def test_forward_motion_is_penalized_when_goal_is_off_axis():
    moving = reward_terms(
        1.0,
        1.0,
        1.0,
        2.0,
        [1.0, 0.0],
        [1.0, 0.0],
    )
    stopped = reward_terms(
        1.0,
        1.0,
        1.0,
        2.0,
        [-1.0, 0.0],
        [-1.0, 0.0],
    )
    assert moving["forward_misalignment"] < 0.0
    assert stopped["forward_misalignment"] == pytest.approx(0.0)
    assert moving["total"] < stopped["total"]


def test_wrap_angle_is_bounded():
    assert -math.pi <= wrap_angle(7.0) <= math.pi


def test_bearing_uses_forward_hemisphere_scale():
    assert normalize_bearing(math.pi / 2.0) == pytest.approx(1.0)
    assert normalize_bearing(-math.pi / 4.0) == pytest.approx(-0.5)
    assert normalize_bearing(math.pi) == pytest.approx(1.0)
    assert normalize_bearing(
        math.pi / 2.0,
        "v1_bearing_pi",
    ) == pytest.approx(0.5)
