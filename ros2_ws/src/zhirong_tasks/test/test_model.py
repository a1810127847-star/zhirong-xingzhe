import json
import math

import pytest

from zhirong_tasks.model import (
    TaskCatalog,
    path_turn_direction,
    polyline_max_deviation,
)


@pytest.fixture
def catalog():
    return TaskCatalog(
        stations={
            "home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "east": {"x": 1.0, "y": 0.0, "yaw": 0.0},
            "north": {"x": 0.0, "y": 1.0, "yaw": 1.57},
        },
        patrols={"demo": ["east", "north", "home"]},
        vision_mappings={
            "color": {"blue": "patrol demo"},
            "qr": {"NAV:HOME": "return_home"},
        },
        default_patrol="demo",
        max_retries=1,
    )


def test_patrol_expands_in_order(catalog):
    parsed = catalog.parse("patrol demo")

    assert parsed.action == "enqueue"
    assert [task.name for task in parsed.tasks] == ["east", "north", "home"]
    assert parsed.tasks[0].yaw == pytest.approx(3.0 * math.pi / 4.0)
    assert parsed.tasks[1].yaw == pytest.approx(-math.pi / 2.0)
    assert parsed.tasks[2].yaw == pytest.approx(-math.pi / 2.0)
    assert [task.pass_through for task in parsed.tasks] == [True, True, True]
    assert [task.patrol_terminal for task in parsed.tasks] == [False, False, True]
    assert parsed.tasks[0].route_id
    assert len({task.route_id for task in parsed.tasks}) == 1
    assert all(task.max_retries == 1 for task in parsed.tasks)


def test_return_home_replaces_current_queue(catalog):
    parsed = catalog.parse("return_home")

    assert parsed.action == "replace"
    assert len(parsed.tasks) == 1
    assert parsed.tasks[0].name == "home"


def test_json_coordinate_command(catalog):
    parsed = catalog.parse(
        json.dumps({"command": "goto", "x": 0.5, "y": -0.25, "yaw": 0.3})
    )

    assert parsed.action == "enqueue"
    assert parsed.tasks[0].x == pytest.approx(0.5)
    assert parsed.tasks[0].y == pytest.approx(-0.25)
    assert parsed.tasks[0].yaw == pytest.approx(0.3)


def test_json_waypoint_patrol_builds_one_continuous_route(catalog):
    parsed = catalog.parse(
        json.dumps(
            {
                "command": "patrol",
                "name": "random_case",
                "waypoints": [
                    {"name": "p1", "x": 0.7, "y": 0.2},
                    {"name": "p2", "x": -0.4, "y": 0.8},
                    {"name": "home", "x": 0.0, "y": 0.0},
                ],
            }
        )
    )

    assert parsed.action == "enqueue"
    assert parsed.argument == "random_case"
    assert [task.name for task in parsed.tasks] == ["p1", "p2", "home"]
    assert len({task.route_id for task in parsed.tasks}) == 1
    assert [task.pass_through for task in parsed.tasks] == [True, True, True]
    assert parsed.tasks[-1].patrol_terminal is True
    assert parsed.tasks[0].yaw == pytest.approx(math.atan2(0.6, -1.1))
    assert parsed.tasks[-1].yaw == pytest.approx(math.atan2(-0.8, 0.4))


def test_vision_mapping_is_case_insensitive(catalog):
    assert catalog.command_for_vision("qr", "nav:home") == "return_home"
    assert catalog.command_for_vision("color", "blue") == "patrol demo"


def test_rejects_unknown_station(catalog):
    with pytest.raises(ValueError, match="Unknown station"):
        catalog.parse("goto nowhere")


def test_path_turn_direction_only_accepts_one_way_curvature():
    assert path_turn_direction([(0, 0), (1, 0), (1, 1), (0, 1)]) == 1
    assert path_turn_direction([(0, 0), (0, 1), (1, 1), (1, 0)]) == -1
    assert path_turn_direction([(0, 0), (1, 0), (1, 1), (2, 1)]) == 0
    assert path_turn_direction([(0, 0), (1, 0), (2, 0)]) == 0


def test_polyline_max_deviation_detects_a_detour():
    guide = [(0, 0), (1, 0), (1, 1)]
    assert polyline_max_deviation([(0.2, 0.0), (1.0, 0.8)], guide) == 0.0
    assert polyline_max_deviation([(0.5, 0.3)], guide) == pytest.approx(0.3)
