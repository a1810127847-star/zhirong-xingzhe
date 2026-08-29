#!/usr/bin/env python3

import sys
from pathlib import Path


script_dir = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "zhirong_bringup"
    / "scripts"
)
sys.path.insert(0, str(script_dir))

from keyboard_hold_teleop import HoldToRunTeleop  # noqa: E402


class FixedValue:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


controller = object.__new__(HoldToRunTeleop)
controller.forward_speed = FixedValue(0.35)
controller.reverse_speed = FixedValue(0.25)
controller.turn_speed = FixedValue(1.0)

cases = (
    (set(), (0.0, 0.0)),
    ({"w"}, (0.35, 0.0)),
    ({"s"}, (-0.25, 0.0)),
    ({"a"}, (0.0, 1.0)),
    ({"d"}, (0.0, -1.0)),
    ({"w", "a"}, (0.35, 1.0)),
    ({"w", "s"}, (0.0, 0.0)),
    ({"a", "d"}, (0.0, 0.0)),
)

for held_keys, expected in cases:
    controller.held_keys = held_keys
    actual = controller.command()
    if actual != expected:
        raise SystemExit(
            f"ERROR: keys={sorted(held_keys)} expected={expected} actual={actual}"
        )

print("KEYBOARD_HOLD_LOGIC_OK")
