#!/usr/bin/env python3

import sys
import time
from pathlib import Path

import rclpy


script_dir = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "zhirong_bringup"
    / "scripts"
)
sys.path.insert(0, str(script_dir))

from keyboard_hold_teleop import (  # noqa: E402
    RELEASE_DEBOUNCE_MS,
    HoldToRunTeleop,
)


rclpy.init()
controller = HoldToRunTeleop()
controller.root.update()

controller.root.event_generate("<KeyPress>", keysym="w")
controller.root.update()
if controller.command() != (0.35, 0.0):
    raise SystemExit(f"ERROR: KeyPress W produced {controller.command()}")

controller.root.event_generate("<KeyRelease>", keysym="w")
deadline = time.monotonic() + (RELEASE_DEBOUNCE_MS + 150) / 1000.0
while time.monotonic() < deadline:
    controller.root.update()
    time.sleep(0.01)

if controller.command() != (0.0, 0.0):
    raise SystemExit(f"ERROR: KeyRelease W produced {controller.command()}")

controller.close()
print("KEYBOARD_HOLD_GUI_EVENTS_OK")
