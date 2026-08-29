#!/usr/bin/env python3

import argparse
import ctypes
import time
from ctypes import wintypes


ERROR_SUCCESS = 0


class XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("buttons", wintypes.WORD),
        ("left_trigger", ctypes.c_ubyte),
        ("right_trigger", ctypes.c_ubyte),
        ("left_x", ctypes.c_short),
        ("left_y", ctypes.c_short),
        ("right_x", ctypes.c_short),
        ("right_y", ctypes.c_short),
    ]


class XInputState(ctypes.Structure):
    _fields_ = [
        ("packet_number", wintypes.DWORD),
        ("gamepad", XInputGamepad),
    ]


class XInputVibration(ctypes.Structure):
    _fields_ = [
        ("left_motor_speed", wintypes.WORD),
        ("right_motor_speed", wintypes.WORD),
    ]


class XInputCapabilities(ctypes.Structure):
    _fields_ = [
        ("device_type", ctypes.c_ubyte),
        ("subtype", ctypes.c_ubyte),
        ("flags", wintypes.WORD),
        ("gamepad", XInputGamepad),
        ("vibration", XInputVibration),
    ]


def load_xinput():
    for library_name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
        try:
            return ctypes.WinDLL(library_name)
        except OSError:
            continue
    raise SystemExit("ERROR: Windows XInput DLL was not found.")


def read_controller(xinput, controller_id):
    state = XInputState()
    result = xinput.XInputGetState(controller_id, ctypes.byref(state))
    if result != ERROR_SUCCESS:
        return None
    return state


def state_values(state):
    gamepad = state.gamepad
    return (
        gamepad.buttons,
        gamepad.left_trigger,
        gamepad.right_trigger,
        gamepad.left_x,
        gamepad.left_y,
        gamepad.right_x,
        gamepad.right_y,
    )


def print_state(controller_id, state):
    gamepad = state.gamepad
    print(f"XINPUT_CONTROLLER={controller_id}")
    print(f"PACKET={state.packet_number}")
    print(f"BUTTONS=0x{gamepad.buttons:04x}")
    print(
        "AXES="
        f"left_x:{gamepad.left_x},left_y:{gamepad.left_y},"
        f"right_x:{gamepad.right_x},right_y:{gamepad.right_y}"
    )
    print(
        "TRIGGERS="
        f"left:{gamepad.left_trigger},right:{gamepad.right_trigger}"
    )
    print(flush=True)


parser = argparse.ArgumentParser()
parser.add_argument("--monitor", type=float, default=0.0)
parser.add_argument("--controller", type=int, default=0)
args = parser.parse_args()

xinput = load_xinput()
xinput.XInputGetState.argtypes = [wintypes.DWORD, ctypes.POINTER(XInputState)]
xinput.XInputGetState.restype = wintypes.DWORD
xinput.XInputGetCapabilities.argtypes = [
    wintypes.DWORD,
    wintypes.DWORD,
    ctypes.POINTER(XInputCapabilities),
]
xinput.XInputGetCapabilities.restype = wintypes.DWORD

capabilities = XInputCapabilities()
if (
    xinput.XInputGetCapabilities(
        args.controller,
        0,
        ctypes.byref(capabilities),
    )
    == ERROR_SUCCESS
):
    print(
        f"CAPABILITIES=type:{capabilities.device_type},"
        f"subtype:{capabilities.subtype},flags:0x{capabilities.flags:04x}",
        flush=True,
    )

if args.monitor > 0:
    deadline = time.monotonic() + args.monitor
    last_values = None
    found = False
    while time.monotonic() < deadline:
        current_state = read_controller(xinput, args.controller)
        if current_state is None:
            time.sleep(0.02)
            continue
        found = True
        current_values = state_values(current_state)
        if current_values != last_values:
            print_state(args.controller, current_state)
            last_values = current_values
        time.sleep(0.02)

    if not found:
        raise SystemExit("NO_XINPUT_CONTROLLER")
else:
    connected = []
    for controller_id in range(4):
        current_state = read_controller(xinput, controller_id)
        if current_state is None:
            continue
        connected.append(controller_id)
        print_state(controller_id, current_state)

    if not connected:
        raise SystemExit("NO_XINPUT_CONTROLLER")
