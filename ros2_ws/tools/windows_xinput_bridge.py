#!/usr/bin/env python3

import argparse
import ctypes
import json
import socket
import subprocess
import time
from ctypes import wintypes


ERROR_SUCCESS = 0
PACKET_MAGIC = "zhirong_xinput_v1"


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


def load_xinput():
    for library_name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
        try:
            return ctypes.WinDLL(library_name)
        except OSError:
            continue
    raise SystemExit("ERROR: Windows XInput DLL was not found.")


def get_wsl_address(distribution):
    creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    output = subprocess.check_output(
        ["wsl.exe", "-d", distribution, "--", "hostname", "-I"],
        text=True,
        stderr=subprocess.DEVNULL,
        creationflags=creation_flags,
        timeout=10,
    )
    addresses = output.strip().split()
    if not addresses:
        raise SystemExit("ERROR: Could not determine the WSL IP address.")
    return addresses[0]


def normalize_axis(raw_value, deadzone):
    value = max(-1.0, min(1.0, raw_value / 32767.0))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return scaled if value > 0 else -scaled


def read_state(xinput, controller_id):
    state = XInputState()
    result = xinput.XInputGetState(controller_id, ctypes.byref(state))
    if result != ERROR_SUCCESS:
        return None
    return state


def make_packet(gamepad, args):
    steering = normalize_axis(gamepad.left_x, args.deadzone)

    if args.mode == "triggers":
        forward = gamepad.right_trigger / 255.0
        reverse = gamepad.left_trigger / 255.0
        throttle = forward - reverse
        enabled = abs(throttle) >= args.trigger_deadzone
        linear = throttle * args.linear_speed if enabled else 0.0
        angular = -steering * args.angular_speed if enabled else 0.0
    else:
        enabled = gamepad.left_trigger >= args.enable_trigger
        throttle = normalize_axis(gamepad.left_y, args.deadzone)
        turbo = gamepad.right_trigger / 255.0
        linear_limit = args.linear_speed + turbo * (
            args.turbo_linear_speed - args.linear_speed
        )
        angular_limit = args.angular_speed + turbo * (
            args.turbo_angular_speed - args.angular_speed
        )
        linear = throttle * linear_limit if enabled else 0.0
        angular = -steering * angular_limit if enabled else 0.0

    return {
        "magic": PACKET_MAGIC,
        "enabled": enabled,
        "linear": round(linear, 5),
        "angular": round(angular, 5),
        "left_trigger": int(gamepad.left_trigger),
        "right_trigger": int(gamepad.right_trigger),
        "timestamp": time.time(),
    }


def send_stop(sock, target):
    packet = json.dumps(
        {
            "magic": PACKET_MAGIC,
            "enabled": False,
            "linear": 0.0,
            "angular": 0.0,
            "timestamp": time.time(),
        }
    ).encode("utf-8")
    for _ in range(3):
        sock.sendto(packet, target)
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distribution", default="Ubuntu-22.04")
    parser.add_argument("--host", default="")
    parser.add_argument("--port", type=int, default=15150)
    parser.add_argument("--controller", type=int, default=0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--mode",
        choices=("triggers", "stick"),
        default="triggers",
    )
    parser.add_argument("--deadzone", type=float, default=0.12)
    parser.add_argument("--trigger-deadzone", type=float, default=0.05)
    parser.add_argument("--enable-trigger", type=int, default=30)
    parser.add_argument("--linear-speed", type=float, default=0.55)
    parser.add_argument("--angular-speed", type=float, default=1.30)
    parser.add_argument("--turbo-linear-speed", type=float, default=0.80)
    parser.add_argument("--turbo-angular-speed", type=float, default=2.00)
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args()

    host = args.host or get_wsl_address(args.distribution)
    target = (host, args.port)
    xinput = load_xinput()
    xinput.XInputGetState.argtypes = [
        wintypes.DWORD,
        ctypes.POINTER(XInputState),
    ]
    xinput.XInputGetState.restype = wintypes.DWORD
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    period = 1.0 / args.rate
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    last_status = None
    next_report = 0.0

    print(
        f"XInput controller {args.controller} -> udp://{host}:{args.port}",
        flush=True,
    )
    if args.mode == "triggers":
        print(
            "RT=forward throttle; LT=reverse throttle; "
            "left stick horizontal=steering; release triggers to stop.",
            flush=True,
        )
    else:
        print(
            "Hold LEFT TRIGGER to enable; left stick controls "
            "throttle/steering; RIGHT TRIGGER adds turbo.",
            flush=True,
        )

    try:
        while deadline is None or time.monotonic() < deadline:
            loop_started = time.monotonic()
            state = read_state(xinput, args.controller)
            if state is None:
                packet = {
                    "magic": PACKET_MAGIC,
                    "enabled": False,
                    "linear": 0.0,
                    "angular": 0.0,
                    "timestamp": time.time(),
                }
                status = "DISCONNECTED"
            else:
                packet = make_packet(state.gamepad, args)
                status = (
                    f"enabled={packet['enabled']} "
                    f"linear={packet['linear']:+.2f} "
                    f"angular={packet['angular']:+.2f} "
                    f"LT={packet['left_trigger']} RT={packet['right_trigger']}"
                )

            sock.sendto(json.dumps(packet).encode("utf-8"), target)
            now = time.monotonic()
            if status != last_status or now >= next_report:
                print(status, flush=True)
                last_status = status
                next_report = now + 1.0

            remaining = period - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        send_stop(sock, target)
        sock.close()
        print("Gamepad bridge stopped; zero command sent.", flush=True)


if __name__ == "__main__":
    main()
