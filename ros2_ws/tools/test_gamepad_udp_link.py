#!/usr/bin/env python3

import json
import socket
import sys
import time
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

from windows_xinput_bridge import (  # noqa: E402
    PACKET_MAGIC,
    get_wsl_address,
)


host = get_wsl_address("Ubuntu-22.04")
target = (host, 15150)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

active_packet = json.dumps(
    {
        "magic": PACKET_MAGIC,
        "enabled": True,
        "linear": 0.10,
        "angular": 0.0,
        "timestamp": time.time(),
    }
).encode("utf-8")
stop_packet = json.dumps(
    {
        "magic": PACKET_MAGIC,
        "enabled": False,
        "linear": 0.0,
        "angular": 0.0,
        "timestamp": time.time(),
    }
).encode("utf-8")

print(f"TEST_PACKET_TARGET={host}:15150", flush=True)
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline:
    sock.sendto(active_packet, target)
    time.sleep(0.05)

for _ in range(3):
    sock.sendto(stop_packet, target)
    time.sleep(0.05)

sock.close()
print("TEST_PACKETS_SENT", flush=True)
