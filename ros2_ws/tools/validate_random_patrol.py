#!/usr/bin/env python3
"""Run reproducible randomized patrols through the normal task interface."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import subprocess
import sys


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=5)
    parser.add_argument(
        "--only-case",
        type=int,
        default=0,
        help="Run one case from the generated set while preserving seed order.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="0 creates a fresh seed; a printed seed reproduces the same routes.",
    )
    parser.add_argument("--case-timeout", type=float, default=140.0)
    parser.add_argument("--min-radius", type=float, default=0.65)
    parser.add_argument("--max-radius", type=float, default=0.90)
    return parser.parse_args()


def build_route(rng: random.Random, case_number: int, direction: int):
    base_angle = rng.uniform(-math.pi, math.pi)
    direction_name = "CW" if direction < 0 else "CCW"
    first_length = rng.uniform(build_route.min_radius, build_route.max_radius)
    second_length = rng.uniform(build_route.min_radius, build_route.max_radius)
    corner_angle = base_angle + direction * (
        math.pi / 2.0 + rng.uniform(-0.16, 0.16)
    )
    first = (
        first_length * math.cos(base_angle),
        first_length * math.sin(base_angle),
    )
    second = (
        second_length * math.cos(corner_angle),
        second_length * math.sin(corner_angle),
    )
    # A rotated, mildly skewed quadrilateral starts and ends at home. Unlike
    # points scattered around a circle, it avoids an artificial near-U-turn
    # at the first point while still varying heading, size and turn direction.
    coordinates = [
        first,
        (first[0] + second[0], first[1] + second[1]),
        second,
    ]
    waypoints = [
        {
            "name": f"r{case_number}_p{point_number}",
            "x": round(x, 3),
            "y": round(y, 3),
        }
        for point_number, (x, y) in enumerate(coordinates, start=1)
    ]
    waypoints.append({"name": "home", "x": 0.0, "y": 0.0})
    return direction_name, waypoints


build_route.min_radius = 0.65
build_route.max_radius = 0.90


def route_text(waypoints):
    return " -> ".join(
        f"{waypoint['name']}({waypoint['x']:.3f},{waypoint['y']:.3f})"
        for waypoint in waypoints
    )


def main():
    args = parse_args()
    if not 1 <= args.cases <= 20:
        raise SystemExit("--cases must be between 1 and 20.")
    if args.only_case and not 1 <= args.only_case <= args.cases:
        raise SystemExit("--only-case must be within the generated case range.")
    if not 0.50 <= args.min_radius < args.max_radius <= 1.50:
        raise SystemExit(
            "Require 0.50 <= --min-radius < --max-radius <= 1.50."
        )

    seed = args.seed or random.SystemRandom().randrange(1, 2**32)
    rng = random.Random(seed)
    build_route.min_radius = args.min_radius
    build_route.max_radius = args.max_radius
    validator = Path(__file__).with_name("validate_task_patrol.py")

    print(f"RANDOM_PATROL_SEED={seed}", flush=True)
    print(f"RANDOM_PATROL_CASES={args.cases}", flush=True)
    directions = [-1, 1]
    directions.extend(rng.choice((-1, 1)) for _ in range(max(0, args.cases - 2)))
    rng.shuffle(directions)
    routes = [
        build_route(rng, case_number, directions[case_number - 1])
        for case_number in range(1, args.cases + 1)
    ]
    selected_cases = (
        [args.only_case]
        if args.only_case
        else list(range(1, args.cases + 1))
    )
    if args.only_case:
        print(f"RANDOM_PATROL_ONLY_CASE={args.only_case}", flush=True)
    passed = 0
    for case_number in selected_cases:
        direction, waypoints = routes[case_number - 1]
        command = json.dumps(
            {
                "command": "patrol",
                "name": f"random_robustness_{case_number}",
                "waypoints": waypoints,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        expected_names = ",".join(point["name"] for point in waypoints)
        print(
            f"RANDOM_PATROL_ROUTE_{case_number}="
            f"{direction} {route_text(waypoints)}",
            flush=True,
        )
        child_command = [
            sys.executable,
            "-u",
            str(validator),
            "--command",
            command,
            "--expected-count",
            str(len(waypoints)),
            "--expected-names",
            expected_names,
            "--timeout",
            str(args.case_timeout),
        ]
        process = subprocess.Popen(
            child_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
        output = []
        assert process.stdout is not None
        for line in process.stdout:
            clean = line.rstrip()
            output.append(clean)
            print(f"[CASE {case_number}/{args.cases}] {clean}", flush=True)
        return_code = process.wait()
        if return_code != 0 or not any(
            "TASK_PATROL_VALIDATION_OK" in line for line in output
        ):
            print(
                f"RANDOM_PATROL_FAILED_CASE={case_number}",
                flush=True,
            )
            print(f"RANDOM_PATROL_REPLAY_SEED={seed}", flush=True)
            raise SystemExit(return_code or 1)
        passed += 1
        print(
            f"RANDOM_PATROL_CASE_{case_number}_OK=1",
            flush=True,
        )

    print(
        f"RANDOM_PATROL_SUCCESS_RATE={passed}/{len(selected_cases)}",
        flush=True,
    )
    print("RANDOM_PATROL_VALIDATION_OK", flush=True)


if __name__ == "__main__":
    main()
