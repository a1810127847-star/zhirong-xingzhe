import json
import math
import uuid
from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Mapping, Sequence


@dataclass
class NavigationTask:
    task_id: str
    name: str
    x: float
    y: float
    yaw: float
    max_retries: int = 1
    attempt: int = 0
    source: str = "command"
    pass_through: bool = False
    patrol_terminal: bool = False
    route_id: str = ""

    def to_dict(self):
        return asdict(self)


@dataclass
class ParsedCommand:
    action: str
    tasks: List[NavigationTask]
    argument: str = ""


class TaskCatalog:
    def __init__(
        self,
        stations: Mapping[str, Mapping],
        patrols: Mapping[str, Sequence[str]],
        vision_mappings: Mapping[str, Mapping[str, str]],
        default_patrol: str = "demo",
        max_retries: int = 1,
    ):
        self.stations = {
            str(name): {
                "x": float(value["x"]),
                "y": float(value["y"]),
                "yaw": float(value.get("yaw", 0.0)),
            }
            for name, value in stations.items()
        }
        self.patrols = {
            str(name): [str(station) for station in station_names]
            for name, station_names in patrols.items()
        }
        self.vision_mappings = {
            str(event_type): {
                str(value): str(command)
                for value, command in mappings.items()
            }
            for event_type, mappings in vision_mappings.items()
        }
        self.default_patrol = default_patrol
        self.max_retries = max(0, int(max_retries))
        if "home" not in self.stations:
            raise ValueError("Task catalog must define a 'home' station.")

    def parse(self, command_text: str, source: str = "command") -> ParsedCommand:
        command_text = command_text.strip()
        if not command_text:
            raise ValueError("Task command is empty.")

        if command_text.startswith("{"):
            return self._parse_json(command_text, source)

        tokens = command_text.split()
        action = tokens[0].lower()
        arguments = tokens[1:]

        if action == "goto":
            if not arguments:
                raise ValueError("Usage: goto <station> or goto <x> <y> [yaw]")
            if arguments[0] in self.stations:
                return ParsedCommand(
                    action="enqueue",
                    tasks=[self.task_for_station(arguments[0], source=source)],
                )
            if len(arguments) == 1:
                self.task_for_station(arguments[0], source=source)
            if len(arguments) not in (2, 3):
                raise ValueError("Usage: goto <x> <y> [yaw]")
            x = float(arguments[0])
            y = float(arguments[1])
            yaw = float(arguments[2]) if len(arguments) == 3 else 0.0
            return ParsedCommand(
                action="enqueue",
                tasks=[
                    NavigationTask(
                        task_id=self._new_id(),
                        name=f"coordinate_{x:.2f}_{y:.2f}",
                        x=x,
                        y=y,
                        yaw=yaw,
                        max_retries=self.max_retries,
                        source=source,
                    )
                ],
            )

        if action == "patrol":
            patrol_name = arguments[0] if arguments else self.default_patrol
            return ParsedCommand(
                action="enqueue",
                tasks=self.tasks_for_patrol(patrol_name, source=source),
                argument=patrol_name,
            )

        if action in ("home", "return_home"):
            return ParsedCommand(
                action="replace",
                tasks=[self.task_for_station("home", source=source)],
                argument="home",
            )

        if action in ("cancel", "clear", "status"):
            return ParsedCommand(action=action, tasks=[])

        if action == "vision":
            if len(arguments) != 1 or arguments[0].lower() not in ("on", "off"):
                raise ValueError("Usage: vision on|off")
            return ParsedCommand(
                action="vision",
                tasks=[],
                argument=arguments[0].lower(),
            )

        raise ValueError(
            "Unknown command. Use goto, patrol, return_home, cancel, clear, "
            "status, or vision on|off."
        )

    def command_for_vision(self, event_type: str, value: str) -> str:
        mappings = self.vision_mappings.get(event_type, {})
        if value in mappings:
            return mappings[value]

        normalized = value.lower()
        for mapping_value, command in mappings.items():
            if mapping_value.lower() == normalized:
                return command
        return ""

    def task_for_station(
        self,
        station_name: str,
        source: str = "command",
    ) -> NavigationTask:
        if station_name not in self.stations:
            available = ", ".join(sorted(self.stations))
            raise ValueError(
                f"Unknown station '{station_name}'. Available: {available}"
            )
        station = self.stations[station_name]
        return NavigationTask(
            task_id=self._new_id(),
            name=station_name,
            x=station["x"],
            y=station["y"],
            yaw=station["yaw"],
            max_retries=self.max_retries,
            source=source,
        )

    def tasks_for_patrol(
        self,
        patrol_name: str,
        source: str = "command",
    ) -> List[NavigationTask]:
        if patrol_name not in self.patrols:
            available = ", ".join(sorted(self.patrols))
            raise ValueError(
                f"Unknown patrol '{patrol_name}'. Available: {available}"
            )
        tasks = [
            self.task_for_station(station_name, source=source)
            for station_name in self.patrols[patrol_name]
        ]
        return self._prepare_route_tasks(tasks)

    def tasks_for_waypoints(
        self,
        waypoints: Sequence[Mapping],
        source: str = "command",
    ) -> List[NavigationTask]:
        if len(waypoints) < 2:
            raise ValueError("A waypoint patrol requires at least 2 points.")
        if len(waypoints) > 20:
            raise ValueError("A waypoint patrol supports at most 20 points.")

        tasks = []
        for index, waypoint in enumerate(waypoints, start=1):
            x = float(waypoint["x"])
            y = float(waypoint["y"])
            yaw = float(waypoint.get("yaw", 0.0))
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError("Waypoint coordinates and yaw must be finite.")
            tasks.append(
                NavigationTask(
                    task_id=self._new_id(),
                    name=str(waypoint.get("name", f"waypoint_{index}")),
                    x=x,
                    y=y,
                    yaw=yaw,
                    max_retries=self.max_retries,
                    source=source,
                )
            )
        return self._prepare_route_tasks(tasks)

    def _prepare_route_tasks(
        self,
        tasks: List[NavigationTask],
    ) -> List[NavigationTask]:
        route_id = self._new_id()
        for task in tasks:
            task.route_id = route_id
        # A patrol is a continuous route. Intermediate goals face the next
        # leg, while the terminal goal keeps the incoming heading. This avoids
        # an otherwise unnecessary stop-and-rotate at every route vertex. A
        # standalone `goto` or `return_home` still uses the station yaw.
        for current, following in zip(tasks, tasks[1:]):
            delta_x = following.x - current.x
            delta_y = following.y - current.y
            if math.hypot(delta_x, delta_y) > 1e-6:
                current.yaw = math.atan2(delta_y, delta_x)
            current.pass_through = True
        if len(tasks) >= 2:
            previous = tasks[-2]
            terminal = tasks[-1]
            terminal.yaw = math.atan2(
                terminal.y - previous.y,
                terminal.x - previous.x,
            )
            terminal.pass_through = True
            terminal.patrol_terminal = True
        return tasks

    def _parse_json(self, command_text: str, source: str) -> ParsedCommand:
        payload = json.loads(command_text)
        command = str(payload.get("command", "")).strip()
        if not command:
            raise ValueError("JSON task command requires a 'command' field.")

        if command == "goto":
            if "station" in payload:
                return self.parse(f"goto {payload['station']}", source=source)
            x = float(payload["x"])
            y = float(payload["y"])
            yaw = float(payload.get("yaw", 0.0))
            return self.parse(f"goto {x} {y} {yaw}", source=source)
        if command == "patrol":
            if "waypoints" in payload:
                return ParsedCommand(
                    action="enqueue",
                    tasks=self.tasks_for_waypoints(
                        payload["waypoints"],
                        source=source,
                    ),
                    argument=str(payload.get("name", "custom")),
                )
            patrol = str(payload.get("name", self.default_patrol))
            return self.parse(f"patrol {patrol}", source=source)
        if command in ("return_home", "cancel", "clear", "status"):
            return self.parse(command, source=source)
        if command == "vision":
            return self.parse(
                f"vision {str(payload.get('enabled', 'off')).lower()}",
                source=source,
            )

        raise ValueError(f"Unsupported JSON command '{command}'.")

    @staticmethod
    def _new_id() -> str:
        return uuid.uuid4().hex[:10]


def quaternion_from_yaw(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def path_turn_direction(points, min_spacing=0.02, min_turn=0.01):
    """Return +1/-1 only when every meaningful path bend has one direction."""
    spaced = []
    for x, y in points:
        point = (float(x), float(y))
        if not spaced or math.hypot(
            point[0] - spaced[-1][0], point[1] - spaced[-1][1]
        ) >= min_spacing:
            spaced.append(point)
    headings = [
        math.atan2(current[1] - previous[1], current[0] - previous[0])
        for previous, current in zip(spaced, spaced[1:])
    ]
    signs = set()
    for previous, current in zip(headings, headings[1:]):
        delta = math.atan2(
            math.sin(current - previous),
            math.cos(current - previous),
        )
        if abs(delta) >= min_turn:
            signs.add(1 if delta > 0.0 else -1)
    if len(signs) == 1:
        return signs.pop()
    return 0


def polyline_max_deviation(points, guide_points):
    """Return the largest point-to-guide-segment distance."""
    guide = [(float(x), float(y)) for x, y in guide_points]
    if len(guide) < 2:
        return float("inf")

    def segment_distance(point, start, end):
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared <= 1e-12:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        projection = max(
            0.0,
            min(
                1.0,
                (
                    (point[0] - start[0]) * delta_x
                    + (point[1] - start[1]) * delta_y
                )
                / length_squared,
            ),
        )
        closest = (
            start[0] + projection * delta_x,
            start[1] + projection * delta_y,
        )
        return math.hypot(point[0] - closest[0], point[1] - closest[1])

    return max(
        (
            min(
                segment_distance(point, start, end)
                for start, end in zip(guide, guide[1:])
            )
            for point in points
        ),
        default=0.0,
    )
