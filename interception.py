"""
Interception planning for multi-robot exit blocking.

All coordinates are world-plane (x, z) in centimeters, and speeds are cm/s.
The module is Isaac-independent and pure: callers may inject dist(a, b) to
represent navmesh path distance; otherwise Euclidean distance is used.
Suspect position may be a camera/odometry estimate rather than simulation
ground truth; use confidence and last_seen_age to make conservative plans.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, List, Tuple, Dict, Any
import math


Point = Tuple[float, float]
DistanceFn = Callable[[Point, Point], float]


@dataclass
class Robot:
    id: str
    x: float
    z: float
    speed: float = 60.0
    available: bool = True
    response_delay: float = 0.0


@dataclass
class Exit:
    id: str
    x: float
    z: float
    priority: float = 1.0
    enabled: bool = True


@dataclass
class Suspect:
    x: float
    z: float
    vx: float = 0.0
    vz: float = 0.0
    speed: float = 80.0
    confidence: float = 1.0       # 0..1 camera/track localization confidence
    last_seen_age: float = 0.0    # seconds since last visual confirmation
    track_id: str = ""


@dataclass
class Assignment:
    robot_id: str
    exit_id: str
    target: Point
    role: str
    robot_eta: float
    suspect_eta: float
    will_arrive_first: bool


@dataclass
class Plan:
    predicted_exit: str
    assignments: List[Assignment]
    contained: bool
    breaches: List[str]


def plan_interception(
    robots: List[Robot],
    exits: List[Exit],
    suspect: Suspect,
    dist: Optional[Callable[[Tuple[float, float], Tuple[float, float]], float]] = None,
) -> Plan:
    """Return greedy robot assignments for blocking exits and pursuing."""
    distance = dist or _euclidean
    active_exits = [e for e in exits if e.enabled]
    if not robots or not active_exits:
        return Plan(
            predicted_exit="",
            assignments=[],
            contained=False,
            breaches=[e.id for e in active_exits],
        )

    predicted = _predict_exit(active_exits, suspect, distance)
    suspect_etas = {
        e.id: _suspect_eta(suspect, e, distance)
        for e in active_exits
    }
    exits_by_urgency = sorted(active_exits, key=lambda e: (suspect_etas[e.id], -float(e.priority), e.id))

    unused = [r for r in robots if r.available]
    assignments: List[Assignment] = []
    blocked_exit_ids = set()

    for exit_ in exits_by_urgency:
        if not unused:
            break
        robot = min(unused, key=lambda r: (_robot_eta(r, exit_, distance), r.id))
        robot_eta = _robot_eta(robot, exit_, distance)
        suspect_eta = suspect_etas[exit_.id]
        assignments.append(Assignment(
            robot_id=robot.id,
            exit_id=exit_.id,
            target=(exit_.x, exit_.z),
            role="block",
            robot_eta=robot_eta,
            suspect_eta=suspect_eta,
            will_arrive_first=robot_eta <= suspect_eta,
        ))
        blocked_exit_ids.add(exit_.id)
        unused.remove(robot)

    if unused and _can_pursue(suspect):
        robot = min(unused, key=lambda r: (_euclidean((r.x, r.z), (suspect.x, suspect.z)), r.id))
        robot_eta = distance((robot.x, robot.z), (suspect.x, suspect.z)) / max(float(robot.speed), 1.0)
        assignments.append(Assignment(
            robot_id=robot.id,
            exit_id=predicted.id,
            target=(suspect.x, suspect.z),
            role="pursue",
            robot_eta=robot_eta,
            suspect_eta=0.0,
            will_arrive_first=False,
        ))

    block_by_exit = {
        a.exit_id: a
        for a in assignments
        if a.role == "block"
    }
    breaches = [
        e.id
        for e in exits_by_urgency
        if e.id not in blocked_exit_ids or not block_by_exit[e.id].will_arrive_first
    ]

    return Plan(
        predicted_exit=predicted.id,
        assignments=assignments,
        contained=len(breaches) == 0,
        breaches=breaches,
    )


def _predict_exit(exits: List[Exit], suspect: Suspect, dist: DistanceFn) -> Exit:
    sp = (suspect.x, suspect.z)
    vmag = math.hypot(suspect.vx, suspect.vz)
    if vmag <= 1e-6 or suspect.confidence < 0.35:
        return min(exits, key=lambda e: (_exit_score_without_velocity(e, sp, dist), e.id))

    vx = suspect.vx / vmag
    vz = suspect.vz / vmag

    def score(exit_: Exit) -> Tuple[float, str]:
        d = dist(sp, (exit_.x, exit_.z))
        dx = exit_.x - suspect.x
        dz = exit_.z - suspect.z
        emeg = math.hypot(dx, dz)
        alignment = 0.0 if emeg <= 1e-6 else (dx / emeg) * vx + (dz / emeg) * vz
        direction_weight = 0.5 * _clamp01(suspect.confidence)
        priority = max(float(exit_.priority), 0.001)
        return (d * (1.0 - direction_weight * alignment)) / priority, exit_.id

    return min(exits, key=score)


def _suspect_eta(suspect: Suspect, exit_: Exit, dist: DistanceFn) -> float:
    vmag = math.hypot(suspect.vx, suspect.vz)
    speed = max(float(suspect.speed), vmag, 1.0)
    eta = dist((suspect.x, suspect.z), (exit_.x, exit_.z)) / speed
    return max(0.0, eta - max(float(suspect.last_seen_age), 0.0))


def _robot_eta(robot: Robot, exit_: Exit, dist: DistanceFn) -> float:
    return max(float(robot.response_delay), 0.0) + dist((robot.x, robot.z), (exit_.x, exit_.z)) / max(float(robot.speed), 1.0)


def _euclidean(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def exits_from_config(config: Dict[str, Any]) -> List[Exit]:
    """Build Exit objects from a zones.json-style dict without file I/O."""
    exits = []
    for item in config.get("exits", []) or []:
        try:
            exits.append(Exit(
                id=str(item.get("id", item.get("name", ""))),
                x=float(item["x"]),
                z=float(item["z"]),
                priority=float(item.get("priority", 1.0)),
                enabled=bool(item.get("enabled", True)),
            ))
        except (KeyError, TypeError, ValueError):
            continue
    return [e for e in exits if e.id]


def _exit_score_without_velocity(exit_: Exit, suspect_pos: Point, dist: DistanceFn) -> float:
    priority = max(float(exit_.priority), 0.001)
    return dist(suspect_pos, (exit_.x, exit_.z)) / priority


def _can_pursue(suspect: Suspect) -> bool:
    return _clamp01(suspect.confidence) >= 0.45 and suspect.last_seen_age <= 1.5


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, float(v)))
