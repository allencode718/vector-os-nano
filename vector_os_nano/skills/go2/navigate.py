"""NavigateSkill — autonomous room-to-room navigation for Go2.

Uses a room map + waypoint graph to navigate between named rooms.
Each room has a center coordinate and a doorway coordinate. Navigation
goes: current_pos → src_door → hallway → dst_door → dst_center.

The house layout must match go2_room.xml (20m x 14m house).
"""
from __future__ import annotations

import logging
import math
from typing import Any

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Room map — must match go2_room.xml layout
# ---------------------------------------------------------------------------

# Room name → (center_x, center_y)
_ROOM_CENTERS: dict[str, tuple[float, float]] = {
    "living_room":    (3.0, 2.5),
    "dining_room":    (3.0, 7.5),
    "kitchen":        (17.0, 2.5),
    "study":          (17.0, 7.5),
    "master_bedroom": (3.5, 12.0),
    "guest_bedroom":  (16.0, 12.0),
    "bathroom":       (8.5, 12.0),
    "hallway":        (10.0, 5.0),
}

# Room name → doorway coordinate (point in the hallway just outside the door)
_ROOM_DOORS: dict[str, tuple[float, float]] = {
    "living_room":    (6.5, 3.0),
    "dining_room":    (6.5, 8.0),
    "kitchen":        (13.5, 3.0),
    "study":          (13.5, 8.0),
    "master_bedroom": (3.0, 10.5),
    "guest_bedroom":  (12.0, 10.5),
    "bathroom":       (8.5, 10.5),
    "hallway":        (10.0, 5.0),  # hallway door = hallway center
}

# Aliases → canonical room name (Chinese + English + shortcuts)
_ROOM_ALIASES: dict[str, str] = {
    # English
    "living room": "living_room", "living": "living_room", "lounge": "living_room",
    "dining room": "dining_room", "dining": "dining_room",
    "kitchen": "kitchen",
    "study": "study", "office": "study",
    "master bedroom": "master_bedroom", "master": "master_bedroom",
    "bedroom": "master_bedroom",  # default bedroom = master
    "guest bedroom": "guest_bedroom", "guest room": "guest_bedroom", "guest": "guest_bedroom",
    "bathroom": "bathroom", "bath": "bathroom", "restroom": "bathroom", "toilet": "bathroom",
    "hallway": "hallway", "hall": "hallway", "corridor": "hallway",
    "laundry": "hallway",  # laundry is in hallway area
    # Chinese
    "客厅": "living_room", "大厅": "living_room",
    "餐厅": "dining_room", "饭厅": "dining_room",
    "厨房": "kitchen",
    "书房": "study", "办公室": "study", "工作室": "study",
    "主卧": "master_bedroom", "卧室": "master_bedroom", "主卧室": "master_bedroom",
    "客卧": "guest_bedroom", "客房": "guest_bedroom", "次卧": "guest_bedroom",
    "卫生间": "bathroom", "浴室": "bathroom", "洗手间": "bathroom", "厕所": "bathroom",
    "走廊": "hallway", "过道": "hallway", "大厅走廊": "hallway",
    "洗衣房": "hallway",
}

_WALK_SPEED: float = 0.4     # m/s
_TURN_SPEED: float = 0.8     # rad/s
_ARRIVAL_RADIUS: float = 0.5  # close enough


def _resolve_room(name: str) -> str | None:
    """Resolve a room name/alias to canonical room key."""
    key = name.strip().lower().replace("_", " ")
    # Direct match
    if key.replace(" ", "_") in _ROOM_CENTERS:
        return key.replace(" ", "_")
    # Alias match
    return _ROOM_ALIASES.get(key)


def _angle_between(x1: float, y1: float, x2: float, y2: float) -> float:
    """Angle from (x1,y1) to (x2,y2) in radians."""
    return math.atan2(y2 - y1, x2 - x1)


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def _normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _detect_current_room(x: float, y: float) -> str:
    """Guess which room the robot is currently in based on position."""
    best_room = "hallway"
    best_dist = float("inf")
    for room, (cx, cy) in _ROOM_CENTERS.items():
        d = _distance(x, y, cx, cy)
        if d < best_dist:
            best_dist = d
            best_room = room
    return best_room


def _navigate_to_waypoint(
    base: Any, target_x: float, target_y: float, label: str,
) -> bool:
    """Turn toward waypoint and walk to it. Returns True if arrived upright."""
    pos = base.get_position()
    cx, cy = pos[0], pos[1]
    heading = base.get_heading()

    dist = _distance(cx, cy, target_x, target_y)
    if dist < _ARRIVAL_RADIUS:
        logger.info("[NAV] Already at %s (%.1fm away)", label, dist)
        return True

    # Calculate turn
    target_angle = _angle_between(cx, cy, target_x, target_y)
    turn_needed = _normalize_angle(target_angle - heading)

    # Turn
    if abs(turn_needed) > 0.1:  # >5.7 degrees
        vyaw = _TURN_SPEED if turn_needed > 0 else -_TURN_SPEED
        turn_dur = abs(turn_needed) / _TURN_SPEED
        logger.info("[NAV] Turn %.0f deg toward %s", math.degrees(turn_needed), label)
        base.walk(0.0, 0.0, vyaw, turn_dur)

    # Walk
    walk_dur = dist / _WALK_SPEED
    logger.info("[NAV] Walk %.1fm to %s", dist, label)
    base.walk(_WALK_SPEED, 0.0, 0.0, walk_dur)

    # Check arrival
    pos = base.get_position()
    if pos[2] < 0.12:
        logger.error("[NAV] Robot fell during navigation")
        return False
    return True


@skill(
    aliases=[
        "navigate", "go to", "goto",
        "去", "到", "走到", "去到", "导航",
    ],
    direct=False,
)
class NavigateSkill:
    """Navigate the Go2 to a named room in the house."""

    name: str = "navigate"
    description: str = (
        "Navigate the robot to a specific room by name. "
        "Available rooms: living_room, dining_room, kitchen, study, "
        "master_bedroom, guest_bedroom, bathroom, hallway."
    )
    parameters: dict = {
        "room": {
            "type": "string",
            "required": True,
            "description": (
                "Target room name. Examples: kitchen, bedroom, study, "
                "bathroom, living_room, dining_room, guest_bedroom, hallway. "
                "Chinese: 厨房, 卧室, 书房, 卫生间, 客厅, 餐厅, 客房, 走廊."
            ),
        },
    }
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects: dict = {"position": "changed"}
    failure_modes: list[str] = ["no_base", "unknown_room", "navigation_failed"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        if context.base is None:
            return SkillResult(
                success=False,
                error_message="No base connected",
                diagnosis_code="no_base",
            )

        room_input = str(params.get("room", ""))
        room_key = _resolve_room(room_input)

        if room_key is None:
            available = ", ".join(sorted(_ROOM_CENTERS.keys()))
            return SkillResult(
                success=False,
                error_message=f"Unknown room: '{room_input}'. Available: {available}",
                diagnosis_code="unknown_room",
            )

        # Current position
        pos = context.base.get_position()
        cx, cy = pos[0], pos[1]
        src_room = _detect_current_room(cx, cy)

        target_center = _ROOM_CENTERS[room_key]

        # Already there?
        if _distance(cx, cy, target_center[0], target_center[1]) < _ARRIVAL_RADIUS:
            return SkillResult(
                success=True,
                result_data={"room": room_key, "position": [cx, cy], "note": "already here"},
            )

        logger.info("[NAV] Navigating from %s to %s", src_room, room_key)

        # Build waypoint sequence:
        # 1) Exit current room via its door (unless already in hallway)
        # 2) Go to target room's door
        # 3) Enter target room center
        waypoints: list[tuple[float, float, str]] = []

        if src_room != "hallway" and src_room != room_key:
            door = _ROOM_DOORS[src_room]
            waypoints.append((door[0], door[1], f"{src_room} door"))

        if room_key != "hallway":
            door = _ROOM_DOORS[room_key]
            waypoints.append((door[0], door[1], f"{room_key} door"))

        waypoints.append((target_center[0], target_center[1], room_key))

        # Execute waypoint sequence
        for wx, wy, label in waypoints:
            ok = _navigate_to_waypoint(context.base, wx, wy, label)
            if not ok:
                return SkillResult(
                    success=False,
                    error_message=f"Navigation failed near {label}",
                    diagnosis_code="navigation_failed",
                )

        final_pos = context.base.get_position()
        return SkillResult(
            success=True,
            result_data={
                "room": room_key,
                "from_room": src_room,
                "position": [round(final_pos[0], 1), round(final_pos[1], 1)],
            },
        )
