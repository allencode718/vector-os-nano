"""Exploration + Spatial Memory skills — learn the environment by moving through it.

Instead of hardcoded room coordinates, the agent:
  1. Explores by navigating to different directions/distances
  2. Remembers locations the user names ("记住这里叫厨房")
  3. Navigates to remembered locations ("去厨房")
  4. Reports its actual position from the nav stack odometry
"""
from __future__ import annotations

import logging
import math
import time
from typing import Any

from vector_os_nano.core.skill import SkillContext, skill
from vector_os_nano.core.types import SkillResult

logger = logging.getLogger(__name__)


def _get_memory(context: SkillContext) -> Any:
    from vector_os_nano.core.spatial_memory import SpatialMemory
    mem = context.services.get("spatial_memory")
    if mem is None:
        mem = SpatialMemory()
        context.services["spatial_memory"] = mem
    return mem


def _get_position(context: SkillContext) -> tuple[float, float] | None:
    """Get actual robot position from base or nav client."""
    if context.base is not None:
        try:
            pos = context.base.get_position()
            return (pos[0], pos[1])
        except Exception:
            pass
    nav = context.services.get("nav")
    if nav is not None:
        odom = nav.get_state_estimation()
        if odom:
            return (odom.x, odom.y)
    return None


def _navigate_and_wait(context: SkillContext, x: float, y: float, timeout: float = 30.0) -> bool:
    """Navigate to (x, y) and verify arrival by checking position."""
    nav = context.services.get("nav")
    if nav is None or not nav.is_available:
        return False

    nav.navigate_to(x, y, timeout=timeout)

    # Check actual position — did we get close?
    pos = _get_position(context)
    if pos is None:
        return False
    dist = math.sqrt((pos[0] - x)**2 + (pos[1] - y)**2)
    return dist < 3.0  # within 3m = close enough


@skill(
    aliases=["explore", "探索", "逛逛", "看看", "explore house", "look around"],
    direct=False,
)
class ExploreSkill:
    """Explore the environment by moving in different directions."""

    name: str = "explore"
    description: str = (
        "Explore the environment. Navigate to a direction and distance to discover new areas. "
        "Use remember_location to save interesting spots."
    )
    parameters: dict = {
        "direction": {
            "type": "string",
            "required": False,
            "default": "forward",
            "enum": ["forward", "left", "right", "back"],
            "description": "Direction to explore relative to current heading",
        },
        "distance": {
            "type": "number",
            "required": False,
            "default": 5.0,
            "description": "How far to explore in meters",
        },
    }
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects: dict = {"position": "changed"}
    failure_modes: list[str] = ["no_base", "no_nav", "navigation_failed"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        pos = _get_position(context)
        if pos is None:
            return SkillResult(success=False, diagnosis_code="no_base",
                             error_message="Cannot get robot position")

        nav = context.services.get("nav")
        if nav is None or not nav.is_available:
            return SkillResult(success=False, diagnosis_code="no_nav",
                             error_message="Navigation stack not available")

        direction = params.get("direction", "forward")
        distance = float(params.get("distance", 5.0))

        # Get current heading
        heading = 0.0
        if context.base:
            try:
                heading = context.base.get_heading()
            except Exception:
                pass

        # Calculate target based on direction
        angle_map = {
            "forward": 0, "left": math.pi/2,
            "right": -math.pi/2, "back": math.pi,
        }
        angle = heading + angle_map.get(direction, 0)
        tx = pos[0] + distance * math.cos(angle)
        ty = pos[1] + distance * math.sin(angle)

        logger.info("[EXPLORE] From (%.1f, %.1f) heading %s %.1fm to (%.1f, %.1f)",
                   pos[0], pos[1], direction, distance, tx, ty)

        ok = _navigate_and_wait(context, tx, ty, timeout=30.0)
        new_pos = _get_position(context)

        # Record visit
        memory = _get_memory(context)
        if new_pos:
            loc_name = memory.current_location_name(new_pos[0], new_pos[1])
            if loc_name:
                memory.visit(loc_name, new_pos[0], new_pos[1])

        return SkillResult(
            success=ok,
            result_data={
                "start": [round(pos[0], 1), round(pos[1], 1)],
                "target": [round(tx, 1), round(ty, 1)],
                "actual": [round(new_pos[0], 1), round(new_pos[1], 1)] if new_pos else None,
                "direction": direction,
                "distance": distance,
            },
        )


@skill(
    aliases=["remember", "记住", "mark", "标记", "save location", "保存位置"],
    direct=False,
)
class RememberLocationSkill:
    """Save the current position with a name for future navigation."""

    name: str = "remember_location"
    description: str = (
        "Save the robot's current position with a custom name. "
        "Later you can navigate back with navigate(room=name). "
        "Example: remember_location(name='kitchen') saves current spot as 'kitchen'."
    )
    parameters: dict = {
        "name": {
            "type": "string",
            "required": True,
            "description": "Name for this location (e.g., 'kitchen', 'charging_station', '厨房')",
        },
    }
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects: dict = {"memory": "updated"}
    failure_modes: list[str] = ["no_position"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        pos = _get_position(context)
        if pos is None:
            return SkillResult(success=False, diagnosis_code="no_position")

        name = params.get("name", "unnamed")
        memory = _get_memory(context)
        memory.add_location(name, pos[0], pos[1], category="landmark", tags=["user_defined"])
        memory.visit(name, pos[0], pos[1])

        # Also add Chinese alias if applicable
        from vector_os_nano.skills.navigate import _ROOM_ALIASES, _ROOM_CENTERS
        _ROOM_ALIASES[name.lower()] = name
        _ROOM_CENTERS[name] = (pos[0], pos[1])

        return SkillResult(
            success=True,
            result_data={
                "saved": name,
                "position": [round(pos[0], 1), round(pos[1], 1)],
                "total_locations": len(memory.get_all_locations()),
            },
        )


@skill(
    aliases=["where am i", "where", "在哪", "我在哪", "位置", "location", "status"],
    direct=True,
)
class WhereAmISkill:
    """Report the robot's current position and nearest known location."""

    name: str = "where_am_i"
    description: str = "Report the robot's current position and which known location it is near."
    parameters: dict = {}
    preconditions: list[str] = []
    postconditions: list[str] = []
    effects: dict = {}
    failure_modes: list[str] = ["no_position"]

    def execute(self, params: dict, context: SkillContext) -> SkillResult:
        pos = _get_position(context)
        if pos is None:
            return SkillResult(success=False, diagnosis_code="no_position")

        memory = _get_memory(context)
        loc_name = memory.current_location_name(pos[0], pos[1])
        nearest = memory.nearest_location(pos[0], pos[1])
        visited = memory.get_visited_locations()

        return SkillResult(
            success=True,
            result_data={
                "position": [round(pos[0], 1), round(pos[1], 1)],
                "current_location": loc_name,
                "nearest_known": nearest.name if nearest else None,
                "visited_count": len(visited),
                "known_locations": [l.name for l in memory.get_all_locations()],
                "memory": memory.summary_for_llm(),
            },
        )
