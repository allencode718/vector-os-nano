"""Level 10: ExploreSkill + VLM auto-look integration tests.

Tests that when ExploreSkill enters a new room during exploration,
it automatically calls VLM to describe the scene and records
observations in the SceneGraph.

All tests use mock VLM (no real API calls) and mock base (no MuJoCo).
"""
from __future__ import annotations

import threading
import time
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from vector_os_nano.core.scene_graph import SceneGraph
from vector_os_nano.core.skill import SkillContext
from vector_os_nano.core.types import SkillResult
from vector_os_nano.skills.go2.explore import (
    ExploreSkill,
    _exploration_loop,
    _explore_cancel,
    _explore_visited,
    cancel_exploration,
    get_explored_rooms,
    is_exploring,
    set_auto_look,
    set_event_callback,
)
from vector_os_nano.skills.navigate import _ROOM_CENTERS


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _make_mock_vlm():
    """Create a mock VLM that returns predictable results."""
    vlm = MagicMock()

    # describe_scene returns SceneDescription-like object
    scene = MagicMock()
    scene.summary = "A room with furniture"
    obj1 = MagicMock()
    obj1.name = "table"
    obj1.confidence = 0.9
    obj2 = MagicMock()
    obj2.name = "chair"
    obj2.confidence = 0.85
    scene.objects = [obj1, obj2]
    vlm.describe_scene.return_value = scene

    # identify_room returns RoomIdentification-like object
    room_id = MagicMock()
    room_id.room = "living_room"
    room_id.confidence = 0.92
    vlm.identify_room.return_value = room_id

    return vlm


def _make_mock_base(rooms_sequence: list[str]):
    """Create a mock base that moves through a sequence of rooms.

    Each call to get_position() returns the center of the next room
    in the sequence.
    """
    base = MagicMock()
    pos_index = [0]

    def _get_position():
        idx = min(pos_index[0], len(rooms_sequence) - 1)
        room = rooms_sequence[idx]
        center = _ROOM_CENTERS.get(room, (0.0, 0.0))
        pos_index[0] += 1
        return (center[0], center[1], 0.28)  # z=0.28 = standing

    def _get_heading():
        return 0.0

    def _get_camera_frame(width=320, height=240):
        return np.zeros((height, width, 3), dtype=np.uint8)

    base.get_position = _get_position
    base.get_heading = _get_heading
    base.get_camera_frame = _get_camera_frame
    base.set_velocity = MagicMock()
    base.walk = MagicMock(return_value=True)
    return base


def _make_context(base, vlm=None, scene_graph=None):
    """Build a SkillContext with optional VLM and SceneGraph."""
    services = {}
    if vlm is not None:
        services["vlm"] = vlm
    if scene_graph is not None:
        services["spatial_memory"] = scene_graph
    return SkillContext(
        bases={"default": base},
        services=services,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestAutoLookCallback:
    """Test that set_auto_look / auto-look callback mechanism works."""

    def test_set_auto_look_is_callable(self):
        """set_auto_look accepts a callable."""
        called = []
        set_auto_look(lambda room: called.append(room))
        # Reset
        set_auto_look(None)

    def test_auto_look_called_on_new_room(self):
        """Auto-look callback fires when exploration enters a new room."""
        observations = []

        def mock_look(room: str) -> dict | None:
            observations.append(room)
            return {"summary": f"Observed {room}", "objects": []}

        set_auto_look(mock_look)

        # Simulate entering two rooms by running a short exploration
        base = _make_mock_base(["living_room", "living_room", "kitchen", "kitchen"])
        _explore_cancel.clear()
        _explore_visited.clear()

        # Run exploration loop briefly with a cancel after a few iterations
        def _cancel_soon():
            time.sleep(1.0)
            _explore_cancel.set()

        cancel_thread = threading.Thread(target=_cancel_soon, daemon=True)
        cancel_thread.start()

        with patch(
            "vector_os_nano.skills.go2.explore._start_tare",
            return_value=False,
        ):
            _exploration_loop(base, has_bridge=False)

        cancel_thread.join(timeout=3.0)

        # Should have observed at least one room
        assert len(observations) >= 1
        assert "living_room" in observations or "kitchen" in observations

        # Cleanup
        set_auto_look(None)

    def test_auto_look_failure_does_not_crash_exploration(self):
        """If auto-look raises, exploration continues."""
        def failing_look(room: str) -> dict | None:
            raise RuntimeError("VLM crashed")

        set_auto_look(failing_look)

        base = _make_mock_base(["living_room", "kitchen"])
        _explore_cancel.clear()
        _explore_visited.clear()

        def _cancel_soon():
            time.sleep(0.5)
            _explore_cancel.set()

        cancel_thread = threading.Thread(target=_cancel_soon, daemon=True)
        cancel_thread.start()

        with patch(
            "vector_os_nano.skills.go2.explore._start_tare",
            return_value=False,
        ):
            # Should not raise
            _exploration_loop(base, has_bridge=False)

        cancel_thread.join(timeout=3.0)

        # Exploration still recorded rooms even though auto-look failed
        assert len(_explore_visited) >= 1

        # Cleanup
        set_auto_look(None)

    def test_auto_look_none_is_noop(self):
        """When auto-look is None, exploration works normally."""
        set_auto_look(None)

        base = _make_mock_base(["hallway", "hallway"])
        _explore_cancel.clear()
        _explore_visited.clear()

        def _cancel_soon():
            time.sleep(0.5)
            _explore_cancel.set()

        cancel_thread = threading.Thread(target=_cancel_soon, daemon=True)
        cancel_thread.start()

        with patch(
            "vector_os_nano.skills.go2.explore._start_tare",
            return_value=False,
        ):
            _exploration_loop(base, has_bridge=False)

        cancel_thread.join(timeout=3.0)
        assert "hallway" in _explore_visited


class TestExploreSkillAutoLookWiring:
    """Test that ExploreSkill.execute() wires auto-look from context."""

    def test_explore_wires_auto_look_when_vlm_available(self):
        """ExploreSkill sets auto-look callback when VLM is in services."""
        vlm = _make_mock_vlm()
        scene_graph = SceneGraph()
        base = _make_mock_base(["living_room"])
        context = _make_context(base, vlm=vlm, scene_graph=scene_graph)

        skill = ExploreSkill()

        # Patch bridge/nav to skip real ROS2
        with patch(
            "vector_os_nano.skills.go2.explore._start_bridge_on_go2",
            return_value=False,
        ):
            result = skill.execute({}, context)

        assert result.success
        assert result.result_data["status"] == "exploration_started"

        # Cancel immediately
        cancel_exploration()

    def test_explore_no_vlm_still_works(self):
        """ExploreSkill works without VLM (no auto-look)."""
        base = _make_mock_base(["hallway"])
        context = _make_context(base, vlm=None, scene_graph=None)

        skill = ExploreSkill()

        with patch(
            "vector_os_nano.skills.go2.explore._start_bridge_on_go2",
            return_value=False,
        ):
            result = skill.execute({}, context)

        assert result.success
        cancel_exploration()


class TestAutoLookSceneGraphIntegration:
    """Test that auto-look observations are recorded in SceneGraph."""

    def test_observations_recorded_in_scene_graph(self):
        """Auto-look records VLM observations via scene_graph.observe_with_viewpoint."""
        vlm = _make_mock_vlm()
        scene_graph = SceneGraph()
        base = _make_mock_base(["living_room", "living_room", "living_room"])
        context = _make_context(base, vlm=vlm, scene_graph=scene_graph)

        skill = ExploreSkill()

        with patch(
            "vector_os_nano.skills.go2.explore._start_bridge_on_go2",
            return_value=False,
        ):
            result = skill.execute({}, context)

        # Wait for auto-look to fire in the background thread
        time.sleep(1.5)
        cancel_exploration()
        time.sleep(0.5)

        # SceneGraph should have the observed room
        visited = scene_graph.get_visited_rooms()
        assert len(visited) >= 1

    def test_room_observed_event_emitted(self):
        """A 'room_observed' event is emitted after auto-look succeeds."""
        events = []

        def capture_event(event_type: str, data: dict):
            events.append((event_type, data))

        set_event_callback(capture_event)

        vlm = _make_mock_vlm()
        scene_graph = SceneGraph()
        base = _make_mock_base(["kitchen", "kitchen", "kitchen"])
        context = _make_context(base, vlm=vlm, scene_graph=scene_graph)

        skill = ExploreSkill()

        with patch(
            "vector_os_nano.skills.go2.explore._start_bridge_on_go2",
            return_value=False,
        ):
            skill.execute({}, context)

        time.sleep(1.5)
        cancel_exploration()
        time.sleep(0.5)

        event_types = [e[0] for e in events]
        assert "room_observed" in event_types

        # Check the room_observed event data
        observed_events = [e for e in events if e[0] == "room_observed"]
        assert len(observed_events) >= 1
        data = observed_events[0][1]
        assert "summary" in data
        assert "objects" in data

        # Cleanup
        set_event_callback(None)
