"""Level 33: Navigation pipeline integrity harness.

Tests verify the full navigate pipeline from proxy to FAR to localPlanner:
  - FAR detection uses /way_point (not /path)
  - Phase 1 probe correctly detects FAR availability
  - Phase 2 only publishes /goal_point (no direct /way_point)
  - Stale waypoint/path rejection
  - SceneGraph drift protection
  - Diagnostic logging
  - Cylinder body safety boundary
"""
from __future__ import annotations

import importlib
import inspect
import math
import os
import re

import pytest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))

from nav_debug_helpers import read_bridge_source

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _proxy_source() -> str:
    mod = importlib.import_module("vector_os_nano.hardware.sim.go2_ros2_proxy")
    return inspect.getsource(mod)


def _navigate_source() -> str:
    mod = importlib.import_module("vector_os_nano.skills.navigate")
    return inspect.getsource(mod)


# ===================================================================
# Part 1: FAR detection must use /way_point, NOT /path
# ===================================================================

class TestFARDetectionSignal:
    """FAR availability must be detected via /way_point, not /path.

    /path from localPlanner is unreliable — it publishes even without
    a valid goal from FAR. /way_point is ONLY published by FAR when
    it has a V-Graph and can route to the goal.
    """

    def test_proxy_subscribes_to_waypoint(self):
        """Proxy must subscribe to /way_point for FAR detection."""
        src = _proxy_source()
        assert "/way_point" in src
        assert "_waypoint_cb" in src or "waypoint_cb" in src

    def test_proxy_has_waypoint_timestamp(self):
        """Proxy tracks _last_waypoint_time for FAR probe."""
        src = _proxy_source()
        assert "_last_waypoint_time" in src

    def test_probe_checks_waypoint_not_path(self):
        """Phase 1 probe must check _last_waypoint_time, not _last_path_time."""
        src = _proxy_source()
        nav_start = src.find("def navigate_to")
        nav_end = src.find("\n    def ", nav_start + 1)
        nav_body = src[nav_start:nav_end]
        assert "_last_waypoint_time" in nav_body, (
            "navigate_to must check _last_waypoint_time for FAR detection"
        )
        # Should NOT use _last_path_time for FAR detection
        assert "_last_path_time > start_time" not in nav_body, (
            "navigate_to must NOT use _last_path_time — /path is unreliable"
        )

    def test_waypoint_timestamp_reset_before_probe(self):
        """_last_waypoint_time must be reset to 0 before Phase 1 probe."""
        src = _proxy_source()
        nav_start = src.find("def navigate_to")
        nav_body = src[nav_start:nav_start + 3000]
        assert "_last_waypoint_time = 0" in nav_body, (
            "Must reset _last_waypoint_time before probe to ignore stale data"
        )


# ===================================================================
# Part 2: Phase 2 must NOT publish direct /way_point
# ===================================================================

class TestPhase2NoDirectWaypoint:
    """Phase 2 must only publish /goal_point. Direct /way_point to
    localPlanner overrides FAR's routed intermediate waypoints and
    causes the dog to navigate straight through walls.
    """

    def test_phase2_publishes_goal_point(self):
        """Phase 2 loop must publish /goal_point."""
        src = _proxy_source()
        # Find the actual Phase 2 code block (after the docstring)
        phase2_comment = src.find("# Phase 2: full navigation")
        if phase2_comment < 0:
            phase2_comment = src.find("Phase 2")
        assert phase2_comment > 0, "Phase 2 section not found"
        phase2 = src[phase2_comment:phase2_comment + 800]
        assert "_publish_goal_point" in phase2

    def test_phase2_no_direct_waypoint(self):
        """Phase 2 must NOT call _publish_waypoint (would override FAR)."""
        src = _proxy_source()
        phase2_start = src.find("Phase 2")
        phase2_end = src.find("far_timeout", phase2_start)
        if phase2_end < 0:
            phase2_end = phase2_start + 800
        phase2 = src[phase2_start:phase2_end]
        assert "_publish_waypoint" not in phase2, (
            "Phase 2 must NOT publish /way_point directly — "
            "it overrides FAR's door routing and causes wall collision"
        )

    def test_phase1_no_direct_waypoint(self):
        """Phase 1 probe must NOT publish /way_point either."""
        src = _proxy_source()
        phase1_start = src.find("Phase 1")
        phase2_start = src.find("Phase 2")
        phase1 = src[phase1_start:phase2_start]
        assert "_publish_waypoint" not in phase1, (
            "Phase 1 must NOT publish /way_point — only /goal_point"
        )


# ===================================================================
# Part 3: SceneGraph drift protection
# ===================================================================

class TestSceneGraphDriftProtection:
    """Navigate must reject SceneGraph positions that are too far from
    the hardcoded room center — these are usually recorded at doorways
    or in the wrong room entirely.
    """

    def test_max_drift_constant_exists(self):
        src = _navigate_source()
        assert "_MAX_DRIFT" in src

    def test_max_drift_value_reasonable(self):
        src = _navigate_source()
        match = re.search(r'_MAX_DRIFT.*?=\s*([\d.]+)', src)
        assert match, "_MAX_DRIFT not found"
        drift = float(match.group(1))
        assert 1.0 <= drift <= 4.0, (
            f"_MAX_DRIFT={drift}m — should be 1.0-4.0m"
        )

    def test_drift_check_in_get_room_center(self):
        """_get_room_center_from_memory must check drift from hardcoded center."""
        src = _navigate_source()
        func_start = src.find("def _get_room_center_from_memory")
        func_end = src.find("\ndef ", func_start + 1)
        func_body = src[func_start:func_end]
        assert "_MAX_DRIFT" in func_body or "drift" in func_body, (
            "_get_room_center_from_memory must check position drift"
        )
        assert "_ROOM_CENTERS" in func_body or "hardcoded" in func_body.lower(), (
            "Must compare against hardcoded room center"
        )

    def test_drift_rejection_logs_warning(self):
        """When drift exceeds threshold, a warning must be logged."""
        src = _navigate_source()
        func_start = src.find("def _get_room_center_from_memory")
        func_end = src.find("\ndef ", func_start + 1)
        func_body = src[func_start:func_end]
        assert "warning" in func_body.lower() or "logger.warn" in func_body, (
            "Must log warning when rejecting drifted SceneGraph position"
        )


# ===================================================================
# Part 4: Cylinder body safety boundary
# ===================================================================

class TestCylinderBodySafety:
    """Path follower must treat the dog as a cylinder — subtract body
    radius from obstacle distances before safety checks.
    """

    def test_body_front_constant(self):
        src = read_bridge_source()
        assert "_BODY_FRONT" in src
        match = re.search(r'_BODY_FRONT\s*=\s*([\d.]+)', src)
        assert match
        val = float(match.group(1))
        assert 0.30 <= val <= 0.40, f"_BODY_FRONT={val} — should be ~0.34m (head)"

    def test_body_side_constant(self):
        src = read_bridge_source()
        assert "_BODY_SIDE" in src
        match = re.search(r'_BODY_SIDE\s*=\s*([\d.]+)', src)
        assert match
        val = float(match.group(1))
        assert 0.15 <= val <= 0.25, f"_BODY_SIDE={val} — should be ~0.19m (hip)"

    def test_gap_calculation(self):
        """Safety must compute gap = obstacle_distance - body_extent."""
        src = read_bridge_source()
        follow_start = src.find("def _follow_path")
        follow_body = src[follow_start:]
        has_gap = (
            "front_gap" in follow_body
            or "left_gap" in follow_body
            or "front_d - _BODY" in follow_body
        )
        assert has_gap, (
            "Must compute gap (obstacle_dist - body_radius), not use raw distance"
        )

    def test_comfort_zone_exists(self):
        """Must define a comfort zone for body-to-wall gap."""
        src = read_bridge_source()
        assert "_COMFORT" in src or "comfort" in src.lower()

    def test_danger_zone_exists(self):
        """Must define a danger zone for imminent body contact."""
        src = read_bridge_source()
        assert "_DANGER" in src or "danger" in src.lower()

    def test_blocks_motion_toward_wall(self):
        """When wall is on one side, must block motion toward that wall."""
        src = read_bridge_source()
        follow_start = src.find("def _follow_path")
        follow_body = src[follow_start:]
        # Should have: if vy > 0: vy = 0.0 (block leftward when wall left)
        assert "vy = 0.0" in follow_body or "vy = 0" in follow_body, (
            "Must block lateral motion toward nearby wall"
        )


# ===================================================================
# Part 5: Behavioral - drift protection simulation
# ===================================================================

class TestDriftProtectionBehavior:

    def test_nearby_position_accepted(self):
        """SceneGraph position within 2m of hardcoded center is accepted."""
        mod = importlib.import_module("vector_os_nano.skills.navigate")

        class FakeRoom:
            center_x = 17.5  # kitchen hardcoded (17.0, 2.5), this is 0.5m off
            center_y = 2.8
            visit_count = 5

        class FakeMemory:
            def get_room(self, key):
                if key == "kitchen":
                    return FakeRoom()
                return None

        result = mod._get_room_center_from_memory(FakeMemory(), "kitchen")
        assert result is not None, "0.5m drift should be accepted"
        assert result == (17.5, 2.8)

    def test_far_position_rejected(self):
        """SceneGraph position >2m from hardcoded center is rejected."""
        mod = importlib.import_module("vector_os_nano.skills.navigate")

        class FakeRoom:
            center_x = 14.0  # 3m from kitchen center (17.0)
            center_y = 2.5
            visit_count = 5

        class FakeMemory:
            def get_room(self, key):
                if key == "kitchen":
                    return FakeRoom()
                return None

        result = mod._get_room_center_from_memory(FakeMemory(), "kitchen")
        assert result is None, "3.0m drift should be rejected"

    def test_low_visit_count_rejected(self):
        """SceneGraph position with < 3 visits is rejected."""
        mod = importlib.import_module("vector_os_nano.skills.navigate")

        class FakeRoom:
            center_x = 17.0
            center_y = 2.5
            visit_count = 1  # only 1 visit — doorway drive-by

        class FakeMemory:
            def get_room(self, key):
                if key == "kitchen":
                    return FakeRoom()
                return None

        result = mod._get_room_center_from_memory(FakeMemory(), "kitchen")
        assert result is None, "1 visit should not be trusted"

    def test_unknown_room_returns_none(self):
        """Unknown room key returns None."""
        mod = importlib.import_module("vector_os_nano.skills.navigate")

        class FakeMemory:
            def get_room(self, key):
                return None

        result = mod._get_room_center_from_memory(FakeMemory(), "nonexistent")
        assert result is None


# ===================================================================
# Part 6: Diagnostic logging
# ===================================================================

class TestNavigateDiagnostics:
    """Navigate must log progress during Phase 2 for debugging."""

    def test_progress_logging_exists(self):
        src = _proxy_source()
        nav_start = src.find("def navigate_to")
        nav_body = src[nav_start:]
        assert "waypoint_age" in nav_body or "wp_age" in nav_body, (
            "Navigate must log waypoint age for debugging"
        )

    def test_far_response_logged(self):
        src = _proxy_source()
        assert "FAR responded" in src, (
            "Must log when FAR responds with /way_point"
        )

    def test_early_fallback_logged(self):
        src = _proxy_source()
        assert "early_fallback" in src, (
            "Must log when falling back to dead-reckoning"
        )
