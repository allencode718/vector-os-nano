"""Level 8 — RViz MarkerArray: scene graph visualisation markers.

Tests ``build_scene_graph_markers()`` from
``vector_os_nano.ros2.nodes.scene_graph_viz``.

The function builds a ``visualization_msgs/MarkerArray`` from a live
``SceneGraph`` instance.  All tests are pure Python and run fast — no
MuJoCo, no real API calls, no ROS2 node spin-up required.  The entire
test class is skipped when ``visualization_msgs`` is not installed so
the suite stays green in bare-Python environments.

Marker namespaces under test
----------------------------
    rooms           — filled CUBE rectangles, one per room (8 total)
    room_labels     — TEXT_VIEW_FACING, one per room (8 total)
    viewpoints      — SPHERE at each ViewpointNode position
    objects         — CUBE for each detected ObjectNode
    object_labels   — TEXT_VIEW_FACING for each ObjectNode
    robot           — ARROW at current robot position
    nav_goal        — SPHERE at navigation target (only when provided)

Reference: ``vector_os_nano/ros2/nodes/scene_graph_viz.py``
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Ensure repo root is importable
# ---------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


# ---------------------------------------------------------------------------
# ROS2 availability guard
# ---------------------------------------------------------------------------

def _ros2_available() -> bool:
    try:
        from visualization_msgs.msg import MarkerArray  # noqa: F401
        return True
    except ImportError:
        return False


# ---------------------------------------------------------------------------
# Test class — skipped entirely when ROS2 message types are absent
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _ros2_available(), reason="ROS2 not available")
class TestLevel8RVizMarkers:
    """MarkerArray generation tests for the scene graph visualiser."""

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_markers(sg, **kwargs):
        """Call build_scene_graph_markers and assert it returns something."""
        from vector_os_nano.ros2.nodes.scene_graph_viz import build_scene_graph_markers
        ma = build_scene_graph_markers(sg, **kwargs)
        assert ma is not None, "build_scene_graph_markers returned None (ROS2 missing?)"
        return ma

    @staticmethod
    def _fresh_sg():
        from vector_os_nano.core.scene_graph import SceneGraph
        return SceneGraph()

    # ------------------------------------------------------------------
    # T8-0  Empty scene graph — room layout always present
    # ------------------------------------------------------------------

    def test_empty_scene_graph_has_room_markers(self):
        """Even with empty SceneGraph, room boundaries and labels are generated."""
        sg = self._fresh_sg()
        ma = self._make_markers(sg, robot_x=10.0, robot_y=3.0)
        # 8 rooms * 2 (boundary + label) + 1 robot = 17 markers minimum
        assert len(ma.markers) >= 17

    # ------------------------------------------------------------------
    # T8-1  Room namespace correctness
    # ------------------------------------------------------------------

    def test_room_boundary_namespaces(self):
        """Room markers use 'rooms' and 'room_labels' namespaces."""
        sg = self._fresh_sg()
        ma = self._make_markers(sg)

        room_markers = [m for m in ma.markers if m.ns == "rooms"]
        label_markers = [m for m in ma.markers if m.ns == "room_labels"]

        # Exactly 8 rooms defined in the static layout
        assert len(room_markers) == 8
        assert len(label_markers) == 8

    # ------------------------------------------------------------------
    # T8-2  Viewpoint markers
    # ------------------------------------------------------------------

    def test_viewpoint_markers_appear(self):
        """After adding viewpoints, green sphere markers appear in 'viewpoints' ns."""
        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe_with_viewpoint(
            "kitchen", 16.5, 2.0, 0.5, ["fridge"], "kitchen scene"
        )
        ma = self._make_markers(sg)

        vp_markers = [m for m in ma.markers if m.ns == "viewpoints"]
        assert len(vp_markers) >= 1

    def test_viewpoint_marker_is_green_sphere(self):
        """Viewpoint markers are SPHERE type with green colour."""
        from visualization_msgs.msg import Marker

        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe_with_viewpoint(
            "kitchen", 16.5, 2.0, 0.5, ["fridge"], "kitchen scene"
        )
        ma = self._make_markers(sg)

        vp_markers = [m for m in ma.markers if m.ns == "viewpoints"]
        assert vp_markers, "Expected at least one viewpoint marker"
        m = vp_markers[0]
        assert m.type == Marker.SPHERE
        assert m.color.g > 0.5, "Viewpoint marker should be predominantly green"
        assert m.color.r < 0.1
        assert m.color.b < 0.1

    # ------------------------------------------------------------------
    # T8-3  Object markers
    # ------------------------------------------------------------------

    def test_object_markers_appear(self):
        """After observing objects, orange cube + text markers appear."""
        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe("kitchen", ["fridge", "counter"], "Kitchen scene")
        ma = self._make_markers(sg)

        obj_markers = [m for m in ma.markers if m.ns == "objects"]
        label_markers = [m for m in ma.markers if m.ns == "object_labels"]
        assert len(obj_markers) >= 2
        assert len(label_markers) >= 2

    def test_object_marker_is_orange_cube(self):
        """Object cube markers have orange colour (r=1.0, g~0.55, b=0.0)."""
        from visualization_msgs.msg import Marker

        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe("kitchen", ["fridge"], "Kitchen scene")
        ma = self._make_markers(sg)

        obj_markers = [m for m in ma.markers if m.ns == "objects"]
        assert obj_markers, "Expected at least one object marker"
        m = obj_markers[0]
        assert m.type == Marker.CUBE
        assert m.color.r == pytest.approx(1.0, abs=0.05)
        assert m.color.g == pytest.approx(0.55, abs=0.1)
        assert m.color.b == pytest.approx(0.0, abs=0.05)

    def test_object_label_text_matches_category(self):
        """Object label text equals the detected object category."""
        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe("kitchen", ["fridge"], "Kitchen scene")
        ma = self._make_markers(sg)

        label_markers = [m for m in ma.markers if m.ns == "object_labels"]
        texts = {m.text for m in label_markers}
        assert "fridge" in texts

    # ------------------------------------------------------------------
    # T8-4  Robot arrow marker
    # ------------------------------------------------------------------

    def test_robot_arrow_marker(self):
        """Robot position is shown as a teal ARROW in the 'robot' namespace."""
        from visualization_msgs.msg import Marker

        sg = self._fresh_sg()
        ma = self._make_markers(sg, robot_x=5.0, robot_y=3.0, robot_heading=1.57)

        robot_markers = [m for m in ma.markers if m.ns == "robot"]
        assert len(robot_markers) == 1

        m = robot_markers[0]
        assert m.type == Marker.ARROW
        assert m.pose.position.x == pytest.approx(5.0)
        assert m.pose.position.y == pytest.approx(3.0)
        # Teal: r=0, g>0.5, b>0.5
        assert m.color.r == pytest.approx(0.0, abs=0.05)
        assert m.color.g > 0.5
        assert m.color.b > 0.5

    def test_robot_arrow_heading_encoded_in_quaternion(self):
        """Robot heading is encoded as a quaternion on the arrow marker."""
        sg = self._fresh_sg()
        heading = math.pi / 4  # 45 degrees
        ma = self._make_markers(sg, robot_heading=heading)

        robot_markers = [m for m in ma.markers if m.ns == "robot"]
        assert robot_markers
        m = robot_markers[0]
        expected_z = math.sin(heading / 2)
        expected_w = math.cos(heading / 2)
        assert m.pose.orientation.z == pytest.approx(expected_z, abs=1e-6)
        assert m.pose.orientation.w == pytest.approx(expected_w, abs=1e-6)

    # ------------------------------------------------------------------
    # T8-5  Navigation goal marker
    # ------------------------------------------------------------------

    def test_nav_goal_marker_absent_by_default(self):
        """No nav_goal marker appears when nav_goal is not provided."""
        sg = self._fresh_sg()
        ma = self._make_markers(sg)

        goal_markers = [m for m in ma.markers if m.ns == "nav_goal"]
        assert len(goal_markers) == 0

    def test_nav_goal_marker(self):
        """Navigation goal is shown as a red SPHERE in 'nav_goal' namespace."""
        from visualization_msgs.msg import Marker

        sg = self._fresh_sg()
        ma = self._make_markers(sg, nav_goal=(12.0, 7.5))

        goal_markers = [m for m in ma.markers if m.ns == "nav_goal"]
        assert len(goal_markers) == 1

        m = goal_markers[0]
        assert m.type == Marker.SPHERE
        assert m.pose.position.x == pytest.approx(12.0)
        assert m.pose.position.y == pytest.approx(7.5)
        # Red: r~1, g<0.2, b<0.2
        assert m.color.r == pytest.approx(1.0, abs=0.05)
        assert m.color.g < 0.2
        assert m.color.b < 0.2

    # ------------------------------------------------------------------
    # T8-6  Room label content
    # ------------------------------------------------------------------

    def test_room_label_shows_visit_info(self):
        """Visited room label includes visit count and coverage."""
        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe("kitchen", ["fridge"], "A kitchen")
        ma = self._make_markers(sg)

        labels = [
            m for m in ma.markers
            if m.ns == "room_labels" and "kitchen" in m.text
        ]
        assert labels, "Expected a room_labels marker for kitchen"
        label_text = labels[0].text
        assert "1x" in label_text, f"Expected visit count '1x' in: {label_text!r}"

    def test_unvisited_room_label_is_plain(self):
        """Unvisited room label shows only the room name, no visit stats."""
        sg = self._fresh_sg()
        # Do not visit any room
        ma = self._make_markers(sg)

        labels = [
            m for m in ma.markers
            if m.ns == "room_labels" and m.text.strip() == "hallway"
        ]
        # hallway label should just be "hallway" when never visited
        assert labels, "Expected a plain 'hallway' label marker"

    def test_coverage_affects_room_label(self):
        """Room coverage percentage appears in label after a viewpoint is added."""
        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe_with_viewpoint(
            "kitchen", 16.5, 2.0, 0.0, ["fridge"], "A kitchen"
        )
        ma = self._make_markers(sg)

        labels = [
            m for m in ma.markers
            if m.ns == "room_labels" and "kitchen" in m.text
        ]
        assert labels
        label_text = labels[0].text
        # Coverage is formatted with % (e.g. "20%")
        assert "%" in label_text, f"Expected coverage % in label: {label_text!r}"

    # ------------------------------------------------------------------
    # T8-7  Structural invariants
    # ------------------------------------------------------------------

    def test_marker_frame_is_map(self):
        """All markers have frame_id='map'."""
        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe("kitchen", ["fridge"], "A kitchen")
        ma = self._make_markers(sg)

        for m in ma.markers:
            assert m.header.frame_id == "map", (
                f"Marker ns={m.ns!r} id={m.id} has frame_id={m.header.frame_id!r}"
            )

    def test_all_marker_ids_unique(self):
        """No duplicate (ns, id) pairs in the marker array."""
        sg = self._fresh_sg()
        sg.visit("kitchen", 17.0, 2.5)
        sg.observe("kitchen", ["fridge", "counter", "table"], "A busy kitchen")
        sg.visit("living_room", 3.0, 2.5)
        sg.observe("living_room", ["sofa", "tv"], "A living room")
        ma = self._make_markers(sg, nav_goal=(10.0, 5.0))

        seen = set()
        for m in ma.markers:
            key = (m.ns, m.id)
            assert key not in seen, f"Duplicate marker key: ns={m.ns!r}, id={m.id}"
            seen.add(key)

    def test_marker_count_grows_with_objects(self):
        """Total marker count increases when objects are added to multiple rooms."""
        sg_empty = self._fresh_sg()
        ma_empty = self._make_markers(sg_empty)
        base_count = len(ma_empty.markers)

        sg_with_objs = self._fresh_sg()
        sg_with_objs.visit("kitchen", 17.0, 2.5)
        sg_with_objs.observe("kitchen", ["fridge", "counter"], "Kitchen")
        sg_with_objs.visit("study", 17.0, 7.5)
        sg_with_objs.observe("study", ["desk", "chair", "lamp"], "Study")
        ma_with_objs = self._make_markers(sg_with_objs)

        # 5 new objects -> 5 cube + 5 label + viewpoints = at least 10 extra markers
        assert len(ma_with_objs.markers) >= base_count + 10
