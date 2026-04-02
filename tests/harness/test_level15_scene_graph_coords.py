"""Level 15 — SceneGraph per-object world coordinates.

Tests that observe_with_viewpoint() accepts an optional
``detected_objects`` parameter carrying per-object world coordinates
from a downstream detector (e.g. GroundingDINO + depth projection).

Test classes
------------
    TestSceneGraphDetectedObjects   L15-0  per-object coords stored correctly
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from vector_os_nano.core.scene_graph import RoomNode, SceneGraph


# ---------------------------------------------------------------------------
# L15-0: per-object world coordinates via detected_objects
# ---------------------------------------------------------------------------


class TestSceneGraphDetectedObjects:
    """L15-0 — observe_with_viewpoint detected_objects integration."""

    def test_observe_with_detected_objects_stores_coords(self) -> None:
        """detected_objects tuples propagate x, y into ObjectNode."""
        sg = SceneGraph()
        sg.add_room(RoomNode(room_id="kitchen", center_x=17.0, center_y=2.5))
        sg.observe_with_viewpoint(
            "kitchen", 15.0, 3.0, 0.0,
            ["fridge", "counter"],
            "kitchen scene",
            detected_objects=[
                ("fridge", 17.5, 4.0),
                ("counter", 18.0, 2.0),
            ],
        )
        objs = sg.find_objects_in_room("kitchen")
        fridge = [o for o in objs if o.category == "fridge"]
        counter = [o for o in objs if o.category == "counter"]
        assert len(fridge) == 1, "fridge not found in kitchen"
        assert abs(fridge[0].x - 17.5) < 0.01, f"fridge.x expected 17.5, got {fridge[0].x}"
        assert abs(fridge[0].y - 4.0) < 0.01, f"fridge.y expected 4.0, got {fridge[0].y}"
        assert len(counter) == 1, "counter not found in kitchen"
        assert abs(counter[0].x - 18.0) < 0.01
        assert abs(counter[0].y - 2.0) < 0.01

    def test_observe_without_detected_objects_backward_compat(self) -> None:
        """Old-style call with plain string list still works; coords default to 0."""
        sg = SceneGraph()
        sg.observe_with_viewpoint("hall", 5.0, 3.0, 0.0, ["chair"], "")
        objs = sg.find_objects_in_room("hall")
        assert len(objs) == 1, f"expected 1 object, got {len(objs)}"
        assert objs[0].category == "chair"
        assert objs[0].x == 0.0, f"expected x=0.0 (no coords given), got {objs[0].x}"
        assert objs[0].y == 0.0, f"expected y=0.0 (no coords given), got {objs[0].y}"

    def test_detected_objects_empty_list_falls_back_to_plain_names(self) -> None:
        """Empty detected_objects falls back to plain string list."""
        sg = SceneGraph()
        sg.observe_with_viewpoint(
            "room", 1.0, 1.0, 0.0, ["a"], "", detected_objects=[],
        )
        objs = sg.find_objects_in_room("room")
        assert len(objs) == 1, f"expected 1 object from plain list, got {len(objs)}"
        assert objs[0].category == "a"

    def test_detected_objects_override_string_list(self) -> None:
        """When detected_objects provided, those replace the plain names."""
        sg = SceneGraph()
        sg.observe_with_viewpoint(
            "room", 1.0, 1.0, 0.0,
            ["a", "b"],               # plain names — should be ignored
            "",
            detected_objects=[("sofa", 3.0, 4.0)],
        )
        objs = sg.find_objects_in_room("room")
        categories = {o.category for o in objs}
        assert "sofa" in categories, f"sofa not found; categories={categories}"
        sofa = [o for o in objs if o.category == "sofa"]
        assert sofa[0].x == 3.0, f"expected sofa.x=3.0, got {sofa[0].x}"
        # Plain names should not appear as extra objects
        assert "a" not in categories, f"'a' should not be present; categories={categories}"
        assert "b" not in categories, f"'b' should not be present; categories={categories}"

    def test_detected_objects_coord_precision(self) -> None:
        """Fractional world coordinates stored without rounding."""
        sg = SceneGraph()
        sg.observe_with_viewpoint(
            "lab", 0.0, 0.0, 0.0,
            [],
            "",
            detected_objects=[("sensor", 1.23456, 7.89012)],
        )
        objs = sg.find_objects_in_room("lab")
        assert len(objs) == 1
        assert abs(objs[0].x - 1.23456) < 1e-5
        assert abs(objs[0].y - 7.89012) < 1e-5

    def test_detected_objects_skipped_viewpoint_stores_coords(self) -> None:
        """When viewpoint is skipped (too close), detected_objects coords still stored."""
        sg = SceneGraph()
        sg.add_room(RoomNode(room_id="office", center_x=5.0, center_y=5.0))
        # First observation creates viewpoint
        sg.observe_with_viewpoint(
            "office", 5.0, 5.0, 0.0,
            ["desk"],
            "office scene",
        )
        # Second observation at very close position — viewpoint skipped
        sg.observe_with_viewpoint(
            "office", 5.1, 5.1, 0.0,
            ["lamp"],
            "",
            detected_objects=[("lamp", 6.0, 7.0)],
        )
        objs = sg.find_objects_in_room("office")
        lamp = [o for o in objs if o.category == "lamp"]
        assert len(lamp) == 1, "lamp not found after skipped viewpoint"
        assert abs(lamp[0].x - 6.0) < 0.01
        assert abs(lamp[0].y - 7.0) < 0.01

    def test_detected_objects_merge_updates_coords(self) -> None:
        """Second observation with higher-confidence detection updates coords."""
        sg = SceneGraph()
        sg.add_room(RoomNode(room_id="living", center_x=0.0, center_y=0.0))
        # First observation — no coords
        sg.observe_with_viewpoint(
            "living", 0.0, 0.0, 0.0,
            ["sofa"],
            "",
        )
        # Second observation at new viewpoint with coords
        sg.observe_with_viewpoint(
            "living", 3.0, 0.0, 0.0,
            [],
            "",
            detected_objects=[("sofa", 5.5, 2.5)],
        )
        objs = sg.find_objects_in_room("living")
        sofa = [o for o in objs if o.category == "sofa"]
        assert len(sofa) == 1, "merge should yield single sofa"
        assert abs(sofa[0].x - 5.5) < 0.01, f"sofa.x expected 5.5, got {sofa[0].x}"
        assert abs(sofa[0].y - 2.5) < 0.01
