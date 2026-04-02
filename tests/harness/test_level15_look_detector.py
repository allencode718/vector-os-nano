"""Level 15: LookSkill + GroundingDINO detector integration tests.

Tests that LookSkill correctly uses the GroundingDINO detector service
for per-object world positioning while delegating room identification
and scene summary to VLM.

All tests use mock detector and mock VLM — no GPU required.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from vector_os_nano.core.scene_graph import SceneGraph, ObjectNode
from vector_os_nano.core.skill import SkillContext
from vector_os_nano.core.types import SkillResult
from vector_os_nano.perception.object_detector import Detection, RobotPose
from vector_os_nano.skills.go2.look import LookSkill


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_vlm(room: str = "living_room", confidence: float = 0.9) -> MagicMock:
    """Return a mock VLM with predictable scene + room identification."""
    vlm = MagicMock()

    scene = MagicMock()
    scene.summary = "A bright living room"
    scene.details = "Walls painted white, natural lighting"
    obj1 = MagicMock()
    obj1.name = "sofa"
    obj1.description = "Grey sofa"
    obj1.confidence = 0.88
    obj2 = MagicMock()
    obj2.name = "table"
    obj2.description = "Wooden coffee table"
    obj2.confidence = 0.75
    scene.objects = [obj1, obj2]
    vlm.describe_scene.return_value = scene

    room_id = MagicMock()
    room_id.room = room
    room_id.confidence = confidence
    room_id.reasoning = "I can see typical living room furniture"
    vlm.identify_room.return_value = room_id

    return vlm


def _make_detections() -> list[Detection]:
    """Return two mock Detection objects with world coordinates."""
    return [
        Detection(
            label="sofa",
            confidence=0.91,
            bbox_u1=50.0, bbox_v1=80.0,
            bbox_u2=200.0, bbox_v2=160.0,
            world_x=4.2, world_y=1.8, world_z=0.3,
            depth_m=2.5,
        ),
        Detection(
            label="table",
            confidence=0.83,
            bbox_u1=120.0, bbox_v1=100.0,
            bbox_u2=220.0, bbox_v2=140.0,
            world_x=3.7, world_y=2.1, world_z=0.1,
            depth_m=1.9,
        ),
    ]


def _make_base(
    position: tuple[float, float, float] = (1.0, 2.0, 0.28),
    heading: float = 0.0,
    depth_frame: np.ndarray | None = None,
) -> MagicMock:
    """Return a mock base with camera and depth frames."""
    base = MagicMock()
    base.get_position.return_value = position
    base.get_heading.return_value = heading
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    base.get_camera_frame.return_value = frame

    if depth_frame is None:
        depth_frame = np.full((240, 320), 2.0, dtype=np.float32)
    base.get_depth_frame.return_value = depth_frame

    return base


def _make_context(
    vlm: Any = None,
    detector: Any = None,
    spatial_memory: Any = None,
    base: Any = None,
) -> SkillContext:
    """Build a SkillContext with the given services."""
    services: dict[str, Any] = {}
    if vlm is not None:
        services["vlm"] = vlm
    if detector is not None:
        services["detector"] = detector
    if spatial_memory is not None:
        services["spatial_memory"] = spatial_memory

    mock_base = base if base is not None else _make_base()

    return SkillContext(
        base=mock_base,
        services=services,
    )


def _make_base_no_depth(
    position: tuple[float, float, float] = (0.0, 0.0, 0.28),
    heading: float = 0.0,
) -> MagicMock:
    """Return a mock base without get_depth_frame attribute."""
    spec_attrs = ["get_position", "get_heading", "get_camera_frame"]
    base = MagicMock(spec=spec_attrs)
    base.get_position.return_value = position
    base.get_heading.return_value = heading
    base.get_camera_frame.return_value = np.zeros((240, 320, 3), dtype=np.uint8)
    return base


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------


class TestLookSkillWithDetector:
    """Core integration: LookSkill calls detector when it is available."""

    def test_look_uses_detector_when_available(self):
        """Detector is called when context.services has 'detector' key."""
        vlm = _make_vlm()
        detections = _make_detections()
        detector_fn = MagicMock(return_value=detections)
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        result = LookSkill().execute({}, ctx)

        assert result.success, f"LookSkill failed: {result.error_message}"
        # detector was called exactly once
        detector_fn.assert_called_once()

    def test_look_falls_back_without_detector(self):
        """No detector in services -> VLM object names used, no world coords."""
        vlm = _make_vlm()
        base = _make_base()
        ctx = _make_context(vlm=vlm, base=base)  # no detector
        result = LookSkill().execute({}, ctx)

        assert result.success
        objects = result.result_data["objects"]
        # VLM returned 2 objects
        assert len(objects) == 2
        # No world_x key since detector not available
        for obj in objects:
            assert "world_x" not in obj

    def test_objects_have_world_coords_in_result(self):
        """result_data['objects'] entries contain world_x, world_y, depth_m."""
        vlm = _make_vlm()
        detections = _make_detections()
        detector_fn = MagicMock(return_value=detections)
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        result = LookSkill().execute({}, ctx)

        assert result.success
        objects = result.result_data["objects"]
        assert len(objects) == 2, f"Expected 2 detector objects, got {len(objects)}"

        for obj in objects:
            assert "world_x" in obj, "Missing world_x in object"
            assert "world_y" in obj, "Missing world_y in object"
            assert "depth_m" in obj, "Missing depth_m in object"
            assert isinstance(obj["world_x"], float)
            assert isinstance(obj["world_y"], float)

        # Verify actual values match detections
        labels = {o["name"]: o for o in objects}
        assert labels["sofa"]["world_x"] == pytest.approx(4.2, abs=1e-3)
        assert labels["sofa"]["world_y"] == pytest.approx(1.8, abs=1e-3)
        assert labels["table"]["world_x"] == pytest.approx(3.7, abs=1e-3)
        assert labels["table"]["depth_m"] == pytest.approx(1.9, abs=1e-3)

    def test_scene_graph_objects_get_detector_coords(self):
        """After look with detector, ObjectNode.x/y are non-zero (detector coords)."""
        vlm = _make_vlm(room="kitchen")
        detections = _make_detections()
        detector_fn = MagicMock(return_value=detections)
        base = _make_base(position=(2.0, 3.0, 0.28), heading=0.5)
        scene_graph = SceneGraph()

        ctx = _make_context(vlm=vlm, detector=detector_fn,
                            spatial_memory=scene_graph, base=base)
        result = LookSkill().execute({}, ctx)

        assert result.success

        # Objects should be stored with world coordinates from detector
        all_objects = scene_graph.find_objects_in_room("kitchen")
        assert len(all_objects) >= 1, "No objects stored in scene graph"

        # At least one object should have non-zero world coordinates
        objects_with_coords = [
            o for o in all_objects
            if o.x != 0.0 or o.y != 0.0
        ]
        assert len(objects_with_coords) >= 1, (
            "No objects have non-zero world coordinates — "
            "detector coords not being passed to merge_object"
        )

    def test_vlm_still_provides_room_id(self):
        """Room identification comes from VLM identify_room, not from detector."""
        vlm = _make_vlm(room="bedroom", confidence=0.95)
        detections = _make_detections()
        detector_fn = MagicMock(return_value=detections)
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        result = LookSkill().execute({}, ctx)

        assert result.success
        assert result.result_data["room"] == "bedroom"
        assert result.result_data["room_confidence"] == pytest.approx(0.95, abs=1e-3)
        # VLM identify_room must have been called
        vlm.identify_room.assert_called_once()
        # VLM describe_scene must also have been called (for summary)
        vlm.describe_scene.assert_called_once()


class TestLookSkillDetectorEdgeCases:
    """Edge cases: failures, empty detections, missing depth."""

    def test_detector_exception_falls_back_to_vlm(self):
        """If detector raises, LookSkill logs warning and uses VLM objects."""
        vlm = _make_vlm()
        detector_fn = MagicMock(side_effect=RuntimeError("GPU OOM"))
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        result = LookSkill().execute({}, ctx)

        # Must not fail — graceful fallback
        assert result.success
        # VLM-based objects returned instead
        objects = result.result_data["objects"]
        assert len(objects) == 2  # from VLM mock
        # No world_x since detector failed
        for obj in objects:
            assert "world_x" not in obj

    def test_empty_detector_returns_falls_back_to_vlm_objects(self):
        """Detector returns [] -> result_data uses VLM object names."""
        vlm = _make_vlm()
        detector_fn = MagicMock(return_value=[])
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        result = LookSkill().execute({}, ctx)

        assert result.success
        objects = result.result_data["objects"]
        # Fall back to VLM objects
        names = {o["name"] for o in objects}
        assert "sofa" in names or "table" in names, (
            "Expected VLM object names when detector returns empty"
        )

    def test_detector_called_with_rgb_and_depth(self):
        """Verify detector is called with correct rgb frame, depth frame, and pose."""
        vlm = _make_vlm()
        detections = _make_detections()
        detector_fn = MagicMock(return_value=detections)

        rgb_frame = np.ones((240, 320, 3), dtype=np.uint8) * 128
        depth_frame = np.full((240, 320), 3.0, dtype=np.float32)
        base = _make_base(
            position=(5.0, 6.0, 0.28),
            heading=1.57,
            depth_frame=depth_frame,
        )
        base.get_camera_frame.return_value = rgb_frame

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        LookSkill().execute({}, ctx)

        detector_fn.assert_called_once()
        call_args = detector_fn.call_args
        called_rgb, called_depth, called_pose = call_args[0]

        assert np.array_equal(called_rgb, rgb_frame), "Wrong RGB frame passed"
        assert np.array_equal(called_depth, depth_frame), "Wrong depth frame passed"
        assert isinstance(called_pose, RobotPose)
        assert called_pose.x == pytest.approx(5.0)
        assert called_pose.y == pytest.approx(6.0)
        assert called_pose.heading == pytest.approx(1.57)

    def test_no_base_depth_frame_skips_detector(self):
        """Base without get_depth_frame -> detector not called, no crash."""
        vlm = _make_vlm()
        detector_fn = MagicMock(return_value=_make_detections())

        # Use spec-based mock so that get_depth_frame is not auto-created
        base = _make_base_no_depth()

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        result = LookSkill().execute({}, ctx)

        assert result.success
        detector_fn.assert_not_called()

    def test_result_summary_still_from_vlm(self):
        """scene summary in result_data always comes from VLM describe_scene."""
        vlm = _make_vlm()
        detector_fn = MagicMock(return_value=_make_detections())
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn, base=base)
        result = LookSkill().execute({}, ctx)

        assert result.success
        assert result.result_data["summary"] == "A bright living room"


class TestSceneGraphDetectorIntegration:
    """Tests verifying SceneGraph records detector world coordinates."""

    def test_scene_graph_receives_world_coords_from_detector(self):
        """merge_object is called with x/y from detector, not zero."""
        vlm = _make_vlm(room="hallway")
        detections = [
            Detection(
                label="chair",
                confidence=0.87,
                bbox_u1=10, bbox_v1=10, bbox_u2=100, bbox_v2=100,
                world_x=7.5, world_y=4.2, world_z=0.2,
                depth_m=3.0,
            )
        ]
        detector_fn = MagicMock(return_value=detections)
        scene_graph = SceneGraph()
        base = _make_base(position=(6.0, 4.0, 0.28), heading=0.0)

        ctx = _make_context(vlm=vlm, detector=detector_fn,
                            spatial_memory=scene_graph, base=base)
        LookSkill().execute({}, ctx)

        chairs = scene_graph.find_objects_by_category("chair")
        assert len(chairs) >= 1, "chair not stored in scene graph"
        chair = chairs[0]
        assert chair.x == pytest.approx(7.5, abs=1e-3), (
            f"Expected x=7.5 from detector, got {chair.x}"
        )
        assert chair.y == pytest.approx(4.2, abs=1e-3), (
            f"Expected y=4.2 from detector, got {chair.y}"
        )

    def test_multiple_detector_objects_stored_separately(self):
        """Each detection becomes a distinct ObjectNode in SceneGraph."""
        vlm = _make_vlm(room="living_room")
        detections = _make_detections()  # sofa + table
        detector_fn = MagicMock(return_value=detections)
        scene_graph = SceneGraph()
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn,
                            spatial_memory=scene_graph, base=base)
        LookSkill().execute({}, ctx)

        objects = scene_graph.find_objects_in_room("living_room")
        categories = {o.category.lower() for o in objects}
        assert "sofa" in categories, f"sofa missing; got {categories}"
        assert "table" in categories, f"table missing; got {categories}"

    def test_detector_object_confidence_stored(self):
        """ObjectNode confidence comes from detector, not VLM."""
        vlm = _make_vlm(room="study")
        detections = [
            Detection(
                label="desk",
                confidence=0.94,
                bbox_u1=0, bbox_v1=0, bbox_u2=50, bbox_v2=50,
                world_x=8.0, world_y=5.0, world_z=0.4,
                depth_m=2.0,
            )
        ]
        detector_fn = MagicMock(return_value=detections)
        scene_graph = SceneGraph()
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn,
                            spatial_memory=scene_graph, base=base)
        LookSkill().execute({}, ctx)

        desks = scene_graph.find_objects_by_category("desk")
        assert desks, "desk not in scene graph"
        assert desks[0].confidence == pytest.approx(0.94, abs=0.01)

    def test_spatial_memory_compat_api_works_with_detector(self):
        """SpatialMemory (non-SceneGraph) still works when detector is active."""
        from vector_os_nano.core.spatial_memory import SpatialMemory

        vlm = _make_vlm(room="bathroom")
        detections = _make_detections()
        detector_fn = MagicMock(return_value=detections)
        # Use old SpatialMemory (no merge_object, no observe_with_viewpoint)
        spatial_mem = SpatialMemory(persist_path=None)
        base = _make_base()

        ctx = _make_context(vlm=vlm, detector=detector_fn,
                            spatial_memory=spatial_mem, base=base)
        result = LookSkill().execute({}, ctx)

        # Should not crash even if spatial_memory lacks SceneGraph methods
        assert result.success
        # Objects should still be in result_data from detector
        objects = result.result_data["objects"]
        assert len(objects) == 2
