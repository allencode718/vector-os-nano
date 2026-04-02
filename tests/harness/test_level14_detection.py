"""Level 14: GroundingDINO object detection + depth projection tests.

Tests the full pipeline:
    RGB → GroundingDINO → per-object bbox → depth at bbox center → world coords

Requires torch + transformers. Skipped if not installed.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Skip if torch/transformers not available
try:
    import torch
    import transformers
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


# ---------------------------------------------------------------------------
# Tests that don't need GPU (mock-based)
# ---------------------------------------------------------------------------


class TestDetectionDataclasses:
    """Test Detection and RobotPose dataclasses."""

    def test_detection_fields(self):
        from vector_os_nano.perception.object_detector import Detection
        d = Detection(
            label="sofa", confidence=0.9,
            bbox_u1=10, bbox_v1=20, bbox_u2=100, bbox_v2=80,
            world_x=3.0, world_y=2.0, world_z=0.3, depth_m=2.5,
        )
        assert d.label == "sofa"
        assert d.confidence == 0.9
        assert d.world_x == 3.0
        assert d.depth_m == 2.5

    def test_detection_defaults(self):
        from vector_os_nano.perception.object_detector import Detection
        d = Detection(
            label="chair", confidence=0.5,
            bbox_u1=0, bbox_v1=0, bbox_u2=50, bbox_v2=50,
        )
        assert d.world_x == 0.0
        assert d.depth_m == 0.0

    def test_robot_pose(self):
        from vector_os_nano.perception.object_detector import RobotPose
        p = RobotPose(x=5.0, y=3.0, z=0.28, heading=1.2)
        assert p.x == 5.0
        assert p.heading == 1.2

    def test_detection_frozen(self):
        from vector_os_nano.perception.object_detector import Detection
        d = Detection(label="x", confidence=0.5,
                      bbox_u1=0, bbox_v1=0, bbox_u2=1, bbox_v2=1)
        with pytest.raises(AttributeError):
            d.label = "y"


class TestDetectAndProject:
    """Test detect_and_project with mock detector output."""

    def test_empty_when_no_model(self):
        """detect_and_project returns empty list if model not loaded."""
        from vector_os_nano.perception import object_detector as od
        # Reset model state
        old_loaded = od._model_loaded
        old_failed = od._model_load_failed
        od._model_loaded = False
        od._model_load_failed = True
        try:
            from vector_os_nano.perception.object_detector import (
                RobotPose, detect_and_project,
            )
            rgb = np.zeros((240, 320, 3), dtype=np.uint8)
            depth = np.full((240, 320), 2.0, dtype=np.float32)
            pose = RobotPose(x=0, y=0, z=0.28, heading=0)
            result = detect_and_project(rgb, depth, pose)
            assert result == []
        finally:
            od._model_loaded = old_loaded
            od._model_load_failed = old_failed

    def test_default_prompt_is_nonempty(self):
        from vector_os_nano.perception.object_detector import _DEFAULT_PROMPT
        assert len(_DEFAULT_PROMPT) > 10
        assert "sofa" in _DEFAULT_PROMPT


# ---------------------------------------------------------------------------
# Tests that need GPU (real model inference)
# ---------------------------------------------------------------------------


@pytest.mark.skipif(not HAS_TORCH, reason="torch not installed")
class TestGroundingDINOInference:
    """Test actual GroundingDINO model inference."""

    def test_detect_objects_returns_list(self):
        from vector_os_nano.perception.object_detector import detect_objects
        img = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        result = detect_objects(img, confidence_threshold=0.1)
        assert isinstance(result, list)

    def test_detection_has_correct_keys(self):
        from vector_os_nano.perception.object_detector import detect_objects
        img = np.full((240, 320, 3), 180, dtype=np.uint8)
        img[80:160, 60:260] = [50, 30, 20]  # dark rectangle
        dets = detect_objects(img, prompt="table . furniture", confidence_threshold=0.1)
        if dets:  # may or may not detect in synthetic image
            d = dets[0]
            assert "label" in d
            assert "confidence" in d
            assert "bbox" in d
            assert len(d["bbox"]) == 4

    def test_bbox_within_image_bounds(self):
        from vector_os_nano.perception.object_detector import detect_objects
        img = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        dets = detect_objects(img, confidence_threshold=0.05)
        for d in dets:
            u1, v1, u2, v2 = d["bbox"]
            assert u1 >= -1 and u2 <= 321  # allow small float rounding
            assert v1 >= -1 and v2 <= 241

    def test_confidence_in_range(self):
        from vector_os_nano.perception.object_detector import detect_objects
        img = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        dets = detect_objects(img, confidence_threshold=0.05)
        for d in dets:
            assert 0.0 <= d["confidence"] <= 1.0

    def test_model_loads_on_gpu(self):
        from vector_os_nano.perception.object_detector import _model
        if torch.cuda.is_available() and _model is not None:
            assert next(_model.parameters()).is_cuda

    def test_detect_and_project_with_depth(self):
        """Full pipeline: detect + depth → world coordinates."""
        from vector_os_nano.perception.object_detector import (
            RobotPose, detect_and_project,
        )
        rgb = np.full((240, 320, 3), 180, dtype=np.uint8)
        rgb[80:160, 100:220] = [40, 30, 25]
        depth = np.full((240, 320), 2.5, dtype=np.float32)
        pose = RobotPose(x=5.0, y=3.0, z=0.28, heading=0.0)

        dets = detect_and_project(rgb, depth, pose, confidence_threshold=0.1)
        # May or may not detect — but should not crash
        assert isinstance(dets, list)
        for d in dets:
            assert hasattr(d, "world_x")
            assert hasattr(d, "depth_m")
            if d.depth_m > 0:
                # World x should be ahead of robot (heading=0 → +X)
                assert d.world_x > 5.0
