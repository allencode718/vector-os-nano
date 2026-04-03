"""Open-vocabulary object detection for accurate world positioning.

Uses GroundingDINO (via HuggingFace transformers) for detection with
D435 depth for 3D projection. Sim-to-real compatible.

Pipeline:
    RGB frame → GroundingDINO → per-object bbox (u1,v1,u2,v2)
    Depth frame → depth at each bbox center
    Camera intrinsics + robot pose → world (x, y, z) per object

Usage::

    detector = ObjectDetector()  # loads model on first call
    detections = detector.detect(rgb_frame, depth_frame, robot_pose)
    # → [Detection(label="sofa", x=4.2, y=1.8, z=0.3, confidence=0.87), ...]

Requires: torch, transformers (pip install torch transformers)
Falls back gracefully when not installed — detect() returns empty list.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

# Lazy-loaded model (heavy — only load when first called)
_model: Any = None
_processor: Any = None
_model_loaded: bool = False
_model_load_failed: bool = False

# Default detection prompt — common indoor furniture and objects
_DEFAULT_PROMPT: str = (
    "sofa . chair . table . desk . bed . fridge . counter . lamp . "
    "tv . bookshelf . plant . door . wardrobe . nightstand . "
    "bathtub . toilet . sink . stool . dresser . rug"
)

_CONFIDENCE_THRESHOLD: float = 0.15
_MODEL_ID: str = "IDEA-Research/grounding-dino-tiny"


# ---------------------------------------------------------------------------
# Detection result
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Detection:
    """A detected object with world coordinates."""

    label: str
    confidence: float
    # Bounding box in image pixels
    bbox_u1: float
    bbox_v1: float
    bbox_u2: float
    bbox_v2: float
    # World coordinates (from depth projection)
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0
    # Depth at bbox center (metres)
    depth_m: float = 0.0


@dataclass(frozen=True)
class RobotPose:
    """Robot pose for depth-to-world projection."""

    x: float
    y: float
    z: float
    heading: float  # yaw in radians
    cam_xpos: Any = None  # (3,) camera world position from MuJoCo
    cam_xmat: Any = None  # (9,) camera rotation matrix from MuJoCo


# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------


def _ensure_model() -> bool:
    """Lazy-load GroundingDINO model. Returns True if available."""
    global _model, _processor, _model_loaded, _model_load_failed

    if _model_loaded:
        return True
    if _model_load_failed:
        return False

    try:
        import os as _os
        _os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")
        import warnings
        warnings.filterwarnings("ignore", message=".*unauthenticated.*HF Hub.*")

        import torch  # noqa: F401
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

        logger.info("[ObjectDetector] Loading %s...", _MODEL_ID)
        _processor = AutoProcessor.from_pretrained(_MODEL_ID)
        _model = AutoModelForZeroShotObjectDetection.from_pretrained(_MODEL_ID)

        # Move to GPU if available
        if torch.cuda.is_available():
            _model = _model.to("cuda")
            logger.info("[ObjectDetector] Model loaded on CUDA")
        else:
            logger.info("[ObjectDetector] Model loaded on CPU")

        _model_loaded = True
        return True

    except ImportError as exc:
        logger.warning(
            "[ObjectDetector] torch/transformers not installed: %s. "
            "Install with: pip install torch transformers",
            exc,
        )
        _model_load_failed = True
        return False
    except Exception as exc:
        logger.error("[ObjectDetector] Model load failed: %s", exc)
        _model_load_failed = True
        return False


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------


def detect_objects(
    rgb: np.ndarray,
    prompt: str | None = None,
    confidence_threshold: float = _CONFIDENCE_THRESHOLD,
) -> list[dict]:
    """Run GroundingDINO on an RGB frame.

    Args:
        rgb: (H, W, 3) uint8 RGB array.
        prompt: Object categories to detect, separated by " . ".
                Defaults to common indoor furniture.
        confidence_threshold: Minimum confidence to keep.

    Returns:
        List of dicts: [{"label": str, "confidence": float,
                         "bbox": (u1,v1,u2,v2)}]
        Returns empty list if model unavailable.
    """
    if not _ensure_model():
        return []

    import torch
    from PIL import Image

    text = prompt or _DEFAULT_PROMPT
    pil_image = Image.fromarray(rgb)

    inputs = _processor(images=pil_image, text=text, return_tensors="pt")

    if torch.cuda.is_available():
        inputs = {k: v.to("cuda") if hasattr(v, "to") else v for k, v in inputs.items()}

    with torch.no_grad():
        outputs = _model(**inputs)

    results = _processor.post_process_grounded_object_detection(
        outputs,
        inputs["input_ids"],
        threshold=confidence_threshold,
        text_threshold=confidence_threshold,
        target_sizes=[pil_image.size[::-1]],  # (H, W)
    )[0]

    detections = []
    h_img, w_img = pil_image.size[::-1]  # (H, W)
    # Labels that are not physical objects (structural elements / background)
    _IGNORE_LABELS = frozenset({"wall", "floor", "ceiling", "room", "space", "area"})

    labels_key = "text_labels" if "text_labels" in results else "labels"
    for score, label, box in zip(
        results["scores"], results[labels_key], results["boxes"]
    ):
        u1, v1, u2, v2 = box.cpu().tolist()
        clean_label = str(label).strip().split()[0] if label else "object"

        # Skip structural elements
        if clean_label.lower() in _IGNORE_LABELS:
            continue

        # Skip bboxes that cover >60% of the image (likely background)
        bbox_area = (u2 - u1) * (v2 - v1)
        img_area = w_img * h_img
        if bbox_area > 0.6 * img_area:
            continue

        # Skip tiny bboxes (<1% of image — likely noise)
        if bbox_area < 0.01 * img_area:
            continue

        detections.append({
            "label": clean_label,
            "confidence": float(score.cpu()),
            "bbox": (u1, v1, u2, v2),
        })

    return detections


def detect_and_project(
    rgb: np.ndarray,
    depth: np.ndarray,
    pose: RobotPose,
    prompt: str | None = None,
    confidence_threshold: float = _CONFIDENCE_THRESHOLD,
) -> list[Detection]:
    """Detect objects and project each to world coordinates using depth.

    Full pipeline: RGB → GroundingDINO → bbox → depth at center → world coords.

    Args:
        rgb: (H, W, 3) uint8 RGB array.
        depth: (H, W) float32 depth in metres (aligned to RGB).
        pose: Robot pose for world projection.
        prompt: Detection categories (default: indoor furniture).
        confidence_threshold: Minimum confidence.

    Returns:
        List of Detection with world coordinates filled in.
        Returns empty list if model unavailable.
    """
    from vector_os_nano.perception.depth_projection import (
        get_intrinsics,
        depth_to_world,
    )

    raw = detect_objects(rgb, prompt, confidence_threshold)
    if not raw:
        return []

    h, w = depth.shape[:2]
    intrinsics = get_intrinsics(w, h, sim=True)  # TODO: detect sim vs real
    results: list[Detection] = []

    for det in raw:
        u1, v1, u2, v2 = det["bbox"]
        # Bbox center
        cu = (u1 + u2) / 2.0
        cv = (v1 + v2) / 2.0

        # --- Robust depth sampling ---
        # Sample depth inside the bbox (not just the center pixel).
        # Use the inner 60% of the bbox to avoid edge depth artifacts.
        bbox_w = u2 - u1
        bbox_h = v2 - v1
        margin_u = bbox_w * 0.2
        margin_v = bbox_h * 0.2
        inner_u1 = int(max(0, u1 + margin_u))
        inner_v1 = int(max(0, v1 + margin_v))
        inner_u2 = int(min(w, u2 - margin_u))
        inner_v2 = int(min(h, v2 - margin_v))

        if inner_u2 <= inner_u1 or inner_v2 <= inner_v1:
            # Bbox too small for inner sampling, use center pixel
            cu_i, cv_i = int(cu), int(cv)
            if 0 <= cu_i < w and 0 <= cv_i < h:
                d_m = float(depth[cv_i, cu_i])
            else:
                continue
        else:
            patch = depth[inner_v1:inner_v2, inner_u1:inner_u2]
            # D435 reliable range: 0.3m – 3.0m
            valid = patch[(patch > 0.3) & (patch <= 3.0)]
            if len(valid) < 5:
                # Not enough valid depth pixels → unreliable, skip
                continue
            d_m = float(np.median(valid))

        # D435 reliable depth range: 0.3 – 3.0m
        if d_m < 0.3 or d_m > 3.0:
            continue

        # --- Bbox quality checks ---
        # Skip tiny bboxes (< 3% of image area) — too small for accurate depth
        bbox_area = bbox_w * bbox_h
        if bbox_area < 0.03 * (w * h):
            continue

        # Depth variance check: if the depth within the bbox varies wildly,
        # the object straddles a depth boundary → position unreliable
        if inner_u2 > inner_u1 and inner_v2 > inner_v1:
            inner_patch = depth[inner_v1:inner_v2, inner_u1:inner_u2]
            inner_valid = inner_patch[(inner_patch > 0.3) & (inner_patch <= 3.0)]
            if len(inner_valid) > 5:
                depth_std = float(np.std(inner_valid))
                if depth_std > 0.5:
                    # High variance → object at a depth discontinuity, skip
                    continue

        # --- Project to world ---
        from vector_os_nano.perception.depth_projection import pixel_to_camera, camera_to_world
        x_cam, y_cam, z_cam = pixel_to_camera(cu, cv, d_m, intrinsics)
        world_pt = camera_to_world(
            x_cam, y_cam, z_cam,
            pose.x, pose.y, pose.z, pose.heading,
            cam_xpos=pose.cam_xpos, cam_xmat=pose.cam_xmat,
        )
        if world_pt is None:
            continue
        wx, wy, wz = world_pt

        # Sanity: world z should be near ground level (< 2m above floor)
        if wz > 2.5 or wz < -0.5:
            continue

        # --- Deduplicate within this frame ---
        # Same category within 1.0m → keep the one with higher confidence
        duplicate = False
        for i_existing, existing in enumerate(results):
            if existing.label == det["label"]:
                dist = math.sqrt((wx - existing.world_x)**2 + (wy - existing.world_y)**2)
                if dist < 1.0:
                    # Keep the higher confidence one
                    if det["confidence"] > existing.confidence:
                        results[i_existing] = Detection(
                            label=det["label"],
                            confidence=det["confidence"],
                            bbox_u1=u1, bbox_v1=v1,
                            bbox_u2=u2, bbox_v2=v2,
                            world_x=wx, world_y=wy, world_z=wz,
                            depth_m=d_m,
                        )
                    duplicate = True
                    break
        if duplicate:
            continue

        results.append(Detection(
            label=det["label"],
            confidence=det["confidence"],
            bbox_u1=u1, bbox_v1=v1,
            bbox_u2=u2, bbox_v2=v2,
            world_x=wx, world_y=wy, world_z=wz,
            depth_m=d_m,
        ))

    return results
