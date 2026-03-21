# Vector OS Nano SDK — Progress

**Last updated:** 2026-03-21  
**Status:** v0.1.0 functional, pick pipeline working, in active tuning

## What Works

- Full NL pipeline: "抓电池" → LLM → scan → detect(VLM) → track(EdgeTAM) → 3D → calibrate → IK → pick → drop
- Direct commands without LLM: home, scan, open, close (instant, no API call)
- Chinese + English natural language
- Live camera viewer: RGB + depth side-by-side, EdgeTAM tracking overlay
- 696 unit tests passing
- ROS2 integration layer (optional, 5 nodes + launch file)
- Textual TUI dashboard
- PyBullet simulation
- SO-101 arm driver (Feetech STS3215 serial)
- Calibration wizard (TUI + readline)

## Current Limitations

### Pick Accuracy
- Empirical XY offsets tuned for specific workspace region
- Calibration matrix Z-row collapsed (all objects at Z=0.005m)
- Gripper asymmetry compensation is position-dependent (left/right/center)
- No look-then-move correction yet (calibration is pose-dependent)
- URDF model doesn't perfectly match real arm (3D-printed, servo backlash)

### Perception
- VLM detection depends on lighting conditions
- EdgeTAM tracking can lose objects if they move fast or get occluded
- Camera serial number hardcoded (335122270413)

### LLM
- Haiku sometimes over-plans (scan→detect even when just told to pick)
- Conversation context reset after each command (no multi-turn memory)

### Architecture
- Calibration only valid at home/scan pose (eye-in-hand, pose-dependent)
- World model cleared after each pick (conservative but loses history)
- No grasp success detection (servo current feedback not implemented)

## Tuning History

| Parameter | Value | Notes |
|-----------|-------|-------|
| z_offset | 10cm | Gripper link to table surface |
| pre_grasp_height | 6cm | Above grasp target |
| X offset | +2cm | Uniform forward compensation |
| Y left | +3cm + 50% proportional | Gripper asymmetry |
| Y right | +1cm | Gripper asymmetry |
| Y center | +2cm | Gripper asymmetry |

## Next Steps

1. Skill Manifest Protocol (ADR-002) — alias-based command routing
2. Re-calibration with more points + Z variation
3. Hand-eye calibration for pose-independent transforms
4. Grasp success detection via servo current/load
5. Multi-object pick-and-place sequences
