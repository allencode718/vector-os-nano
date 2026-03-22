# Vector OS Nano SDK — Progress

**Last updated:** 2026-03-22 13:30 UTC  
**Status:** v0.1.0 complete — full NL pick pipeline working, TUI dashboard mature, unified launcher deployed

## What Works

- Full NL pipeline: "抓电池" → LLM → scan → detect(VLM) → track(EdgeTAM) → 3D → calibrate → IK → pick → drop
- Direct commands without LLM: home, scan, open, close (instant, no API call)
- Chinese + English natural language
- Live camera viewer: RGB + depth side-by-side, EdgeTAM tracking overlay
- 696 unit tests passing (100%)
- ROS2 integration layer (optional, 5 nodes + launch file)
- Textual TUI dashboard (5 tabs, real-time status, camera preview, joint visualization)
- PyBullet simulation
- SO-101 arm driver (Feetech STS3215 serial)
- Calibration wizard (TUI + readline)

## Current Release: v0.1.0

### Unified Launcher: run.py

Single entry point with three modes:

1. **CLI Mode** (default): `python run.py`
   - Interactive readline shell
   - Natural language commands
   - Fast startup, headless-friendly

2. **Dashboard Mode**: `python run.py --dashboard` or `python run.py -d`
   - Rich TUI with 5 tabs (Dashboard, Log, Skills, World, Camera)
   - Real-time visualization (joint angles, status, camera feed)
   - F1-F5 tab switching, F6 fullscreen camera
   - Production-grade monitoring interface

3. **Testing Modes**: `--no-arm`, `--no-perception`
   - Full simulation without hardware
   - Perception testing without arm
   - Control testing without camera

## Test Coverage

| Category | Count | Notes |
|----------|-------|-------|
| Unit tests | 696 | 100% passing, 85%+ coverage |
| Integration tests | 42 | Mock hardware, ROS2 stack |
| System tests | 5 | Full pipeline validation |
| Manual tests | 3 | Hardware, perception, dashboard |

## Core Features (Architecture)

| Layer | Component | Status |
|-------|-----------|--------|
| LLM | Claude Haiku via OpenRouter | Working |
| Planning | Task decomposition + executor | Working |
| Perception | VLM + EdgeTAM + RealSense D405 | Working |
| Control | Pinocchio FK/IK solver | Working |
| Hardware | SO-101 arm + gripper | Working |
| Skills | 6 built-ins (pick, place, home, scan, detect, track) | Working |
| CLI | Readline shell + TUI dashboard | Working |
| ROS2 | 5 nodes + launch file (optional) | Working |

## Known Limitations

### Pick Accuracy
- Calibration valid only at home/scan pose (pose-dependent)
- Z-row collapsed (all objects at z=0.005m)
- Gripper asymmetry requires manual offsets per robot
- No look-then-move correction

### Perception
- VLM detection depends on lighting
- EdgeTAM tracking can lose objects in occlusion
- Camera serial hardcoded (TODO: parameterize)

### LLM
- Haiku sometimes over-plans (scan→detect even when just told to pick)
- No multi-turn conversation memory (reset after each command)

## TODO (Next Priorities)

### 1. MuJoCo Simulation
- Replace/supplement PyBullet with MuJoCo for higher fidelity physics
- MuJoCo has better contact modeling, faster simulation, native Python bindings
- Goal: full pick pipeline testable in simulation without real hardware
- Load SO-101 URDF into MuJoCo, implement `SimulatedArm(ArmProtocol)` + `SimulatedCamera`
- Enable CI/CD testing of the full pick pipeline

### 2. LLM Agent Brain Upgrade
- **Skill Manifest Protocol (ADR-002)**: YAML skill registry with aliases, auto_steps, rich descriptions
- **Better task decomposition**: handle complex multi-step instructions ("sort by color", "stack blocks")
- **Generalization**: handle novel objects, spatial reasoning ("put it next to the blue one")
- **Multi-turn memory**: conversation context across commands
- **Model upgrade path**: Sonnet for complex planning, Haiku for simple commands (auto-select)
- See `docs/ADR-002-skill-manifest-protocol.md`

### 3. Pick Accuracy
- Re-calibration with more points + Z variation
- Hand-eye calibration for pose-independent transforms
- Grasp success detection via servo current/load feedback

### 4. Merge & Release
- Merge feat/vector-os-nano-python-sdk → main
- Tag v0.1.0 release
- PyPI publish

## Deployment

**Python SDK:** Installable via `pip install vector_os_nano[all]`

**ROS2 Integration:** Optional. Nodes available if `rclpy` installed.

**Tested On:**
- Ubuntu 22.04 LTS
- Python 3.10+
- PyTorch 2.x (nightly for RTX 5080)
- ROS2 Humble (optional)

**Key Dependencies:**
- Moondream2 VLM (~1.8GB model download)
- EdgeTAM tracker (GPU acceleration)
- Pinocchio IK solver
- Textual TUI framework
