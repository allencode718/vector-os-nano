# Vector OS Nano — Architecture Guide

## Repository Structure

```
~/Desktop/vector_os/              # Project root (git repo)
├── vector_os/                    # Python source package (THE CODE)
│   ├── core/                     # Agent engine (pure Python)
│   │   ├── agent.py              # Main entry point — Agent class
│   │   ├── executor.py           # Deterministic task executor
│   │   ├── world_model.py        # Object/robot state tracking
│   │   ├── skill.py              # Skill protocol + registry
│   │   ├── types.py              # Shared data types (frozen dataclasses)
│   │   └── config.py             # YAML config loading
│   │
│   ├── llm/                      # LLM providers (pure Python)
│   │   ├── claude.py             # OpenRouter/Anthropic API
│   │   ├── openai_compat.py      # OpenAI-compatible APIs
│   │   ├── local.py              # Ollama local models
│   │   └── prompts.py            # Planning system prompt
│   │
│   ├── perception/               # Camera + VLM + tracking (GPU)
│   │   ├── realsense.py          # Intel RealSense D405 driver
│   │   ├── vlm.py                # Moondream VLM detector
│   │   ├── tracker.py            # EdgeTAM video segmentation
│   │   ├── pointcloud.py         # RGBD → 3D (numpy vectorized)
│   │   ├── pipeline.py           # Orchestrator + background tracking
│   │   └── calibration.py        # Camera-to-arm transform
│   │
│   ├── hardware/                 # Arm drivers (pure Python)
│   │   ├── so101/                # SO-101 implementation
│   │   │   ├── arm.py            # SO101Arm (ArmProtocol)
│   │   │   ├── gripper.py        # SO101Gripper (GripperProtocol)
│   │   │   ├── joint_config.py   # Encoder↔radian mapping
│   │   │   ├── serial_bus.py     # SCS protocol (scservo_sdk)
│   │   │   └── ik_solver.py      # Pinocchio FK/IK
│   │   ├── sim/                  # PyBullet simulation
│   │   └── urdf/                 # Robot model files
│   │
│   ├── skills/                   # Built-in skills (pure Python)
│   │   ├── pick.py               # Pick skill (full pipeline)
│   │   ├── place.py              # Place skill
│   │   ├── home.py               # Home position
│   │   ├── scan.py               # Scan position
│   │   ├── detect.py             # VLM detection + 3D
│   │   └── calibration.py        # Calibration transform helpers
│   │
│   ├── cli/                      # User interfaces
│   │   ├── simple.py             # readline CLI
│   │   ├── dashboard.py          # Textual TUI dashboard
│   │   └── calibration_wizard.py # Interactive calibration
│   │
│   └── ros2/                     # Optional ROS2 layer
│       ├── nodes/                # ROS2 node wrappers
│       └── launch/               # Launch files
│
├── tests/                        # Test suite (696 tests)
│   ├── unit/                     # No hardware needed
│   └── integration/              # Mock hardware
│
├── config/                       # Configuration
│   ├── default.yaml              # Default settings
│   ├── user.yaml                 # User overrides (gitignored)
│   └── workspace_calibration.yaml # Calibration data (gitignored)
│
├── docs/                         # Documentation
│   ├── architecture.md           # This file
│   ├── ADR-001-core-design.md    # Architecture decision records
│   ├── ADR-002-skill-manifest.md # Skill protocol design
│   ├── dependencies.md           # Third-party packages
│   └── testing-guide.md          # Test suite documentation
│
├── examples/                     # Usage examples
├── .sdd/                         # SDD planning documents
├── agents/devlog/                # Progress tracking
├── run.py                        # Full system launcher
├── pyproject.toml                # Package definition
├── README.md                     # Quick start guide
└── LICENSE                       # MIT
```

**Note:** `vector_os_nano/` in the project root is a Python virtual environment (venv), NOT source code. It is gitignored and not part of the repository.

---

## Data Flow

### User Command → Hardware Motion

```
User: "抓电池"
  ↓
Agent.execute(command)
  ↓
LLM Planner (Claude Haiku via OpenRouter)
  → Returns: TaskPlan([scan, detect("电池"), pick("电池")])
  ↓
TaskExecutor
  ├─→ ScanSkill
  │   └─→ arm.move_joints(scan_pose)
  │
  ├─→ DetectSkill
  │   ├─→ VLM.detect("电池")
  │   │   └─→ 2D bbox (pixel coordinates)
  │   ├─→ EdgeTAM.init_track(bbox)
  │   │   └─→ pixel mask
  │   ├─→ RGBD + mask → Pointcloud.filter()
  │   │   └─→ 3D centroid [x, y, z]_cam
  │   ├─→ Calibration.transform_to_base()
  │   │   └─→ [x, y, z]_base
  │   └─→ WorldModel.add_object("电池", [x, y, z]_base)
  │
  └─→ PickSkill
      ├─→ VLM.detect("电池") [always fresh]
      ├─→ 20-frame density clustering
      ├─→ Calibration.apply_offsets()
      │   ├─→ Z-row: all objects at z = 0.005m
      │   ├─→ X offset: +2cm uniform
      │   └─→ Y offset: ±1-3cm (left/right/center asymmetry)
      ├─→ IKSolver.solve_pick(target_pose)
      │   └─→ joint angles (max 5 iterations)
      ├─→ Motion sequence:
      │   ├─→ move_joints(pre_grasp_pose) [6cm above]
      │   ├─→ move_linear(descend to grasp)
      │   ├─→ gripper.grip()
      │   ├─→ move_linear(lift)
      │   ├─→ move_joints(home_pose)
      │   └─→ gripper.open() [drop]
      └─→ WorldModel.clear() [forget all objects]

Direct Commands (bypass LLM):
  home() → move_joints(home_pose) [0 ms]
  scan() → move_joints(scan_pose) [0 ms]
  open() → gripper.open() [0 ms]
  close() → gripper.close() [0 ms]
```

---

## Key Modules

### core/

**agent.py**
- `Agent` class: main entry point
- Methods: `execute(command)`, `execute_skill(name, **kwargs)`
- Integrates: LLM planning, executor, world model, CLI

**executor.py**
- `TaskExecutor` class: deterministic task sequencing
- Runs `TaskPlan` (list of skills) with dependency tracking
- Error handling: stops on first failure, logs traceback

**world_model.py**
- `WorldModel` class: object + robot state
- Methods: `add_object(label, pose)`, `get_object(label)`, `clear()`
- Immutable updates (frozen dataclasses)

**skill.py**
- `SkillProtocol` (abstract): `execute(executor, **kwargs) → Result`
- `SkillRegistry`: global registry of available skills
- Result type: `Union[Dict, None]` (success) or raises exception (failure)

**types.py**
- Frozen dataclasses: `Pose3D`, `BoundingBox`, `PointCloud3D`, `ObjectInstance`
- Config dataclass: `PerceptionConfig`, `HardwareConfig`, `LLMConfig`

**config.py**
- `load_config(path)` → parsed YAML + environment variable expansion
- Validates required keys, provides defaults

### hardware/

**so101/arm.py**
- `SO101Arm(ArmProtocol)`: 6-DOF arm driver
- Methods: `move_joints(angles)`, `move_pose(pose)`, `get_joint_angles()`, `shutdown()`
- Uses: `Feetech STS3215` servos via serial bus

**so101/gripper.py**
- `SO101Gripper(GripperProtocol)`: parallel jaw gripper
- Methods: `open()`, `close()`, `grip(force)`
- Servo ID: 8 (standard in SO-101)

**so101/ik_solver.py**
- `IKSolver` class: Pinocchio-based FK/IK
- Methods: `forward_kinematics(angles)`, `inverse_kinematics(target_pose)`, `get_jacobian(angles)`
- Max 5 IK iterations, convergence threshold 5mm

**so101/joint_config.py**
- Joint encoder ↔ radian mapping
- Offsets, ranges, inversion flags per joint

**sim/pybullet_arm.py**
- `SimulatedArm`, `SimulatedGripper`: PyBullet physics
- 1:1 API compatible with real hardware drivers

### perception/

**realsense.py**
- `RealSenseCamera` class: Intel D405 RGB-D camera
- Methods: `get_rgb()`, `get_depth()`, `get_intrinsics()`
- Camera serial: `335122270413` (hardcoded, TODO: parameterize)

**vlm.py**
- `VLMDetector` class: Moondream visual language model
- Method: `detect(image, label)` → `BoundingBox` (pixel coordinates)
- Runs on GPU, batches detections for efficiency

**tracker.py**
- `EdgeTAMTracker` class: video segmentation
- Methods: `init_track(bbox)`, `track(frame)` → `mask` (binary array)
- Background thread for continuous tracking (decoupled from detection)

**pointcloud.py**
- `PointCloud3D` class: RGBD → 3D points
- Methods: `from_rgbd(rgb, depth, intrinsics)`, `filter_by_mask(mask)`, `centroid()`
- Vectorized (numpy), no loops

**pipeline.py**
- `PerceptionPipeline` class: orchestrator
- Manages: RealSense, VLM, EdgeTAM, PointCloud
- Runs background tracking thread (independent of main pick task)

**calibration.py**
- `CameraCalibration` class: camera-to-base-frame transform
- Method: `transform_to_base(point_cam)` → `point_base`
- Empirical 4x4 matrix, loaded from YAML

### llm/

**claude.py**
- `ClaudeProvider` class: OpenRouter/Anthropic API
- Method: `plan(query)` → `TaskPlan` (list of skills + params)
- Model: Claude Haiku (fast, cheap)

**openai_compat.py**
- `OpenAIProvider` class: OpenAI-compatible APIs
- Same interface as Claude (swappable)

**local.py**
- `OllamaProvider` class: local Ollama instance
- For offline development/testing

**prompts.py**
- `PLANNING_PROMPT`: system message for LLM
- Action-oriented: "Execute immediately, no clarification questions"
- Includes skill registry (TODO: dynamic per ADR-002)

### skills/

**pick.py**
- `PickSkill` class: full pick pipeline
- Flow: detect → 3D → calibrate → IK → motion sequence → drop
- Always re-detects (never uses cached world model position)

**place.py**
- `PlaceSkill` class: place object at target location
- Target: either object label or 3D pose

**home.py, scan.py**
- Direct pose moves (no perception)
- Used as intermediate waypoints

**detect.py**
- `DetectSkill` class: VLM detection + 3D point
- Stores result in world model

**calibration.py**
- Helper functions: `apply_gripper_asymmetry()`, `apply_z_offset()`, `apply_xy_offset()`
- Tuning parameters: z_offset=10cm, x_offset=+2cm, y asymmetry

### cli/

**simple.py**
- `RCLIPShell` class: readline-based REPL
- Commands: `pick <target>`, `place <target>`, `home`, `scan`, `open`, `close`, `quit`
- Integrates with Agent

**dashboard.py**
- `Dashboard` class: Textual TUI
- Real-time display: robot state, camera feed, detected objects, task log

**calibration_wizard.py**
- Interactive calibration procedure
- Prompts user to place objects, collects points, builds calibration matrix

### ros2/

**nodes/perception_node.py**
- ROS2 Node: subscribes to raw camera topics, publishes detected objects
- Wraps perception pipeline

**nodes/hardware_bridge.py**
- ROS2 Node: subscribes to motion commands, publishes joint angles + gripper state

**nodes/world_model_node.py**
- ROS2 Node: maintains world model, publishes object list

**launch/nano.launch.py**
- Launches all 5 ROS2 nodes
- Parameters: sim/real switching, LLM provider, camera calibration path

---

## Design Decisions

### 1. **ROS2 is Optional**
- Core SDK has zero ROS2 imports
- Pure Python, works standalone on any computer
- ROS2 layer added as thin wrapper nodes for integration
- **Rationale:** maximize flexibility, minimal dependencies for standalone operation

### 2. **Calibration is Pose-Dependent**
- Camera-to-arm transform only valid at home/scan position (eye-in-hand)
- Different poses = different optical axis
- **Trade-off:** simplified calibration (single matrix) vs. look-then-move correction
- **Current approach:** empirical offsets + frequent re-detection

### 3. **Empirical Offsets Over Model-Based Correction**
- Real arm (3D-printed, servo backlash) doesn't match URDF
- FK/IK errors: ~5-20mm typical
- **Solution:** measure offsets (X, Y, Z asymmetry), add to target before IK
- **Tuning history:** 10+ iterations, converged on x=+2cm, y=±1-3cm (asymmetric)

### 4. **World Model Cleared After Each Pick**
- Conservative approach: don't accumulate stale positions
- Every pick **re-detects** with VLM (fresh sensor data)
- **Trade-off:** slower (more VLM calls) vs. guaranteed accuracy

### 5. **Direct Commands Bypass LLM**
- `home()`, `scan()`, `open()`, `close()` execute instantly
- No API call, no latency
- **Rationale:** frequent calibration/testing, speed critical

### 6. **Skill Protocol is Simple**
- `execute(executor, **kwargs)` → `Result` (dict or None)
- Executor passes itself (allows access to world model, hardware)
- No message passing overhead (direct function calls)

### 7. **Task Executor is Deterministic**
- Linear task plan execution (no loops, no conditionals)
- Dependencies are explicit (task 2 depends on task 1's output)
- **Rationale:** predictable, testable, debuggable

---

## Testing Strategy

### Unit Tests (350+ tests)
- Pure functions: calibration math, pointcloud filtering, IK solving
- Mock hardware: `FakeArm`, `FakeGripper`, `FakeCamera`
- No real sensors needed

### Integration Tests (200+ tests)
- Multi-component: executor + skills + mock hardware
- Full pick pipeline end-to-end (PyBullet physics)
- LLM mocked with deterministic response

### Hardware Tests (150+ tests)
- Real SO-101 arm + RealSense camera
- Calibration validation, pick accuracy measurement
- Skipped in CI (requires physical robot)

**Coverage:** 85%+ (core modules 95%+, hardware 70%+)

---

## Performance Notes

- **LLM latency:** ~2s (API call + planning)
- **VLM detection:** ~500ms (GPU)
- **EdgeTAM tracking:** real-time (30 FPS background thread)
- **IK solving:** ~100ms (5 iterations max)
- **Pick cycle:** ~15-20s total (scan + detect + motion + drop)

---

## Next Steps

### ADR-002: Skill Manifest Protocol
- Phase 1: YAML registry with aliases (`"battery" → PickSkill(label="电池")`)
- Phase 2: LLM context enrichment (available skills → prompt injection)
- Phase 3: Dynamic skill discovery + runtime loading
- Phase 4: Multi-agent skill coordination (ROS2 topic broadcast)

See `docs/ADR-002-skill-manifest-protocol.md` for detailed design.
