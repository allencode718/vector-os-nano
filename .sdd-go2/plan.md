# Go2 MuJoCo Integration — Technical Plan

- Status: draft
- Author: Lead Architect (Opus)
- Date: 2026-03-24
- Prereq: spec.md approved (gait controller changed from sinusoidal to convex MPC per CEO decision)

## 1. Architecture Overview

```
ToolAgent / CLI
    │
    ├── WalkSkill.execute(direction, distance, speed)
    │       └── context.base.walk(vx, vy, vyaw, duration)
    │
    ├── TurnSkill.execute(direction, angle)
    │       └── context.base.walk(0, 0, vyaw, duration)
    │
    └── StandSkill / SitSkill / LieDownSkill
            └── context.base.stand() / sit() / lie_down()
                    │
                    ▼
            ┌─────────────────────────────────────────┐
            │           MuJoCoGo2                      │
            │  (vector_os_nano/hardware/sim/mujoco_go2)│
            │                                          │
            │  walk(vx, vy, vyaw, duration):           │
            │    ┌──────────────────────────┐           │
            │    │  convex_mpc stack        │           │
            │    │  PinGo2Model (dynamics)  │           │
            │    │  ComTraj (trajectory)    │           │
            │    │  CentroidalMPC (QP)     │           │
            │    │  LegController (torques) │           │
            │    │  Gait (contact schedule) │           │
            │    └──────────────────────────┘           │
            │                                          │
            │  stand() / sit() / lie_down():           │
            │    PD joint interpolation                │
            │                                          │
            │  MuJoCo_GO2_Model (sim interface)        │
            └─────────────────────────────────────────┘
```

The MuJoCoGo2 class wraps the entire go2-convex-mpc control stack behind the
same high-level interface spec'd in spec.md Section 4. Skills see only
`walk(vx, vy, vyaw, duration)`, `stand()`, `sit()`, etc.

## 2. Technical Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Locomotion controller | go2-convex-mpc (Convex MPC) | Proven stable, velocity-tracking, zero training, CPU-only |
| Dependency strategy | `pip install -e ~/Desktop/go2-convex-mpc` + pip casadi | pinocchio 3.9 already in .venv, casadi 3.7 pip installable |
| MJCF model | go2-convex-mpc bundled models | Includes URDF (for Pinocchio) + MJCF (for MuJoCo), matched |
| Scene MJCF | Our own go2_room.xml including go2-convex-mpc's scene.xml | Indoor room with furniture for demo |
| Stand/sit/lie_down | PD joint interpolation (same as unitree stand_go2.py) | MPC is for locomotion; posture transitions use direct PD |
| Sim rates | SIM=1kHz, CTRL=200Hz, MPC=~48Hz (from go2-convex-mpc) | Proven working rates from the library |
| Viewer | MuJoCo passive viewer (same as MuJoCoArm) | Consistent UX |

## 3. Module Design

### Module A: MuJoCoGo2 (`hardware/sim/mujoco_go2.py`)

**Responsibility:** Wrap go2-convex-mpc into a BaseProtocol-compatible class.

**Key insight:** go2-convex-mpc's example scripts run a monolithic sim loop.
We need to refactor this into a stateful controller that can be called
step-by-step from our MuJoCoGo2 methods.

**Internal state:**
```python
class MuJoCoGo2:
    # convex_mpc objects (created on connect)
    _pin_model: PinGo2Model         # Pinocchio dynamics
    _mj_model: MuJoCo_GO2_Model     # MuJoCo interface
    _mpc: CentroidalMPC             # QP solver
    _leg_ctrl: LegController        # Swing/stance torques
    _gait: Gait                     # Contact schedule
    _traj: ComTraj                  # Reference trajectory

    # Sim loop state
    _sim_hz: int = 1000
    _ctrl_hz: int = 200
    _tau_hold: np.ndarray           # Last computed torques (12,)
    _ctrl_counter: int = 0          # Tracks ctrl decimation
```

**Methods:**

| Method | Implementation |
|--------|---------------|
| `connect()` | Init PinGo2Model, MuJoCo_GO2_Model (with custom scene XML), Gait, ComTraj, CentroidalMPC, LegController. Initial forward pass. Optional viewer. |
| `disconnect()` | Close viewer, release model. |
| `get_position()` | Read `mj_data.qpos[0:3]` → (x, y, z) |
| `get_heading()` | Extract yaw from quaternion `mj_data.qpos[3:7]` |
| `get_velocity()` | Read `mj_data.qvel[0:3]` body velocity, transform to body frame |
| `get_joint_positions()` | Read `mj_data.qpos[7:19]` → 12 joint angles |
| `walk(vx, vy, vyaw, duration)` | Run MPC control loop for `duration` seconds (see below) |
| `stand()` | PD interpolate to standing pose `[0, 0.9, -1.8] x 4` over 2s |
| `sit()` | PD interpolate to sit pose `[0, 1.5, -2.5] x 4` over 2s |
| `lie_down()` | PD interpolate to lie pose `[0, 2.0, -2.7] x 4` over 2s |
| `stop()` | Zero velocity: set tau_hold to standing PD |
| `step(n)` | Advance sim by n timesteps |
| `render(camera, w, h)` | Render from named camera |

**walk() control loop (core logic):**

```python
def walk(self, vx, vy, vyaw, duration):
    vx = np.clip(vx, -0.8, 0.8)
    vy = np.clip(vy, -0.4, 0.4)
    vyaw = np.clip(vyaw, -4.0, 4.0)

    sim_steps = int(duration * self._sim_hz)
    ctrl_decim = self._sim_hz // self._ctrl_hz
    mpc_dt = self._gait.gait_period / 16
    steps_per_mpc = max(1, int(self._ctrl_hz // (1.0 / mpc_dt)))

    ctrl_i = 0
    for k in range(sim_steps):
        if k % ctrl_decim == 0:
            # Sync Pinocchio from MuJoCo
            self._mj_model.update_pin_with_mujoco(self._pin_model)

            # MPC update
            if ctrl_i % steps_per_mpc == 0:
                self._traj.generate_traj(
                    self._pin_model, self._gait, self._sim_time,
                    vx, vy, 0.27, vyaw, time_step=mpc_dt,
                )
                sol = self._mpc.solve_QP(self._pin_model, self._traj, False)
                self._U_opt = ... # extract from sol

            # Leg torques (all 4 legs)
            tau = self._compute_all_leg_torques(ctrl_i)
            tau = np.clip(tau, -TAU_LIM, TAU_LIM)
            self._tau_hold = tau
            ctrl_i += 1

        # Sim step with held torques
        mj.mj_step1(self._mj_model.model, self._mj_model.data)
        self._mj_model.set_joint_torque(self._tau_hold)
        mj.mj_step2(self._mj_model.model, self._mj_model.data)
        self._sim_time += 1.0 / self._sim_hz

        # Sync viewer
        if self._viewer is not None and k % 8 == 0:
            self._viewer.sync()

    return True
```

**PD posture control (stand/sit/lie_down):**

Adapted from unitree_mujoco's `stand_go2.py`:
```python
def _pd_interpolate(self, target_joints, duration=2.0):
    """Smoothly interpolate to target joint positions using PD control."""
    current = self.get_joint_positions()
    steps = int(duration * self._sim_hz)
    for k in range(steps):
        phase = np.tanh(k / steps * 3.0)  # smooth sigmoid
        target = phase * target_joints + (1 - phase) * current
        # PD torques
        q_err = target - self.get_joint_positions()
        dq = self._mj_model.data.qvel[6:]
        tau = 50.0 * q_err - 3.5 * dq
        tau = np.clip(tau, -TAU_LIM, TAU_LIM)
        self._mj_model.set_joint_torque(tau)
        mj.mj_step(self._mj_model.model, self._mj_model.data)
        if self._viewer and k % 8 == 0:
            self._viewer.sync()
```

### Module B: Go2 Scene MJCF (`hardware/sim/go2_room.xml`)

**Responsibility:** Indoor room scene that includes the Go2 model.

**Key issue:** go2-convex-mpc's `MuJoCo_GO2_Model` hardcodes its own
`models/MJCF/go2/scene.xml` path. We have two options:

1. Patch `MuJoCo_GO2_Model.__init__` to accept custom XML path
2. Subclass and override

**Choice:** Option 1 — simple and clean. We add an optional `xml_path`
parameter to `MuJoCo_GO2_Model` construction, or we construct our own
`mj.MjModel.from_xml_path()` and pass model/data directly.

**Actually:** We don't need to modify go2-convex-mpc at all. We'll construct
our own MuJoCo model/data from our scene XML, then pass model+data to a
slightly adapted initialization. The PinGo2Model is independent of MuJoCo
(it uses URDF). We only need MuJoCo model/data for simulation stepping.

**Scene layout:** 5m x 5m room (from spec Section 6), but the Go2 model
comes from go2-convex-mpc's bundled MJCF.

### Module C: Go2 Skills (`skills/go2/`)

**Responsibility:** SkillFlow-decorated skills for quadruped commands.

**Files:**
- `skills/go2/__init__.py` — `get_go2_skills()` registration
- `skills/go2/walk.py` — WalkSkill
- `skills/go2/turn.py` — TurnSkill
- `skills/go2/stance.py` — StandSkill, SitSkill, LieDownSkill

Skills are thin wrappers that translate parameters into `context.base.xxx()`
calls. All heavy logic lives in MuJoCoGo2.

### Module D: Agent Integration (modified files)

**core/world_model.py:**
- Add `position_xy: tuple[float, float] = (0.0, 0.0)` and `heading: float = 0.0` to RobotState
- Add these to `update_robot_state()` valid_fields
- Add to `to_dict()` / `from_dict()`

**core/agent.py:**
- Add `base: Any = None` parameter to `__init__`
- Store as `self._base`
- In `_build_context()`: pass `base=self._base`
- In `_sync_robot_state()`: sync base position/heading when base is present

**core/tool_agent.py:**
- In `_build_system_prompt()`: detect Go2 mode (base present, arm absent), inject Go2 state info

**run.py:**
- Add `--sim-go2` / `--sim-go2-headless` flags
- Add `_init_sim_go2(cfg, gui)` function
- Register Go2 skills instead of arm skills
- Pass `base=` to Agent constructor

## 4. Data Flow

```
User: "往前走两步"
    │
    ▼
ToolAgent._build_system_prompt()
  → "Mode: Go2 quadruped, Position: (2.5, 2.5), Heading: 0 deg"
  → LLM sees walk/turn/stand/sit tools
    │
    ▼
LLM calls walk(direction="forward", distance=2.0)
    │
    ▼
WalkSkill.execute({"direction":"forward","distance":2.0}, context)
  → vx = 0.3, duration = 2.0 / 0.3 = 6.67s
  → context.base.walk(0.3, 0.0, 0.0, 6.67)
    │
    ▼
MuJoCoGo2.walk(0.3, 0.0, 0.0, 6.67)
  → 6670 sim steps @ 1kHz
  → MPC solves QP every ~4 ctrl steps
  → LegController computes joint torques every 5 sim steps
  → Viewer syncs every 8 sim steps
    │
    ▼
SkillResult(success=True, result_data={"new_position": (2.5, 4.5, 0.27)})
```

## 5. Directory Structure

```
vector_os_nano/
├── hardware/sim/
│   ├── mujoco_go2.py          # NEW — MuJoCoGo2 class (~350 lines)
│   └── go2_room.xml           # NEW — Indoor scene MJCF (~80 lines)
├── skills/go2/
│   ├── __init__.py            # NEW — get_go2_skills()
│   ├── walk.py                # NEW — WalkSkill
│   ├── turn.py                # NEW — TurnSkill
│   └── stance.py              # NEW — StandSkill, SitSkill, LieDownSkill
├── core/
│   ├── agent.py               # MOD — add base param
│   ├── world_model.py         # MOD — add position_xy, heading
│   └── tool_agent.py          # MOD — Go2 system prompt
├── run.py                     # MOD — --sim-go2 flag
└── tests/unit/
    ├── test_mujoco_go2.py     # NEW — lifecycle, stand, walk, turn tests
    ├── test_go2_skills.py     # NEW — skill integration tests
    └── test_world_model_base.py  # NEW — RobotState base fields
```

## 6. Key Implementation Details

### 6.1 MuJoCo Model Loading

go2-convex-mpc's `MuJoCo_GO2_Model` hardcodes its XML path. Rather than
monkey-patching, MuJoCoGo2 will:

1. Load our `go2_room.xml` via `mj.MjModel.from_xml_path()`
2. Create a lightweight adapter that wraps model+data with the same interface
   as `MuJoCo_GO2_Model` (set_joint_torque, base_bid, etc.)
3. PinGo2Model uses URDF (separate from MuJoCo) — no scene dependency

```python
class _MuJoCoModelAdapter:
    """Adapter to make our custom scene work with convex_mpc's LegController."""
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.base_bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base_link")

    def set_joint_torque(self, torque):
        # Same actuator naming as go2-convex-mpc
        for i, leg in enumerate(["FL", "FR", "RL", "RR"]):
            for j, joint in enumerate(["hip", "thigh", "calf"]):
                aid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{joint}")
                self.data.ctrl[aid] = torque[i*3 + j]

    def update_pin_with_mujoco(self, go2):
        # Identical to MuJoCo_GO2_Model.update_pin_with_mujoco
        ...
```

### 6.2 Scene XML Strategy

Our `go2_room.xml` will NOT use `<include>` (fragile path resolution).
Instead, it will be a self-contained scene that loads the Go2 MJCF
assets from go2-convex-mpc's bundled model path using `<include>` with
the absolute path resolved at runtime.

**Actually simpler:** We load go2-convex-mpc's existing `scene.xml`
(which already includes the Go2 model) and add our room geometry
programmatically via MuJoCo's `mj_loadXML` with a modified XML string,
or we create a composite XML that includes their scene.

**Simplest approach:** For Milestone 1, just use go2-convex-mpc's scene
as-is (flat ground, no room). Add the room in a follow-up. The Go2 walking
on a flat plane is the critical deliverable.

### 6.3 Viewer Sync

go2-convex-mpc uses a separate replay loop. We need real-time display:
- Launch `mujoco.viewer.launch_passive()` in connect()
- Sync viewer every ~8 sim steps (125 Hz visual update)
- Same pattern as MuJoCoArm

### 6.4 Dependency Management

All dependencies already available via pip in our .venv:
- `pinocchio` (pin 3.9.0) — already installed
- `casadi` (3.7.2) — just installed
- `convex_mpc` (1.0.0) — installed as editable from ~/Desktop/go2-convex-mpc

For pyproject.toml, add optional extras:
```toml
[project.optional-dependencies]
go2 = ["casadi>=3.6", "pin>=3.0"]
```
(convex_mpc will need to be vendored or published for PyPI)

## 7. Test Strategy

### Unit Tests (TDD — write first)

| Test | File | What it verifies |
|------|------|-----------------|
| T1: lifecycle | test_mujoco_go2.py | connect/disconnect, _connected flag |
| T2: standing | test_mujoco_go2.py | stand() → height ~0.27m, 12 joints |
| T3: walk forward | test_mujoco_go2.py | walk(0.3,0,0,2) → displacement > 10cm |
| T4: turn | test_mujoco_go2.py | walk(0,0,0.5,2) → heading change > 0.3 rad |
| T5: sit | test_mujoco_go2.py | sit() → z < stand_z |
| T6: stability | test_mujoco_go2.py | walk 5s → still upright (z > 0.15) |
| T7: walk skill | test_go2_skills.py | WalkSkill.execute → success=True |
| T8: world model | test_world_model_base.py | position_xy, heading fields work |
| T9: PD controller | test_mujoco_go2.py | set_joint_positions → within 0.1 rad |
| T10: MPC import | test_mujoco_go2.py | convex_mpc imports OK, model loads |

### Integration Tests (post-implementation)

| Test | What it verifies |
|------|-----------------|
| Agent + Go2 skills | Agent(base=go2) registers Go2 skills |
| ToolAgent Go2 prompt | System prompt includes Go2 state |
| --sim-go2 launch | run.py flag works end-to-end |

### Coverage Target

| Module | Target |
|--------|--------|
| mujoco_go2.py | 80% |
| skills/go2/*.py | 90% |
| world_model.py changes | 95% |

### Test Speed

MuJoCo Go2 tests will be slower than arm tests due to MPC computation:
- T1 (lifecycle): ~1s
- T2 (stand): ~3s (2s interpolation + sim steps)
- T3 (walk): ~5s (2s walk at 1kHz sim)
- T6 (stability): ~8s (5s walk)

Total estimated: ~40s for all Go2 tests. Mark with `@pytest.mark.slow` if needed.

## 8. Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| MPC too slow on CPU for real-time sim | Walk commands block for too long | go2-convex-mpc solves in ~2.7ms, well within 20ms budget. If slow: reduce MPC frequency |
| Go2-convex-mpc scene.xml path conflicts | Model won't load | Use adapter pattern, load our own scene |
| pinocchio/casadi version conflict with .venv | Import errors | Already verified: pin 3.9 + casadi 3.7 work together |
| MPC instability at edge velocities | Dog falls over | Clamp velocities conservatively (vx<0.5, vy<0.3) for Milestone 1 |
| convex_mpc not on PyPI | Can't pip install for distribution | Vendor or fork for v1.0 release; editable install fine for dev |
| MJCF model mismatch (menagerie vs convex_mpc bundled) | Joint naming conflicts | Use convex_mpc's bundled models exclusively (URDF + MJCF matched) |

## 9. Milestone 1 Success Criteria (from spec)

1. `python run.py --sim-go2` launches MuJoCo with Go2
2. Dog stands up on startup
3. "往前走两步" → walk forward ~2m
4. "左转" → turn left ~90 degrees
5. "坐下" → dog sits
6. Viewer shows the scene in real-time
7. All 10 test contracts pass
8. Dog does not fall over during normal walking (T6)
