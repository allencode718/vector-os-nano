# Go2 MuJoCo Integration Spec

- Status: draft
- Author: Lead Architect (Opus)
- Date: 2026-03-24
- Scope: Milestone 1 -- stand, walk, turn, sit in indoor room via ToolAgent

## 1. Problem Statement

Vector OS Nano controls a SO-101 arm in MuJoCo via `MuJoCoArm`. We need to add
Unitree Go2 quadruped support so the dog can walk around a simple indoor room,
controlled by the same ToolAgent / SkillFlow architecture.

The official `unitree_sdk2py` requires DDS network communication (Cyclone DDS
over loopback), a multi-process architecture, and heavyweight dependencies. This
is designed for sim-to-real transfer, not for our lightweight embedded agent
demo.

We will write a `MuJoCoGo2` class that controls the MuJoCo model directly --
identical in spirit to how `MuJoCoArm` controls the SO-101 arm -- and a simple
sinusoidal trotting gait controller for locomotion.

## 2. Architecture Decision: Direct MuJoCo Control

### Option A: unitree_sdk2py + DDS bridge (rejected)

- Requires `unitree_sdk2py` pip package with Cyclone DDS
- Three-process architecture: MuJoCo sim, DDS bridge, control program
- Network loopback communication with discovery overhead
- Heavy for a demo; complex failure modes

### Option B: Direct MuJoCo API (chosen)

- Zero external dependencies beyond `mujoco` (already a dependency)
- Single-process, in-memory: load MJCF, set ctrl, call mj_step
- Consistent with existing MuJoCoArm pattern
- Gait controller runs in-process as plain Python

If Yusen later wants real Go2 hardware or the official SDK locomotion controller,
a `UnitreeSDKGo2` adapter class can be added as a separate implementation of the
same BaseProtocol. The MuJoCo-native version comes first.

### Option C: MuJoCo MPC (mjpc) for locomotion (deferred)

- MuJoCo has a built-in MPC-based locomotion planner
- Much better locomotion quality, but adds mjpc dependency
- Deferred to Milestone 2 if sinusoidal gait is insufficient

## 3. Go2 Hardware Model

### 3.1 Joint Layout

12 actuators in 4 legs, 3 joints per leg (hip/thigh/calf):

```
Leg     | hip_joint       | thigh_joint     | calf_joint
--------|-----------------|-----------------|------------------
FL      | FL_hip_joint    | FL_thigh_joint  | FL_calf_joint
FR      | FR_hip_joint    | FR_thigh_joint  | FR_calf_joint
RL      | RL_hip_joint    | RL_thigh_joint  | RL_calf_joint
RR      | RR_hip_joint    | RR_thigh_joint  | RR_calf_joint
```

### 3.2 Joint Ranges (from mujoco_menagerie go2.xml)

| Joint class  | Range (rad)           | Torque limit |
|--------------|-----------------------|--------------|
| abduction    | [-1.0472, 1.0472]     | 23.7 Nm      |
| front_hip    | [-1.5708, 3.4907]     | 23.7 Nm      |
| back_hip     | [-0.5236, 4.5379]     | 23.7 Nm      |
| knee         | [-2.7227, -0.83776]   | 45.43 Nm     |

### 3.3 Standing Pose (home keyframe)

From the MJCF keyframe:
```
Base position: (0, 0, 0.27)
Base quaternion: (1, 0, 0, 0) -- upright
Per leg: hip=0, thigh=0.9, calf=-1.8
```

### 3.4 MJCF Source

We use `mujoco_menagerie/unitree_go2/go2.xml` because:
- Higher friction (0.6 vs 0.4) -- more stable walking
- Higher joint damping (2.0 vs 0.1) -- less oscillation
- Better foot contact parameters (solimp specified)
- Established, maintained open-source model

The scene file (`go2_scene.xml`) will include this model via `<include>`.

### 3.5 Freejoint

The Go2 base has a `freejoint` (7 qpos: xyz + quaternion). This means:
- `qpos[0:3]` = base position (x, y, z)
- `qpos[3:7]` = base orientation quaternion (w, x, y, z) -- MuJoCo uses wxyz
- `qpos[7:19]` = 12 joint angles (3 per leg x 4 legs)

## 4. MuJoCoGo2 Class Design

### 4.1 BaseProtocol

The class implements a `BaseProtocol` (duck-typed, like `ArmProtocol` for
MuJoCoArm). Skills access it via `context.base`.

```python
class MuJoCoGo2:
    """Unitree Go2 in MuJoCo simulation.

    BaseProtocol-compatible. Drop-in base controller for quadruped skills.
    """

    def __init__(self, gui: bool = False, scene_xml: str | None = None) -> None: ...
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...

    # --- State queries ---
    def get_position(self) -> tuple[float, float, float]: ...
    def get_orientation(self) -> tuple[float, float, float, float]: ...
    def get_heading(self) -> float: ...
    def get_velocity(self) -> tuple[float, float, float]: ...
    def get_joint_positions(self) -> list[float]: ...

    # --- High-level commands ---
    def stand(self) -> bool: ...
    def sit(self) -> bool: ...
    def lie_down(self) -> bool: ...
    def walk(self, vx: float, vy: float, vyaw: float, duration: float) -> bool: ...
    def stop(self) -> None: ...

    # --- Low-level ---
    def set_joint_positions(self, positions: list[float], duration: float) -> bool: ...
    def step(self, n: int = 1) -> None: ...
    def render(self, camera_name: str, width: int, height: int) -> Any: ...
```

### 4.2 Lifecycle (mirrors MuJoCoArm)

```
__init__(gui, scene_xml)  -- store params, no MuJoCo import
connect()                 -- load MJCF, create MjData, cache IDs, launch viewer
disconnect()              -- close viewer, release model
```

Lazy MuJoCo import pattern: `_get_mujoco()` global, identical to mujoco_arm.py.

### 4.3 State Queries

**get_position()** -> (x, y, z):
Read base body position from `data.body("base").xpos`. Returns world-frame
coordinates in meters.

**get_heading()** -> float:
Extract yaw from base quaternion. MuJoCo uses (w, x, y, z) quaternion format.
Yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2)).

**get_velocity()** -> (vx, vy, vyaw):
Read from `data.body("base").cvel` (6D spatial velocity in world frame).
Transform to body-frame vx/vy using heading rotation, extract vyaw from
angular velocity.

**get_joint_positions()** -> list[float]:
Read 12 joint qpos values in canonical order:
FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf.

### 4.4 Joint Position Control

The mujoco_menagerie Go2 model uses torque-mode motors (`<motor>`). To get
position control, MuJoCoGo2 must implement a PD controller:

```
tau = kp * (q_target - q_current) + kd * (0 - q_vel)
data.ctrl[actuator_id] = tau
```

PD gains (from unitree_mujoco reference):
- kp = 20.0 for all joints (hip, thigh, calf)
- kd = 0.5 for all joints

These match the Go2's factory PD controller gains. The actuator ctrl values are
torques (Nm), not positions.

### 4.5 High-Level Commands

**stand():**
Interpolate current joint positions to the standing pose
`[0, 0.9, -1.8] x 4 legs` over 2 seconds. Returns True when complete.

**sit():**
Interpolate to a sitting pose:
`[0, 1.5, -2.5] x 4 legs` (thighs more forward, calves more bent).
Dog's rear lowers while front stays up.

**lie_down():**
Interpolate to a lying pose:
`[0, 2.0, -2.7] x 4 legs` (all legs folded, body on ground).

**walk(vx, vy, vyaw, duration):**
Execute the sinusoidal trotting gait controller (Section 5) for `duration`
seconds at the requested velocity.

- vx: forward speed (m/s), clamped to [-0.5, 0.5]
- vy: lateral speed (m/s), clamped to [-0.3, 0.3]
- vyaw: turning rate (rad/s), clamped to [-1.0, 1.0]
- duration: seconds to walk

**stop():**
Set all actuator targets to current joint positions (zero velocity).

### 4.6 Rendering

Same pattern as MuJoCoArm.render(). Create an mj.Renderer, render from the
named camera, return (H, W, 3) uint8 BGR array. The scene XML defines cameras.

## 5. Gait Controller: Sinusoidal Trotting

### 5.1 Approach

A trotting gait pairs diagonal legs (FL+RR, FR+RL) that swing together.
Each joint trajectory is a sinusoidal offset from the standing pose:

```
q_hip(t)   = q_stand_hip   + A_hip   * sin(2*pi*f*t + phi_hip)
q_thigh(t) = q_stand_thigh + A_thigh * sin(2*pi*f*t + phi_thigh)
q_calf(t)  = q_stand_calf  + A_calf  * sin(2*pi*f*t + phi_calf)
```

Diagonal legs are in phase, adjacent legs are 180 degrees out of phase.

### 5.2 Gait Parameters

| Parameter      | Value   | Unit   | Notes                             |
|----------------|---------|--------|-----------------------------------|
| f (frequency)  | 2.0     | Hz     | Steps per second (moderate trot)  |
| A_hip          | 0.0     | rad    | No lateral sway (abduction)       |
| A_thigh        | 0.3     | rad    | Swing amplitude for thigh         |
| A_calf         | 0.3     | rad    | Complementary calf swing          |
| phi_thigh      | 0       | rad    | Reference phase                   |
| phi_calf       | pi/2    | rad    | Calf leads thigh by 90 degrees    |

These produce a stable trot for the Go2's kinematics. The amplitudes and
phases were chosen based on:
- Go2 leg length: thigh=0.213m, calf=0.213m
- Standing height: 0.27m
- Foot clearance requirement: ~2-3cm during swing

### 5.3 Velocity Modulation

Forward speed (vx):
- Scale A_thigh proportionally: `A_thigh = 0.3 * abs(vx) / 0.5`
- Reverse direction: negate amplitude for backward walking
- This naturally produces longer strides at higher speeds

Lateral speed (vy):
- Modulate A_hip: `A_hip = 0.15 * vy / 0.3`
- Left legs and right legs get opposite hip phase

Turning (vyaw):
- Differential stride length: inner legs reduce amplitude, outer legs increase
- `A_thigh_left  = A_thigh * (1 + vyaw * 0.5)`
- `A_thigh_right = A_thigh * (1 - vyaw * 0.5)`

### 5.4 Foot Contact Phasing

Trot pattern (diagonal pairs in sync):
```
Phase 0.0-0.5: FL+RR stance (on ground), FR+RL swing (in air)
Phase 0.5-1.0: FR+RL stance (on ground), FL+RR swing (in air)
```

Implemented via phase offset in the sinusoid:
```python
phase_offset = {
    "FL": 0,      "FR": pi,
    "RL": pi,     "RR": 0,
}
```

### 5.5 Feasibility Analysis

**Is sinusoidal trot sufficient for indoor navigation?**

YES, for the demo scope:
- Flat floor, no steps/ramps
- Low speeds (< 0.5 m/s)
- No external disturbances
- MuJoCo's contact solver handles foot-ground interaction
- The Go2 MJCF has proper mass/inertia for stable dynamics

Limitations (acceptable for Milestone 1):
- No terrain adaptation (stairs, slopes)
- No dynamic balance recovery (push recovery)
- Gait transitions (walk/trot/gallop) not supported
- Drift accumulation in heading over long walks

These are acceptable because we are building a demo, not a production locomotion
controller. The sinusoidal gait has been used successfully in quadruped research
(Raibert's original work, MIT Cheetah early prototypes) for flat-ground walking.

## 6. Scene: go2_room.xml

### 6.1 Layout

A simple 5m x 5m indoor room:

```
+---------------------------+
|                           |
|   [table]                 |
|                           |
|          Go2              |
|          -->              |
|                           |
|              [chair]      |
|                           |
+---------------------------+
```

### 6.2 MJCF Structure

```xml
<mujoco model="go2_room">
  <!-- Include the Go2 model from mujoco_menagerie -->
  <include file="go2.xml"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
  </visual>

  <asset>
    <!-- Floor texture -->
    <texture type="2d" name="floor_tex" builtin="checker"
             rgb1="0.8 0.8 0.8" rgb2="0.6 0.6 0.6" width="512" height="512"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="10 10"/>
    <!-- Wall material -->
    <material name="wall_mat" rgba="0.9 0.9 0.85 1"/>
    <!-- Furniture material -->
    <material name="wood_mat" rgba="0.55 0.35 0.15 1"/>
  </asset>

  <worldbody>
    <light pos="2.5 2.5 3" dir="0 0 -1" directional="true"/>
    <light pos="2.5 2.5 2.5" dir="0 0 -1" diffuse="0.3 0.3 0.3"/>

    <!-- Floor -->
    <geom name="floor" type="plane" size="2.5 2.5 0.1"
          material="floor_mat" pos="2.5 2.5 0"/>

    <!-- Walls (4 box geoms) -->
    <geom name="wall_north" type="box" size="2.5 0.05 1.2"
          pos="2.5 5.0 1.2" material="wall_mat"/>
    <geom name="wall_south" type="box" size="2.5 0.05 1.2"
          pos="2.5 0.0 1.2" material="wall_mat"/>
    <geom name="wall_east"  type="box" size="0.05 2.5 1.2"
          pos="5.0 2.5 1.2" material="wall_mat"/>
    <geom name="wall_west"  type="box" size="0.05 2.5 1.2"
          pos="0.0 2.5 1.2" material="wall_mat"/>

    <!-- Table (simple box) -->
    <body name="table" pos="1.5 3.5 0">
      <geom name="table_top" type="box" size="0.4 0.3 0.02"
            pos="0 0 0.7" material="wood_mat"/>
      <geom name="table_leg1" type="box" size="0.03 0.03 0.35"
            pos="-0.35 -0.25 0.35" material="wood_mat"/>
      <geom name="table_leg2" type="box" size="0.03 0.03 0.35"
            pos="0.35 -0.25 0.35" material="wood_mat"/>
      <geom name="table_leg3" type="box" size="0.03 0.03 0.35"
            pos="-0.35 0.25 0.35" material="wood_mat"/>
      <geom name="table_leg4" type="box" size="0.03 0.03 0.35"
            pos="0.35 0.25 0.35" material="wood_mat"/>
    </body>

    <!-- Chair (simple box) -->
    <body name="chair" pos="3.5 1.5 0">
      <geom name="chair_seat" type="box" size="0.2 0.2 0.02"
            pos="0 0 0.4" material="wood_mat"/>
      <geom name="chair_back" type="box" size="0.2 0.02 0.3"
            pos="0 -0.18 0.72" material="wood_mat"/>
      <geom name="chair_leg1" type="box" size="0.02 0.02 0.2"
            pos="-0.17 -0.17 0.2" material="wood_mat"/>
      <geom name="chair_leg2" type="box" size="0.02 0.02 0.2"
            pos="0.17 -0.17 0.2" material="wood_mat"/>
      <geom name="chair_leg3" type="box" size="0.02 0.02 0.2"
            pos="-0.17 0.17 0.2" material="wood_mat"/>
      <geom name="chair_leg4" type="box" size="0.02 0.02 0.2"
            pos="0.17 0.17 0.2" material="wood_mat"/>
    </body>

    <!-- Cameras -->
    <camera name="overhead" pos="2.5 2.5 4.5" xyaxes="1 0 0 0 1 0"/>
    <camera name="tracking" pos="2.5 0.5 1.5" xyaxes="1 0 0 0 0.4 0.9"/>
  </worldbody>
</mujoco>
```

Notes:
- Go2 model is included via `<include>`. Its base starts at pos="0 0 0.445"
  in the MJCF; we override via keyframe to (2.5, 2.5, 0.27) standing in room
  center.
- Overhead camera provides top-down view for rendering.
- Tracking camera provides perspective view.
- Furniture bodies are static (no freejoint) -- they are obstacles, not
  manipulable objects.

## 7. Go2 Skills

### 7.1 Directory: `vector_os_nano/skills/go2/`

Skills follow the exact same `@skill` decorator pattern as arm skills.
They access `context.base` instead of `context.arm`.

### 7.2 WalkSkill

```python
@skill(
    aliases=["walk", "go", "move", "走", "走路", "往前走", "前进", "后退"],
    direct=False,
)
class WalkSkill:
    name = "walk"
    description = "Walk the robot in a direction for a distance or duration."
    parameters = {
        "direction": {
            "type": "string",
            "required": False,
            "default": "forward",
            "enum": ["forward", "backward", "left", "right"],
            "description": "Direction to walk",
        },
        "distance": {
            "type": "number",
            "required": False,
            "default": 1.0,
            "description": "Distance in meters (approximate)",
        },
        "speed": {
            "type": "number",
            "required": False,
            "default": 0.3,
            "description": "Speed in m/s (0.1 to 0.5)",
        },
    }
    preconditions = ["robot_standing"]
    postconditions = []
    effects = {"position": "changed"}
    failure_modes = ["no_base", "collision", "timeout"]
```

Execute logic:
1. Validate `context.base is not None`
2. Map direction to (vx, vy): forward=(speed,0), backward=(-speed,0),
   left=(0,speed), right=(0,-speed)
3. Calculate duration = distance / speed
4. Call `context.base.walk(vx, vy, 0.0, duration)`
5. Return SkillResult with new position

### 7.3 TurnSkill

```python
@skill(
    aliases=["turn", "rotate", "转", "转弯", "转向", "左转", "右转"],
    direct=False,
)
class TurnSkill:
    name = "turn"
    description = "Turn the robot in place by a given angle."
    parameters = {
        "direction": {
            "type": "string",
            "required": False,
            "default": "left",
            "enum": ["left", "right"],
            "description": "Turn direction",
        },
        "angle": {
            "type": "number",
            "required": False,
            "default": 90.0,
            "description": "Turn angle in degrees",
        },
    }
    preconditions = ["robot_standing"]
    postconditions = []
    effects = {"heading": "changed"}
    failure_modes = ["no_base"]
```

Execute logic:
1. Map direction to vyaw sign: left=+1, right=-1
2. vyaw = 0.5 * sign (0.5 rad/s turning speed)
3. duration = abs(angle_rad) / abs(vyaw)
4. Call `context.base.walk(0, 0, vyaw, duration)`

### 7.4 StanceSkills

```python
@skill(aliases=["stand", "站", "站起来", "起立"], direct=True)
class StandSkill:
    name = "stand"
    description = "Stand up."
    # ... execute calls context.base.stand()

@skill(aliases=["sit", "坐", "坐下"], direct=True)
class SitSkill:
    name = "sit"
    description = "Sit down."
    # ... execute calls context.base.sit()

@skill(aliases=["lie down", "lie", "趴", "趴下", "躺下"], direct=True)
class LieDownSkill:
    name = "lie_down"
    description = "Lie down."
    # ... execute calls context.base.lie_down()
```

These are direct=True (no LLM planning needed -- immediate execution).

### 7.5 Skill Registration

New file: `vector_os_nano/skills/go2/__init__.py`:
```python
def get_go2_skills() -> list:
    return [WalkSkill(), TurnSkill(), StandSkill(), SitSkill(), LieDownSkill()]
```

Skills are registered in `_init_sim_go2()` in run.py, NOT in the global
`get_default_skills()`. Go2 skills only make sense when a Go2 base is present.
Arm skills (pick, place, scan, etc.) are NOT registered in Go2 mode.

## 8. Agent Integration

### 8.1 Agent.__init__ changes

Add `base` parameter to Agent constructor:

```python
class Agent:
    def __init__(
        self,
        arm: Any = None,
        gripper: Any = None,
        perception: Any = None,
        base: Any = None,        # NEW
        llm: Any = None,
        ...
    ) -> None:
```

### 8.2 _build_context() changes

Pass `base` to SkillContext:

```python
def _build_context(self) -> SkillContext:
    return SkillContext(
        arm=self._arm,
        gripper=self._gripper,
        perception=self._perception,
        world_model=self._world_model,
        calibration=self._calibration,
        config=self._config,
        base=self._base,  # NEW
    )
```

### 8.3 _sync_robot_state() changes

When `self._base` is not None, also sync base position/heading:

```python
def _sync_robot_state(self) -> None:
    # Existing arm sync...

    if self._base is not None:
        try:
            pos = self._base.get_position()
            heading = self._base.get_heading()
            self._world_model.update_robot_state(
                position_xy=(pos[0], pos[1]),
                heading=heading,
            )
        except Exception as exc:
            logger.debug("Could not sync base state: %s", exc)
```

### 8.4 run.py: _init_sim_go2()

New function parallel to `_init_sim()`:

```python
def _init_sim_go2(cfg: dict, gui: bool = False) -> tuple:
    from vector_os_nano.hardware.sim.mujoco_go2 import MuJoCoGo2

    print("Starting Go2 MuJoCo simulation...")
    base = MuJoCoGo2(gui=gui)
    base.connect()
    base.stand()  # Start standing

    pos = base.get_position()
    print(f"Go2 standing at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

    # No arm, no gripper, no perception for Go2 milestone 1
    return None, None, None, None, base
```

New CLI flag: `--sim-go2` launches Go2 mode.

### 8.5 ToolAgent system prompt

The ToolAgent system prompt must be adapted when base is present and arm is not:

```python
if agent._base:
    parts.append("Mode: Go2 quadruped robot in MuJoCo simulation")
    parts.append("Robot type: Unitree Go2 (4-legged robot dog)")
    pos = agent._base.get_position()
    heading = agent._base.get_heading()
    parts.append(f"Position: ({pos[0]:.1f}, {pos[1]:.1f}) m")
    parts.append(f"Heading: {math.degrees(heading):.0f} deg")
```

## 9. WorldModel Extensions

### 9.1 RobotState additions

Add two optional fields to RobotState:

```python
@dataclass(frozen=True)
class RobotState:
    joint_positions: tuple[float, ...] = ()
    gripper_state: str = "open"
    held_object: str | None = None
    is_moving: bool = False
    ee_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # NEW -- mobile robot fields
    position_xy: tuple[float, float] = (0.0, 0.0)
    heading: float = 0.0  # radians, 0 = +X axis
```

### 9.2 update_robot_state() changes

Add `position_xy` and `heading` to the valid_fields set. Existing code is
unchanged -- new fields have defaults, so arm-only mode is unaffected.

### 9.3 New predicates

Add to WorldModel.check_predicate():
- `robot_standing` -- True when base is connected and not lying down
  (always True for now; future: track stance state)
- `robot_near(x,y,radius)` -- True when robot position is within radius of (x,y)

## 10. File Manifest

### New files

| File | Description | Lines (est.) |
|------|-------------|--------------|
| `hardware/sim/mujoco_go2.py` | MuJoCoGo2 class + PD controller + gait | ~350 |
| `hardware/sim/go2_room.xml` | Indoor room MJCF scene | ~80 |
| `skills/go2/__init__.py` | Go2 skill registration | ~15 |
| `skills/go2/walk.py` | WalkSkill | ~80 |
| `skills/go2/turn.py` | TurnSkill | ~60 |
| `skills/go2/stance.py` | StandSkill, SitSkill, LieDownSkill | ~90 |

### Modified files

| File | Change | Risk |
|------|--------|------|
| `core/agent.py` | Add `base` param, pass to context | LOW -- additive |
| `core/world_model.py` | Add position_xy, heading to RobotState | LOW -- defaults preserve compat |
| `core/skill.py` | No change (base already in SkillContext) | NONE |
| `core/tool_agent.py` | Adapt system prompt for Go2 mode | LOW |
| `run.py` | Add `_init_sim_go2()`, `--sim-go2` flag | LOW |

## 11. Test Contracts

### T1: MuJoCoGo2 lifecycle

```python
def test_connect_disconnect():
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    assert go2._connected
    pos = go2.get_position()
    assert len(pos) == 3
    assert pos[2] > 0.1  # not on the floor yet (initial z=0.445)
    go2.disconnect()
    assert not go2._connected
```

### T2: Standing pose

```python
def test_stand():
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    go2.stand()
    pos = go2.get_position()
    # Standing height should be approximately 0.27m
    assert 0.2 < pos[2] < 0.4
    joints = go2.get_joint_positions()
    assert len(joints) == 12
    go2.disconnect()
```

### T3: Walking moves the robot forward

```python
def test_walk_forward():
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    go2.stand()
    start_pos = go2.get_position()
    go2.walk(vx=0.3, vy=0.0, vyaw=0.0, duration=2.0)
    end_pos = go2.get_position()
    # Robot should have moved forward (positive X in body frame)
    displacement = ((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)**0.5
    assert displacement > 0.1  # moved at least 10cm in 2 seconds
    assert end_pos[2] > 0.15   # still upright, not fallen
    go2.disconnect()
```

### T4: Turning changes heading

```python
def test_turn():
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    go2.stand()
    start_heading = go2.get_heading()
    go2.walk(vx=0.0, vy=0.0, vyaw=0.5, duration=2.0)
    end_heading = go2.get_heading()
    # Should have turned approximately 1 radian (0.5 rad/s * 2s)
    delta = abs(end_heading - start_heading)
    assert delta > 0.3  # at least 0.3 rad turn
    go2.disconnect()
```

### T5: Sit lowers the robot

```python
def test_sit():
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    go2.stand()
    stand_z = go2.get_position()[2]
    go2.sit()
    sit_z = go2.get_position()[2]
    assert sit_z < stand_z  # robot is lower when sitting
    go2.disconnect()
```

### T6: Robot stays upright during walk

```python
def test_walk_stability():
    """Robot should not fall over during normal walking."""
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    go2.stand()
    go2.walk(vx=0.3, vy=0.0, vyaw=0.0, duration=5.0)
    pos = go2.get_position()
    # Z should stay above 0.15 (not fallen)
    assert pos[2] > 0.15
    go2.disconnect()
```

### T7: WalkSkill integration

```python
def test_walk_skill():
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    go2.stand()
    context = SkillContext(
        arm=None, gripper=None, perception=None,
        world_model=WorldModel(), calibration=None,
        base=go2,
    )
    skill = WalkSkill()
    result = skill.execute({"direction": "forward", "distance": 0.5}, context)
    assert result.success
    go2.disconnect()
```

### T8: WorldModel base state

```python
def test_world_model_base_fields():
    wm = WorldModel()
    wm.update_robot_state(position_xy=(1.5, 2.3), heading=0.5)
    robot = wm.get_robot()
    assert robot.position_xy == (1.5, 2.3)
    assert robot.heading == 0.5
    # Existing fields should have defaults
    assert robot.gripper_state == "open"
```

### T9: Scene loads without error

```python
def test_scene_loads():
    """go2_room.xml should load and simulate without crashes."""
    go2 = MuJoCoGo2(gui=False, scene_xml="path/to/go2_room.xml")
    go2.connect()
    go2.step(100)  # 100 timesteps without crash
    go2.disconnect()
```

### T10: PD controller tracks target position

```python
def test_pd_controller():
    """PD controller should drive joints to target within tolerance."""
    go2 = MuJoCoGo2(gui=False)
    go2.connect()
    target = [0, 0.9, -1.8] * 4  # standing pose
    go2.set_joint_positions(target, duration=2.0)
    actual = go2.get_joint_positions()
    for t, a in zip(target, actual):
        assert abs(t - a) < 0.1  # within 0.1 rad
    go2.disconnect()
```

## 12. Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Sinusoidal gait falls over | Medium | High | Tune PD gains + gait params iteratively; fallback to MuJoCo MPC if needed |
| MJCF include path resolution | Low | Medium | Use absolute path or copy go2.xml + assets into our sim directory |
| Performance (PD + gait at 1kHz) | Low | Low | Pure numpy; no ML inference in the loop |
| Heading drift during long walks | Medium | Low | Acceptable for demo; add heading correction in M2 |
| Agent registers both arm and Go2 skills | Low | Medium | Skills are mode-specific; only register relevant set |

## 13. Out of Scope (Milestone 1)

- Obstacle avoidance / collision detection logic
- SLAM or localization
- Navigation planning (path planning, costmaps)
- RL-trained locomotion policies
- Real Go2 hardware interface (unitree_sdk2py)
- Perception from the Go2 (camera on the dog)
- Arm mounted on Go2 (Go2 + arm combo)
- Multi-robot coordination
- Terrain adaptation (stairs, slopes, rough ground)

## 14. Milestone 1 Success Criteria

1. `python run.py --sim-go2` launches MuJoCo with Go2 in an indoor room
2. The dog stands up on startup
3. User says "往前走两步" -> ToolAgent calls walk(forward, ~1m) -> dog walks forward
4. User says "左转" -> ToolAgent calls turn(left, 90) -> dog turns left
5. User says "坐下" -> ToolAgent calls sit() -> dog sits down
6. Overhead camera renders the scene (visible in viewer or via render())
7. All 10 test contracts pass
8. The dog does not fall over during normal walking (T6)
