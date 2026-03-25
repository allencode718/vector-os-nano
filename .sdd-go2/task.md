# Go2 MuJoCo Integration — Task List

## Execution Status
- Total tasks: 7
- Completed: 0
- In progress: 0
- Pending: 7

## Tasks

### Task 1: WorldModel base fields + Agent base param
- **Status**: [ ] pending
- **Agent**: gamma
- **Depends**: none
- **Package**: core
- **Input**: plan.md Section 3 Module D
- **Output**: modified core/world_model.py, core/agent.py
- **Test file**: tests/unit/test_world_model_base.py
- **TDD Deliverables**:
  - RED: test RobotState has position_xy/heading, update_robot_state accepts them, to_dict/from_dict round-trip
  - RED: test Agent(base=mock) stores base, _build_context passes base
  - GREEN: add fields to RobotState, update valid_fields, add base param to Agent
  - REFACTOR: ensure all existing tests still pass
- **Acceptance Criteria**:
  - [ ] RobotState.position_xy and .heading exist with defaults (0,0) and 0.0
  - [ ] update_robot_state(position_xy=, heading=) works
  - [ ] Agent(base=obj) stores and passes to SkillContext
  - [ ] All existing tests pass (no regression)
- **Verify**: `cd ~/Desktop/vector_os_nano && source .venv/bin/activate && python -m pytest tests/unit/test_world_model.py tests/unit/test_world_model_base.py tests/unit/test_agent.py -x -q`

### Task 2: MuJoCoGo2 core class — lifecycle + state queries + PD posture
- **Status**: [ ] pending
- **Agent**: alpha
- **Depends**: none
- **Package**: hardware/sim
- **Input**: plan.md Section 3 Module A, go2-convex-mpc source
- **Output**: vector_os_nano/hardware/sim/mujoco_go2.py
- **Test file**: tests/unit/test_mujoco_go2.py
- **TDD Deliverables**:
  - RED: T1 (lifecycle), T2 (stand), T5 (sit), T9 (PD), T10 (import)
  - GREEN: MuJoCoGo2 class with connect/disconnect/get_position/get_heading/get_joint_positions/stand/sit/lie_down
  - REFACTOR: clean up, extract constants
- **Acceptance Criteria**:
  - [ ] connect() loads MuJoCo + Pinocchio models
  - [ ] get_position() returns (x,y,z), get_heading() returns float
  - [ ] stand() brings dog to ~0.27m height
  - [ ] sit() lowers dog below standing height
  - [ ] PD controller tracks target joints within 0.1 rad
- **Verify**: `cd ~/Desktop/vector_os_nano && source .venv/bin/activate && python -m pytest tests/unit/test_mujoco_go2.py -x -q -k "not walk and not turn and not stability"`

### Task 3: MuJoCoGo2 walk() — MPC locomotion integration
- **Status**: [ ] pending
- **Agent**: alpha
- **Depends**: Task 2
- **Package**: hardware/sim
- **Input**: plan.md Section 3 Module A walk() loop, go2-convex-mpc ex00_demo.py
- **Output**: walk() method added to mujoco_go2.py
- **Test file**: tests/unit/test_mujoco_go2.py (additional tests)
- **TDD Deliverables**:
  - RED: T3 (walk forward), T4 (turn), T6 (stability)
  - GREEN: implement walk(vx, vy, vyaw, duration) with full MPC control loop
  - REFACTOR: extract _mpc_step helper, tune velocity clamps
- **Acceptance Criteria**:
  - [ ] walk(0.3, 0, 0, 2) moves robot forward > 10cm
  - [ ] walk(0, 0, 0.5, 2) changes heading > 0.3 rad
  - [ ] walk(0.3, 0, 0, 5) keeps robot upright (z > 0.15)
  - [ ] Viewer syncs during walk (no freeze)
- **Verify**: `cd ~/Desktop/vector_os_nano && source .venv/bin/activate && python -m pytest tests/unit/test_mujoco_go2.py -x -q`

### Task 4: Go2 Skills — walk, turn, stance
- **Status**: [ ] pending
- **Agent**: beta
- **Depends**: none (uses mock base for testing)
- **Package**: skills/go2
- **Input**: spec.md Section 7, plan.md Section 3 Module C
- **Output**: skills/go2/__init__.py, walk.py, turn.py, stance.py
- **Test file**: tests/unit/test_go2_skills.py
- **TDD Deliverables**:
  - RED: T7 (walk skill execute with mock base), turn skill, stance skills
  - GREEN: implement all 5 skills with @skill decorator
  - REFACTOR: ensure parameter validation, error handling
- **Acceptance Criteria**:
  - [ ] WalkSkill.execute calls context.base.walk with correct args
  - [ ] TurnSkill.execute maps direction/angle to vyaw/duration
  - [ ] StandSkill/SitSkill/LieDownSkill are direct=True
  - [ ] All skills return SkillResult with appropriate result_data
  - [ ] get_go2_skills() returns list of 5 skill instances
- **Verify**: `cd ~/Desktop/vector_os_nano && source .venv/bin/activate && python -m pytest tests/unit/test_go2_skills.py -x -q`

### Task 5: run.py --sim-go2 + ToolAgent Go2 prompt
- **Status**: [ ] pending
- **Agent**: gamma
- **Depends**: Task 1, Task 2, Task 4
- **Package**: run.py, core/tool_agent.py
- **Input**: plan.md Section 3 Module D
- **Output**: modified run.py, modified core/tool_agent.py
- **TDD Deliverables**:
  - RED: test _init_sim_go2 returns (None, None, None, None, base), test ToolAgent prompt includes Go2 info
  - GREEN: implement _init_sim_go2, add --sim-go2 flag, adapt _build_system_prompt
  - REFACTOR: ensure CLI help text is clear
- **Acceptance Criteria**:
  - [ ] `python run.py --sim-go2` launches Go2 sim with viewer
  - [ ] `python run.py --sim-go2-headless` launches headless
  - [ ] Agent constructed with base=go2, arm=None
  - [ ] Go2 skills registered (walk, turn, stand, sit, lie_down)
  - [ ] ToolAgent system prompt shows Go2 mode + position + heading
- **Verify**: `cd ~/Desktop/vector_os_nano && source .venv/bin/activate && python run.py --sim-go2-headless --help` (smoke test)

### Task 6: Integration testing — full pipeline
- **Status**: [ ] pending
- **Agent**: beta
- **Depends**: Task 3, Task 5
- **Package**: tests
- **Input**: all previous deliverables
- **Output**: tests/integration/test_go2_integration.py
- **TDD Deliverables**:
  - Agent(base=go2) + Go2 skills + execute("walk forward")
  - ToolAgent with Go2 base generates correct system prompt
  - Full walk + turn + sit sequence without crash
- **Acceptance Criteria**:
  - [ ] Agent with Go2 base registers 5 skills
  - [ ] execute("walk forward") triggers WalkSkill → base.walk()
  - [ ] Sequential walk → turn → sit completes without error
- **Verify**: `cd ~/Desktop/vector_os_nano && source .venv/bin/activate && python -m pytest tests/integration/test_go2_integration.py -x -q`

### Task 7: pyproject.toml + dependency docs
- **Status**: [ ] pending
- **Agent**: gamma
- **Depends**: Task 6
- **Package**: root
- **Input**: plan.md Section 6.4
- **Output**: modified pyproject.toml, updated progress.md
- **TDD Deliverables**:
  - Add `go2` optional extras to pyproject.toml
  - Update progress.md with Go2 status
- **Acceptance Criteria**:
  - [ ] `pip install -e ".[go2]"` installs casadi and pin
  - [ ] progress.md reflects Go2 milestone 1 status
- **Verify**: `cd ~/Desktop/vector_os_nano && source .venv/bin/activate && pip install -e ".[go2]"`

## Dependency Graph
```
Task 1 (world model + agent base) ──┐
Task 2 (MuJoCoGo2 lifecycle+PD) ────┤──> Task 5 (run.py + prompt) ──> Task 6 (integration) ──> Task 7 (deps)
Task 4 (Go2 skills) ────────────────┘
Task 2 ──> Task 3 (MPC walk)  ──────────> Task 6
```

## Execution Waves
| Wave | Tasks | Agents | Gate |
|------|-------|--------|------|
| 1 | T1, T2, T4 | Gamma, Alpha, Beta | unit tests pass |
| 2 | T3, T5 | Alpha, Gamma | unit tests pass |
| 3 | T6 | Beta | integration tests pass |
| 4 | T7 | Gamma | deps verified |
