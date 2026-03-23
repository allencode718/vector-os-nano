# Vector OS Nano SDK — Progress

**Last updated:** 2026-03-23
**Current version:** v0.1.0 (stable)
**In development:** v0.2.0 — LLM Memory + Model Router + MCP Server

## v0.2.0 Task Completion

- [x] `llm/router.py` — ModelRouter + ModelSelection (Beta, 2026-03-23)
  - Heuristic complexity scoring (5 rules, score >= 2 → complex)
  - SPATIAL_WORDS, MULTI_ACTION_PATTERNS (EN + ZH)
  - for_classify / for_plan / for_chat / for_summarize
  - 34 unit tests passing (tests/unit/test_router.py)

## v0.1.0 — Stable

Complete and working:
- Full NL pipeline: "抓杯子" → classify → plan → execute → summarize
- Multi-stage Agent Pipeline: MATCH → CLASSIFY → PLAN → EXECUTE → ADAPT → SUMMARIZE
- AI Chat (V): multi-turn conversation with Claude Haiku, context-aware
- SkillFlow protocol: declarative @skill decorator routing
- MuJoCo simulation: SO-101 with 13 STL meshes, 6 graspable objects
- Simulated perception: ground-truth object detection, Chinese/English NL queries
- Web Dashboard: localhost:8000, real-time WebSocket chat
- Direct commands (zero LLM): home, scan, open, close
- 733+ unit tests passing
- ROS2 integration layer (optional, 5 nodes)
- Textual TUI dashboard (5 tabs)
- SO-101 hardware driver (Feetech STS3215 serial + Pinocchio IK)

## v0.2.0 — In Development

Two features being built:

### Feature 1: LLM Memory + Model Routing
- SessionMemory class: persistent cross-task conversation memory
- ModelRouter class: auto Haiku/Sonnet selection (simple vs complex tasks)
- Fixes broken task memory (currently resets between commands)
- Enables anaphora resolution ("now put it on the left")
- Plan: docs/plan-llm-memory-mcp.md (Section 1)

### Feature 2: MCP Server
- Expose skills via Model Context Protocol (stdio + SSE transports)
- Claude Desktop can directly control simulated robot
- World state and camera resources as MCP resources
- Builds on Feature 1 memory for cross-task context
- Plan: docs/plan-llm-memory-mcp.md (Section 2)

---

## Architecture

### Multi-Stage Agent Pipeline

```
User Input
    |
    v
[Stage 1: MATCH]   — @skill alias matching (zero LLM)
[Stage 2: CLASSIFY] — Haiku intent detection (chat/task/query)
[Stage 3: PLAN]     — Model router selects Haiku/Sonnet
[Stage 4: EXECUTE]  — Deterministic step execution
[Stage 5: ADAPT]    — Retry/explain on failure
[Stage 6: SUMMARIZE] — Haiku result report
```

### SkillFlow Protocol

All command routing via `@skill` decorator:

```python
@skill(aliases=["grab", "抓"], auto_steps=["scan", "detect", "pick"])
class PickSkill: ...

@skill(aliases=["close", "夹紧"], direct=True)
class GripperCloseSkill: ...
```

### System Layers

```
vector_os_nano/
├── core/           Agent, Executor, WorldModel, Skill protocol
├── llm/            Claude/OpenAI providers, prompts
├── perception/     RealSense, Moondream VLM, EdgeTAM tracker
├── hardware/
│   ├── so101/      SO-101 arm driver (Feetech serial, Pinocchio)
│   └── sim/        MuJoCo simulation
├── skills/         pick, place, home, scan, detect, gripper
├── cli/            Interactive CLI with Rich + prompt_toolkit
├── web/            FastAPI + WebSocket dashboard
├── mcp/            (v0.2.0) MCP tools + resources
└── ros2/           Optional ROS2 nodes
```

---

## Launcher Commands

Real hardware:
```bash
python run.py                  # CLI mode
python run.py --dashboard      # TUI dashboard
python run.py -v               # Verbose
```

MuJoCo simulation:
```bash
python run.py --sim            # With viewer
python run.py --sim-headless   # Headless
python run.py --sim -d         # With TUI
python run.py --web --sim      # Web dashboard
```

Testing:
```bash
python run.py --no-arm         # No hardware
python run.py --no-perception  # No camera
```

---

## CLI Commands

```
vector> 你好                    # Chat
vector> 桌上有什么              # Query + detection
vector> 抓杯子                  # Task (LLM plan)
vector> home                    # Direct command
vector> open / close            # Gripper
vector> scan / detect           # Move / perceive
vector> status                  # Show state
vector> world                   # Show objects
vector> help / q                # Help / quit
```

---

## MuJoCo Simulation

- SO-101 arm: 13 real STL meshes from CAD
- 6 graspable objects: banana, mug, bottle, screwdriver, duck, lego
- Weld-constraint grasping (reliable)
- Smooth real-time motion + 60fps viewer sync
- Simulated perception: ground-truth positions
- Jacobian IK solver (< 2mm accuracy)
- Camera rendering for renders

---

## AI Agent (V)

- Name: V, calls user "主人"
- System prompt: config/agent.md
- (v0.1.0) 30-turn conversation memory
- (v0.2.0) SessionMemory + cross-task continuity
- Context-aware: knows robot mode, gripper state, visible objects
- Task planning: decomposes to skill sequences
- Post-execution summarization
