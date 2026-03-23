# Vector OS Nano SDK — Progress

**Last updated:** 2026-03-22
**Status:** v0.1.0 — SkillFlow protocol + MuJoCo sim + Multi-stage Agent Pipeline + AI Chat

## What Works

- Full NL pipeline: "抓杯子" → classify → plan → execute (scan→detect→pick→place→home) → summarize
- Multi-stage Agent Pipeline: CLASSIFY → ROUTE → PLAN → EXECUTE → ADAPT → SUMMARIZE
- AI Chat (V): multi-turn conversation with Claude Haiku, context-aware (knows robot state + objects)
- MuJoCo simulation: SO-101 with real STL meshes, 6 mesh objects, weld grasping, smooth real-time motion
- Simulated perception: ground-truth object detection, Chinese/English NL queries
- Web Dashboard: localhost:8000, real-time WebSocket chat + status
- Direct commands without LLM: home, scan, open, close (instant)
- Chinese + English natural language
- Live camera viewer: RGB + depth side-by-side, EdgeTAM tracking overlay
- 733+ unit tests passing
- ROS2 integration layer (optional, 5 nodes + launch file)
- Textual TUI dashboard (5 tabs)
- SO-101 arm driver (Feetech STS3215 serial)

## Architecture

### Multi-Stage Agent Pipeline

```
User Input
    |
    v
[Stage 1: MATCH] — @skill alias matching (zero LLM)
    Match + direct=True  → Execute immediately (home, open, close)
    Match + auto_steps   → Expand chain (scan→detect→pick→home)
    Match + complex      → Stage 3 (LLM plan)
    No match             → Stage 2
    |
[Stage 2: CLASSIFY] — Haiku, fast intent detection
    → chat | task | query
    |
[Stage 3: PLAN] — Haiku, task decomposition
    Input: user goal + @skill schemas + world state
    Output: { message: "好的主人...", steps: [...] }
    |
[Stage 4: EXECUTE] — deterministic, no LLM
    Run skills step by step, show progress
    |
[Stage 5: ADAPT] — on failure, retry or explain
    |
[Stage 6: SUMMARIZE] — Haiku, result report
```

### SkillFlow Protocol

All routing is declarative via `@skill` decorator — zero hard-coded command matching:

```python
@skill(aliases=["grab", "抓", "拿"], auto_steps=["scan", "detect", "pick"])
class PickSkill: ...

@skill(aliases=["close", "grip", "夹紧"], direct=True)
class GripperCloseSkill: ...
```

See docs/skill-protocol.md for full specification.

### System Layers

```
vector_os_nano/
├── core/          Agent (multi-stage pipeline), Planner, Executor, WorldModel, Skill protocol
├── llm/           Claude/OpenAI providers, classify/plan/chat/summarize prompts
├── perception/    RealSense camera, Moondream VLM, EdgeTAM tracker, pointcloud
├── hardware/
│   ├── so101/     SO-101 arm driver (Feetech STS3215 serial, Pinocchio IK)
│   └── sim/       MuJoCo simulation (arm, gripper, perception, 6 mesh objects)
├── skills/        pick, place, home, scan, detect
├── cli/           Interactive CLI with AI chat (V), braille logo
├── web/           FastAPI + WebSocket dashboard (localhost:8000)
└── ros2/          Optional ROS2 nodes + launch file (5 nodes)
```

### Config Files

```
config/
├── default.yaml              # SDK defaults (arm, camera, LLM, skills)
├── user.yaml                 # User overrides (API keys, gitignored)
└── agent.md                  # V's system prompt (Identity, Safety, Skills, Behavior)
```

## Launcher Commands

```bash
# ─── Real Hardware ───
python run.py                  # CLI mode (readline + AI chat)
python run.py --dashboard      # Textual TUI dashboard
python run.py -v               # Verbose mode (show all skill logs)

# ─── MuJoCo Simulation ───
python run.py --sim            # Sim with MuJoCo viewer + CLI
python run.py --sim-headless   # Sim without viewer (headless)
python run.py --sim -d         # Sim + TUI dashboard

# ─── Web Dashboard ───
python run.py --web            # Web dashboard at localhost:8000
python run.py --web --sim      # Web + MuJoCo sim

# ─── Testing ───
python run.py --no-arm         # No arm hardware
python run.py --no-perception  # No camera/perception
```

## CLI Commands

```
vector> 你好                    # AI chat (V responds)
vector> 桌上有什么              # Query (scan + detect + V describes)
vector> 抓杯子                  # Task (plan + execute + summarize)
vector> 随便做点什么            # Creative task (LLM plans multi-step)
vector> home                    # Direct command (instant, no LLM)
vector> open / close            # Gripper control (instant)
vector> scan                    # Move to scan position (instant)
vector> detect                  # Detect all objects (instant)
vector> status                  # Show robot status + objects
vector> world                   # Show world model JSON
vector> help                    # Show all commands
vector> q                       # Quit
```

## MuJoCo Simulation

- SO-101 arm with 13 real STL meshes from CAD model
- 6 graspable objects: banana, mug, bottle, screwdriver, duck, lego brick
- Weld-constraint grasping (reliable, no contact/friction issues)
- Smooth real-time motion with linear interpolation + 60fps viewer sync
- Pick sequence: open → approach → grasp → lift → rotate 90deg → drop → home
- Simulated perception: ground-truth positions, NL queries (Chinese + English)
- Jacobian-based IK solver (< 2mm accuracy)
- Camera rendering for future VLM integration

## AI Agent (V)

- Name: V, calls user "主人"
- System prompt: config/agent.md (Identity, Safety, Communication, Skills, Behavior)
- Multi-turn conversation memory (30 turns)
- Context-aware: knows robot mode, arm status, gripper state, objects on table
- Intent classification: chat vs task vs direct vs query
- Task planning: decomposes complex instructions into skill sequences
- Post-execution summarization: reports results to user

## TODO (Next Priorities)

### 1. ~~MuJoCo Simulation~~ DONE
### 2. ~~Multi-stage Agent Pipeline~~ DONE
### 3. ~~AI Chat (V)~~ DONE

### 4. ~~SkillFlow Protocol~~ DONE
- @skill decorator with aliases, direct, auto_steps
- Alias-based routing replaces all hard-coded commands
- GripperOpen/Close as proper skill classes

### 5. LLM Agent Enhancements
- Multi-turn planning memory across commands
- Model auto-select (Haiku for simple, Sonnet for complex)
- MCP server to expose skills externally

### 6. Pick Accuracy
- Re-calibration, hand-eye calibration
- Grasp success detection via servo current/load

### 6. Web Dashboard Enhancement
- MuJoCo camera render in browser
- 3D joint visualization
- Settings panel

### 7. Merge & Release
- Merge feat/vector-os-nano-python-sdk → master
- Tag v0.1.0 release, PyPI publish
