# Development Status — v0.2.0 COMPLETE

**Session Date:** 2026-03-23  
**Project:** Vector OS Nano SDK  
**Status:** v0.2.0 stable release complete, ready for v0.3.0 planning

---

## v0.2.0 Final Completion Summary

### Features Delivered

**Phase 1: LLM Memory + Model Router**
- SessionMemory: Persistent cross-task conversation history (50 entries max)
- ModelRouter: Complexity-driven model selection (Haiku for simple, Sonnet for complex)
- Agent integration: Fixed conversation history reset bug
- 120 new tests passing (78 unit + 42 integration)

**Phase 2: MCP Server**
- MCP tools: 7 skill tools (pick, place, home, scan, detect, open, close) + natural_language meta-tool
- MCP resources: 6 resources (world state, 3 camera views, object list, detailed state)
- VectorMCPServer: Full MCP protocol implementation with stdio transport
- Entry points: --sim, --sim-headless, --hardware modes
- Claude Desktop integration: .mcp.json auto-connect config

**Phase 2.5: Bug Fixes**
- JSON Schema float/int type mapping (was causing Claude API 400 errors)
- Python 3.10+ asyncio.get_event_loop() fix

### Test Metrics
| Category | Count | Status |
|----------|-------|--------|
| v0.2.0 new tests | 120 | PASS |
| v0.1.0 existing tests | 732 | PASS |
| Skipped (ROS2 conditional) | 10 | SKIP |
| **TOTAL** | **852** | **PASS** |

### Files Modified / Created
**New:**
- `vector_os_nano/core/memory.py` (SessionMemory + MemoryEntry)
- `vector_os_nano/llm/router.py` (ModelRouter + ModelSelection)
- `vector_os_nano/mcp/__init__.py`, `__main__.py`, `tools.py`, `resources.py`, `server.py`
- `.mcp.json` (Claude Desktop auto-connect)
- Tests: 5 new unit test modules, 6 new integration tests

**Modified:**
- `vector_os_nano/core/agent.py` (SessionMemory + ModelRouter integration)
- `vector_os_nano/llm/claude.py`, `base.py`, `openai_compat.py` (model_override parameter)
- `config/default.yaml` (models + mcp sections)
- `pyproject.toml` (mcp>=1.0 optional dependency)

---

## MCP Server — Quick Reference

### Auto-Connect (Claude Desktop)
```bash
# .mcp.json already configured, server runs automatically
python -m vector_os_nano.mcp --sim --stdio
```

### Manual Start
```bash
# Simulation with viewer
python -m vector_os_nano.mcp --sim --stdio

# Headless simulation (no viewer)
python -m vector_os_nano.mcp --sim-headless --stdio

# Real hardware mode (SO-101 + RealSense + VLM)
python -m vector_os_nano.mcp --hardware --stdio
```

### Switch Hardware Mode
Edit `.mcp.json` args:
- `"args": ["-m", "vector_os_nano.mcp", "--sim", "--stdio"]` (current — simulation)
- `"args": ["-m", "vector_os_nano.mcp", "--hardware", "--stdio"]` (to switch)

### Tools Available
1. `pick(object)` — Grasp object
2. `place(location)` — Release at location
3. `home()` — Safe home position
4. `scan()` — Move and detect objects
5. `detect(query)` — Identify object by description
6. `open()` — Open gripper
7. `close()` — Close gripper
8. `natural_language(query)` — Full pipeline: plan → execute → report

### Resources Available
- `world://state` — Full robot state (JSON)
- `world://objects` — Detected objects (JSON array)
- `camera://overhead` — Bird's eye view (PNG)
- `camera://left` — Left camera (PNG)
- `camera://right` — Right camera (PNG)

---

## v0.3.0 Next Steps

### Proposed (awaiting Yusen approval)
1. Claude Code agent team integration
2. Real RealSense camera feed
3. Moondream VLM open-vocabulary detection
4. EdgeTAM continuous tracking
5. Documentation: Architecture diagrams, MCP setup guide, API reference

### Agent Readiness
| Agent | Model | Status | Notes |
|-------|-------|--------|-------|
| Lead/Architect | opus | Ready | v0.3.0 spec writing |
| Alpha | sonnet | Ready | Claude Code testing |
| Beta | sonnet | Ready | Claude Code testing |
| Gamma | sonnet | Ready | Claude Code testing |
| QA | — | Ready | Code review for v0.3.0 |
| Scribe | haiku | Ready | Docs tracking |

---

## Known Issues (v0.2.0)

None. All critical bugs fixed.

---

## Documentation Status

**Updated:**
- `progress.md` — Complete v0.2.0 feature list, MCP entry points, CLI/launcher commands
- `agents/devlog/status.md` — This file

**Pending v0.3.0:**
- `README.md` — MCP section, Claude Desktop setup, hardware mode switcher
- `docs/architecture.md` — SessionMemory + ModelRouter + MCP flow diagrams
- `docs/api.md` — MCP tools + resources reference (auto-generated)
- `QUICKSTART.md` — MCP server startup guide (if needed)

---

## Session Notes

- v0.2.0 implementation completed without blockers
- All 120 new tests pass, no regressions in v0.1.0 tests
- MCP module optional—doesn't block existing features
- `.mcp.json` ready for immediate Claude Desktop use
- Ready to transition to v0.3.0 planning
