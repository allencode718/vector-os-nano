# Agent Status — Vector OS Nano SDK

**Session:** 2026-03-21 | **Phase:** Tuning | **696 TESTS PASSING**

---

## Executive Summary

Full SDK complete and functional on hardware. Pick pipeline end-to-end: NL command → LLM planning → sensor perception → IK → arm motion → gripper control. All 696 tests passing. Current focus: empirical pick accuracy tuning and calibration refinement. Next major work: Skill Manifest Protocol (ADR-002) for alias-based command routing.

| Agent | Status | Current Work | Branch | Notes |
|-------|--------|--------------|--------|-------|
| Lead (Opus) | idle | — | — | Architecture approved, awaiting next phase |
| Alpha (Sonnet) | idle | — | — | All wave tasks complete |
| Beta (Sonnet) | idle | — | — | All wave tasks complete |
| Gamma (Sonnet) | idle | — | — | All wave tasks complete |
| Scribe (Haiku) | active | Documentation update | dev | Status + architecture docs |

**Test Status:** 696/696 passing (100%), coverage 85%+

---

## Quality Metrics

| Metric | Value |
|--------|-------|
| Test success rate | 100% (696/696) |
| Code coverage | 85% |
| Source files | 54+ |
| Lines of code | 10,000+ |
| Protocols defined | 5 |
| Regressions | 0 |
| Days in active development | 1 |

---

## Current Phase: Tuning

### Pick Reliability (In Progress)
- Empirical XY offsets calibrated for workspace region
- Z calibration simplified (pose-dependent, all objects at z=0.005m)
- Gripper asymmetry compensation: left/right/center branches
- Test data: 50+ successful picks on battery-like objects

### Outstanding Issues
- Look-then-move correction disabled (calibration is pose-dependent)
- World model cleared after each pick (prevents stale position data)
- No grasp success detection yet (servo current feedback not implemented)
- Camera serial hardcoded (335122270413)

---

## Completed Work

### Wave 1 (Foundation) — 281 tests
- Core types, protocols, world model
- Hardware drivers (SO101Arm, SO101Gripper)
- Simulation layer (PyBullet)

### Wave 2 (Intelligence) — +171 tests (452 total)
- LLM integration (Claude, OpenAI, Ollama)
- Perception pipeline (RealSense, VLM, EdgeTAM, 3D)
- Skill framework + registry

### Wave 3 (Integration) — +87 tests (539 total)
- Agent executor (planning + dispatch)
- CLI (readline shell, calibration wizard)
- ROS2 integration (5 nodes + launch file)

### Wave 4 (Polish + Sim) — +103 tests (642 total)
- Textual TUI dashboard
- PyBullet arm simulation
- Documentation + examples

### Post-Wave 4 — +54 tests (696 total)
- Background EdgeTAM tracking
- Dashboard fixes
- Calibration refinement

---

## Next Phase: Skill Manifest Protocol (ADR-002)

Starting implementation:
1. **Phase 1:** YAML skill registry with aliases
2. **Phase 2:** LLM context enrichment (available skills → prompt injection)
3. **Phase 3:** Dynamic skill discovery + routing
4. **Phase 4:** Multi-agent skill coordination (ROS2 integration)

See `docs/ADR-002-skill-manifest-protocol.md` for design details.

---

## Blockers

None currently. All hardware interfaces working, calibration tuning ongoing, next major feature (Skill Manifest) ready to begin.

---

## File Dependencies

- `vector_os/core/agent.py` — main Agent class, executor, world model
- `vector_os/hardware/so101/` — SO-101 arm driver
- `vector_os/perception/pipeline.py` — camera + VLM + tracking orchestrator
- `vector_os/skills/pick.py` — full pick skill implementation
- `vector_os/cli/simple.py` — readline shell
- `config/workspace_calibration.yaml` — calibration matrix (gitignored)
