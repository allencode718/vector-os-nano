# Development Status — v0.2.0 Feature Wave 1.1

**Session Date:** 2026-03-23  
**Project:** Vector OS Nano SDK  
**Status:** Active development

---

## Wave Status

| Wave | Feature | Status | Target |
|------|---------|--------|--------|
| 1.1 | LLM Memory + Model Router (parallel) | **IN_PROGRESS** | SessionMemory + ModelRouter classes + unit tests |
| 1.2 | Integrate memory + router into Agent | Pending | Modify agent.py, LLM providers |
| 1.3 | Config updates | Pending | default.yaml models section |
| 2.1 | MCP tools + resources | Pending (depends on 1.2) | mcp/tools.py, mcp/resources.py |
| 2.2 | MCP server | Pending | mcp/server.py + integration tests |
| 2.3 | Mount on run.py + config | Pending | CLI flags, pyproject.toml |

---

## Agent Assignments — Wave 1.1

### Alpha (Sonnet 4.6)
- **Task:** Create `vector_os_nano/core/memory.py` + `tests/unit/test_memory.py`
- **Scope:** SessionMemory + MemoryEntry classes (~200 lines code, ~200 lines tests)
- **Status:** Not started
- **Branch:** feat/alpha-session-memory
- **Blocker:** None

### Beta (Sonnet 4.6)
- **Task:** Create `vector_os_nano/llm/router.py` + `tests/unit/test_router.py`
- **Scope:** ModelRouter + ModelSelection classes (~120 lines code, ~150 lines tests)
- **Status:** Not started
- **Branch:** feat/beta-model-router
- **Blocker:** None

### Parallel Independence
- Both tasks are **independent** — no file conflicts expected
- Alpha + Beta can execute simultaneously
- Wave 1.1 completion = both PRs merged

---

## Next Steps (Wave 1.2)

After 1.1 merges:

1. **Gamma:** Modify `llm/claude.py`, `llm/base.py`, `llm/openai_compat.py`
   - Add `model_override` parameter to all LLM methods
   - ~30 lines total changes

2. **Alpha:** Modify `core/agent.py`
   - Replace `_conversation_history` with `SessionMemory` instance
   - Create `ModelRouter` instance in `__init__`
   - Update `_handle_task`, `_handle_chat`, `_handle_query` methods
   - Use router to select model for planning
   - ~80 lines changes

3. **Beta:** Modify `config/default.yaml`
   - Add `models` section with Haiku/Sonnet assignments
   - Update existing integration tests

---

## File Manifest (After v0.2.0 Complete)

New files:
- `vector_os_nano/core/memory.py` — SessionMemory, MemoryEntry
- `vector_os_nano/llm/router.py` — ModelRouter, ModelSelection
- `vector_os_nano/mcp/__init__.py`
- `vector_os_nano/mcp/server.py` — VectorMCPServer
- `vector_os_nano/mcp/tools.py` — Skill-to-MCP conversion
- `vector_os_nano/mcp/resources.py` — World state + camera resources
- `tests/unit/test_memory.py`
- `tests/unit/test_router.py`
- `tests/unit/test_mcp_server.py`
- `tests/unit/test_mcp_tools.py`
- `tests/unit/test_mcp_resources.py`

Modified files:
- `vector_os_nano/core/agent.py` — Use SessionMemory + ModelRouter
- `vector_os_nano/llm/claude.py` — Add model_override param
- `vector_os_nano/llm/base.py` — Add model_override to Protocol
- `vector_os_nano/llm/openai_compat.py` — Add model_override param
- `vector_os_nano/web/app.py` — Mount MCP SSE endpoint
- `run.py` — Add --mcp flag, stdio entry point
- `pyproject.toml` — Add mcp optional dependency
- `config/default.yaml` — Add models + mcp sections

---

## Architecture Changes (v0.2.0)

### SessionMemory (replaces direct `_conversation_history` list)
```python
memory = SessionMemory(max_entries=50)
memory.add_user_message("pick the red cup", entry_type="task")
memory.add_task_result(instruction, execution_result, world_diff)
llm_history = memory.get_llm_history(max_turns=20)  # For LLM API
last_task = memory.get_last_task_context()           # For anaphora
```

**Impact:** Fixes broken task memory, enables cross-task references ("now put it on the left" understands previous task context)

### ModelRouter (selects Haiku vs Sonnet per stage)
```python
router = ModelRouter(config['llm'])
selection = router.for_plan(instruction, world_state)
# selection.model = "haiku" for simple, "sonnet" for complex
llm.plan(..., model_override=selection.model)
```

**Impact:** Reduces cost while maintaining quality (Sonnet only for complex tasks)

### MCP Server (Feature 2)
- Exposes skills as MCP tools (pick, place, home, etc.)
- Meta-tool `natural_language` for full agent pipeline
- Resources: `world://state`, `world://objects`, `camera://overhead`, etc.
- Transports: SSE (FastAPI mount) + stdio (standalone entry)

**Impact:** Claude Desktop can directly control simulated robot

---

## Documentation Status

Files to review after code is written:
- `docs/architecture.md` — Will need new sections for memory.py, router.py, mcp/
- `README.md` — Will need MCP usage examples, Claude Desktop setup guide

**Action:** After Wave 1.1, Scribe will note what needs updating. Updates deferred to after code merge.

---

## Known Risks

| Risk | Severity | Status |
|------|----------|--------|
| MCP SDK API stability | Medium | Mitigation: pin version, wrap in types |
| Memory bloat with long sessions | Low | Mitigation: bounded to 50 entries |
| Model router heuristic inaccuracy | Low | Mitigation: defaults to safe model |
| SSE transport not available | Medium | Mitigation: stdio fallback always works |
| Agent.execute() async/sync mismatch | Medium | Mitigation: already handled in web/app.py |

---

## Collaboration Notes

- **Alpha + Beta Wave 1.1:** No file conflicts expected, run in parallel
- **After merge:** All agents must read Wave 1.1 PRs before starting Wave 1.2
- **Code review:** tdd-guide + code-reviewer agents gate Wave 1.1 PRs
- **Scribe role:** Track status, detect conflicts, update docs after each wave

---

## Session Timeline

- **[scribe] 14:xx** — Initialize status.md, update progress.md
- **[alpha] TBD** — Start SessionMemory implementation
- **[beta] TBD** — Start ModelRouter implementation
- **[code-review] TBD** — Review PRs
- **[scribe] TBD** — Wave 1.1 completion report
