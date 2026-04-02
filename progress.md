# Vector OS Nano SDK — Progress

**Last updated:** 2026-04-02
**Version:** v0.7.0-dev
**Branch:** master

---

## Current: Go2 VLM + SceneGraph + Auto-Look + RViz Visualization

### Architecture
```
User
  ├── "巡逻全屋" ──→ MobileAgentLoop ──→ [navigate→look→navigate→look→...]
  ├── "去厨房看看" ──→ TaskPlanner ──→ navigate(kitchen) + look()
  ├── RViz teleop ──→ /joy ──→ bridge (direct velocity, 0.8 m/s)
  └── /goal_point ──→ FAR planner ──→ /way_point ──→ localPlanner

MuJoCoGo2 (convex MPC, 1kHz)
  ├── get_camera_frame() ──→ Go2VLMPerception (GPT-4o via OpenRouter)
  │     ├── describe_scene() → SceneDescription (summary, objects, room_type)
  │     ├── identify_room() → RoomIdentification (room, confidence)
  │     └── find_objects() → [DetectedObject]
  ├── publishes: /state_estimation (200Hz), /registered_scan (10Hz)
  ├── publishes: /camera/image (5Hz, 320x240), /speed (2Hz)
  └── SceneGraph: 3-layer (rooms→viewpoints→objects), persistent YAML

Skills (12 total):
  walk, turn, stand, sit, lie_down, navigate, explore,
  where_am_i, stop, look, describe_scene, patrol

RViz Visualization:
  ├── Room fills (semi-transparent, color-coded per room)
  ├── Room borders (LINE_STRIP outlines, visited=bright, unvisited=dim)
  ├── Room labels (name + visit count + coverage% + object count)
  ├── Viewpoint spheres (teal-green) + FOV cones (TRIANGLE_LIST)
  ├── Object cubes (category-colored) + text labels
  ├── Robot arrow (teal) + footprint cylinder
  ├── Trajectory trail (LINE_STRIP, grey→teal fade)
  └── Nav goal beacon (red cylinder + disc + "GOAL" label)
```

### Harness Results
| Suite | Result | Details |
|-------|--------|---------|
| Locomotion (L0-L4) | **26/26** | physics → navigation |
| Agent+Go2 | **5/5** | walk, turn, stand, sit, skills |
| VLM API (L0) | **4/4** | GPT-4o reachable, JSON parse, latency, cost |
| Camera→VLM (L1) | **6/6** | MuJoCo frame → GPT-4o → scene description |
| Scene Skills (L2) | **17/17** | LookSkill, DescribeSceneSkill (mock VLM) |
| Task Planning (L3) | **18/18** | fallback planner, JSON parse, Chinese rooms |
| E2E Patrol (L4) | **4/4** | 2-room patrol, real API, spatial memory |
| VLM Accuracy (L5) | **1-2/8** | Diagnostic: MuJoCo rendering limits room ID |
| ToolAgent (L5) | **6/6** | 中文指令, navigate, look, multi-turn context |
| Robustness (L6) | **32/32** | VLM errors, nav edge cases, spatial memory |
| SceneGraph (L7) | **55/55** | 3-layer graph, viewpoints, coverage, merge, persist |
| RViz Markers (L8) | **38/38** | room fills/borders, FOV cones, trajectory, nav goal |
| Proxy E2E (L9) | **26/26** | Go2ROS2Proxy camera → LookSkill → SceneGraph |
| Persistence (L9) | **28/28** | SceneGraph save/load lifecycle, edge cases |
| Auto-Look (L10) | **8/8** | ExploreSkill + VLM auto-observe on new room |
| Mobile Loop (L11) | **14/14** | LLM planning, fallback, execution, auto-observe |
| **Total harness** | **236+** | 0 collection errors |

### What's New (v0.7.0-dev)
- **VLM Auto-Look on Explore**: ExploreSkill automatically calls VLM when entering a new room, records observations in SceneGraph with viewpoint-aware positioning
- **RViz Visualization Upgrade**: Apple-quality markers — color-coded rooms, FOV cone fans, trajectory trail with gradient fade, nav goal beacon, category-colored object cubes
- **SceneGraph Persistence**: Auto-load on startup from ~/.vector_os_nano/scene_graph.yaml, auto-save on exit. Rooms, viewpoints, objects survive across sessions
- **MobileAgentLoop LLM Fix**: Fixed chat() interface mismatch (was calling messages= keyword, now uses user_message/system_prompt). Auto-observe uses SceneGraph viewpoint-aware API
- **Proxy Camera E2E Verified**: Go2ROS2Proxy._camera_cb → get_camera_frame → LookSkill → VLM → SceneGraph pipeline fully tested
- **Test Harness Expanded**: L8 from 17→38 tests, new L9 (proxy+persistence), L10 (auto-look), L11 (mobile loop). 236+ tests total

### What Works
- Go2 walks with unitree convex MPC (auto-detected, sinusoidal fallback)
- Livox MID360 simulation: 30 deg tilt, -7/+52 deg FOV, 10k+ points/scan
- Vector Nav Stack: localPlanner, pathFollower, terrain_analysis, FAR planner
- TARE autonomous exploration (frontier-based TSP)
- Camera RGB from MuJoCo → GPT-4o scene understanding
- VLM room identification with confidence scores
- Multi-room patrol with spatial memory recording
- Agent SDK: natural language → Go2 skills (12 skills)
- SceneGraph persists across sessions (rooms, viewpoints, objects)
- Auto-look during exploration captures VLM scene at each new room
- MobileAgentLoop plans via LLM with SceneGraph context
- RViz shows room boundaries, FOV cones, trajectory, nav goal, objects

### Known Issues
1. FAR planner publishes /way_point but not /global_path (graph_decoder issue)
2. Camera depth rendering intermittent (MuJoCo API)
3. VLM look via proxy `/camera/image` — tested in mock, needs real ROS2 E2E verification
4. L5 VLM accuracy limited by MuJoCo room texture quality

### TODO
- [ ] Live ROS2 E2E test: start launch_vnav.sh + proxy, verify real /camera/image frames
- [ ] Improve VLM room accuracy (higher res, multi-angle, better scene textures)
- [ ] Go2ROS2Proxy: add sit/lie_down via ROS2 service (currently just zero velocity)
- [ ] MobileAgentLoop: test with real OpenRouter API for LLM planning
- [ ] SceneGraph: add room connectivity edges (door graph) for smarter navigation

### Scripts
| Script | Purpose |
|--------|---------|
| `./scripts/launch_explore.sh` | Autonomous exploration (TARE + VNav) |
| `./scripts/launch_vnav.sh` | Vector Nav Stack + RViz (manual/goal) |
| `./scripts/launch_nav2.sh --rviz` | Nav2 + AMCL alternative |
| `./scripts/launch_slam.sh` | SLAM real-time mapping |
| `.venv-nano/bin/python3 run.py --sim-go2` | Agent mode (NL + VLM) |
| `.venv-nano/bin/python3 -m pytest tests/harness/ -v` | Full harness |
