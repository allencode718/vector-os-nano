# Vector OS Nano SDK — Progress

**Last updated:** 2026-04-02
**Version:** v0.9.0-dev
**Branch:** master

---

## Current: GroundingDINO Object Detection + RGBD Positioning

### Sensor Configuration (sim-to-real)
```
Unitree Go2 quadruped
  ├── Livox MID-360 LiDAR  → /registered_scan (10Hz, 10k+ pts)
  │     30° tilt, -7/+52° FOV, sim: MuJoCo raycasting
  └── RealSense D435 RGBD  → /camera/image (5Hz, RGB8 320x240)
        69° HFOV              → /camera/depth (5Hz, 32FC1 320x240)
        sim: MuJoCo depth renderer, real: rs2 aligned_depth_to_color
```

### Architecture
```
User
  ├── "巡逻全屋" ──→ MobileAgentLoop ──→ [navigate→look→navigate→look→...]
  ├── "去厨房看看" ──→ TaskPlanner ──→ navigate(kitchen) + look()
  ├── RViz teleop ──→ /joy ──→ bridge (direct velocity, 0.8 m/s)
  └── /goal_point ──→ FAR planner ──→ /way_point ──→ localPlanner

Subprocess (launch_vnav.sh)
  MuJoCoGo2 (convex MPC, 1kHz)
  Go2VNavBridge (200Hz odom, 10Hz scan, 5Hz RGBD)
  localPlanner + pathFollower + terrainAnalysis + FAR planner
  sensorScanGeneration → /state_estimation_at_scan → TARE

Agent Process (vector-cli / run.py)
  Go2ROS2Proxy ←→ ROS2 topics ←→ Bridge
  ├── get_camera_frame()  → VLM (GPT-4o via OpenRouter)
  │     describe_scene() → SceneDescription
  │     identify_room()  → RoomIdentification
  ├── get_depth_frame()   → depth_projection.project_center_to_world()
  │     pixel + depth + D435 intrinsics + robot pose → world (x,y,z)
  └── SceneGraph: 3-layer (rooms→viewpoints→objects), persistent YAML
        objects positioned via depth projection (sim-to-real)

Skills (12 total):
  walk, turn, stand, sit, lie_down, navigate, explore,
  where_am_i, stop, look, describe_scene, patrol

RViz Visualization (anti-flicker: 3s interval, 5s lifetime):
  ├── Room fills (semi-transparent, color-coded per room)
  ├── Room borders (LINE_STRIP outlines, visited=bright, unvisited=dim)
  ├── Room labels (name + visit count + coverage% + object count)
  ├── Viewpoint spheres (teal-green) + FOV cones (TRIANGLE_LIST)
  ├── Object cubes (depth-projected position, category-colored) + labels
  ├── Robot arrow (teal) + footprint cylinder
  ├── Trajectory trail (LINE_STRIP, grey→teal fade)
  └── Nav goal beacon (red cylinder + disc + "GOAL" label)
```

### Object Positioning Pipeline (sim-to-real)
```
D435 RGBD camera
  ├── RGB → GroundingDINO → per-object bbox (u1,v1,u2,v2)  [IN PROGRESS]
  │         "sofa" at (50,80,200,180), "table" at (220,90,300,170)
  │
  └── Depth → depth at each bbox center → per-object metric depth
                  ↓
      depth_to_world(bbox_center, depth, D435_intrinsics, robot_pose)
                  ↓
      ObjectNode(label="sofa", x=2.1, y=1.3)  ← accurate per-object position
      ObjectNode(label="table", x=3.8, y=2.0)
                  ↓
      RViz markers at correct world locations

Current (until GroundingDINO integration complete):
  RGB → VLM (gpt-4o-mini) → object names (no positions)
  Objects clustered ~2m in front of viewpoint heading
```

### GroundingDINO Integration Status
- [x] object_detector.py module created (detect_objects + detect_and_project)
- [x] D435 depth_projection.py (pixel→camera→world transforms)
- [ ] Install torch + transformers in .venv-nano (running)
- [ ] Test GroundingDINO model loading + inference on MuJoCo frames
- [ ] Wire into LookSkill: replace VLM object list with GroundingDINO detections
- [ ] Wire into auto-look: per-object positioning during exploration
- [ ] Harness tests: L14 detection + projection accuracy

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
| TARE Chain (L12) | **20/20** | wander interval, duty cycle, QoS compat |
| Depth Projection (L13) | **24/24** | D435 intrinsics, pixel→world, center_depth |
| **Total harness** | **280+** | 0 collection errors |

### What's New (v0.9.0-dev)
- **GroundingDINO Integration (in progress)**: Open-vocabulary object detector for per-object bounding boxes + depth → accurate world positioning. Model: grounding-dino-tiny on RTX 5080.
- **VLM Fix**: Switched to gpt-4o-mini + image resize to 160px (was timing out on OpenRouter with large base64). 1-2s response now.
- **RealSense D435 RGBD Simulation**: MuJoCoGo2 renders aligned RGB+depth via MuJoCo depth renderer. Same `get_rgbd_frame()` interface for sim and real.
- **Depth-Based Object Positioning**: Objects placed at world coordinates computed from D435 depth + camera intrinsics + robot pose. Replaces heading-based guessing.
- **depth_projection.py**: D435 intrinsics, `pixel_to_camera()`, `camera_to_world()`, `center_depth()`, `project_center_to_world()`. Sim-to-real compatible.
- **Anti-Flicker Markers**: Publish interval 1Hz→3Hz, marker lifetime=5s, hash-based change detection. Only re-publish when scene graph changes.
- **TARE Wander Fix**: Exploration loop sends velocity every 0.8s (was 2.0s), 62.5% duty cycle. TARE gets continuous scan data instead of starving.
- **`/clear_memory` Command**: Reset scene graph and delete persist file from CLI.
- **Object Position Fix**: Markers use depth-projected coords (priority 1), viewpoint heading projection (priority 2), room center (priority 3). All clamped to room bounds.

### What Works
- Go2 walks with unitree convex MPC (auto-detected, sinusoidal fallback)
- Livox MID360 + RealSense D435 simulation (LiDAR + RGBD)
- Vector Nav Stack: localPlanner, pathFollower, terrain_analysis, FAR planner
- TARE autonomous exploration with continuous wander velocity
- RGBD camera → GPT-4o scene understanding + depth object positioning
- VLM room identification with confidence scores
- Multi-room patrol with spatial memory recording
- Agent SDK: natural language → Go2 skills (12 skills)
- SceneGraph persists across sessions (rooms, viewpoints, objects)
- Auto-look during exploration: RGBD capture + VLM + depth projection at each new room
- MobileAgentLoop plans via LLM with SceneGraph context
- RViz: color-coded rooms, FOV cones, trajectory, objects at depth-projected positions
- `/clear_memory` to reset scene graph

### Known Issues
1. FAR planner publishes /way_point but not /global_path (graph_decoder issue)
2. TARE may still struggle with viewpoint generation in tight spaces (MuJoCo house geometry)
3. VLM accuracy limited by MuJoCo room texture quality (L5 diagnostic)
4. Depth projection accuracy depends on MuJoCo depth buffer fidelity

### TODO
- [ ] Verify TARE produces waypoints with wander fix (check /tmp/vector_tare.log)
- [ ] Improve VLM room accuracy (higher res, multi-angle, better scene textures)
- [ ] Go2ROS2Proxy: sit/lie_down via ROS2 service (currently just zero velocity)
- [ ] SceneGraph: room connectivity edges (door graph) for smarter navigation
- [ ] Real D435 driver integration (rs2 → /camera/image + /camera/depth)

### CLI Commands
| Command | Purpose |
|---------|---------|
| `/clear_memory` | Reset scene graph, delete persist file |
| `/model <name>` | Switch LLM model (OpenRouter) |
| `/status` | Show hardware, tools, session info |
| `/tools` | List all registered tools |
| `/help` | Show all commands |

### Scripts
| Script | Purpose |
|--------|---------|
| `./scripts/launch_explore.sh` | Autonomous exploration (TARE + VNav) |
| `./scripts/launch_vnav.sh` | Vector Nav Stack + RViz (manual/goal) |
| `./scripts/launch_nav2.sh --rviz` | Nav2 + AMCL alternative |
| `./scripts/launch_slam.sh` | SLAM real-time mapping |
| `.venv-nano/bin/python3 run.py --sim-go2` | Agent mode (NL + VLM) |
| `.venv-nano/bin/python3 -m pytest tests/harness/ -v` | Full harness |
