# Vector OS Nano SDK — Progress

**Last updated:** 2026-04-05
**Version:** v1.3.0-dev (sim-to-real ready)
**Branch:** master

## Architecture

```
vector-cli (agent process)          launch_explore.sh (subprocess)
  LLM + SceneGraph + VLM              MuJoCoGo2 (convex MPC, 1kHz)
  Go2ROS2Proxy ◄── ROS2 ──►           Go2VNavBridge (200Hz odom)
  12 Go2 skills                        localPlanner + FAR + TARE
                                       terrainAnalysis + RViz
```

## Navigation Pipeline (sim-to-real ready)

```
explore:
  TARE autonomous → VLM identifies rooms → SceneGraph learns centers
  Room transitions → SceneGraph learns door positions (add_door)

proxy.navigate_to(x, y):
  Phase 1 (5s): send /goal_point, detect FAR /way_point
  Phase 2: FAR V-Graph routing (/goal_point only, FAR publishes /way_point)
  Phase 3: door-chain fallback (SceneGraph BFS → /way_point → localPlanner)

No hardcoded coordinates. SceneGraph is the only map source.
Navigate without explore → "explore first" error.
```

## SceneGraph (3-layer spatial memory)
- Rooms: center positions learned via running average during explore
- Doors: transition positions learned when VLM detects room change
- Objects: VLM-detected items with world coordinates
- Pathfinding: BFS on room adjacency for door-chain navigation
- Persistence: YAML save/load (~/.vector_os_nano/scene_graph.yaml)

## Path Follower (C++ pathFollower port)
- Heading-gated acceleration (dirDiffThre=0.1 rad from C++)
- Cross-track vy: vx=speed*cos(err), vy=-speed*sin(err)
- Cylinder body safety: gap = obstacle_dist - body_radius (front 0.34m, side 0.19m)
- Wall escape: two-phase (reverse 1s → strafe 1.5s)
- Spot turn: vx=0.05 creep for quadruped gait

## Terrain Persistence
- TerrainAccumulator: 2D voxel grid saved to ~/.vector_os_nano/terrain_map.npz
- Auto-save every 30s during explore
- Delayed replay (20s) on startup to seed FAR
- Replay publishes to /registered_scan + /terrain_map + /terrain_map_ext (bypasses range filter)

## Harness Tests: 700+ total
| Suite | Tests | Status |
|-------|-------|--------|
| Locomotion L0-L4 | 26 | pass |
| Agent+Go2 | 5 | pass |
| VLM+Scene L0-L9 | 200+ | pass |
| Nav L17-L33 | 247 | pass |
| Sim-to-Real L34-L36 | 78 | pass |
| Other (robustness, TARE, etc.) | 150+ | pass |

## Known Limitations
- TARE sometimes stops at 7/8 rooms
- VLM room identification accuracy affects door/room learning quality
- First navigate requires explore (by design — no hardcoded fallback)
- SLAM integration not yet tested (interface compatible via /state_estimation)
