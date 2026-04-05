# Vector OS Nano SDK — Progress

**Last updated:** 2026-04-05
**Version:** v1.2.0-dev
**Branch:** master

## Architecture

```
vector-cli (agent process)          launch_explore.sh (subprocess)
  LLM + SceneGraph + VLM              MuJoCoGo2 (convex MPC, 1kHz)
  Go2ROS2Proxy ◄── ROS2 ──►           Go2VNavBridge (200Hz odom)
  12 Go2 skills                        localPlanner + FAR + TARE
                                       terrainAnalysis + RViz
```

## Navigation Pipeline

```
proxy.navigate_to(x, y):
  Phase 1 (5s): send /goal_point, detect FAR /way_point
  Phase 2: FAR V-Graph routing (/goal_point only, FAR publishes /way_point)
  Phase 3: door-chain fallback (SceneGraph doors → /way_point → localPlanner)
```

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

## Harness Tests: 600+ total
| Suite | Tests | Status |
|-------|-------|--------|
| Locomotion L0-L4 | 26 | pass |
| Agent+Go2 | 5 | pass |
| VLM+Scene L0-L9 | 200+ | pass |
| Nav L17-L33 | 247 | pass |

## Known Limitations
- _ROOM_CENTERS/_ROOM_DOORS hardcoded for go2_room.xml (NOT sim-to-real ready)
- Terrain replay partial (terrainAnalysis range filter)
- Dead-reckoning fallback still exists as last resort
- TARE sometimes stops at 7/8 rooms

## Next: Sim-to-Real Refactor
- Remove all hardcoded coordinates (8 source files)
- SceneGraph as only map source
- Door learning during explore (VLM room transition)
- Terrain replay fix (publish to FAR topics directly)
- SDD spec: .sdd/spec.md
