# Brainstorm

## Goal (2026-06-16)

Robot drives autonomously around house, finds all cans and cups, remembers where they are, reports or navigates to them on command.

---

## Key insights from thinking

- Persistent saved map not required — within-session live SLAM sufficient
- Avoids saved-map fragility (starting pose, long-lived map drift)
- Collision avoidance is separate from localization — reactive costmap handles it
- Architecture should be localization-source agnostic (indoor SLAM today, GPS/RTK outdoors later)
- Current dome_nav structure not a constraint — design fresh

---

## Architecture proposal (draft)

### Layers

```
┌─────────────────────────────────────────┐
│           Mission Layer                 │
│  explore, detect, record, report, goto  │
├─────────────────────────────────────────┤
│         Object Layer                    │
│  detect cans/cups, pin to map frame,    │
│  dedup, maintain object list            │
├─────────────────────────────────────────┤
│         Navigation Layer                │
│  move_base / Nav2, costmap, avoid obs   │
├─────────────────────────────────────────┤
│         Localization Layer              │  ← swappable
│  live SLAM (indoor) | GPS/RTK (outdoor) │
└─────────────────────────────────────────┘
```

### Nodes / components

**Localization decision:** live SLAM + ArUco fiducials.
- SLAM builds map in real time, no pre-built map required
- Fiducials = fixed real-world anchors, replace "where I started" with absolute pose
- Robot starts anywhere, drives until it sees a marker, snaps to known world frame
- Multiple markers = redundancy + drift correction
- Cross-session object memory works because coordinate frame is anchored to markers, not start pose
- Outdoor extension: swap fiducials for GPS, rest of pipeline unchanged

| Component | Role | Technology |
|-----------|------|------------|
| `localizer` | Provides robot pose in world frame | slam_toolbox online mode + ArUco fiducials for absolute anchor |
| `explorer` | Drives coverage of unknown space | frontier-based or lawnmower waypoints |
| `detector` | Finds cans/cups in camera feed | YOLO v8, RGB or RGBD |
| `object_mapper` | Projects detections into map frame, deduplicates | tf2 + camera_info + clustering |
| `object_store` | Maintains list of found objects + poses | in-memory (session) or sqlite |
| `mission_ctrl` | Orchestrates explore/detect/report/goto | ROS2 action server or state machine |

### Data flow

```
camera → detector → (object_id, bbox, confidence)
                         ↓
robot_pose (from localizer) + depth/camera_info
                         ↓
object_mapper → (object_id, map_pose)
                         ↓
object_store → deduplicated object list
                         ↓
mission_ctrl → respond to "report" / "goto" commands
```

### Coverage strategy (open choice)

- **Frontier exploration**: explore_lite drives to map frontiers — systematic but needs growing map
- **Lawnmower waypoints**: pre-specified grid of waypoints — simple, no map needed, brittle to furniture
- **Random walk + bounce**: simplest, poor coverage guarantees

Recommendation: frontier exploration using live SLAM map (map grows during session, discarded after).

---

## Open questions

- Camera: RGB or RGBD? RGBD gives direct depth for map-frame projection; RGB needs monocular depth estimation.
- Detection model: pretrained COCO (has "cup", "bottle") may be sufficient without fine-tuning.
- Dedup strategy: cluster detections within N meters as same object?
- Command interface: CLI, ROS topic, web UI?
- "Cup" and "can" scope — any cup/can or specific ones?

---

## Mapping strategy: Roomba-style lifelong mapping (DECIDED)

- First run: no map, SLAM builds from scratch, saved at end of session
- Every subsequent run: load prior map, localize against it immediately, update as robot moves, save again at end
- Map improves over time, never needs to be "built first"
- Robot explores via random walk while SLAM runs in background — no waiting for map
- Fiducial confirms correct world frame at startup

Technology: slam_toolbox lifelong mapping mode.

**Critical dependency: fiducials are the initialization mechanism, not optional.**
- Lifelong mapping loads prior map but doesn't know where in map robot is
- Robot wanders until it sees an ArUco marker → instant absolute pose in map frame
- That pose fed to slam_toolbox as initial pose estimate → localization locks in
- First run (no map): fiducial defines world frame origin, map built anchored to it
- Every subsequent run: load map, wander, see fiducial, snap to pose, localized
- No fixed start spot required — just needs to see a marker early in the run

Solves:
- No fixed start spot required
- Not fragile — fiducial gives hard absolute anchor, not drift-prone odometry guess
- Works on first run with no map at all
- Map coordinate frame stable across sessions (anchored to physical markers)

---

---

## Package responsibilities in new scheme

No new package needed. All three packages exist; gaps are specced but unbuilt.

### dome_nav (infrastructure — SLAM + Nav)
Changes needed:
- Remove `map_start_at_dock: true`
- Switch slam_toolbox to lifelong mapping mode
- Add fiducial (ArUco) initialization: on startup, subscribe to fiducial pose, feed to slam_toolbox as initial pose estimate
- Everything else (Nav2, NavigateToPose, costmap, `/dome_nav/nav_status`) unchanged

### dome_vision (detection + target confirmation)
Already built: OAK-D pipeline, YOLO inference, `/oak/detections_3d` in camera frame.
Needs to build: **Layer 6** (already specced in dome_vision spec):
- Subscribe to `/oak/detections_3d`
- tf2 lookup: camera frame → map frame using live TF chain
- Multi-frame confirmation: PotentialTarget → ConfirmedTarget
- Publish `/targets/confirmed`, `/targets/markers`

### dome_control (mission orchestration)
Already has: behavior_manager, motion_behavior, perception_behavior, navigation_commands, survey_commands.
Needs: explore behavior (random walk), report command (dump `/targets/confirmed`), goto command (send NavigateToPose goal via dome_nav).

### Data flow (full system)
```
OAK-D ──► dome_vision ──► /oak/detections_3d
                                  ↓
              tf2 (map←camera) + confirmation
                                  ↓
                    /targets/confirmed
                         ↓              ↓
              dome_control          dome_nav
          (report / goto cmd)   (NavigateToPose)

lidar ──► dome_nav (slam_toolbox lifelong + Nav2)
ArUco ──► dome_nav (initial pose on startup)
```

---

---

## Nav2 + slam_toolbox simultaneously — verified

Officially supported. Nav2 has dedicated "Navigating while Mapping" tutorial. slam_toolbox provides `/map` + `map→odom` TF; Nav2 consumes both. No AMCL or map_server needed.

**Loop closure map-jump problem:**
- Loop closure shifts map frame → active Nav2 goals can be invalidated → replanning triggered
- Real practical issue, not documented explicitly by Nav2/slam_toolbox
- Low impact for random-walk exploration — no fixed goals to invalidate, reactive costmap handles obstacles
- Higher impact for "goto object" commands — but map is more stable by then (exploration done)

**Lifelong mapping startup — confirmed mechanism:**
slam_toolbox README lists three startup options:
1. Near predefined dock
2. At specific node ID
3. **Any location via `/initialpose` topic** ← this is what fiducials publish to

Fiducials → `/initialpose` → slam_toolbox localizes → lifelong mapping continues. Documented mechanism, not a workaround.

**Odometry caveat:**
Localization mode requires "quite a bit of tuning" and "high quality odometry." Wheel odometry alone may not be sufficient — may need IMU fusion.

---

---

## MAJOR ALTERNATIVE: Static map + AMCL

### Concept

One-time human mapping drive → saved map → AMCL forever. No continuous SLAM during operation.

### How it works

**Phase 0 (once): Map building**
- Human teleoperate robot around house
- slam_toolbox online_async builds map
- Save map to disk (`map.yaml` + `map.pgm`)
- Never run slam_toolbox again unless layout changes significantly

**Phase 1 (every run): Startup**
- Load saved map via `map_server`
- AMCL particle filter initializes — no initial pose needed, converges from lidar alone
- Nav2 starts with static map

**Phase 2 (every run): Search**
- Nav2 drives robot to pre-defined search waypoints covering house
- YOLO detects objects at each waypoint
- Object positions recorded in map frame
- Report/goto on command

### Why this is more robust than lifelong SLAM

| Issue | Lifelong SLAM | Static map + AMCL |
|-------|--------------|-------------------|
| Initial pose required | Yes (fiducials needed) | No (particle filter converges) |
| Map frame jumps | Yes (loop closure) | No (map is static) |
| Nav2 goal invalidation | Possible | Never |
| Odometry quality needed | High + IMU maybe | Moderate |
| Hardware extras | ArUco markers | None |
| Maturity | Newer, less tuned | 20 years battle-tested |

### dome_nav responsibilities in this scheme

Two explicit operating modes:

**Mode A — Map build** (run once, human at keyboard)
- Launch slam_toolbox online_async
- Accept teleop input (dome_control drives)
- On shutdown: serialize map to `~/.dome/map.yaml` + `~/.dome/map.pgm`

**Mode B — Navigation** (every normal run)
- Launch `map_server` with saved map
- Launch AMCL with saved map (particle filter, global localization)
- Launch Nav2 (planner + controller + costmap)
- Expose `/navigate_to_pose` action
- Publish `/dome_nav/slam_status`, `/dome_nav/nav_status`
- Localization source: **swappable plugin** (indoor=AMCL, outdoor=GPS/RTK)

### Outdoor future

Same dome_nav, swap localization source:
- Indoor: AMCL (lidar-based particle filter)
- Outdoor: `robot_localization` EKF fusing GPS/RTK + IMU
- Nav2, costmap, NavigateToPose interface: **unchanged**
- Map: outdoor may use GPS-framed costmap instead of occupancy grid, or pre-built satellite map

dome_nav exposes a stable interface. Localization internals are a config/launch choice, not an API change.

### What dome_nav does NOT do

- No object detection
- No exploration logic
- No object store
- No mission orchestration
- Just: localize + navigate + report status

---

---

## Package interfaces (current code, verified)

### Topic/action map

```
dome_control ──/intent──► behavior_manager ──► NavigateToPose ──► dome_nav
dome_control ◄──/dome_nav/nav_status────────────────────────────── dome_nav
dome_control ──map.save / map.serialize cmds──► dome_nav services

dome_nav ──TF (map→odom→base_link→camera_link)──► dome_vision (passive, TF broadcast)

dome_vision ──/oak/detections_3d──► SemanticMapNode (tf2 → world coords)
dome_vision ──/targets/confirmed (JSON)──► dome_control
dome_vision ──/describe_scene service──► dome_control
```

### dome_vision SemanticMapNode — already built (not future)

`semantic_map_node.py` exists and implements Layer 6: subscribes to `/oak/detections_3d`,
transforms to world frame via tf2, confirms targets, publishes `/targets/confirmed` + `/targets/markers`.
Also saves/loads object list to disk (`map_persist_path`).

**Critical bug for cross-session memory:**
```python
ODOM_FRAME = "odom"  # semantic_map_node.py:26
```
Objects stored in `odom` frame. `odom` resets every session (relative to start position).
For cross-session object memory, must store in `map` frame (stable across AMCL sessions).
Fix: change transform target from `odom` to `map`.

### Three concrete gaps to close

1. **dome_vision** — change `ODOM_FRAME = "odom"` → `"map"` in `semantic_map_node.py`
2. **dome_nav** — add Mode A (map build: slam_toolbox) / Mode B (navigate: AMCL + static map)
3. **dome_control** — add exploration commands (drive to search waypoints), `goto <object>` (lookup `/targets/confirmed` → NavigateToPose)

---

## What is NOT decided

- Whether dome_nav stays as one package or splits into multiple
- Whether localizer is a wrapper or a thin launch config
- Session persistence (lose objects on shutdown, or save to disk?)
