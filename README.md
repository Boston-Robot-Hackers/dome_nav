# dome_nav

Navigation and SLAM management package for the DOME robot.

Wraps slam_toolbox and Nav2. Owns all SLAM/navigation config, manages map
persistence across sessions, and exposes navigation **primitives** — including
autonomous exploration as the `ExploreArea` action (`dome_nav_msgs`).

Mission sequencing and go-to-label were extracted to the neutral **dome_mission**
package (F35); dome_nav no longer owns `/intent` or semantic knowledge.

## Related packages

- **dome_mission** — mission FSM; owns `/intent`, drives `ExploreArea` + Nav2.
- **dome_nav_msgs** — the `ExploreArea` action interface (ament_cmake).
- **dome_semantic_msgs** — `SemanticTarget` / `SemanticTargetArray`.

## Quick start

```bash
colcon build --packages-select dome_nav
source install/setup.bash
bl dome_nav robot.launch.py
```

## Map persistence

Maps saved to `~/.dome/slam_map.*` on clean shutdown. Loaded automatically on
next run. Robot must start at the same physical origin each session.

## See also

- `02-doc/spec.md` — full interface specification
- `02-doc/current.md` — session status and next steps
- `02-doc/notes.md` — architecture decisions
