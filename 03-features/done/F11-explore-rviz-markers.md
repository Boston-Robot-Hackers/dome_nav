# F11 — RViz2 Exploration Markers

**Priority**: Medium
**Done:** yes (2026-06-26)
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** N/A — RViz2 visual, verified live
**Description**: Publish `visualization_msgs/MarkerArray` on `/explore/markers` so RViz2
can display exploration state in real time: frontier cells, blacklisted positions, and
the current nav goal. Aids tuning and live debugging without reading raw telemetry.

## Scope

- `dome_nav/explore_manager_node.py` — add `MarkerArray` publisher + `publish_markers()`
  called each tick alongside `publish_status()`
- `package.xml` — add `visualization_msgs` dependency
- RViz2 config (optional) — pre-configure MarkerArray display for `/explore/markers`

## Marker namespaces

| namespace | type | color | content |
|---|---|---|---|
| `frontiers` | POINTS | yellow | all cells from clusters passing `MIN_FRONTIER_SIZE` |
| `blacklist` | POINTS | red | all positions in `self.blacklist` |
| `goal` | SPHERE | cyan | `current_goal_xy` (DELETE when no active goal) |

## Notes

- Store `self.latest_clusters` and `self.latest_map_info` as instance state, updated
  each tick in `find_and_send_frontier`
- Frontier points can be dense (hundreds of cells) — only show large clusters to reduce noise
- When `state != "exploring"` publish DELETE markers to clear stale display
