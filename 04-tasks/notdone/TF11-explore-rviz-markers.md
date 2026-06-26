# TF11 — RViz2 Exploration Markers for F11

## T01 — Add visualization_msgs dependency
**Status**: notdone
**Description**: Add `<depend>visualization_msgs</depend>` to `package.xml`.

## T02 — Add publisher and instance state
**Status**: notdone
**Description**: In `explore_manager_node.py` `__init__`:
- Import `Point`, `Marker`, `MarkerArray` from `geometry_msgs` / `visualization_msgs`
- `self.marker_pub = self.create_publisher(MarkerArray, "/explore/markers", 10)`
- `self.latest_clusters: list[list[int]] = []`
- `self.latest_map_info: MapInfo | None = None`

## T03 — Store clusters each tick
**Status**: notdone
**Description**: In `find_and_send_frontier`, after computing `clusters` and `info`,
store `self.latest_clusters = clusters` and `self.latest_map_info = info`.

## T04 — Implement publish_markers()
**Status**: notdone
**Description**: New method publishing a `MarkerArray` with three namespaces:
- `frontiers` (id=0): POINTS, yellow, all cells from clusters with `len >= MIN_FRONTIER_SIZE`
- `blacklist` (id=1): POINTS, red, all `(x,y)` in `self.blacklist`
- `goal` (id=2): SPHERE, cyan, `current_goal_xy` — action=DELETE when no active goal
When `state != "exploring"`, send DELETE for frontiers and goal markers.

## T05 — Call publish_markers() from explore_tick
**Status**: notdone
**Description**: Add `self.publish_markers()` call in `explore_tick()` alongside
`self.publish_status(self.state)`.

## T06 — Manual smoke test in RViz2
**Status**: notdone
**Description**: Launch Mode E, open RViz2, add MarkerArray display on `/explore/markers`.
Verify frontier cells appear yellow, blacklisted positions appear red, current goal
appears as cyan sphere. Verify markers clear when exploration stops.
