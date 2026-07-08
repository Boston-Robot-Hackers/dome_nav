---
version: "1.1"
generated: "2026-07-08"
---

# explore_markers.py — RViz Visualization for Exploration

This module builds the `MarkerArray` published on `/explore/markers` so a human
watching RViz can see what the explorer is thinking: which frontiers it sees,
which spots it has given up on, and where it's currently headed. It is pure
message construction — no ROS node, no state — which is why it was extracted from
the node file (both for the node's length budget and so the marker shapes can be
built and inspected in isolation).

## Three markers, three stable namespaces

The public entry point assembles exactly three markers into one array:

```python
def build_explore_markers(now, is_exploring, clusters, min_frontier_size,
                          map_info, blacklist, goal_xy) -> MarkerArray:
    markers = MarkerArray()
    markers.markers.append(build_frontier_marker(now, is_exploring, clusters,
                                                 min_frontier_size, map_info))
    markers.markers.append(build_blacklist_marker(now, blacklist))
    markers.markers.append(build_goal_marker(now, is_exploring, goal_xy))
    return markers
```

Each has a fixed `ns`/`id` so RViz updates them in place rather than piling up:

| namespace | id | type | color | content |
|---|---|---|---|---|
| `frontiers` | 0 | POINTS | yellow | cells of clusters ≥ `min_frontier_size` |
| `blacklist` | 1 | POINTS | red | every blacklisted point |
| `goal` | 2 | SPHERE | cyan | the current goal (or removed) |

## The `ADD` / `DELETE` idiom

The subtle part is how markers *disappear* when they shouldn't be shown. RViz
keeps a marker until told otherwise, so simply not-appending it would leave a
stale one on screen. Instead the code sends the same `ns`/`id` with
`action = Marker.DELETE`:

```python
marker.action = Marker.ADD if is_exploring else Marker.DELETE
```

The frontier marker deletes itself when not exploring; the goal marker deletes
itself when not exploring *or* when there is no goal:

```python
if is_exploring and goal_xy is not None:
    marker.action = Marker.ADD
    marker.pose.position.x = goal_xy[0]
    ...
else:
    marker.action = Marker.DELETE
```

So the visualization always reflects the live state — when exploration stops, the
frontier cloud and goal sphere vanish, while the blacklist (which is always
`ADD`) persists as a record of where the robot struggled.

## Frontiers mirror the algorithm's own filter

The frontier marker only draws clusters that pass the same `min_frontier_size`
gate the algorithm uses, so what you see is what the picker actually considers —
not raw noise:

```python
if is_exploring and map_info is not None:
    for cluster in clusters:
        if len(cluster) >= min_frontier_size:
            for idx in cluster:
                wx, wy = cell_to_world(idx, map_info)
                ...
```

`cell_to_world` (from `frontier_explorer`) is the shared index→world conversion,
so markers land exactly where the algorithm believes the cells are. All markers
are stamped in the `map` frame.

## Observations / possible improvements

- **`min_frontier_size` filtering here duplicates the algorithm's gate.** They
  must stay in sync (the node passes the same param to both), but a marker that
  showed *rejected* clusters in a dimmer color would make "why didn't it pick
  that frontier?" visually obvious.
- **The three `build_*` functions are near-identical boilerplate** (header, ns,
  id, scale, color). A small helper that stamps the common fields would cut the
  repetition, though the current explicitness is easy to read.
- **No visualization of the distance band or blacklist radius.** Drawing the
  `min/max_frontier_dist` ring around the robot, or the blacklist radius as
  spheres rather than points, would make the tuning parameters legible in RViz.
