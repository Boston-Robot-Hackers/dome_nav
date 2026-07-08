---
version: "1.5"
generated: "2026-07-08"
---

# frontier_explorer.py — Frontier Detection from an Occupancy Grid

This is the algorithmic heart of autonomous exploration, and it is deliberately
pure: it takes a flat list of occupancy values plus grid geometry and returns
where to go next, with no ROS, no TF, and no I/O. That purity is what lets the
whole thing be tested against hand-drawn grids. The module answers three
questions in sequence — *where are the frontiers?*, *which one should we drive
to?*, and *where exactly should the goal go?* — plus a diagnostic helper for when
the answer is "none."

## The grid model

A ROS `OccupancyGrid` is a 1-D array; `MapInfo` carries the geometry needed to
map between a flat index and world coordinates.

```python
@dataclass
class MapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
```

Cells encode `0` = free, `-1` = unknown, `>0` = occupied. Index arithmetic is the
recurring idiom: `row = idx // width`, `col = idx % width`, and back with
`cell_to_world`, which returns the *center* of the cell:

```python
def cell_to_world(idx, info):
    r, c = divmod(idx, info.width)
    x = info.origin_x + (c + 0.5) * info.resolution
    y = info.origin_y + (r + 0.5) * info.resolution
    return (x, y)
```

## What counts as a frontier (the buffer-cell definition)

The classic definition of a frontier is "a free cell adjacent to unknown." This
module uses a deliberately stricter one: a frontier cell is free, does **not
itself** touch unknown, but has at least one 4-neighbor that does. In other
words, there is always one known "buffer" cell between the goal and the unknown
region it borders.

```python
touches_unknown: set[int] = set()
for idx in range(width * height):
    if data[idx] != 0:
        continue
    for nb in neighbors4(idx):
        if data[nb] == -1:
            touches_unknown.add(idx)
            break

is_frontier: set[int] = set()
for idx in touches_unknown:
    for nb in neighbors4(idx):
        if data[nb] == 0 and nb not in touches_unknown:
            is_frontier.add(nb)
```

Why the extra step? Goals sitting *directly* on the ragged known/unknown boundary
are exactly where Nav2's planners were historically unreliable (the NavFn "legal
potential" bug) and where costmap geometry is most ambiguous. Keeping every
candidate one cell deeper into confirmed-known space made goals more reliably
plannable. This is a real, hard-won design choice, not a cosmetic one.

Adjacent frontier cells are then grouped into clusters by an 8-connectivity
flood-fill, so a long wall-opening becomes one cluster rather than dozens of
singletons:

```python
for seed in is_frontier:
    if seed in visited: continue
    cluster, stack = [], [seed]
    while stack:
        cell = stack.pop()
        if cell in visited or cell not in is_frontier: continue
        visited.add(cell); cluster.append(cell)
        for nb in neighbors8(cell):
            if nb not in visited and nb in is_frontier:
                stack.append(nb)
    clusters.append(cluster)
```

```mermaid
flowchart TD
    A[grid cells] --> B[free cells touching unknown]
    B --> C[their known free neighbors = frontier cells]
    C --> D[8-connectivity flood fill]
    D --> E[clusters of frontier cell indices]
```

## Picking the goal: nearest cell, not centroid

`pick_best_frontier` chooses among clusters, and its most important decision is
what point *within* a cluster to aim at. It uses the nearest qualifying **cell**,
not the cluster centroid.

```python
for cluster in clusters:
    if len(cluster) < min_size:
        continue
    ...
    for cell_idx in cluster:
        wx, wy = cell_to_world(cell_idx, info)
        # skip blacklisted, too-close, too-far cells...
        is_better = d > goal_dist if prefer_farthest else d < goal_dist
        if is_better:
            goal_dist, goal = d, (wx, wy)
```

The comment explains why: a large frontier that *surrounds* the robot (a ring)
has a centroid ≈ the robot's own position — useless as a goal — while its
individual cells are out at the map boundary where the robot actually needs to
go. So the centroid is used only for the `max_radius` cluster-level filter, never
as the goal itself.

The filters stack per cell: **blacklist** (within `blacklist_radius` of a failed
point), **`min_dist`/`max_dist`** distance band, and cluster-level `min_size` and
`max_radius`. The `prefer_farthest` flag flips every "nearer is better"
comparison to "farther is better" — the same code path, one operator — used in
sim to escape locally-stuck regions.

## Placing the goal off the boundary: `nudge_toward_robot`

Even a buffer cell can sit near the costmap edge. `nudge_toward_robot` pulls the
final goal a fixed `inset_m` back toward the robot:

```python
def nudge_toward_robot(xy, robot_xy, inset_m):
    dx, dy = robot_xy[0] - xy[0], robot_xy[1] - xy[1]
    dist = math.sqrt(dx*dx + dy*dy)
    if dist < inset_m:
        return xy
    scale = inset_m / dist
    return (xy[0] + dx * scale, xy[1] + dy * scale)
```

This keeps the sent goal inside known, navigable space and avoids Nav2
`worldToMap` errors at the grid edge. (It's why `min_frontier_dist` is set 0.3 m
higher than the desired sent-goal floor — see `explore_context`.)

## When nothing qualifies: `frontier_diag`

If `pick_best_frontier` returns `None`, the node needs to know *why* — was
everything too small, all blacklisted, or all outside the distance band?
`frontier_diag` does one extra pass to count exactly that, and is only called on
the None path so it never taxes the normal case:

```python
def frontier_diag(clusters, info, robot_xy, min_size, min_dist, max_dist=0.0):
    too_small = sum(1 for c in clusters if len(c) < min_size)
    large = [c for c in clusters if len(c) >= min_size]
    all_out_of_range = sum(
        1 for cluster in large
        if all(cell_out_of_range(cell_to_world(i, info), robot_xy, min_dist, max_dist)
               for i in cluster)
    )
    return {"too_small": too_small, "large_clusters": len(large),
            "all_cells_out_of_range": all_out_of_range}
```

Those three counts are what turn a mute "no frontier found" into an actionable
telemetry record. (`cell_out_of_range` is the shared distance-band predicate,
also used implicitly by the picker's own filters.)

## Observations / possible improvements

- **Two full grid scans per call** (`touches_unknown`, then its neighbors). At
  current map sizes and 2 Hz this is fine; for large maps a single pass that
  records both sets would halve the work.
- **`pick_best_frontier` has grown to eleven parameters.** They all matter, but
  it's at the edge of readability — bundling the filter params (they already
  travel together as `ExploreParams`) would tighten the signature.
- **The buffer-cell definition assumes 1 cell of margin is enough.** On coarser
  resolutions or noisier maps a configurable buffer depth might be worth it; for
  now one cell has been the reliable sweet spot.
- **`frontier_diag` recomputes `cell_to_world` for every cell of every large
  cluster.** Only on the failure path, so acceptable, but it duplicates work the
  picker just did.
