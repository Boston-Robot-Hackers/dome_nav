---
version: "1.1"
generated: "2026-07-05"
---

# algo_demo — Interactive Frontier Exploration Simulator

## What It Is and Why It Exists

`tools/algo_demo.py` is a terminal-based simulator for the `FrontierAlgorithm`. It lets
you watch the algorithm run step-by-step on a hand-crafted ASCII map — no ROS2, no robot,
no Nav2 — just pure Python. The motivating question it answers is: *does the frontier
algorithm behave sensibly on a map I can reason about by eye?*

The simulator sits entirely outside the algorithm. `FrontierAlgorithm.next_goal(ctx)` sees
only an `ExplorationContext` dataclass; it knows nothing about terminals, sensor physics, or
travel constraints. The demo is the harness that assembles the context, executes moves, and
renders the world.

---

## Map Representation

Each built-in map is a list of ASCII strings. Three cell types exist:

| Char | Meaning | Parsed as |
|------|---------|-----------|
| `?`  | Unknown (not yet sensed) | `CELL_UNK = -1` |
| `0`  | Free space | `CELL_FREE = 0` |
| `#`  | Occupied (wall or obstacle) | `CELL_OCC = 100` |
| `R`  | Robot start (also free) | `CELL_FREE` |

`parse_map` flattens the 2-D list into a 1-D array indexed row-major, consistent with how
`FrontierExplorer` consumes ROS2 `OccupancyGrid` messages.

```python
def parse_map(rows: list[str]) -> tuple[list[int], MapInfo]:
    height = len(rows)
    width = max(len(r) for r in rows)
    data: list[int] = []
    for row in rows:
        for ch in row.ljust(width):
            if ch in ("0", "R"):
                data.append(CELL_FREE)
            elif ch == "#":
                data.append(CELL_OCC)
            else:
                data.append(CELL_UNK)
    info = MapInfo(width=width, height=height, resolution=1.0,
                   origin_x=0.0, origin_y=0.0)
    return data, info
```

`resolution=1.0` means one cell equals one metre, so world coordinates and cell indices
stay easy to reason about.

---

## The Maps

Five built-in maps test progressively harder exploration scenarios:

| Name | Size | Challenge |
|------|------|-----------|
| `room` | 11×8 | Baseline: single open room |
| `corridor` | 11×7 | Two rooms separated by walls (no door) |
| `ring` | 13×9 | Inner room inside outer ring — two disjoint free regions |
| `maze` | 13×9 | Internal walls creating dead ends |
| `large` | 30×30 | Three rooms joined by corridors |
| `compound` | 40×40 | Large room with internal obstacles, gap + side corridor |

`compound` is the most demanding. Its design:

```
cols:   0        28 29 30       39
         ┌────────┐#┌──────────┐
rows 0–4 │  room  │#│ unknown  │
rows 5–8 │ ##     │#│ unknown  │  ← 4×4 obstacle A
rows 9–14│        │#│ unknown  │
rows15–24│        │ │ corridor │  ← gap in wall (10 rows)
rows25–27│        │#│ unknown  │
rows28–31│     ## │#│ unknown  │  ← 4×4 obstacle B
rows32–39│        │#│ unknown  │
         └────────┘#└──────────┘
```

The robot starts at row 20, col 12 — aligned with the gap vertically but far from it
horizontally. It must first explore the room, approach the wall, and eventually "see
through" the gap to discover the corridor.

---

## Sensor Simulation: Line-of-Sight Reveal

In reality, a 2-D lidar reveals cells by casting rays in all directions until a ray hits an
obstacle. The naive approach — reveal every unknown cell within a radius — lets the robot
"see through" walls, which defeats the purpose of the `compound` map's corridor design.

The simulator uses **Bresenham's line algorithm** to cast a ray from the robot to each
candidate cell, checking every intermediate grid cell for occupancy.

### Bresenham's Line

The classic integer rasterisation algorithm produces the tightest possible set of grid cells
connecting two endpoints, with no floating-point arithmetic in the inner loop:

```python
def bresenham_cells(c0: int, r0: int, c1: int, r1: int) -> list[tuple[int, int]]:
    cells = []
    dc = abs(c1 - c0)
    dr = abs(r1 - r0)
    sc = 1 if c1 > c0 else -1
    sr = 1 if r1 > r0 else -1
    err = dc - dr
    c, r = c0, r0
    while True:
        cells.append((c, r))
        if c == c1 and r == r1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells
```

The error accumulator `err` is initialised to `dc - dr` and updated by `±2·dc` and
`±2·dr` each step, avoiding any division. The result is a list of `(col, row)` pairs from
start to end inclusive.

### LOS Check

`has_line_of_sight` slices off the start and end cells — the robot's own cell (always free)
and the candidate target cell (which we're deciding whether to reveal) — and checks only
the intermediates:

```python
def has_line_of_sight(data, info, from_xy, to_xy) -> bool:
    c0 = int((from_xy[0] - info.origin_x) / info.resolution)
    r0 = int((from_xy[1] - info.origin_y) / info.resolution)
    c1 = int((to_xy[0] - info.origin_x) / info.resolution)
    r1 = int((to_xy[1] - info.origin_y) / info.resolution)
    for c, r in bresenham_cells(c0, r0, c1, r1)[1:-1]:
        if 0 <= r < info.height and 0 <= c < info.width:
            if data[r * info.width + c] == CELL_OCC:
                return False
    return True
```

`[1:-1]` is the key: adjacent cells always have LOS (empty intermediate set), and the
endpoint itself is excluded because revealing an unknown free cell cannot be blocked by
itself.

### Reveal

`uncover_around_robot` applies the radius + LOS filter together:

```python
def uncover_around_robot(data, info, robot_xy, radius) -> list[int]:
    data = list(data)
    rx, ry = robot_xy
    for idx in range(info.width * info.height):
        if data[idx] != CELL_UNK:
            continue
        wx, wy = cell_to_world(idx, info)
        if math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2) <= radius:
            if has_line_of_sight(data, info, robot_xy, (wx, wy)):
                data[idx] = CELL_FREE
    return data
```

The function returns a new list rather than mutating in place, so callers can chain calls
easily (as `uncover_along_path` does).

### Path Scanning

`uncover_along_path` models continuous lidar scanning during transit. Rather than revealing
cells only at the destination, it sweeps the robot along the straight-line path in steps of
`radius/2` and calls `uncover_around_robot` at each intermediate position:

```python
def uncover_along_path(data, info, from_xy, to_xy, radius) -> list[int]:
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    dist = math.sqrt(dx ** 2 + dy ** 2)
    steps = max(1, int(dist / (radius / 2)))
    for i in range(steps + 1):
        t = i / steps
        pos = (from_xy[0] + t * dx, from_xy[1] + t * dy)
        data = uncover_around_robot(data, info, pos, radius)
    return data
```

Step size `radius/2` guarantees that sensor circles at consecutive positions overlap, so no
strip of cells is skipped between steps. With LOS checking in place, walls along the path
continue to block reveal on their far side.

---

## Travel Constraint

The demo uses straight-line teleportation: the robot jumps to the goal each step rather than
planning a path around obstacles. Without an additional check, the robot could jump through
a wall to a frontier on the far side.

Before committing a move, the main loop checks whether the straight-line path from robot to
goal crosses any occupied cell — reusing `has_line_of_sight`:

```python
elif not has_line_of_sight(data, info, robot_xy, goal_xy):
    no_frontier_count = 0
    print(f"\nStep {step}: path to ({goal_xy[0]:.2f},{goal_xy[1]:.2f})"
          f" blocked by obstacle — blacklisting")
    blacklist.add(goal_xy)
```

A blocked goal is blacklisted so the algorithm picks a different frontier next tick. This
is a simulation approximation: a real robot would ask Nav2 to plan around the obstacle, not
give up on the goal entirely. In practice, once the robot reaches an angle from which the
path is clear, it reaches those frontier cells via a different goal.

---

## Goal Nudging: Toward Robot vs. Away From Unknown

A frontier cell is, by definition, a free cell adjacent to unknown space — the sensor's
current edge of visibility. Sending Nav2 a goal sitting exactly on that boundary invites
trouble: `worldToMap` lookups can fail near an unmapped region, and (as live Gazebo
telemetry later confirmed — see `04-tasks/notdone/TF13-gazebo-simulation.md` T04m/T04n) the
NavFn global planner can fail with "legal potential found, no path" precisely when the goal
sits on the ragged known/unknown edge. Some inset off that edge, back into confirmed free
space, is needed before the goal is usable.

The original approach, `nudge_toward_robot`, pulls the raw frontier cell a fixed distance
(`goal_inset_m`) toward the robot's current position:

```python
def nudge_toward_robot(xy, robot_xy, inset_m):
    dx = robot_xy[0] - xy[0]
    dy = robot_xy[1] - xy[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < inset_m:
        return xy
    scale = inset_m / dist
    return (xy[0] + dx * scale, xy[1] + dy * scale)
```

This is a reasonable proxy — the robot is usually on the known side of the frontier — but it
is only a proxy. The direction "toward the robot" and the direction "away from the unknown
boundary" are the same only when the robot happens to sit on the boundary's normal. Approach
the same frontier cell from an oblique angle and the two directions diverge; a synthetic
concave-corner test (an L-shaped known region, frontier tip exposed to unknown on two sides)
showed `nudge_toward_robot`'s effective clearance from the unknown boundary dropping from a
full 3-cell inset to just 1 cell, depending solely on where the robot happened to be standing.

`nudge_away_from_unknown` targets the boundary directly instead of using the robot as a
proxy. For every unknown cell within a small window around the target, it accumulates a unit
vector pointing from that unknown cell back toward the target; the sum of those vectors is
the escape direction, robust to the robot's position:

```python
def nudge_away_from_unknown(target_xy, data, info, inset_m, search_cells=2):
    tc = int((target_xy[0] - info.origin_x) / info.resolution)
    tr = int((target_xy[1] - info.origin_y) / info.resolution)
    vx, vy = 0.0, 0.0
    for dr in range(-search_cells, search_cells + 1):
        for dc in range(-search_cells, search_cells + 1):
            if dr == 0 and dc == 0:
                continue
            nr, nc = tr + dr, tc + dc
            if not (0 <= nr < info.height and 0 <= nc < info.width):
                continue
            if data[nr * info.width + nc] != CELL_UNK:
                continue
            dist = math.sqrt(dr * dr + dc * dc)
            vx += -dc / dist
            vy += -dr / dist
    mag = math.sqrt(vx * vx + vy * vy)
    if mag == 0.0:
        return target_xy
    dir_x, dir_y = vx / mag, vy / mag

    for scale in (1.0, 0.66, 0.33):
        step = inset_m * scale
        cand_xy = (target_xy[0] + dir_x * step, target_xy[1] + dir_y * step)
        cc = int((cand_xy[0] - info.origin_x) / info.resolution)
        cr = int((cand_xy[1] - info.origin_y) / info.resolution)
        if 0 <= cr < info.height and 0 <= cc < info.width:
            if data[cr * info.width + cc] == CELL_FREE:
                return cand_xy
    return target_xy
```

Three details matter here:

- **Vectors are weighted by inverse distance, not counted equally.** A cell diagonally two
  steps away contributes less than an adjacent one, so the escape direction favors moving
  away from the *closest* unknown cells first — the ones most responsible for the boundary
  instability.
- **Step size backs off in thirds (`1.0, 0.66, 0.33`) rather than failing outright.** If the
  full inset would land on an occupied or unknown cell (a tight corner where "away from
  unknown" leads toward a wall instead), a shorter step is tried before giving up and
  returning the raw target unchanged.
- **`mag == 0.0` falls back to the raw target**, not to `nudge_toward_robot`. This can only
  happen if the search window found no unknown neighbor at all — which shouldn't occur for a
  genuine frontier cell, but the function stays defensive rather than assuming.

Both strategies are wired into the demo behind `--nudge-mode {robot,unknown}` so they can be
compared side-by-side on the same map and frontier selection, without touching the
production `frontier_algorithm.py` path. This is deliberately a throwaway comparison
harness, not a refactor of the real nudge logic — see Observations below.

---

## Rendering

The renderer produces one character per cell, colour-coded with ANSI 256-colour escapes.
Priority order (first match wins): robot > target cell > goal cell > blacklisted > frontier
cluster > free > occupied > unknown.

```mermaid
flowchart TD
    A[cell (r,c)] --> B{robot?}
    B -- yes --> R[cyan R]
    B -- no --> C{target?}
    C -- yes --> T[pink T]
    C -- no --> D{goal?}
    D -- yes --> G[yellow G]
    D -- no --> E{blacklisted?}
    E -- yes --> BL[red B]
    E -- no --> F{frontier cluster?}
    F -- yes --> CL[cluster letter A-Z]
    F -- no --> H{free?}
    H -- yes --> FR[dim . ]
    H -- no --> I{occupied?}
    I -- yes --> W[white #]
    I -- no --> U[dim ?]
```

Frontier clusters are labelled A–Z and coloured from an 8-colour palette that cycles for
larger maps. The cluster legend below the map shows each label with its cell count.

---

## The Main Loop

```mermaid
sequenceDiagram
    participant Loop
    participant Clusters as find_frontier_clusters
    participant Pick as pick_best_frontier
    participant Nudge as nudge_toward_robot / nudge_away_from_unknown
    participant Map as map data

    Loop->>Clusters: find_frontier_clusters(data, info)
    Clusters-->>Loop: clusters
    Loop->>Pick: pick_best_frontier(clusters, ...)
    Pick-->>Loop: target_xy or None

    alt target is None
        Loop->>Loop: run _frontier_diag, increment no_frontier_count
        Loop->>Loop: break if >= PATIENCE
    else target found
        Loop->>Nudge: nudge(target_xy, ..., inset_m)
        Nudge-->>Loop: goal_xy
        alt path blocked by obstacle
            Loop->>Loop: blacklist goal_xy, stay put
        else clear path
            Loop->>Map: uncover_along_path(robot→goal)
            Loop->>Loop: robot_xy = goal_xy
            Loop->>Loop: blacklist goal_xy
        end
    end
    Loop->>Loop: render + print step info
```

The main loop calls `find_frontier_clusters` and `pick_best_frontier` directly rather than
going through `FrontierAlgorithm.next_goal()`, so it can select which nudge function runs on
the result (`--nudge-mode`) — `next_goal()` always applies `nudge_toward_robot` internally
and has no hook for swapping it out. This is the one place the demo diverges from calling
the production algorithm as a black box, and it exists solely to compare the two nudge
strategies (see previous section); it is not how the real pluggable node computes a goal.

`PATIENCE = 6` consecutive no-frontier ticks triggers termination. Each tick where a path
is blocked does not count against patience — only ticks where `pick_best_frontier` itself
returns `None`.

---

## Observations and Potential Improvements

**Straight-line travel is a poor proxy for Nav2.** The blacklist-on-block workaround handles
obstacles, but a robot that can only travel in straight lines will get stuck in concave
regions of a map. A breadth-first reachability check (flood-fill from robot position) would
let the demo verify that a goal is reachable before committing, rather than discovering
the block after the algorithm picks it.

**LOS check is conservative at corners.** Bresenham's algorithm steps through grid cells
discretely; a ray aimed at a cell just past an obstacle corner will clip the corner cell
and be blocked, even though a real lidar beam at a slight angle would pass through. This
makes the sensor slightly pessimistic at convex obstacle edges.

**`uncover_along_path` performance.** For large maps, the O(width × height) scan in
`uncover_around_robot` at each step of `uncover_along_path` can be slow. A bounding-box
pre-filter (skip cells outside `(robot ± radius)` in both axes) would cut the constant by
roughly 4×.

**No multi-algorithm comparison.** The pluggable design (`ExplorationAlgorithm` protocol)
supports injecting any algorithm, but the demo hard-codes `FrontierAlgorithm`. Adding a
`--algorithm` flag that can select `RandomWalkAlgorithm` or a stub would make the demo
useful as a benchmarking harness.

**Map data embedded in source.** The compound map alone adds 40 string literals to the
file, pushing it well past the 300-line guideline. Loading maps from external YAML or text
files would separate content from logic and allow user-defined maps without editing source.

**`--nudge-mode` bypasses `FrontierAlgorithm.next_goal()`.** The comparison harness calls
`find_frontier_clusters`/`pick_best_frontier` directly so it can swap in
`nudge_away_from_unknown`, which duplicates a few lines of `next_goal()`'s body. This is
fine for a throwaway comparison tool, but if `nudge_away_from_unknown` (or a `nudge_mode`
parameter) is ported into `frontier_algorithm.py` for real use, this demo should go back to
calling `next_goal()` as a black box rather than reimplementing its internals.
