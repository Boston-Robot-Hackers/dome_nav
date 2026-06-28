---
version: "1.0"
generated: "2026-06-28"
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
    participant Algo as FrontierAlgorithm
    participant Map as map data

    Loop->>Algo: next_goal(ctx)
    Algo-->>Loop: goal_xy or None

    alt goal is None
        Loop->>Loop: increment no_frontier_count
        Loop->>Loop: break if >= PATIENCE
    else path blocked by obstacle
        Loop->>Loop: blacklist goal_xy, stay put
    else clear path
        Loop->>Map: uncover_along_path(robot→goal)
        Loop->>Loop: robot_xy = goal_xy
        Loop->>Loop: blacklist goal_xy
    end
    Loop->>Loop: render + print step info
```

`PATIENCE = 6` consecutive no-frontier ticks triggers termination. Each tick where a path
is blocked does not count against patience — only ticks where the algorithm itself returns
`None`.

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
Human: add any maps at the top of this file that you want in that format and pass the directory with --maps-dir.
