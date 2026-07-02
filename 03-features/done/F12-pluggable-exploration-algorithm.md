# F12 — Pluggable Exploration Algorithm

**Priority**: High
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Isolate the exploration decision logic behind a protocol so
different algorithms (frontier, random-walk, local-scan, direct-motor) can be
swapped without touching Nav2 or slam_toolbox plumbing. Algorithm is pure
Python, fully testable without a robot.

---

## Motivation

`ExploreManagerNode` currently fuses three concerns:

1. **Infrastructure** — ROS subscriptions, TF, Nav2 action client, telemetry
2. **Session management** — blacklist, goal tracking, timeout, patience counter
3. **Exploration algorithm** — frontier clustering, scoring, goal selection

Tangling them makes it impossible to:
- unit-test the algorithm without spinning a ROS node
- swap frontier detection for a scan-based or random strategy
- run the algorithm against a recorded map in a notebook
- experiment with skipping Nav2 entirely (direct velocity commands)

---

## Design

### `ExplorationContext` dataclass

Passed **in** to the algorithm each tick. Contains everything the algorithm
needs; nothing it does not.

```python
@dataclass
class ExplorationContext:
    # Map data — either full OccupancyGrid cells or a local scan window
    map_data: list[int]
    map_info: MapInfo          # width/height/resolution/origin

    # Robot pose in the map frame
    robot_xy: tuple[float, float]

    # Session state the algorithm may read and mutate
    blacklist: set[tuple[float, float]]
    start_xy: tuple[float, float] | None

    # Tuning knobs (read-only from algorithm's perspective)
    params: ExploreParams      # dataclass of all the MIN_* / GOAL_* constants
```

### `ExploreParams` dataclass

All numeric tuning constants extracted from `ExploreManagerNode` class vars:

```python
@dataclass
class ExploreParams:
    min_frontier_size: int   = 10
    blacklist_radius: float  = 0.5
    min_frontier_dist: float = 0.8
    goal_inset_m: float      = 0.3
    max_explore_radius: float = 0.0
```

### `ExplorationAlgorithm` protocol

```python
class ExplorationAlgorithm(Protocol):
    def next_goal(self, ctx: ExplorationContext) -> tuple[float, float] | None:
        """Return world-frame (x, y) goal, or None if no valid target."""
        ...
```

`None` return is the single signal for "no frontier / exploration done."
The caller (`ExploreManagerNode`) handles patience counting; the algorithm
does not need to know about it.

### `FrontierAlgorithm` (existing logic, refactored)

Wraps current `find_frontier_clusters` + `pick_best_frontier` + `nudge_toward_robot`:

```python
class FrontierAlgorithm:
    def next_goal(self, ctx: ExplorationContext) -> tuple[float, float] | None:
        clusters = find_frontier_clusters(ctx.map_data, ctx.map_info)
        target = pick_best_frontier(clusters, ctx.map_info, ctx.robot_xy, ...)
        if target is None:
            return None
        return nudge_toward_robot(target, ctx.robot_xy, ctx.params.goal_inset_m)
```

Algorithm also stores `latest_clusters` on itself so the node can read it
for marker publishing — keeps visualization working without coupling.

### `ExploreManagerNode` (thinned)

- Builds `ExplorationContext` each tick from latest map + TF
- Calls `self.algorithm.next_goal(ctx)`
- If result is not `None`, sends Nav2 goal
- Patience counter lives in the node, not the algorithm
- Algorithm is injected at `__init__` (defaults to `FrontierAlgorithm`)

```python
def __init__(self, algorithm: ExplorationAlgorithm | None = None):
    ...
    self.algorithm = algorithm or FrontierAlgorithm()
```

---

## Future algorithm slots (not implemented in F12)

| Algorithm | Map input | Navigator | Notes |
|---|---|---|---|
| `FrontierAlgorithm` | full OccupancyGrid | Nav2 | existing, default |
| `LocalScanAlgorithm` | LiDAR scan window | Nav2 | no slam_toolbox dep |
| `RandomWalkAlgorithm` | none | Nav2 | baseline / debug |
| `DirectVelocityAlgorithm` | LiDAR scan | cmd_vel | no Nav2 dep |

Only `FrontierAlgorithm` is implemented as part of F12. Others are listed to
validate the interface is wide enough.

---

## File changes

| File | Change |
|---|---|
| `dome_nav/explore_context.py` | new — `ExploreParams`, `ExplorationContext`, `ExplorationAlgorithm` protocol |
| `dome_nav/frontier_algorithm.py` | new — `FrontierAlgorithm` (logic from `explore_manager_node.py`) |
| `dome_nav/frontier_explorer.py` | unchanged (pure functions stay) |
| `dome_nav/explore_manager_node.py` | refactored — thin shell, inject algorithm |
| `test/test_frontier_algorithm.py` | new — unit tests, zero ROS deps |

---

## Testing without a robot

Because `FrontierAlgorithm.next_goal` takes only plain Python dataclasses:

```python
def test_picks_nearest_frontier():
    ctx = ExplorationContext(
        map_data=[...],      # hand-crafted grid
        map_info=MapInfo(width=10, height=10, resolution=0.05,
                         origin_x=0.0, origin_y=0.0),
        robot_xy=(0.25, 0.25),
        blacklist=set(),
        start_xy=None,
        params=ExploreParams(),
    )
    algo = FrontierAlgorithm()
    goal = algo.next_goal(ctx)
    assert goal is not None
    assert goal[0] > 0.0
```

No `rclpy`, no node spin, no TF, no Nav2.

---

## How to Demo

**Setup**: `bl dome_nav robot_explore.launch.py` with robot in unknown space.

**Steps**:
1. Confirm exploration starts as before — `FrontierAlgorithm` is default.
2. Run `pytest test/test_frontier_algorithm.py` — passes without ROS.
3. Swap in a stub algorithm at node init, confirm node calls `next_goal`.

**Expected output**: Exploration behavior identical to F10/F11 baseline.
All unit tests pass. `ExploreManagerNode` has no direct calls to
`find_frontier_clusters` or `pick_best_frontier`.
