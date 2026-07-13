# F14 — Preferred Goal Distance Frontier Ranking

**Priority**: Medium
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Replace the binary `prefer_farthest` flag with a continuous ranking
that scores each frontier candidate by how close its distance is to a configurable
`preferred_goal_distance` parameter (default 1.0 m). The best candidate is the one
whose distance from the robot is nearest to the preferred value — not simply the
closest or farthest. This generalises both existing modes: preferred=0 approximates
nearest-first; preferred=inf approximates farthest-first.

## Scope

- `dome_nav/explore_context.py` — add `preferred_goal_distance: float = 1.0` to
  `ExploreParams`. Deprecate `prefer_farthest` (keep for one release, map True →
  preferred=max_frontier_dist, False → preferred=0.0, with a deprecation log).
- `dome_nav/frontier_explorer.py` — replace the `prefer_farthest` bool in
  `pick_best_frontier()` with `preferred_dist: float`. Selection criterion becomes
  `min |d - preferred_dist|` across all qualifying (non-blacklisted, in-range, size-ok)
  cells, both within a cluster and across clusters. All existing filters
  (blacklist, min_dist/max_dist, max_radius, min_size) apply first.
- `dome_nav/frontier_algorithm.py` — pass `preferred_dist` through from
  `ExploreParams` to `pick_best_frontier()`.
- `dome_nav/pluggable_explore_manager_node.py` — declare ROS parameter
  `preferred_goal_distance` (default 1.0); remove `prefer_farthest` ROS parameter
  (or keep as deprecated alias). Build `ExploreParams` with the new field.
- Launch files (`sim_nav_full.launch.py`, `sim_explore.launch.py`,
  `sim_explore_node.launch.py`, `robot_explore.launch.py`) — replace
  `prefer_farthest` arg with `preferred_goal_distance` arg. Sim default 2.0 m
  (avoids the near-cluster retry loop seen with nearest-first in multi_room.world).
  Real default 1.0 m (conservative, within reliable Nav2 planning range).

## Constraints

- Pure Python change; no new ROS dependencies.
- `prefer_farthest` must not silently do nothing — either map it or raise with a
  clear deprecation message so callers know to update.
- All existing `pick_best_frontier` filter logic (blacklist, min/max dist,
  max_radius, min_size) is unchanged.
- `frontier_diag` / `dump_frontier_exhaustion` diagnostics are unaffected
  (they report filter-stage counts, not selection criterion).

## How to Demo

**Setup**: sim stack running (`bl dome_nav sim_nav_full.launch.py --map_name f14test
--world_name multi_room`). `ros2 topic echo /explore/status`.

**Steps**:
1. Launch with `--preferred_goal_distance 1.0` — observe goals cluster around 1 m.
2. Relaunch with `--preferred_goal_distance 3.0` — observe goals target ~3 m away.
3. Relaunch with `--preferred_goal_distance 0.5` — observe nearest-first behaviour.

**Expected output**: `goal_sent` telemetry `dist_m` values centre around the
configured `preferred_goal_distance` across a full exploration session, with coverage
proceeding smoothly without the ping-pong oscillation seen under `prefer_farthest`.
