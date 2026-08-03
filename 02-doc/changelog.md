# dome_nav — Changelog

Session entries older than ~1 week, moved out of `02-doc/current.md` to keep
that file scannable (rule in `.claude/process.md`). Reverse-chronological,
same format as `current.md`. Detailed history also lives in git log and the
`04-tasks/` files — this is a narrative supplement, not the source of truth.

## This session (2026-07-24, evening) — parameter inventory + algo_demo repair + wedge diagnosis

- **`02-doc/tunable_parameters.md` created**: full inventory of every tuning knob
  — explorer-manager ROS params, `FrontierParams` ROS params (incl. deprecated
  `prefer_farthest`/`novelty_top_n`), slam_manager params, launch-file exposure
  matrix, slam_toolbox/nav2 yaml deltas, and code-edit-only constants. Notes the
  two gaps: `blacklist_radius` has no ROS/launch exposure; `goal_inset_m` is
  ROS-declared but set by no launch file.
- **`tools/algo_demo.py` repaired** (was `TypeError`-dead since the F23/F31 split;
  chores.md entry): CLI args → `FrontierParams`, `merge_tuning` → `FrontierTuning`,
  new `pick_best_frontier(tuning, blacklist=, data=)` signature. Verified with
  `--auto` runs on room/maze/corridor + `--nudge-mode unknown`. Literate
  `10-algo_demo.md` regenerated v1.1. Full suite **270 pass**.
- **Wedge investigation narrowed (no code change)**: `collision_monitor_state`
  confirmed oscillating 0 (DO_NOTHING) ↔ 3 (APPROACH, FootprintApproach). RViz
  measure showed the costmap is *correct*: obstacle 0.20 m from body edge ⇒ robot
  center ~0.35 m ⇒ global cost 0 (global `inflation_radius` 0.2 ends at 0.20 m;
  the pink halo is geometrically right). Ghost-cell/mismapping theory ruled out.
  **Open**: split `cmd_vel_smoothed` vs `cmd_vel` during a stall to decide
  MPPI-stall vs monitor-gate. Prime suspect if MPPI: local `inflation_radius`
  0.25 > actual 0.20 m clearance (global is 0.2) — body edge sits at ~cost 217
  locally, path-through-obstacle + path-alignment critics stall all trajectories.
  Secondary: slam `minimum_travel_distance`/`minimum_travel_heading` 0.5/0.5 (sim
  uses 0.1/0.1) starves map updates while stuck — chicken-egg worth fixing
  regardless.

## This session (2026-07-24) — docs cleanup + package-wide style-guide pass

No behavior change. Cosmetic/quality + docs only; full suite still **266 pass**
(4 live-stack `test_map_validation` need a robot), `colcon build` clean.

- **Style guide grew a rule** (`.claude/style_guide.md` §Comments And Types, MUST):
  complicated/obscure functions explain what + inputs/outputs, **preferably in the
  docstring**; explicitly does not license in-body narrative `#` blocks (resolves
  the clash with the existing narrative-block ban). Working rule applied everywhere:
  *whole-function/class "what" → docstring; per-line "why" → `#` comment.*
- **Package-wide style pass on all 14 `dome_nav/*.py`**: `#`-what-blocks → docstrings;
  16 lines wrapped to ≤88; DRY (`explore_markers` `new_marker`/`add_point`,
  `explore_telemetry` `telemetry_dir`); single-letter renames (`d/m/r/t/v/n/a/b` →
  intention-revealing). One line left >88 = pre-existing aligned field-comment
  (`frontier_explorer.py:191`).
- **Docs fixed**: `02-doc/notes.md` map-persistence (no auto-resume by design +
  how to resume; corrected stale "map_file_name hardcoded" claim);
  `01-literate/X07` stale `sim_explore.launch.py` refs (file deleted 07-20) +
  `pluggable_explore_manager_node` node name.
- **Launch audit**: all 12 launch files justified, none dead. Doc-rot flagged:
  `spec.md` outdoor row is a planned-future file (kept); `robot_nav_outdoor` /
  `sim_explore` refs otherwise only in `done/` archives.
- **Deferred (needs decision)**: `frontier_params.declare_frontier_params` +
  `merge_tuning` hand-transcribe 13/16 dataclass fields — style_guide SHOULD (:52)
  prefers `dataclasses.fields()`; behavior-sensitive (ROS param types), not done.
- **Known stale literate (pre-existing, not this session)**: `X05-explore_telemetry.md`
  shows a retired `next_run_number`/`exp-NNNN` scheme; real code is
  `build_telemetry_filename` (`e<name><date>.json`). Needs a regen.

## This session (2026-07-23) — F31 pipeline T03–T05 (novelty scorer + clearance)

TF31 T01/T02 were already landed (uncommitted) at session start. Completed:

- **T03** — novelty migrated to a weighted scorer; the two-stage
  short-list-then-re-rank branch in `select_target` is gone. `novelty_top_n` is
  now a deprecated no-op (kept declared so configs don't break). Added
  pipeline-level novelty tests (tie-break toward higher-unknown path).
- **T04** — new pure `clearance_field(data, info)` (multi-source relaxation BFS
  from wall cells `>= OCCUPIED_THRESHOLD` 65 on the SLAM `/map`, 8-connected,
  diagonal √2). Added `keep_clearance_floor` cell filter + `score_clearance_bonus`
  scorer, registered when `w_clearance > 0`.
- **T05** — `FrontierParams`/`FrontierTuning`/`merge_tuning`/`declare` carry
  `w_distance`/`w_novelty`/`w_clearance`, `robot_radius` (R_inscribed source, not a
  separate param), `clearance_margin_m`. `_minmax_normalize` made inf-safe
  (`lo == hi ⇒ zeros`) so a no-wall map's all-inf clearance column is a no-op.
- **Default `w_clearance` 1.0 ⇒ clearance ON by default** (spec intent: fix
  wall-hug). Not sim-verified yet — **T07 must tune**. Maps with no occupied cell
  are unaffected (clearance all-inf → floor/bonus no-op).
- **Tests**: full suite **266 pass** (`/usr/bin/python3 -m pytest test/`, excl. 4
  live-stack `test_map_validation` tests that need a running robot). `colcon build
  --packages-select dome_nav` clean. Gotcha logged: PATH `python` is the platformio
  venv (no numpy) — use `/usr/bin/python3` (memory `project_pure_test_run_recipe`).
- **Params plumbed into launch**: `w_distance`/`w_novelty`/`w_clearance`/
  `robot_radius`/`clearance_margin_m` now exposed on all three explorer launch
  surfaces — `robot_explore` (real, explicit values, robot_radius 0.17),
  `sim_explore_node` (sim args), `just_explorer` (-1 sentinels, the tuning
  harness). `w_clearance:=0.0` = the T07 baseline; default 1.0 = clearance on.
- Remaining TF31: **T06** (feature flags + literate regen), **T07** sim,
  **T08** live.

## This session (2026-07-22) — gate probe (preliminary) + F31 scoring pipeline

Goal: pin down **which gate** stops the wedged robot (F29 mandatory probe).

- `ros2 topic echo /collision_monitor_state` while wedged shows alternating
  `action_type: 3` / `polygon_name: FootprintApproach` ↔ `action_type: 0` (empty).
- Enum verified against jazzy `nav2_msgs/CollisionMonitorState.msg`:
  **3 = APPROACH** (not LIMIT — LIMIT is 4). So the gate is confirmed
  **FootprintApproach APPROACH**, and **no `STOP` (1) appears ⇒ the
  invalid-source / TF-starvation stop is ruled out** for this stall.
- Toggling 3↔0 = publish-on-change: Nav2 commands motion → zeroed → command
  drops → retry. Classic wedge loop.
- **Still open — static check vs approach simulation** (the F29 go/no-go):
  APPROACH covers both the velocity-blind static check (≥ `min_points` 6 scan
  points inside the 0.17 m footprint ⇒ ALL cmd_vel zeroed incl. reverse) and the
  forward simulation (only motion toward points gated). Next measurements:
  - `ros2 topic echo /collision_monitor/collision_points_marker` (lazy topic) —
    count points within 0.17 m of base center.
  - Compare monitor input vs output cmd_vel `linear.x`: ratio 0.0 ⇒ full gate
    (static check likely); 0<r<1 ⇒ simulation throttle.
  - While wedged, publish a small negative-x cmd_vel into the monitor and watch
    the output — direct test whether reverse passes.
  - Decision: ≥6 points inside footprint ⇒ static check ⇒ F29 BackUp escape
    needs the `FootprintApproach.enabled false` dynamic toggle; <6 ⇒ plain
    BackUp viable.
- F29 has **no task file yet** (TF29 needed before code; probe should be T01).

Also this session — **frontier goals hug walls** → wrote F31 goal-scoring pipeline:

- **Root cause**: `find_frontier_clusters` buffers only against *unknown*
  (`buffer_cells` rings from the `-1` boundary); occupied cells never checked.
  `best_cell_in_cluster` filters size/blacklist/dist but has no obstacle-proximity
  test (no `data` in scope). F27 rejects goals *on* lethal cells; near-lethal
  passes → picks hug walls → feeds the F29 collision_monitor wedge.
- **F31 written** (`03-features/notdone/F31-goal-scoring-pipeline.md`, High):
  refactor selection into **filters + weighted scorers** (Nav2 `CriticManager`
  shape) with a per-cycle `CellCtx` + registry from `FrontierParams`. Kills the
  scattered `if params.x` guards and the F15 two-stage novelty hack (novelty
  becomes just a weighted scorer). First new tenant = **obstacle clearance**: new
  `clearance_field` BFS (distance-to-nearest-occupied), floor filter
  (`clearance ≥ R_inscribed + margin`) + bonus scorer (`-clearance`). Two
  must-gets: per-cycle [0,1] normalization; keep cluster/cell two-phase. Parity
  test (clearance weight 0 + novelty migrated = old behavior) is the regression
  anchor. F15/F30 migrate in as tenants; F27 relocates as a cell filter.
- Wall-standoff YAML lever (local `cost_scaling_factor` 5.0→3.0) still pending
  `colcon build` + real retest — F31 attacks the same wall-hug at goal selection,
  upstream of costmap tuning.

## Prior sessions (2026-07-18/20/21) — shipped + diagnosed

- **F27 lethal-goal guard shipped + verified live** (code/tests done; sim T06 /
  live T07 verification tasks still open). Robot no longer sends goals onto
  lethal cells. Fixed the `on_goal_result` stale-callback race (live `TypeError`
  crash) with regression tests.
- **Mini-config crash tuning** (`nav2_params_explore_real_mini.yaml`):
  `time_before_collision` 0.5→1.0, `robot_radius` 0.15→0.17 (true 0.16 + flared
  post base), MPPI+smoother linear speed 0.5→0.25. `STUCK_T_S` 7→20 so the
  explorer stops pre-empting Nav2 recovery.
- **Wedge diagnosis:** start-wedged robot never moves; Nav2's inner recovery is
  ClearLocalCostmap+retry (useless vs a real obstacle); Spin/BackUp live in the
  outer recovery it never reaches within the stuck window. → **F29** custom
  BackUp escape (feature written; collision_monitor source read 2026-07-21
  upgraded the probe to mandatory — reverse may be gated by the static check).
- **F28** (reason-tagged goal exclusion, per-reason TTL) and **F30**
  (path-distance Dijkstra frontier ranking, replaces Euclidean — kills
  through-wall goal picks) feature files written 2026-07-20/21. Neither has a
  task file yet.
- Pi is CPU-starved during nav: MPPI 8.6 Hz vs 20 desired; slam_toolbox TF
  queue-full drops. Throughput problem, upstream of any tolerance tuning.
