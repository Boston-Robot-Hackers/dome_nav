# dome_nav — Current Session Handoff

Concise cold-start orientation covering roughly the last week. Older entries
move to `02-doc/changelog.md` (rule in `.claude/process.md`) — do **not**
re-narrate git history here.

**Date:** 2026-08-21 · **Branch:** main

## This session (2026-08-21) — dev VM host migrated to M4 Pro, 8 vCPUs

Dev VM's host machine changed to a MacOS M4 Pro; the VM itself now reports
8 vCPUs (`nproc` inside the VM), up from the documented 1-core bottleneck.
Exceeds the 4–6 vCPU target from the Pi-CPU/Nav2-serialization notes below.
Doc-only update — no code changed. Still pending: live-verify that the
intermittent Nav2 action-ACK timeouts (attributed to single-core
serialization) are actually resolved under the new host.

## This session (2026-08-03) — F33/TF33 feature and task records relocated to dome_semantic

`03-features/notdone/F33-semantic-exploration.md` and
`04-tasks/notdone/TF33-semantic-exploration.md` moved to the sibling
`dome_semantic` repo (same paths there). Completes the relocate-with-the-code
step T02 deferred ("F33/TF33 records stay in dome_nav for now... moving only
once the extraction is further along") — the extraction (T01–T04, T06) is far
enough along that the records now live with the code they describe.
`dome_nav` no longer has any `F33`/`TF33` `.md` records of its own; historical
entries below that reference the old in-`dome_nav` location remain as an
accurate record of what was true when written.

## This session (2026-08-03) — checkpoint: ruff clean, literate synced, T04/T06 committed

Full `/checkpoint` run over today's `dome_semantic` work (T04 + T06). Tests:
dome_semantic 127/127; dome_nav 236/241 (4 known live-stack `test_map_validation`
need a robot + 1 flaky concurrency test that passes in isolation — not a
regression, no dome_nav Python source touched this session). `ruff
check`/`ruff format` now clean across `dome_semantic`, including fenced
Python blocks inside `01-literate/*.md`. One deliberate reversal of a prior
checkpoint's call: 2026-08-02 left two `SIM102` nested-if findings in
`association.py` (T02 port) unfixed as out-of-scope; revisited and applied
since the merge is a pure boolean identity, not a control-flow reshape — see
`dome_semantic/02-doc/current.md` for the full reasoning. Simplified
`semantic_persistence.py`'s legacy-format handling per review (dropped the
speculative old-bare-array migration branch — no such file can exist at the
new keyed path yet).

## This session (2026-08-03) — TF33 T06 done: persistence keyed to SLAM map identity

Continuation of today's TF33 work in the sibling `dome_semantic` package —
the semantic map now survives a node restart.

- **New `dome_semantic/semantic_persistence.py`**: `save_semantic_map`/
  `load_semantic_map` write/read `~/.dome/semantic_maps/<map_name>.json`
  (same `~/.dome/` root as `slam_maps/`/`telemetry/`; `map_name` mirrors
  `explorer_manager_node`'s param and `slam_manager`'s `--map_name`).
  Envelope format `{"map_name": ..., "targets": [...]}` wraps
  `WorldTracker.to_json()` rather than teaching the pure tracking core about
  map identity.
- **Never a silent merge**: the filename is the primary key, but the
  sanitizing regex is lossy (`"room a"`/`"room#a"` collide) — the
  envelope's own `map_name` is a second check. Wrong `map_name` and
  unrecognized formats (e.g. an old dome_vision-era bare-array file) share
  one fallback path: fresh tracker + warning, never a guess. **Decided
  against migrating the old bare-array format** — no such file can exist at
  the new keyed path (map_name-keying didn't exist before this task), so a
  migration branch would be speculative code; the safe fresh-map fallback
  satisfies the style-guide MUST without one.
- **Node wiring**: new `map_name` ROS param (default `"unknown"`); tracker
  now built via `load_semantic_map(...)` instead of a bare `WorldTracker()`;
  new `node.save()` called from `main()`'s `finally` (plain `Node`, not
  `LifecycleNode`, so no `on_shutdown` hook exists to use instead).
- **Tests**: 8 new (6 `test_semantic_persistence.py`, 2
  `test_semantic_map_node.py`) — 127 total dome_semantic tests pass;
  `colcon build --packages-select dome_semantic` clean.
- **Literate**: new `12-semantic_persistence.md` chapter; renumbered
  `semantic_map_node.py`'s chapter 12→13 (gained a "Persistence" section);
  `00-overview.md` diagram/reading-order/"not wired in" list updated to
  match — full set now 14 files (00 + 13 chapters).
- **Also this session**: mermaid diagrams across the literate set
  reoriented to `flowchart TD` (vertical) instead of `LR`/side-by-side
  subgraphs, so they don't shrink to fit on the page.

## This session (2026-08-03) — TF33 T04 done: typed publishing + node wiring

Cross-repo work in the sibling `dome_semantic` package (F33/TF33 records stay
in dome_nav per the T02 pattern). `SemanticMapNode` now publishes a real
`/semantic/targets` feed — `dome_mission`'s consumer (TF35 T05) is unblocked
end-to-end for the first time.

- **New `/semantic/targets` (`SemanticTargetArray`), `/targets/markers`,
  `/targets/assoc_diag`, `/describe_scene`** — ported from dome_vision's
  retired node, retargeted to `map` frame. `/semantic/targets` and
  `/targets/markers` throttled by a new `publish_every_n` ROS param (default
  5); `/targets/assoc_diag` stays unthrottled (tuning-diagnostic feed).
  Target orientation always published identity — `WorldTracker` has no
  object-facing estimate, so there's nothing non-arbitrary to publish; the
  mission-layer consumer uses `pose.orientation` for the robot's own arrival
  heading, not a claim about the object.
- **New `dome_semantic/tracker_params.py`**: `declare_tracker_config(node)`
  walks `WorldTrackerConfig.model_fields` (pydantic) and declares every field
  as a ROS param — mirrors dome_nav's F34 "dataclass is the single source of
  truth" pattern, adapted for pydantic. Pure, node-duck-typed function,
  tested with a `FakeNode` (no live rclpy needed), same testability trade as
  `declare_frontier_params`.
- **Considered, deferred**: matching dome_nav's fuller pattern
  (`ParameterDescriptor` descriptions, `ros_important`/`ros_dynamic`
  metadata, strict bool/int/float type gating) — not a direct port since
  `WorldTrackerConfig` is pydantic (not a dataclass) and already has a `str`
  field dome_nav's type gate doesn't support; the metadata flags are
  documentation-only no-ops in dome_nav's own code today. Worth a real
  `ParameterDescriptor(description=...)` pass later if `ros2 param describe`
  becomes a workflow anyone relies on.
- **Tests**: 9 new (4 `test_tracker_params.py`, 5 `test_semantic_map_node.py`)
  — 119 total dome_semantic tests pass.
- **Build gotcha hit and fixed**: `dome_semantic_msgs` failed with `CMake
  Error: source directory ".../ros2_ws/dome_semantic_msgs" does not exist` —
  a stale `build/dome_semantic_msgs` CMake cache from an earlier
  misconfigured first build, pointing at a path missing `src/`. Fixed by
  clearing `build/`/`install/`/`log/latest_build` for just that package
  (regenerable artifacts, outside any git tree — `~/ros2_ws` itself isn't a
  git repo) and rebuilding. `colcon build --packages-select
  dome_semantic_msgs dome_semantic` now clean.
- **Literate**: full `01-literate/` set generated for `dome_semantic` — a
  `00-overview.md` theory-of-operation plus one dependency-ordered chapter
  per module (12 total). The package had no literate docs before this
  session.
- **Considered and rejected (for now)**: extracting the "declare a typed
  config object as ROS params" pattern into a shared, DOME-independent
  package. Genuinely general in intent, but the two existing implementations
  (dome_nav's dataclass+descriptor version, this session's bare pydantic
  version) haven't converged — extracting now means designing a
  dataclass-and-pydantic API from a sample of two. Revisit on a third
  consumer, or once the two are deliberately unified.

## This session (2026-08-03) — current.md refresh + changelog split

`current.md` had drifted: its last dated entry was 2026-07-31 despite three
more days of committed work (2026-08-01/02), and the recorded branch
(`semantic-exploration`) no longer matched actual (`main`, clean). Backfilled
the missing sessions below from git log + the `TF33`/`F33` files, corrected
the branch, and split entries older than ~1 week into `02-doc/changelog.md`
per a new `.claude/process.md` rule. No code changed.

## This session (2026-08-02) — TF33 T02/T03 done (dome_semantic package + map-frame TF/re-basing); I01 grid-fetch race fixed

- **TF33 T02 — `dome_semantic` package created.** New sibling repo
  `~/ros2_ws/src/dome_semantic` (full `.claude`/doc/feature/task bootstrap;
  F33/TF33 records stay in dome_nav for now, same relocate-with-the-code
  pattern F35 used). Ported `world_tracker.py` + its full pure dependency
  closure (`association.py`, `class_profiles.py`, `size_estimate.py`,
  `targets.py`, `tracker_config.py`, `geometry.py`), behavior-preserving.
  Deliberate deviation: extracted `cosine_sim` into a new 15-line
  `embedding_similarity.py` instead of importing dome_vision's `embedding.py`
  whole (avoids pulling `torch`/`torchvision`/`cv2` into a lightweight
  tracking-only package). `dome_vision_ros`'s live `semantic_map_node.py`
  still uses the old `dome_vision.world_tracker` directly — cleanup deferred
  until `dome_semantic`'s own node (T03/T04) is proven. 83 ported tests pass,
  `colcon build --packages-select dome_semantic` clean.
- **TF33 T03 — `map`-frame recording + re-basing on map jumps.** Not a port —
  this logic never existed (the old node only transformed camera→`odom`, no
  re-basing). New `tf_adapter.py` (retargeted to `map`; transform lookup now
  uses the detection's own stamp per F33 G1, not time-zero/latest). New pure
  `map_rebasing.py` (`PlanarTransform`, SE(2) compose/invert/apply,
  `has_jumped`, `rebase_delta`, `rebase_tracker`). New `semantic_map_node.py`
  subscribes `/oak/detections_3d`, re-bases stored targets on a detected
  `map→odom` jump; missing TF warns (throttled) and drops the detection, no
  crash. Scope boundary: no `SemanticTargetArray` publishing yet (that's
  T04) — this node only proves the TF integration. 25 new tests, 110 total
  dome_semantic tests pass, `colcon build` clean.
- **I01 (dome_mission) — grid-fetch race fixed.** `fetch_grid()`'s
  create/destroy-a-subscription-per-tick pattern raced the
  `MultiThreadedExecutor`'s wait-set rebuild on another thread, occasionally
  raising `InvalidHandle` live. Fixed with standing grid subscriptions
  (`start_grids`/`stop_grids`, same lifecycle as the TF listener; callbacks
  just cache the latest message). Also carries the Kilted
  `nav2_params_explore_real.yaml` fix (`error_code_names` →
  `error_code_name_prefixes`, `enable_stamped_cmd_vel: false`) that TF35 T07
  applied live but never committed.
- **Docs sync**: `.claude/literate.md` + `.claude/process.md` refreshed from
  the master `j3` kit (density/skimmability rules; `.md`-writing formatting
  rules folded in).

## This session (2026-08-01) — live sim bring-up verified; housekeeping

- **TF35 T07 live sim bring-up verified**: `ExploreArea` action server +
  `/intent`-free explorer confirmed against the sim stack, closing the
  "pending a sim host" gap noted 2026-07-31 below.
- **Housekeeping**: `.gitignore` trimmed (common patterns moved to the global
  gitignore); `.claude/` config synced from the `j3` kit
  (`bootstrap.md`, `checkpoint.md` command updates).

## This session (2026-07-31) — F35 mission-layer extracted to dome_mission

New sibling package **dome_mission** (own repo `Boston-Robot-Hackers/dome_mission`)
now owns `/intent` and mission sequencing; dome_nav is navigation **primitives
only**. TF35 T01–T08 all done (T08 landed same day). Live sim bring-up
verified 2026-08-01 (see above).

- **Layering**: `/intent` → dome_mission FSM → `ExploreArea` action (dome_nav
  explorer) + Nav2 `NavigateToPose` (direct). Semantic map (`SemanticTargetArray`,
  `dome_semantic_msgs`) → dome_mission label→pose. dome_nav never depends on
  `dome_semantic_msgs`.
- **dome_nav changes**: `nav_manager` + `nav_manager_node` **deleted** (go-to-label
  moved to dome_mission `label_resolver` + `mission_node`). `explorer_manager_node`
  dropped `/intent`, now exposes the **`ExploreArea` action** (`explore_area`;
  blocking execute + `MultiThreadedExecutor` + reentrant group so the 1 Hz tick and
  feedback run concurrently; `session_outcome` set at the DONE paths). Added
  `dome_nav_msgs` dep. F02/F08 records + `tools/nav_intent_check.py` relocated to
  dome_mission. Suite **231 pass** (4 known live-stack `test_map_validation` need a
  robot).
- **New interface pkg** `dome_nav_msgs` (ament_cmake): `ExploreArea.action`.
- **Verify**: live smoke confirmed `/explore_area` advertised + explorer has no
  `/intent` sub (single-handler invariant met).

## This session (2026-07-30) — F34 tuning single-source DONE + F33 written

F34 complete (T01–T05); F33 written. Committed in `d4a16f4`.

- **F34 tuning single-source — DONE.** Dataclass = single source of truth for
  explorer tuning. **T01**: `declare_frontier_params` declares/reads via
  `dataclasses.fields()` loop; `merge_tuning` deduped to `fields()` iteration +
  shared overlay; per-field metadata (`ros_description`/`ros_important`/
  `ros_dynamic`), scorer weights carry `FloatingPointRange(from_value=0.0)`.
  `prefer_farthest` deleted. **T02**: `blacklist_radius` now a real ROS param
  (was silently pinned 0.5). **T03**: ownership rule settled — *shared iff the
  node itself reads it*; `preferred_goal_distance` moved
  `ExploreParams` → `FrontierParams` (scorer-only); `HelloWorldAlgorithm` gained
  its own same-named step param; shared overlay now exactly 2 fields
  (`max_explore_radius`, `blacklist_radius`); node telemetry key preserved via
  `FrontierAlgorithm.session_params()`. **T04**: launch move transparent (all 5
  files set it by name, now declared by the algorithm); `tunable_parameters.md`
  reconciled. **T05**: suite 281 pass, colcon clean, literate `07`/`08`
  regenerated, DRY chore removed, F34/TF34 moved to `done/`. F34 is the
  **enabler** for F33 Phase B. **Committed** (`d4a16f4`, with F33 files).
- **F33 semantic exploration** (dome_nav × dome_vision) written — explore + recognize
  objects → semantic map in SLAM-map coords, reusable by Mode B go-to-label.
  Settled: frame of record = `map`; contract = typed `SemanticTarget` msg in new
  `dome_semantic_msgs` pkg; map owned by new neutral `dome_semantic` pkg. Phased
  A (contract+adapter+launch) / C (explore-then-survey) / B (vision-aware explore,
  depends F32 revival). **TF33 = Phase A only, T01–T10 not started.** Motivation:
  `02-doc/analysis.md`.
- **Committed** — F34 (T01–T05) + F33 feature/task files landed in `d4a16f4`.

## Status

F35 done: dome_nav is navigation primitives only; `dome_mission` (sibling repo)
owns `/intent` and mission sequencing. **F33 Phase A in progress**: T01–T04
and T06 done (msgs pkg, `dome_semantic` package with ported tracker,
map-frame TF+re-basing, typed publishing, persistence keyed to SLAM map
identity) — `dome_mission`'s consumer is unblocked end-to-end and the
semantic map survives a node restart. T05 superseded/done. **T07 next**
(fake detection producer for sim); T08–T10 not started.

Sim exploration works; robot drives and covers the map (~16 goals over ~9×9 m).
Full sim stack healthy. Real robot: explore runs but **start-wedged near an
obstacle it stalls** (F29, deferred). Mode B (go-to-label) now lives in
dome_mission, not live-verified there yet.

**Dev VM now has 8 vCPUs** (host migrated to a MacOS M4 Pro, 2026-08-21) — was
1 core, the documented cause of Nav2 multi-process serialization/intermittent
action-ACK timeouts. vCPU count now exceeds the 4–6 target; live verification
that the timeouts are actually gone is still pending.

Known-but-unfixed nav tuning:
- Planner choice unsettled: real configs SmacPlanner2D, sim NavFn.
- Real-robot MPPI CPU high; candidates `batch_size` 1000→500, freq 20→10 Hz.
- `FootprintApproach` `enabled: true` needs restoring in
  `nav2_params_explore_sim.yaml` + `nav2_params_explore_real.yaml` (disabled for
  diagnostics).

## Architecture essentials

- **One explorer node for sim and real:** `explorer_manager_node.py`
  (injected `ExplorationAlgorithm`, default `FrontierAlgorithm`). Sim vs real
  differ only by ROS params. Exposes the `ExploreArea` action (`dome_nav_msgs`);
  no `/intent` — that's dome_mission's.
- **F23 decoupling:** node knows nothing about frontiers. Protocol =
  `next_goal(ctx) -> GoalDecision` (`NEW_GOAL/NO_TARGETS_BLOCKED/EXPLORED_DONE`);
  viz/diag/telemetry are optional opaque hooks via `getattr`. Frontier params
  self-declared by the algorithm (`frontier_params.FrontierParams`).
- **No YAML patching.** `config/` holds standalone commented copies of upstream
  defaults; launch files load them verbatim. Derived configs mark deltas with
  `# UPSTREAM <val>: why`.
- **slam** runs via upstream `online_async_launch.py`; maps persisted by
  `slam_manager_node` (`--map_name`). Re-running an existing name **overwrites**.
- **Gotcha — copy-install:** `colcon build --packages-select dome_nav` after
  every source edit.
- **Gotcha — orphan processes:** stale nodes/`gz sim` cause TF/clock collisions;
  `ps` audit + `kill -9` beats `pkill -f`.

## Key params (real default / sim override)

Frontier params owned + self-declared by `FrontierAlgorithm`; node declares only
the shared set.

- `min_frontier_dist`: 0.5 / **0.9** m; `max_frontier_dist`: 0.0 / **15.0** m
- `min_frontier_size`: 15 default / **5** sim / 10 real (launch)
- `frontier_buffer_cells`: 2; `goal_inset_m`: 0.3
- `preferred_goal_distance`: 1.0 real / 2.0 sim — `min |d - preferred|`
- `use_novelty_scoring`: False (F15, opt-in)
- `max_explore_radius`: 0.0; `blacklist_radius`: 0.5 m
- Node constants: `EXPLORE_HZ` 1, `NO_FRONTIER_PATIENCE` 14,
  `GOAL_TIMEOUT_S` 25, `STUCK_T_S` 20, `MAX_GOAL_ATTEMPTS` 8,
  `LETHAL_THRESHOLD` 99 (scaled OccupancyGrid — one scale, node + diagnostics)

## Launch

```bash
# Real robot — base stack first (no nav), then a mode:
bl dome2 robot.launch.py --options "drivers control vision voice"
bl dome_nav robot_map.launch.py --map_name <name>      # Mode A: mapping (slam)
bl dome_nav robot_nav.launch.py                        # Mode B: AMCL nav
bl dome_nav robot_explore.launch.py --map_name <name>  # Mode E: autonomous explore

# Sim — single command:
bl dome_nav sim_nav_full.launch.py --map_name <name> --world_name multi_room
# sim_rviz.launch.py separate optional window.

# Experiment harness (trimmed nav2, C2 CPU fix):
bl dome_nav nav_experiment.launch.py
```

`just_explorer.launch.py` is new + untracked (explorer node alone).

## Exploration control

```bash
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_stop\",  \"source\": \"cli\", \"slots\": {}}"'
ros2 topic echo /explore/status
tail -f ~/.dome/telemetry/e*.json       # e<mapname><dd-mmm>.json (F17)
```

`/intent` is now published by **dome_mission**, not dome_nav; the commands
above only work if dome_mission is running. Intent contract: `nav go <label>`→
`navigation_go {label}`, `nav cancel`→ `navigation_cancel`, `nav explore`→
`exploration_start`, `nav explore stop`→ `exploration_stop`. `/explore/markers`:
frontiers yellow, blacklist red, goal cyan.

## Collision monitor probe commands (F29 investigation, deferred)

```bash
ros2 topic echo /collision_monitor_state                      # action change; 3=APPROACH, 1=STOP
ros2 topic echo /collision_monitor/collision_points_marker    # base-frame points (lazy)
# throttle = out/in cmd_vel linear.x ratio
ros2 param set /collision_monitor FootprintApproach.enabled false   # dynamic escape toggle
```

## Next steps

1. **TF33 T07** — fake `/oak/detections_3d` producer for sim in
   `dome_semantic` (analogous to `dome_mission`'s `tools/nav_intent_check.py`),
   unlocking sim-based regression without OAK hardware and feeding T08.
2. ~~Finish gate probe → write TF29~~ — **F29 deferred 2026-07-29 (intentional).**
   The wedge cure is parked; live-verify blocked on it (F10/F27/F31) is parked too.
   Probe artifacts kept for whenever F29 is revived: `scratchpad/count_footprint_points.py`
   (R=0.17, min_points=6).
3. ~~Give the dev VM 4–6 vCPUs~~ — **done, 2026-08-21** (VM now on new M4 Pro
   host, 8 vCPUs). Live-verify the action-ACK timeouts are gone under the new
   host before fully closing this out.
4. **Restore `FootprintApproach` enabled** in both explore configs.
5. TF15 T05 live verify (novelty on vs off).
6. Real-robot retest of wall standoff (local `cost_scaling_factor` 5.0→3.0).

## In-flight features

- **F33** semantic exploration, Phase A: **in progress** — T01–T04 and T06
  done (`dome_semantic_msgs`, `dome_semantic` package with ported tracker,
  map-frame TF + re-basing, typed `/semantic/targets` publishing, persistence
  keyed to SLAM map identity — the `dome_mission` consumer is unblocked
  end-to-end and the semantic map survives a restart), T05 superseded by F35
  (consumer is dome_mission), **T07 next** (fake detection producer for
  sim), T08–T10 not started. dome_nav never depends on `dome_semantic_msgs`
  (F33 G9).
- **F35** mission-layer extraction: **DONE (2026-07-31)** — relocated to
  sibling repo `dome_mission`; F35/TF35 files moved out of dome_nav.
- **F34** tuning single-source: **DONE (2026-07-29)** — moved to `done/`.
- **F27** lethal-goal guard: **DONE (2026-07-29)** — code+tests done, live-observed;
  T06 sim/T07 live marked done with the caveat that both are very hard to really
  verify (can't force a nudged goal onto a lethal cell on demand). Feature + task
  moved to `done/`.
- **F29** BackUp escape: **deferred (2026-07-29, intentional)** — feature file
  only, no TF29; moved to `03-features/deferred/`. Was the intended start-wedge
  cure; parking it means the F10/F27/F31 live-verify blocked on the wedge is
  parked too.
- **F31** goal-scoring pipeline + clearance: **DONE (2026-07-29)** — T01–T08
  complete, sim+live verified. Feature + task moved to `done/`.
- **F30** path-distance ranking: **deferred (2026-07-29)** — feature file only,
  no TF30; moved to `03-features/deferred/`.
- **F28** reason-tagged exclusion: **deferred (2026-07-29)** — feature file only,
  no TF28; moved to `03-features/deferred/`.
- **F32** candidate-source abstraction: **deferred (2026-07-29)** — feature file
  only, no TF32; moved to `03-features/deferred/`. (Depended on F31, now landed;
  also a prerequisite for F33 Phase B.)
- **F26** survey-algorithms paper: TF26 T01–T05 not started.
- **F15** novelty scoring: code done; T05 live verify + literate regen pending.
- **F10** exploration: implementation + unit tests done; open only on live
  verify T06/T07, blocked on the start-wedge — and its cure F29 is deferred,
  so live verify is parked indefinitely.
- **F09** dome_control integration: T04 live smoke pending.
- **F05** rosbag integration test: TF05 written, T01–T07 not started. Also
  covers the F27/F31 heuristic-firing gap (T06).

## Open issues

`05-issues/open/` is empty.
