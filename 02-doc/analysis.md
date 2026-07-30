# Architecture Analysis — dome_nav, and the Vision–Nav Semantic Exploration Requirement

**Date:** 2026-07-29 · **Status:** analysis only — no code changes proposed or made.

Part 1 analyzes the current dome_nav architecture. Part 2 states the new
requirement (marry dome_vision with dome_nav: explore from scratch while building
a semantic map of recognized objects — soda cans and coffee cups). Part 3 surveys
what dome_vision already provides. Part 4 identifies the gaps and tensions. Part 5
lays out architectural options for the refactor.

---

## Part 1 — Current dome_nav architecture

### Layering

Four layers, dependencies only point downward. This is the strongest property of
the codebase.

- **L0 pure (no ROS imports):** `utils.py`, `explore_telemetry.py`, `nav_manager.py`
- **L0.5 contract (ROS msg types, no rclpy):** `explore_context.py` — the
  `ExplorationAlgorithm` protocol (`next_goal(ctx) -> GoalDecision`),
  `ExploreParams`, `RenderContext`
- **L1 pure algorithm core:** `frontier_explorer.py` (grid math, F31
  filters+scorers pipeline), `frontier_params.py`
- **L1.5 msg-coupled helpers:** `explore_markers.py`, `explore_diagnostics.py`
- **L2 algorithm plugins:** `frontier_algorithm.py`, `hello_world_algorithm.py`
- **L3 nodes (rclpy):** `explorer_manager_node.py`, `nav_manager_node.py`,
  `slam_manager_node.py`

The three nodes never talk to each other. Explorer and nav_manager share only
`/intent` and the Nav2 `navigate_to_pose` action server (different modes, never
run together). slam_manager only watches `/map` and calls slam_toolbox services.

### Node roles

- **ExplorerManagerNode** — 1 Hz tick; fetches `/map` + costmaps on demand via
  `wait_for_message` (TRANSIENT_LOCAL) rather than standing subs (Pi CPU);
  TF listener exists only while exploring; owns the session FSM
  (IDLE→EXPL→DONE), goal watchdogs (stuck 20 s, timeout 25 s, wedge = 3
  same-pose stucks → stop), blacklist policy, and costmap guards (lethal,
  bounds). Session constants are class attrs, not ROS params.
- **NavManagerNode** — thin ROS shell over pure `NavManager`: intent-driven
  go-to-label navigation using `/targets/confirmed`, AMCL localization status.
- **SlamManagerNode** — LifecycleNode; persists/restores slam_toolbox posegraphs
  under `~/.dome`; synchronous save on shutdown.

### Algorithm abstraction (F23/F31)

- Node↔algorithm contract: `next_goal(ctx) -> GoalDecision`
  (`new_goal`/`blocked`/`done`), optional `declare_params(node)`, opaque hooks
  via `getattr`: `render_markers`, `exhaustion_report`, `failure_report`,
  `telemetry_extra`, `session_params`.
- F31 pipeline in `frontier_explorer.py`: per-tick `Registry` of cluster
  filters, cell filters, and weighted scorers; per-cycle min-max normalization;
  novelty and clearance are ordinary tenants. Distance-only configuration
  reproduces pre-F31 behavior exactly.

### Pure/ROS split

Fully pure: `utils`, `explore_telemetry`, `nav_manager`, `frontier_explorer`,
`frontier_params`. This split is load-bearing: it enables the ~270-test no-ROS
suite and `tools/algo_demo.py` (pure CLI driving `FrontierAlgorithm` on ASCII
maps).

### Known seams and tensions

- **Param plumbing hand-transcribed 4×** — `FrontierParams` fields repeated in
  the dataclass, `FrontierTuning`, `merge_tuning`, `declare_frontier_params`,
  then again in 3–4 launch files. Highest-leverage cleanup before any new
  algorithmic knob.
- **Tuning in three places with three override mechanics** — node class
  constants, node-declared shared params (`ExploreParams`), algorithm-declared
  params (`FrontierParams`). The `blacklist_radius` split needs a docstring to
  be comprehensible.
- **Fat node** — `explorer_manager_node.py` is ~690 lines with ~6
  responsibilities (FSM, watchdogs, blacklist, costmap guards, status/markers/
  telemetry, TF lifecycle). The natural dumping ground for new behavior, and
  where the design erodes first.
- **Deprecated live wires** — `prefer_farthest`, `novelty_top_n` (no-op),
  vestigial `exploration_resume`/`paused_on_failure`, Mode B hardcoded
  `basement1.yaml` map.
- **Recovery-policy boundary is blurred** — goal watchdogs and costmap guards
  (arguably planner-side policy) live in the node. Parked with F29, but any
  future escape/recovery feature must decide node-vs-Nav2 ownership first.

---

## Part 2 — The new requirement

> The robot starts **without a map** and **explores** the space autonomously
> (Mode E) while the vision package **recognizes objects** (soda cans, coffee
> cups). Each recognized object is **placed on a semantic map** using
> coordinates derived during mapping. Result: after exploration, a metric map
> plus a semantic map of object locations.

This fuses two pipelines that today run in different modes and were never run
together:

- **dome_nav Mode E** — frontier exploration + SLAM (continuous motion).
- **dome_vision spin-survey** — stationary, step-spinning observation epochs
  feeding a `WorldTracker` that confirms targets and publishes them.

"It will require rethinking and refactoring the architecture" — agreed; Part 4
shows the coupling points are real, not cosmetic.

---

## Part 3 — What dome_vision already provides

(From a read-only survey of `/home/pitosalas/ros2_ws/src/dome_vision` —
`dome_vision` lib, `dome_vision_ros` wrapper, `dome_telemetry_msgs`.)

- **OakRoboflowNode** (`dome_vision`): opens the OAK-D directly via DepthAI
  (no ROS camera topics). Roboflow YOLO (`cups-and-cans-again` v9), classes
  **`can` and `cup`**. Publishes `/oak/detections` (Detection2DArray),
  `/oak/detections_3d` (Detection3DArray, xyz in camera frame via on-device
  stereo depth), images, `/oak/markers`, `/telemetry/oak`. Broadcasts
  `target_<track_id>` TF frames under `camera_link`.
- **SemanticMapNode** (`semantic_map`): the existing semantic mapper. Subs
  `/oak/detections_3d` (does TF itself, `inline_tf=True`); pubs
  **`/targets/confirmed`** (String JSON), `/targets/markers` (odom frame),
  `/targets/assoc_diag`; service `/describe_scene` (Trigger → NL summary).
  Persists a JSON map file (`~/.dome/spin_survey_map.json`).
- **WorldTracker** — two-tier confirmation, fully automatic: PotentialTarget →
  ConfirmedTarget after `min_frames=5` spanning `min_time_s=1.0 s`;
  label-constrained distance association (0.3 m / 0.6 m tolerances); lost after
  5 s unseen, removed after 30 s. Optional CLIP-like embedding refinement,
  exchanged only during survey pauses.
- **SpinSurveyNode** (lives in dome_control) — step-spins 360°, pausing 1 s
  every ~50°; `/spin_survey/paused` gates whether SemanticMapNode ingests
  observations. Launch docs cap spin at ≤0.3 rad/s (motion blur).

Key properties of the existing pipeline:

- World frame is **`odom`**, not `map`.
- `/targets/confirmed` payload per target:
  `{target_id, label, x, y, z, observation_count, last_seen_s, track_ids}` —
  **position only, no yaw**.
- Confirmation and association tolerances **assume a stationary or
  pause-stepping robot**; nothing compensates for motion blur or large
  inter-frame displacement while driving.
- Hard dependency: the `odom → base_link → camera_link` TF chain must be up,
  else all 3D detections are dropped.

---

## Part 4 — Gaps and tensions

### G1. Frame mismatch: `odom` vs `map`

Vision builds the semantic map in `odom`; exploration/SLAM produces `map`, and
`map→odom` drifts and **jumps** as SLAM corrects. A semantic map recorded in
raw `odom` during a long explore will smear and will not line up with the saved
SLAM map. Options: record in `map` frame (needs `map→odom` at observation time
and re-basing on map jumps), record in `odom` and accept drift, or store
observations and re-associate post-hoc. This is the single most consequential
decision in the integration.

### G2. `/targets/confirmed` contract mismatch — the two ends disagree today

- dome_vision publishes `{target_id, label, x, y, z, ...}` (odom frame, no yaw).
- dome_nav `nav_manager.is_valid_target` (`nav_manager.py:10-23`) requires
  `{"label", "xyz_world": [x, y, ...]}` and the node consumes `yaw_world`
  (`nav_manager_node.py:83`). Every dome_vision target would be **silently
  dropped** at ingest today. (The current contract was written against
  `tools/nav_intent_check.py`, which fakes it.)

Whichever side adapts, this needs an explicit, versioned contract — ideally a
real ROS msg instead of schemaless JSON (a recurring pattern: intent,
targets, embeddings are all JSON-in-String).

### G3. Stationary-observation assumption vs continuous exploration

The tracker/association tuning (0.3/0.6 m, 5 frames over 1 s, blur caps)
assumes pauses. Mode E drives continuously at up to 0.4 m/s. Either exploration
adopts look-around behaviors (pause at frontiers, scan sweeps), or the vision
side gets retuned/extended for in-motion observation (tighter gating,
velocity-scaled tolerances, blur rejection). This is a behavioral coupling, not
just a data-path one: **goal selection may need to become vision-aware** (e.g.
prefer frontiers that bring unobserved space into camera view, dwell to
confirm a potential target).

### G4. Who owns the semantic map?

Candidates: keep it in dome_vision's SemanticMapNode (it exists, has
persistence and association), move/duplicate it into dome_nav, or a new neutral
package. Ownership determines where the `map`-frame re-basing (G1) and the
nav-side consumption (G2) live. Today dome_nav has zero vision dependencies and
dome_vision has zero nav dependencies — both packages' docs treat the other as
external. A shared-contract package (msgs only) may be the cleanest seam.

### G5. NavManager vs Explorer as the semantic consumer

Two different consumers want target data: NavManager (go-to-label, Mode B) and,
potentially, the exploration algorithm itself (G3: vision-aware goal selection,
e.g. as an F31 scorer or candidate source — F32 candidate-source abstraction
was deferred for exactly this shape). Decide whether the semantic map is only
an *output* of exploration or also an *input* to it.

### G6. Launch/lifecycle composition

Vision is launched from dome2 (`--options "... vision"`), nav/explore from
dome_nav, spin_survey from dome_control. The combined mode needs a single
launch story: OAK-D + depth, slam_toolbox, Nav2, explorer, semantic map — plus
ordering constraints (TF chain before detections flow; `sim_nav_full` already
had to solve a similar map→odom wait). Also: **no sim story for vision** — the
OAK-D is hardware-only, so sim-based regression of the integrated mode needs a
fake-detection producer (analogous to `tools/nav_intent_check.py` but for
`/oak/detections_3d`), or the F05 bag approach extended with vision topics.

### G7. CPU budget on the Pi

Pi is already CPU-starved during nav (MPPI 8.6 Hz vs 20 desired; TF queue
drops). Adding OAK-D depth + YOLO + tracker + semantic association on the same
machine during exploration is a real risk. On-device NN helps; image pubs
(`image_raw`, annotated) should stay off (they already are in the robot
launch). May force rate caps or offboard processing.

### G8. Persistence unification

Three separate persisted artifacts would result: SLAM posegraph+map
(slam_manager, `~/.dome`), semantic map JSON (vision, `~/.dome`), explore
telemetry (dome_nav, `~/.dome/telemetry`). If the semantic map is to be
reusable in Mode B (go-to-can in a saved map), it must be keyed to the **SLAM
map identity** and re-based consistently — ties back to G1 and to
slam_manager's map naming (`--map_name`, overwrite-on-rerun semantics).

---

## Part 5 — Architectural options

### Option A — Minimal coupling (adapter only)

Keep both pipelines as-is. Add a small adapter that (a) converts
`/targets/confirmed` to the nav contract (or version the contract), (b)
transforms targets odom→map at read time, (c) a combined launch file.
Exploration remains vision-unaware; semantic map is purely an output.

- **Pros:** smallest change; respects both packages' independence; ships the
  demo fastest. **Cons:** G3 unaddressed (detections while moving are
  degraded); map-jump smear (G1) handled only at read time, approximately;
  semantic map quality depends on luck of viewpoint coverage.

### Option B — Vision-aware exploration (semantic layer as first-class tenant)

Semantic map becomes an input to goal selection: new F31 scorers/candidate
sources (e.g. viewpoint coverage, dwell-to-confirm), vision-aware behaviors
(pause-and-scan at intervals or at frontiers), contract moved to a real msg in
a shared package, targets recorded in `map` frame with re-basing.

- **Pros:** the actual stated objective (good semantic coverage *while*
  exploring); uses the F31/F32 seams the architecture already grew for this.
  **Cons:** touches the explorer FSM (dwell behaviors conflict with the
  node-level watchdogs — the Part 1 "fat node" tension), larger refactor,
  harder to test without a vision sim story (G6).

### Option C — Two-phase (explore, then survey)

Explore and build the metric map first (Mode E as-is), then run the existing
spin-survey at chosen vantage points with Nav2 navigation between them. Vision
never runs while driving.

- **Pros:** zero changes to the tracker's stationary assumption; reuses both
  pipelines at full quality; simplest data story. **Cons:** not the stated
  objective (objects noted *during* exploration); two passes over the space;
  vantage-point selection is itself a new (smaller) problem.

### Recommendation sketch

Phase it: **A's contract+adapter work is mandatory in all options** (G2 is a
bug today regardless). Then C as the low-risk integration milestone (it only
needs A plus launch composition), with B as the target architecture, scheduled
behind the deferred F32 candidate-source abstraction and a decision on the
node-watchdog boundary. Decide G1 (frame/re-basing) before writing any task
files — it constrains everything downstream.

### Open questions to settle before a feature file

1. Frame of record for the semantic map: `odom`, `map`, or observations +
   post-hoc re-basing? (G1)
2. Contract: version the JSON, or introduce a `SemanticTarget` msg in a shared
   package? Which side adapts? (G2, G4)
3. Is the semantic map an input to exploration (B) or only an output (A/C)? (G5)
4. Where does dwell/look-around behavior live if B: node FSM, algorithm
   plugin, or Nav2 behavior tree? (G3 + the Part 1 boundary tension)
5. Sim/test strategy for the integrated mode without OAK hardware: fake
   detection producer vs rosbag (F05 synergy)? (G6)
6. CPU headroom measurement on the Pi with vision + explore both live. (G7)
