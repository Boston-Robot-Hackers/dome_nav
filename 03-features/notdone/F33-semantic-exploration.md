# F33 — Semantic Exploration (dome_nav × dome_vision Integration)

**Priority**: High
**Done:** no
**Tasks File Created:** yes (TF33 — Phase A only; C/B get their own files later)
**Tests Written:** no
**Test Passing:** no
**Description**: The robot starts without a map, explores autonomously (Mode E)
while dome_vision recognizes objects (soda cans, coffee cups), and each confirmed
object is placed on a **semantic map** in SLAM-map coordinates. End result of a
run: a metric map (slam_toolbox, as today) plus a semantic map of object
locations aligned with it, reusable later by Mode B go-to-label navigation.

Motivation and full gap analysis: `02-doc/analysis.md` (2026-07-29). This
feature implements the integration it describes.

## Settled decisions (2026-07-30, author)

- **G1 frame of record: `map`.** Detections are transformed into the `map`
  frame at observation time via live TF (`map→odom→base_link→camera_link`).
  The semantic-map owner re-bases recorded targets when SLAM corrections jump
  `map→odom`, so the semantic map stays aligned with the saved SLAM map and
  works for Mode B reuse (G8).
- **G2 contract: real ROS msg in a shared package.** A new msgs-only package
  (working name `dome_semantic_msgs`) defines a typed, versioned
  `SemanticTarget` msg replacing the schemaless `/targets/confirmed` JSON.
  This also fixes the live bug where `nav_manager.is_valid_target` silently
  drops every dome_vision target today.
- **G2a msgs-package scope: domain-scoped, not a broad `dome_msgs`**
  (2026-07-31). Decided narrow. Rationale: (1) follows the existing
  `dome_telemetry_msgs` precedent (workspace already scopes interface pkgs by
  concern; no broad `dome_msgs` exists); (2) interface pkgs force all
  dependents to rebuild on any change — a grab-bag couples unrelated domains,
  a narrow pkg isolates blast radius (same reason `nav2_msgs`/`geometry_msgs`
  are domain-scoped, not "everything"); (3) matches G4 neutral ownership —
  `dome_semantic_msgs` expresses who owns the contract; a shared bag blurs it;
  (4) YAGNI — no concrete cross-domain second consumer exists. Scope the pkg to
  the whole **semantic-perception** domain (SemanticTarget, SemanticTargetArray,
  future SemanticMap / describe-scene types land here), so it is extensible
  without becoming a dumping ground. Revisit only if a truly cross-cutting type
  shared by unrelated domains (nav + voice + control) ever appears.
- **G4 ownership: new neutral package.** The semantic-map node (adapted from
  dome_vision's `SemanticMapNode` + `WorldTracker`) moves to a new package
  (working name `dome_semantic`) that depends only on the msgs package and TF.
  Neither dome_nav nor dome_vision depends on the other.
- **G5 scope: semantic map is an output first, an input later.** Phase A/C
  treat it as output only; Phase B (below) makes it an input to goal selection.
- **G9 mission-sequencing layer → new `dome_mission` package (F35)**
  (2026-07-31). High-level sequencing (`explore` / `locate targets` /
  `go to target`) and the `/intent` contract move out of dome_nav into a new
  neutral `dome_mission` package, **extracted in Phase A** (see F35). Effect on
  this feature: the typed-msg consumer (**T05** below) moves to dome_mission —
  label→pose resolution lives there, and **dome_nav never depends on
  `dome_semantic_msgs`**. dome_nav becomes navigation primitives only. This also
  resolves the G3 open question (where dwell/sequencing lives: dome_mission,
  not the explorer node FSM / algorithm plugin / Nav2 BT). TF33 T05 is
  superseded by the F35 task file; keep the G2 contract bug fix (typed msg
  replacing schemaless JSON) but its consumer is dome_mission.

## Phasing (per analysis.md recommendation sketch)

Task files are written per phase, not all upfront — each phase is
independently shippable and later phases depend on decisions validated by
earlier ones.

- **Phase A — contract + adapter + launch (this feature's first milestone).**
  Msgs package, neutral semantic-map package with `map`-frame recording and
  re-basing, nav-side consumption via the new msg, combined launch file
  (OAK-D + slam_toolbox + Nav2 + explorer + semantic map, with TF-ordering
  constraints). Exploration stays vision-unaware. Fixes the G2 contract bug.
- **Phase C — two-phase explore-then-survey.** Explore and save the metric map
  (Mode E as-is), then run the existing spin-survey at chosen vantage points
  with Nav2 navigation between them. Vision never runs while driving; reuses
  both pipelines at full quality. Low-risk integration milestone.
- **Phase B — vision-aware exploration (target architecture).** Semantic
  targets become inputs to goal selection: F31 scorers / F32 candidate sources
  (viewpoint coverage, dwell-to-confirm), pause-and-scan behaviors.
  **Depends on reviving F32** (candidate-source abstraction, per analysis.md
  Part 6) and settling where dwell behavior lives (node FSM vs algorithm
  plugin vs Nav2 BT) — the open G3 question.

## Scope

- New `dome_semantic_msgs` package: `SemanticTarget` msg (target_id, label,
  pose in `map` frame incl. yaw, observation_count, last_seen, track_ids,
  schema version).
- New `dome_semantic` package: semantic-map node adapted from dome_vision's
  SemanticMapNode/WorldTracker — subscribes `/oak/detections_3d`, records in
  `map` frame, re-bases on `map→odom` jumps, persists JSON keyed to SLAM map
  identity (`--map_name`, G8), publishes `SemanticTarget` array + markers +
  `/describe_scene` service.
- dome_nav: `nav_manager` consumes the typed msg (replaces the JSON
  `is_valid_target` contract; `tools/nav_intent_check.py` updated to match).
- Combined launch: `robot_explore_semantic.launch.py` (name TBD) composing
  vision + explore + semantic map with documented ordering constraints.
- Sim/test strategy (G6): a fake `/oak/detections_3d` producer (analogous to
  `tools/nav_intent_check.py`) so the integrated mode is testable without OAK
  hardware; synergy with F05 bag-replay approach.

## Constraints

- Cross-repo work: `dome_semantic`/`dome_semantic_msgs` are new packages in
  the workspace; dome_vision's SemanticMapNode/WorldTracker move out of
  dome_vision (adaptation, not fork — dome_vision keeps OAK/depth/tracking).
- No changes to dome_control's intent contract.
- Association/confirmation tolerances still assume stationary or
  pause-stepping observation until Phase B; Phase A/C must not run the
  semantic ingestion while the robot is driving fast (gate on motion or
  accept degraded association — decided in the Phase A task file).
- Pi CPU budget (G7): vision + explore concurrently is a measured risk; image
  pubs stay off, on-device NN only. A CPU-headroom measurement on the Pi is
  part of Phase A acceptance.
- No YAML patching; launch composition via `better_launch` per style guide.

## Open questions (deferred to later phases)

- Where dwell/look-around behavior lives in Phase B (G3 + node-watchdog
  boundary tension, analysis.md Part 1).
- F32 revival framing and the param-plumbing dedup that must land with it
  (analysis.md Part 6) — prerequisite for Phase B, may become its own task
  series under this feature or a revived F32.

## How to Demo

**Setup**: Phase A complete; real robot with OAK-D; fresh `map_name`; cans
and cups placed in the space.

**Steps**:
1. `bl dome_nav robot_explore_semantic.launch.py --map_name <name>`
2. `nav explore` from dome_control (or `/intent` CLI)
3. Robot explores; `/targets/confirmed` (typed msg) accumulates confirmed
   cans/cups in `map` frame; markers visible in RViz/Foxglove
4. On completion: SLAM map saved under `~/.dome/slam_maps/`, semantic map JSON
   saved under `~/.dome/` keyed to the same map name
5. Restart in Mode B against the saved map; `nav go can` drives to a recorded
   can location

**Expected output**: metric map + aligned semantic map from one autonomous
exploration run; Mode B reuses the semantic map without re-surveying.
