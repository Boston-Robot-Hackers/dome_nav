# TF33 — Semantic Exploration, Phase A (contract + adapter + launch) for F33

Phase A only. Phases B and C get their own task files once Phase A validates
the decisions recorded in the F33 feature file.

**Settled decisions**: frame of record = `map`; contract = a typed
`SemanticTarget` msg in a new msgs package; the semantic map is owned by a new,
neutral package (`dome_semantic`).

## Build order — interleaved with TF35 (F33 G9)

**TF35 (the `dome_mission` mission-sequencing layer) is now fully complete** —
merged to `dome_nav`'s `main` at `894c799` (2026-08-02). `/intent` ownership and
the semantic-target consumer live in `dome_mission`, not here. This task file
is *not* an independent sequence; the real build order across the two features
was:

1. ~~TF35 T01–T04~~ — *done*: `dome_mission` skeleton, `ExploreArea` action,
   mission FSM, `/intent` moved out of dome_nav.
2. ~~TF33 T01–T04 (this file)~~ — *done (2026-08-03)*: msgs, tracker port,
   map-frame recording, typed publishing. `SemanticMapNode` now publishes a
   real, live `/semantic/targets` feed.
3. ~~TF35 T05~~ — *done*: `dome_mission` consumes `SemanticTargetArray`. Was
   blocked on T04 publishing something real; now unblocked end-to-end
   (message type since T01, live publisher since T04) — not yet
   live-verified together on one running stack (that's T08).
4. ~~TF35 T06/T07~~ — *done*: dome_nav cleanup, `ExploreArea` action server,
   top-level launch, live sim bring-up verified 2026-08-01.
5. ~~TF33 T06~~ — *done (2026-08-03)*: persistence keyed to SLAM map identity.
6. **TF33 T07 (fake detection producer for sim) is the next unstarted piece
   of work** — see below. T08 (sub-stack launch composition) still needs T07
   first (its own test plan leans on the fake producer).

**Consumer of everything this task publishes is `dome_mission`, never
dome_nav** — dome_nav must never gain a `dome_semantic_msgs` dependency.

---

## T01 — `dome_semantic_msgs` package

**Status**: done

Already exists as a sibling workspace package (`src/dome_semantic_msgs/`,
committed, `ament_cmake`, no node deps) and **matches the original spec
exactly**:

- `SemanticTarget.msg` — `schema_version` (uint8), `target_id` (string),
  `label` (string), `pose` (`geometry_msgs/Pose`, `map` frame, yaw via
  orientation), `observation_count` (uint32), `last_seen`
  (`builtin_interfaces/Time`), `track_ids` (string[]).
- `SemanticTargetArray.msg` — `std_msgs/Header` + `SemanticTarget[]`.

Both message comments already document the ownership split ("Owned/published
by dome_semantic; consumed by the dome_mission go-to-target behavior").

**Test**: `colcon build` clean; `ros2 interface show` matches spec — *verify
this still holds*, but no further authoring work is needed here.

## T02 — `dome_semantic` package: port the tracker core (pure)

**Status**: done (2026-08-02)

New sibling repo `~/ros2_ws/src/dome_semantic` (full bootstrap: `.claude/`
copied from `~/j3`, `02-doc/`, `03-features/`, `04-tasks/`, `05-issues/`
scaffolded — F33/TF33 records **stay in dome_nav** for now, same
relocate-with-the-code pattern F35 used, moving only once the extraction is
further along).

**Ported, behavior-preserving** (tolerances/thresholds unchanged):
`world_tracker.py` + its full pure dependency closure —  `association.py`,
`class_profiles.py`, `size_estimate.py`, `targets.py`, `tracker_config.py`,
`geometry.py` (needed transitively: `size_estimate.estimate_size_m` uses
`geometry.Intrinsics`, and the ported tests exercise it).

**One deliberate deviation**: `world_tracker.py` imports only `cosine_sim`
from `dome_vision`'s `embedding.py` — but that file also defines
`EmbeddingExtractor`, which pulls in `torch`/`torchvision`/`cv2` at module
level. Copying it whole would force those heavy ML/CV deps onto a package
meant to be a lightweight, ROS-free tracking core. Instead, `cosine_sim` was
extracted into a new 15-line `embedding_similarity.py`; `EmbeddingExtractor`
stays in `dome_vision` (it does image inference, not tracking math — no
test imported `embedding.py`, so nothing lost).

**Not done**: deleting anything from `dome_vision`. `dome_vision_ros`'s
`semantic_map_node.py` still imports `dome_vision.world_tracker.WorldTracker`
directly and is the live, currently-deployed tracker — cleanup waits until
`dome_semantic`'s own ROS node (T03/T04) is built and proven, per the same
delete-after-validation pattern F35/TF35 T06 used. See
`dome_semantic/02-doc/notes.md`'s "Watch list" section: four **not-done**
`dome_vision` features (F39 duplicate-target collapse, F40 odom-stopped tight
radius, F56 WorldTracker architecture split, F57 target data-model
consolidation) target this same code and stay `dome_vision`'s concern until
that cleanup happens — after which they'd need re-authoring against
`dome_semantic`, not a silent carry-over.

**Test**: dome_vision's existing pure tracker tests came along and pass
under plain `pytest` in the new package — 83 pass
(`/usr/bin/python3 -m pytest test/` inside `dome_semantic`); `colcon build
--packages-select dome_semantic` clean. Two of the original tests
(`test_appconfig_world_tracker_section`,
`test_appconfig_world_tracker_defaults_when_absent`) were dropped as
out-of-scope — they test `dome_vision`'s own `AppConfig`, not `WorldTracker`
itself. `tests/conftest.py`'s cwd-fixture and the
`examples/configs/class_profiles.yaml` fixture were ported alongside the
code so the remaining tests resolve their relative paths correctly.

## T03 — `map`-frame recording + re-basing on map jumps

**Status**: done (2026-08-02)

**Not a port** — unlike T02, none of this existed anywhere in the codebase.
`dome_vision_ros`'s current `semantic_map_node.py` only transforms
camera→**`odom`** (never `map`) and has no re-basing logic at all; F33's own
G1 decision (frame of record = `map`, re-base on `map→odom` jumps) had never
been implemented.

**`dome_semantic/tf_adapter.py`** — ported `TFAdapter` (generic
source/target-frame point transform via an injected `tf2_ros.Buffer`),
retargeted to `map`. One deliberate behavior change from the original: the
transform lookup now uses the detection's own stamp instead of
time-zero/"latest available" — F33 G1 says "at observation time," and the
original silently ignored its own `stamp` parameter.

**`dome_semantic/map_rebasing.py`** (new, pure, no ROS imports) — `PlanarTransform(x, y, yaw)`
plus SE(2) `apply_transform`/`invert_transform`/`compose_transforms`;
`has_jumped(old, new, epsilon_m, epsilon_rad)`; `rebase_delta(old, new)`
composes the transform mapping an old-map-frame point to its new-map-frame
position; `rebase_tracker(tracker, delta)` re-bases every stored
`WorldTracker` target's `xyz_world`, `position_history`, and
`pos_welford_mean` in place (z untouched — SLAM corrections are planar).

**`dome_semantic/semantic_map_node.py`** (new) — subscribes
`/oak/detections_3d`; `maybe_rebase()` looks up `map→odom` each tick via
`lookup_transform`, seeds a baseline on first success (no false jump on
frame one), and re-bases via the above when a jump is detected; missing TF
warns (throttled) and drops the affected detection(s) without crashing.
**Scope boundary vs. T04, matching this task file's own split**: no
`SemanticTargetArray` publishing or param wiring yet — this node only
proves the TF integration end-to-end, using a default `WorldTrackerConfig()`.

**Test**: 25 new tests (`test_tf_adapter.py`, `test_map_rebasing.py`,
`test_semantic_map_node.py`) — known transform → expected map-frame
position; synthetic `map→odom` jump → stored targets re-based to identical
positions (confirmed via SE(2) composition round-trip and an
identity-delta no-op case); missing TF → warned + dropped, no crash. Node
tests follow `dome_nav`'s own `tf2_ros.TransformListener`-patching pattern.
110 total dome_semantic tests pass; `colcon build --packages-select
dome_semantic` clean.

## T04 — Typed publishing + node wiring

**Status**: done (2026-08-03)

`SemanticMapNode` now publishes `SemanticTargetArray` on `/semantic/targets`
(the topic `mission_node` already subscribes — `dome_mission`'s
`label_resolver.py`), `/targets/markers` in `map` frame (sphere + label +
leader-line per confirmed target, faint dots for potentials), unthrottled
`/targets/assoc_diag` (JSON, only when `use_class_profiles` is on), and a
`/describe_scene` Trigger service. `/semantic/targets` and `/targets/markers`
are throttled by a new `publish_every_n` ROS int param (default 5) — a
publish-cadence concern kept separate from `WorldTrackerConfig`, which owns
tracking-behavior tuning only.

**Confirmation-threshold params**: new `dome_semantic/tracker_params.py`
(`declare_tracker_config(node)`) walks `WorldTrackerConfig.model_fields` and
declares every field as a ROS param (skips `class_profiles_inline`, a
non-scalar type, and `fx`/`fy`, unused until size estimation is wired in) —
mirrors dome_nav's F34 "dataclass is the single source of truth" pattern,
adapted for pydantic's `model_fields` instead of `dataclasses.fields()`. The
function is a pure `node`-duck-typed helper (declare_parameter/get_parameter
only), so it's tested with a `FakeNode`, no live rclpy context needed —
same testability trade as dome_nav's `declare_frontier_params`.

**Considered and deferred**: matching dome_nav's fuller pattern
(`ParameterDescriptor(description=...)` per field, `ros_important`/
`ros_dynamic` metadata, strict bool/int/float-only type gating) — not a
direct port since `WorldTrackerConfig` is pydantic (not a dataclass) and
already has a `str` field (`class_profile_path`) dome_nav's narrow type gate
doesn't support. `ros_important`/`ros_dynamic` are documentation-only no-ops
in dome_nav's own code today, so skipping them costs nothing functional.
Worth revisiting if `tracker_params.py` grows a real `ros2 param describe`
consumer.

**Consumer is `dome_mission` (already built, TF35 T05) — this task only
publishes.** Nothing in dome_nav subscribes to this topic.

**Test**: `test_tracker_params.py` (4 tests — declare/read round-trip, field
coverage minus exclusions, launch-override precedence, `None`-default
handling) + 5 new `test_semantic_map_node.py` tests (confirmed-target
publish with correct label/pose/count/track_ids, potentials excluded,
throttle honored, `/describe_scene` with/without targets). 119 total
dome_semantic tests pass; `colcon build --packages-select
dome_semantic_msgs dome_semantic` clean (after clearing a stale
`build/dome_semantic_msgs` CMake cache left over from an earlier
misconfigured first build — unrelated to this task's code).

**Literate**: full 01-literate/ set generated for dome_semantic (00-overview.md
theory-of-operation plus one dependency-ordered chapter per module) — the
package had none before this task. Extended again under T06 below to cover
the new persistence module.

## T05 — nav_manager consumes the typed msg

**Status**: superseded by F35 (2026-07-31) — consumer moved to `dome_mission`,
not dome_nav (F33 G9)

Label→pose resolution lives in the mission layer; dome_nav never depends on
`dome_semantic_msgs`. **This is already done**, as `dome_mission`'s
`label_resolver.py` + `mission_node`'s `SemanticTargetArray` subscription
(schema-version-gated). The G2 contract bug fix (typed msg replacing
schemaless JSON) landed there too. This task stays here only as a pointer —
*no work remains under this task file*.

## T06 — Persistence keyed to SLAM map identity

**Status**: done (2026-08-03)

New `dome_semantic/semantic_persistence.py`: `save_semantic_map`/
`load_semantic_map` write/read `~/.dome/semantic_maps/<map_name>.json`
(mirrors `explorer_manager_node`'s `map_name` param and `slam_manager`'s
`--map_name` CLI convention — same `~/.dome/` root as `slam_maps/` and
`telemetry/`). File is a small envelope, `{"map_name": ..., "targets":
[...]}`, wrapping `WorldTracker.to_json()`'s existing bare-list format
rather than teaching the pure tracking core about map identity.

**Matching a save to a load**: primary key is the filename (derived from
`map_name`, sanitized), but the sanitizing regex is lossy (`"room a"` and
`"room#a"` both become `room_a.json`) — so the envelope's own `map_name`
field is a second, independent check. Missing file → fresh tracker (normal
first-run case, no warning). Anything else that isn't a matching envelope —
wrong `map_name`, or an unrecognized shape (e.g. an old dome_vision-era bare
JSON array, which predates map_name-keying entirely and can't exist at a
`map_name`-derived path today) — falls into one unified path: fresh tracker
+ warning, **never a silent merge**. **Decided against migrating the old
bare-array format**: no such file has ever existed at the new keyed path,
so a migration branch would be speculative code for a case that can't occur
yet; the style-guide MUST (persisted-format changes preserve old files or
ship defaults) is satisfied by the safe fresh-map fallback, not a format
converter.

**Node wiring**: new `map_name` ROS param (default `"unknown"`, matching
`explorer_manager_node`'s own default) selects which saved map to restore.
`SemanticMapNode.__init__` now builds its tracker via
`load_semantic_map(self.map_name, declare_tracker_config(self), ...)`
instead of a bare `WorldTracker(...)`; a new `node.save()` method is called
from `main()`'s `finally` block (this node is a plain `Node`, not a
`LifecycleNode`, so `main`'s teardown is the shutdown hook, not
`on_shutdown`).

**Test**: `test_semantic_persistence.py` (6 tests — round-trip, missing
file, unrecognized-format fallback with warning, real filename-collision
mismatch with warning + fresh tracker, path sanitization, envelope
contents) + 2 new `test_semantic_map_node.py` tests (construction restores
a pre-existing map, `save()` writes current tracker state). 127 total
dome_semantic tests pass; `colcon build --packages-select dome_semantic`
clean. Literate: new `12-semantic_persistence.md` chapter;
`13-semantic_map_node.md` (renumbered from 12) gained a "Persistence"
section; `00-overview.md` architecture diagram, reading-order table, and
"what's not wired in yet" list updated to match.

## T07 — Fake detection producer for sim

**Status**: not done

`tools/fake_detections.py` (or a `dome_semantic` test util) publishes
`/oak/detections_3d` from a scripted scenario (object list + noise + dropout),
analogous to `tools/nav_intent_check.py`. This unlocks sim-based regression of
the integrated mode without OAK hardware (G6) and feeds the T08 integration
test.

**Test**: smoke test — node under test confirms a scripted object end-to-end
in sim time.

## T08 — Perception+SLAM+explore+semantic sub-stack

**Status**: not done

**Launch ownership is already settled** (F33 G9 / TF35 T07): the top-level
launch is `dome_mission`'s `mission_explore.launch.py`, with `dome_mission` as
the `/intent` front-end. *This task owns only the sub-stack it composes into
that launch* — not a second top-level launch.

Delivers the OAK-D + `slam_toolbox` + Nav2 + `explorer_manager_node` +
`dome_semantic` group (in `robot_explore_semantic.launch.py` or a dedicated
include file), with documented ordering constraints — TF chain before
detections flow, reusing the `sim_nav_full` `map→odom` wait pattern already
proven in TF35 T07's live sim bring-up.

No direct `/intent` wiring into the explorer here — that's `dome_mission`'s
(done, TF35 T04). `mission_explore.launch.py` currently `bl.include`s this
sub-stack's *absence* with an explicit "TF33 uncoded" note; landing this task
means updating that include, not writing a fresh launch file.

**Also includes** the Phase A ingestion-gating decision (F33 constraints):
gate semantic ingestion on robot motion, or accept degraded association while
driving — *measure and pick one, record why*. Sim variant uses the T07 fake
producer.

**Test**: sim integration test (marked ROS2-runtime per style guide) — explore
with fake detections → semantic map contains the scripted objects in `map`
frame within tolerance. Manual: real-robot run recorded in notes.

## T09 — Pi CPU headroom measurement

**Status**: not done, manual

Run vision + explore + semantic map together on the Pi; record MPPI control
rate (baseline 8.6 Hz vs 20 desired), TF queue drops, load average. Image pubs
off, on-device NN only. Decides whether Phase A ships on-Pi or needs rate
caps/offboard (G7).

**Test**: manual — numbers recorded in `02-doc/notes.md`.

## T10 — Docs, literate, current.md

**Status**: not done

Update `02-doc/current.md` (F33 underway, Phase A status), regenerate literate
for every changed dome_nav Python module, note the new packages in
README/CLAUDE.md if layout docs reference package lists.

**Test**: full suite green (`/usr/bin/python3 -m pytest test/`) + `colcon
build` clean across dome_nav, dome_semantic, dome_semantic_msgs.
