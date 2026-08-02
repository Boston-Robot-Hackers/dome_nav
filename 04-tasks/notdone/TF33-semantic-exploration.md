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
2. **TF33 T01–T04 (this file)** — msgs, tracker port, map-frame recording,
   publish. *Only T01 is done so far* (see below) — **this is the actual next
   piece of work**, and nothing downstream of it can proceed without it.
3. ~~TF35 T05~~ — *done*: `dome_mission` consumes `SemanticTargetArray` once
   T04 below publishes it. Currently unblocked on the message *type* (T01 is
   done) but still blocked on there being no live *publisher* (T04 not done).
4. ~~TF35 T06/T07~~ — *done*: dome_nav cleanup, `ExploreArea` action server,
   top-level launch, live sim bring-up verified 2026-08-01.
5. **TF33 T08 sub-stack** — still owed here; composed *into* `dome_mission`'s
   existing top-level launch (`mission_explore.launch.py`), which today
   explicitly omits the dome_semantic/OAK-D group with a "TF33 uncoded" note.

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

**Status**: not done

Port `WorldTracker` (two-tier confirmation, label-constrained association,
lost/removed timeouts) from `dome_vision` into a new `dome_semantic` package as
a pure, ROS-free core module — same pure/ROS split discipline as dome_nav's
L0/L1. *Behavior-preserving port, not a redesign*: tolerances and thresholds
carry over unchanged. `dome_vision` keeps OAK/depth/detection publishing; only
the map/tracker moves out.

**Test**: dome_vision's existing pure tracker tests come along and pass under
plain `pytest` in the new package (marker policy: no ROS imports in the core).

## T03 — `map`-frame recording + re-basing on map jumps

**Status**: not done

The ROS shell subscribes `/oak/detections_3d`, transforms each observation
camera→`map` at observation time (TF2, with a timeout/drop policy when the
chain is unavailable — *report, don't silently drop*). It tracks the
`map→odom` transform each tick; when it jumps beyond a small epsilon (a SLAM
correction), it re-bases all stored target poses so they stay in the current
`map` frame. Frame-convention naming per the style guide (camera-frame vs
world-frame values named distinctly).

**Test**: frame-convention tests (style-guide MUST for TF/xyz math) —

- known transform → expected map-frame position
- synthetic `map→odom` jump → stored targets re-based to identical world
  positions
- missing TF → warning + drop counted, no crash

## T04 — Typed publishing + node wiring

**Status**: not done

Node publishes `SemanticTargetArray` on `/semantic/targets` (the topic
`mission_node` already subscribes — see `dome_mission`'s `label_resolver.py`),
plus `/targets/markers` in `map` frame, `/targets/assoc_diag`, and a
`/describe_scene` Trigger service. Confirmation-threshold params
(`min_frames`, `min_time_s`, tolerances) are ROS-declared and
launch-overridable.

**Consumer is `dome_mission` (already built, TF35 T05) — this task only
publishes.** Nothing in dome_nav subscribes to this topic.

**Test**: node-level test with a stubbed TF buffer — *N* detections over *T*
seconds → one confirmed target in the array with the correct label/pose/count.

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

**Status**: not done

Semantic map JSON under `~/.dome/`, keyed to the `slam_manager` `map_name`
(G8), so Mode B reloads the semantic map matching the loaded SLAM map. Save on
shutdown, restore on startup when the key matches. Old dome_vision-era JSON
files: migrate, or ignore-with-warning (style-guide MUST: format changes
preserve old files or handle defaults).

**Test**: round-trip save/restore; missing optional fields from an older file
deserialize with defaults; mismatched `map_name` → fresh map + warning, never a
silent merge.

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
