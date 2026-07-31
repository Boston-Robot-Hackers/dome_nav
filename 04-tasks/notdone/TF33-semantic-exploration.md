# TF33 — Semantic Exploration, Phase A (contract + adapter + launch) for F33

Phase A only. Phases C and B get their own task files when Phase A validates
the decisions (see F33 feature file). Settled decisions: frame of record =
`map`; contract = typed `SemanticTarget` msg in a new msgs package; semantic
map owned by a new neutral package.

## T01 — `dome_semantic_msgs` package
**Status**: not done
**Description**: New msgs-only package in the workspace. `SemanticTarget.msg`:
schema_version (uint8), target_id (string), label (string), pose
(geometry_msgs/Pose, in `map` frame — includes yaw, fixing the no-yaw gap),
observation_count (uint32), last_seen (builtin_interfaces/Time),
track_ids (string[]). `SemanticTargetArray.msg` wrapping the confirmed set.
No node dependencies — pure interface package so both dome_nav and
dome_semantic can depend on it cheaply.
**Test**: `colcon build` clean; `ros2 interface show` matches spec; fields
cover everything the old JSON payload carried plus yaw and version.

## T02 — `dome_semantic` package: port the tracker core (pure)
**Status**: not done
**Description**: Port `WorldTracker` (two-tier confirmation, label-constrained
association, lost/removed timeouts) from dome_vision into `dome_semantic` as a
pure, ROS-free core module (same pure/ROS split discipline as dome_nav L0/L1).
Behavior-preserving port, not a redesign — tolerances and thresholds carry
over. dome_vision keeps OAK/depth/detection publishing; the map/tracker moves.
**Test**: dome_vision's existing pure tracker tests come along and pass under
plain pytest in the new package (marker policy: no ROS imports in the core).

## T03 — `map`-frame recording + re-basing on map jumps
**Status**: not done
**Description**: The ROS shell subscribes `/oak/detections_3d`, transforms each
observation camera→`map` at observation time (TF2, with timeout/drop policy
when the chain is unavailable — report, don't silently drop). Track the
`map→odom` transform each tick; when it jumps beyond a small epsilon (SLAM
correction), re-base all stored target poses so they stay in the current
`map` frame. Frame-convention naming per style guide (camera-frame vs
world-frame values named distinctly).
**Test**: frame-convention tests (style-guide MUST for TF/xyz math): known
transform → expected map-frame position; synthetic `map→odom` jump → stored
targets re-based to identical world positions; missing TF → warning + drop
counted, no crash.

## T04 — Typed publishing + node wiring
**Status**: not done
**Description**: Node publishes `SemanticTargetArray` on `/targets/confirmed`
(replacing the JSON String — coordinated with T05), `/targets/markers` in
`map` frame, `/targets/assoc_diag`, and the `/describe_scene` Trigger service.
Params for confirmation thresholds (min_frames, min_time_s, tolerances) are
ROS-declared and launch-overridable.
**Test**: node-level test with a stubbed TF buffer: N detections over T
seconds → one confirmed target in the array with correct label/pose/count.

## T05 — nav_manager consumes the typed msg
**Status**: not done
**Description**: Replace the schemaless JSON contract in
`dome_nav/nav_manager.py` (`is_valid_target`, `nav_manager.py:10-23`) and
`nav_manager_node.py:83` with subscription to `SemanticTargetArray`. Label
lookup + yaw now come from typed fields. Update `tools/nav_intent_check.py`
to publish the typed msg (it faked the old JSON). This is the G2 bug fix —
today every real dome_vision target is silently dropped at ingest.
**Test**: regression test — the exact payload shape dome_vision produced
(old JSON) is rejected with a clear log, and a valid `SemanticTargetArray`
parses into the same go-to-label behavior as the current unit tests.

## T06 — Persistence keyed to SLAM map identity
**Status**: not done
**Description**: Semantic map JSON under `~/.dome/` keyed to the slam_manager
`map_name` (G8) so Mode B reloads the semantic map that matches the loaded
SLAM map. Save on shutdown, restore on startup when the key matches. Old
dome_vision-era JSON files: migrate or ignore-with-warning (style-guide MUST:
format changes preserve old files or handle defaults).
**Test**: round-trip save/restore; missing optional fields from an older file
deserialize with defaults; mismatched map_name → fresh map + warning, not a
silent merge.

## T07 — Fake detection producer for sim
**Status**: not done
**Description**: `tools/fake_detections.py` (or a dome_semantic test util):
publishes `/oak/detections_3d` from a scripted scenario (object list + noise +
dropout), analogous to `tools/nav_intent_check.py`. Unlocks sim-based
regression of the integrated mode without OAK hardware (G6) and feeds the
T08 integration test.
**Test**: smoke test — node under test confirms a scripted object end-to-end
in sim time.

## T08 — Combined launch + integration test
**Status**: not done
**Description**: `launch/robot_explore_semantic.launch.py` (better_launch):
OAK-D + slam_toolbox + Nav2 + explorer_manager + dome_semantic, with
documented ordering constraints (TF chain before detections flow — reuse the
`sim_nav_full` map→odom wait pattern). Includes the Phase A ingestion-gating
decision (F33 constraints): gate semantic ingestion on robot motion or accept
degraded association while driving — measure and pick one, record why.
Sim variant uses the T07 fake producer.
**Test**: sim integration test (marked ROS2-runtime per style guide): explore
with fake detections → semantic map contains the scripted objects in `map`
frame within tolerance. Manual: real-robot run recorded in notes.

## T09 — Pi CPU headroom measurement
**Status**: not done
**Description**: Manual. Run vision + explore + semantic map together on the
Pi; record MPPI control rate (baseline 8.6 Hz vs 20 desired), TF queue drops,
load average. Image pubs off, on-device NN only. Decides whether Phase A
ships on-Pi or needs rate caps/offboard (G7).
**Test**: manual — numbers recorded in `02-doc/notes.md`.

## T10 — Docs, literate, current.md
**Status**: not done
**Description**: Update `02-doc/current.md` (F33 underway, Phase A status),
regenerate literate for every changed dome_nav Python module
(`nav_manager.py`, `nav_manager_node.py`, anything else touched), note the new
packages in README/CLAUDE.md if layout docs reference package lists.
**Test**: full suite green (`/usr/bin/python3 -m pytest test/`) + `colcon build`
clean across dome_nav, dome_semantic, dome_semantic_msgs.
