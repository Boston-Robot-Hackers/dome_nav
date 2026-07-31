# TF35 — dome_mission Mission-Sequencing Layer for F35

Extract mission sequencing out of dome_nav into a new neutral `dome_mission`
package, in step with F33 Phase A. Settled (F35): extract in Phase A;
dome_mission owns `/intent`; dome_nav becomes primitives only and never depends
on `dome_semantic_msgs`. Interleaves with TF33 — ordering notes on each task.

## T01 — Settle the dome_nav primitive interface
**Status**: done (2026-07-31)
**Decision (locked 2026-07-31)**:
- **Transport: ROS actions** (not topics/services). Rationale below.
- **Interface home = new `dome_nav_msgs` (ament_cmake)** (2026-07-31).
  dome_nav is `ament_python` and cannot generate rosidl interfaces, so the
  action lives in a domain-scoped interfaces package `dome_nav_msgs` — parity
  with G2a (`dome_semantic_msgs` for semantic) and the `dome_telemetry_msgs`
  precedent. Not a broad shared bag; isolates rebuild blast-radius.
- **explore = new `ExploreArea` action** in `dome_nav_msgs`. Long-running,
  cancellable (preempt on `explore stop`), feedback for progress.
  - **Goal**: `string map_name` (SLAM map identity, matches slam_manager;
    empty = use the running session's map). No bounds field for now — explore
    runs until no frontiers (spec is whole-space survey); add bounds later only
    if a scoped-area verb appears.
  - **Feedback**: `uint32 frontiers_remaining`, `float32 explored_area_m2`,
    `geometry_msgs/Point current_goal` — enough for the mission FSM/telemetry
    without leaking frontier internals (stays behind F23 decoupling).
  - **Result**: `uint8 outcome` with `EXPLORED_DONE=0` (no frontiers left),
    `STOPPED=1` (preempted), `NO_TARGETS_BLOCKED=2` (all blacklisted) —
    mirrors the existing `GoalDecision` enum so the explorer maps 1:1.
- **drive-to-pose: no new dome_nav surface — dome_mission calls Nav2
  `NavigateToPose` directly.** dome_nav adds *only* `ExploreArea`. Label→pose
  resolution lives in dome_mission (T05); it already holds the semantic map, so
  routing the pose back through dome_nav would be a pointless hop. Keeps
  dome_nav free of any go-to-label / semantic knowledge (F35 core goal).
  Consequence: dome_nav's primitive surface = `ExploreArea` action + whatever
  the explorer already exposes; `nav_manager` label logic is deleted (T06),
  not re-exposed.
**Description**: Decide and spec how dome_nav exposes primitives to
dome_mission: **explore start/stop** and **drive-to-pose**. Options weighed in
F35 (ROS actions / command+status topics / services). Deliverable is a written
decision in this task + the interface definition (action/msg/srv names, fields,
which are new vs reuse Nav2's `NavigateToPose`). Recommendation to evaluate:
**ROS action for explore** (long-running, cancellable, feedback/result — the
mission FSM needs preempt on `explore stop`) + **reuse Nav2 `NavigateToPose`**
for drive-to-pose (already an action; dome_mission can call it directly, so
"drive-to-pose" may need no new dome_nav surface at all). Settle whether
drive-to-pose routes through dome_nav or dome_mission talks to Nav2 directly.
Gate: nothing else in TF35/TF33 builds until this lands.

**BT-compatibility rationale (2026-07-31).** Choose ROS actions partly for
FSM→BT forward-compatibility. Behavior-tree leaves are long-running,
cancellable, and report `RUNNING/SUCCESS/FAILURE` — a 1:1 map onto ROS action
goal/feedback/result/cancel (`py_trees_ros` ships action-client leaves for
exactly this). Command/status topics are a poor BT substrate (no result
contract, no clean cancel; each leaf re-implements status). Picking actions now
means a later FSM→BT swap is *swap the orchestrator, keep the interface*, not a
re-plumb. dome_mission stays an **FSM for Phase A/C** (3 verbs, mostly
sequential — a BT is YAGNI here); BT is a **Phase-B** candidate for reactive
vision-aware behaviors, and if adopted it is **`py_trees_ros` (Python)**, a
separate higher tree — *not* Nav2's internal C++ BT (which we never touch, and
which would break the pure-Python discipline).
**Test**: n/a (design decision). Interface files get tests when defined in T02.

## T02 — `dome_mission` package skeleton + interface artifacts
**Status**: done (2026-07-31)
**Description**: Two parts. (a) **`dome_nav_msgs`** (ament_cmake) holding
`ExploreArea.action` per the T01 spec — the interface package, built first
(ROS convention: interfaces before nodes). (b) **`dome_mission`** package
(ament_python, pure/ROS split per dome_nav L0/L1); empty FSM node boots.
`dome_mission` depends on `dome_nav_msgs` + `dome_semantic_msgs`.
**Done (2026-07-31)**: `dome_nav_msgs` (ament_cmake) holds
`action/ExploreArea.action` matching T01 exactly (goal `map_name`; result
`outcome` w/ EXPLORED_DONE/STOPPED/NO_TARGETS_BLOCKED; feedback
`frontiers_remaining`/`explored_area_m2`/`current_goal`). `dome_mission`
(ament_python) skeleton created: `mission_node` boots+idles, depends
`dome_nav_msgs`+`dome_semantic_msgs`. Both are ungit'd `ws/src/` siblings of
dome_nav (not in this repo). `colcon build` clean; `ros2 interface show`
verified; node spins idle.
**Test**: `colcon build` clean across `dome_nav_msgs`, `dome_mission`;
`ros2 interface show dome_nav_msgs/action/ExploreArea` matches the T01 spec;
node starts and idles.

## T03 — Mission FSM core (pure) + the three behaviors

**Status**: done (2026-07-31)

**Description**: Pure, ROS-free mission FSM: states for idle / exploring /
locating / going-to-target, transitions driven by intent events + behavior
completion. Three behaviors defined as sequencing logic (not yet wired to ROS):

**explore** (start→run→done-on-no-frontiers), **locate targets** (explore or
survey while semantic ingest runs — the stateful cross-package sequence, F33
Phase C's home), **go to target** (label→pose→drive). Boundary decision
(F35 open q): which stop/abort authority stays in dome_nav's explorer watchdog
vs moves up to the mission FSM — record it here.

**Done (2026-07-31)**: `dome_mission/mission_fsm.py` — pure, no ROS import.
`MissionFsm.on_intent(intent, label, map_name)` and `on_done(outcome)` return
`list[Command]` and mutate `state`; stray/inapplicable events are no-ops
(return `[]`, state unchanged) so the FSM can't wedge. States `IDLE /
EXPLORING / LOCATING / GOING_TO_TARGET`; commands `START_EXPLORE /
CANCEL_EXPLORE / DRIVE_TO_TARGET / CANCEL_DRIVE`. LOCATING drives the same
explore primitive (semantic ingest is external always-on; differs only in
mission intent). GO_TO_TARGET is IDLE-only for now (concurrent preempt deferred
to T05). `Command.label` carries the payload; label→pose resolution stays
downstream (T05).

**Boundary decision (settled)**: per-goal stop/abort authority (goal timeouts,
`STUCK_T_S`, blacklist exhaustion, per-goal reselection) stays DOWN in
dome_nav's explorer watchdog and never surfaces as an FSM event. The FSM owns
only mission-level authority — start/stop/preempt intents + the single terminal
`ExploreArea` outcome (EXPLORED_DONE / STOPPED / NO_TARGETS_BLOCKED). The
watchdog's internal recovery is invisible here; only its final give-up
(NO_TARGETS_BLOCKED) ends the behavior. Recorded in the module docstring.

**Test**: `test/test_mission_fsm.py` — 17 pure unit tests over intent sequences
→ expected transitions + emitted commands, no ROS graph. All pass
(`/usr/bin/python3 -m pytest`).

## T04 — `/intent` ownership moves to dome_mission
**Status**: not done
**Description**: dome_mission node subscribes `/intent` and drives the FSM.
dome_nav's `explorer_manager_node` and `nav_manager_node` **stop** subscribing
`/intent`; they expose the T01 primitives instead. Exactly one `/intent`
handler in the system after this task. Coordinate with TF33 T04 (which assumed
dome_nav-side consumption).
**Test**: node test — `/intent` payloads (`exploration_start/stop`,
`navigation_go {label}`, `navigation_cancel`) drive the FSM to the right
behavior; regression asserting dome_nav nodes no longer subscribe `/intent`.

## T05 — go-to-target: typed-msg consumer + label→pose (supersedes TF33 T05)
**Status**: not done
**Description**: This is the relocated TF33 T05 (F33 G9). dome_mission
subscribes `SemanticTargetArray` (`dome_semantic_msgs`), resolves label→pose
(typed fields incl. yaw), and issues drive-to-pose via the T01 interface. The
G2 contract bug fix lives here — schemaless JSON gone, typed msg in — but the
consumer is dome_mission, so **dome_nav keeps no `dome_semantic_msgs`
dependency**. Retarget `tools/nav_intent_check.py` to publish the typed msg to
dome_mission.
**Test**: regression — old JSON payload rejected with a clear log; valid
`SemanticTargetArray` + `navigation_go can` resolves to the recorded can pose
and emits the correct drive-to-pose command. Carry over the current
go-to-label unit expectations.

## T06 — dome_nav cleanup: primitives only
**Status**: not done
**Description**: Remove mission/label logic from dome_nav: `nav_manager`
label lookup and `is_valid_target` (`nav_manager.py:10-23`,
`nav_manager_node.py:83`) move to dome_mission or delete; `explorer_manager_node`
loses `/intent`, keeps its explore primitive + watchdog (per T03 boundary
decision). dome_nav `package.xml` has no `dome_semantic_msgs` /
`dome_semantic` dep. Verify dome_nav still builds and its unit suite is green.
**Test**: full dome_nav suite green (`/usr/bin/python3 -m pytest test/`);
`colcon build --packages-select dome_nav` clean; grep-assert no
`dome_semantic` import/dep in dome_nav.

## T07 — Top-level launch (composes TF33 T08 sub-stack)
**Status**: not done
**Description**: **Owns the top-level launch (settled 2026-07-31).** Composes
via `bl.include`: pulls in the TF33 T08 sub-stack (OAK-D + slam + Nav2 +
explorer_manager + dome_semantic) and adds `dome_mission` on top as the
`/intent` front-end. No `/intent` wiring into the explorer anywhere — that is
dome_mission's alone (T04). `better_launch` per style guide. TF33 T08 is the
included sub-stack, not a second top-level launch.
**Test**: sim bring-up: `nav explore` via dome_mission drives exploration;
`nav go <label>` drives to a recorded target. Marked ROS2-runtime.

## T08 — Docs, literate, current.md
**Status**: not done
**Description**: Update `02-doc/current.md` (F35 underway, new layering),
regenerate literate for every changed/removed dome_nav module (`nav_manager.py`,
`nav_manager_node.py`, `explorer_manager_node.py`), add dome_mission to package
lists in README/CLAUDE.md, note the layering diagram. Cross-check F33/TF33
reflect the T05 relocation.
**Test**: full suite green across dome_nav + dome_mission; `colcon build` clean.
