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
**Status**: done (2026-07-31)

**Description**: dome_mission node subscribes `/intent` and drives the FSM.
dome_nav's `explorer_manager_node` and `nav_manager_node` **stop** subscribing
`/intent`; they expose the T01 primitives instead. Exactly one `/intent`
handler in the system after this task. Coordinate with TF33 T04 (which assumed
dome_nav-side consumption).

**Done (2026-07-31)**: dome_mission side wired. `intent_parser.py` (pure):
`parse_intent(json_str) -> ParsedIntent | None` maps the JSON contract
(`exploration_start`→EXPLORE_START w/ optional `slots.map_name`,
`exploration_stop`→EXPLORE_STOP, `navigation_go`→GO_TO_TARGET w/ `slots.label`,
`navigation_cancel`→CANCEL; unknown/malformed → None). `mission_node`
subscribes `/intent`, parses, drives `MissionFsm`, and executes emitted
commands (execution stubbed to logging — real ExploreArea/NavigateToPose
clients land T05/T07).

**Caveat — single-handler invariant not yet complete**: dome_nav's
`explorer_manager_node` + `nav_manager_node` **still** subscribe `/intent`;
their removal is deferred to **T07** (the `/intent`→ExploreArea action swap,
see T06 scope note). Until then, running dome_mission alongside dome_nav
double-handles `/intent`. The "exactly one handler" state is reached at T07,
not here.

**Test**: `test/test_intent_parser.py` (10 pure) + `test/test_mission_node.py`
(6, rclpy — feeds `/intent` String payloads, asserts FSM state + the
`/intent`-subscription invariant). 33 dome_mission tests pass total.

**Test**: node test — `/intent` payloads (`exploration_start/stop`,
`navigation_go {label}`, `navigation_cancel`) drive the FSM to the right
behavior; regression asserting dome_nav nodes no longer subscribe `/intent`.

## Feature/task record relocation (staged, 2026-07-31)

dome_mission is now a full package (own `.claude`, bootstrap scaffold, git repo
`Boston-Robot-Hackers/dome_mission`). Feature/task *records* move from dome_nav
in step with the code, so a record never leads its implementation:

- **F08 / TF08** (typed intent/status, deferred) — **moved now** to
  `dome_mission/03-features/deferred/` + `04-tasks/deferred/` (no code to
  desync; `/intent`+`nav_status` become dome_mission's contract).
- **F02** (intent-navigation, done) — **moves at T06**, when `nav_manager`
  label/nearest-target logic physically relocates/deletes.
- **F35 / TF35** (this file) — **moves at T08** as dome_mission's founding
  record, once the extraction is complete. Stays in dome_nav until then because
  it also drives the dome_nav-side removal (T06).
- **Stay in dome_nav**: all frontier/SLAM/nav primitives (F10/F15/F31/F27/F34/
  F22/F23/F01/F03/F13…), F33 explore side (its go-to-label consumer already
  relocated into T05). F06 localization-status borderline → stays for now.

## T05 — go-to-target: typed-msg consumer + label→pose (supersedes TF33 T05)
**Status**: done (2026-07-31)

**Description**: This is the relocated TF33 T05 (F33 G9). dome_mission
subscribes `SemanticTargetArray` (`dome_semantic_msgs`), resolves label→pose
(typed fields incl. yaw), and issues drive-to-pose via the T01 interface. The
G2 contract bug fix lives here — schemaless JSON gone, typed msg in — but the
consumer is dome_mission, so **dome_nav keeps no `dome_semantic_msgs`
dependency**. Retarget `tools/nav_intent_check.py` to publish the typed msg to
dome_mission.

**Done (2026-07-31)**: `label_resolver.py` (pure) — `TargetPose` dataclass
(`x_m`/`y_m`/`yaw_rad`), `SemanticTargetStore.resolve(label, robot_xy)` = typed
successor to nav_manager `find_nearest_confirmed` (nearest match; `robot_xy`
None → first), `yaw_from_quaternion` helper. `mission_node` subscribes
`SemanticTargetArray` on `/semantic/targets`, gates `schema_version`
(EXPECTED=1, mismatches dropped + warned), converts each `Pose`→`TargetPose` at
the boundary (pure stays ROS-free), tracks robot pose from `/amcl_pose`. On a
`DRIVE_TO_TARGET` command it resolves the label and drives via Nav2
`NavigateToPose` directly (no dome_nav hop); a missing label warns +
`on_done(DRIVE_FAILED)` so the FSM settles back to IDLE.

**Tool**: `nav_intent_check.py` retargeted + **moved to `dome_mission/tools/`**
(it needs `dome_semantic_msgs`, which T06 forbids in dome_nav). Publishes a
typed `SemanticTargetArray` on `/semantic/targets`; the schemaless
`/targets/confirmed` JSON path is gone. Terminal-status assertion dropped —
dome_mission has no status topic yet (F08).

**Contract note**: topic `/semantic/targets` is the dome_semantic → dome_mission
contract; must match the dome_semantic publisher (TF33). `schema_version` gating
is the "old payload rejected" path (wrong version dropped with a warning).

**Test**: `test/test_label_resolver.py` (8: nearest-match carried over from
test_nav_manager, robot-None-first, yaw conversion) + node tests in
`test_mission_node.py` (ingest populates store; schema mismatch dropped;
Pose→yaw conversion; unknown label fails cleanly to IDLE; `goal_pose` encodes
x/y/yaw). **45 dome_mission tests pass.** Full live drive verification lands in
T07 sim bring-up.

## T06 — dome_nav cleanup: primitives only
**Status**: done (2026-07-31) — deletion scope; explorer `/intent`→action swap deferred to T07

**Description**: Remove mission/label logic from dome_nav: `nav_manager`
label lookup and `is_valid_target` (`nav_manager.py:10-23`,
`nav_manager_node.py:83`) move to dome_mission or delete; `explorer_manager_node`
loses `/intent`, keeps its explore primitive + watchdog (per T03 boundary
decision). dome_nav `package.xml` has no `dome_semantic_msgs` /
`dome_semantic` dep. Verify dome_nav still builds and its unit suite is green.

**Scope decision (2026-07-31)**: T06 = **deletion only**. The go-to-label logic
fully landed in dome_mission at T05, so it is deleted here. The
`explorer_manager_node` `/intent`→`ExploreArea` action-server swap is
**deferred to T07**, paired with mission_node's ExploreArea client + sim verify,
so an untested action server never ships and `/intent`-triggered explore never
breaks mid-sequence. Consequence: explorer_manager_node **still subscribes
`/intent`** after T06 — the single-`/intent`-handler invariant (T04) is met at
**T07**, not here (until then explorer + mission_node both handle `/intent`).

**Done (2026-07-31)**:
- Deleted `nav_manager.py`, `nav_manager_node.py`, `test_nav_manager.py`,
  `test_nav_manager_pure.py`, and their literate (`01-literate/03-nav_manager_node.md`,
  `05-nav_manager.md`).
- Removed the `nav_manager_node` entry point (`setup.py`) and the `nav_manager`
  `bl.node` block from `robot_nav.launch.py` + `robot_map.launch.py` (both now
  carry a one-line note that dome_mission provides go-to-target via T07).
- Moved F02/TF02 records to `dome_mission/03-features/done/` +
  `04-tasks/done/` with a relocation banner.
- dome_nav has no `dome_semantic` / `dome_semantic_msgs` dep (never did; F33
  uncoded). explorer_manager_node untouched.

**Test**: full dome_nav suite green (`/usr/bin/python3 -m pytest test/`);
`colcon build --packages-select dome_nav` clean; grep-assert no
`dome_semantic` import/dep in dome_nav.

**Result**: `colcon build --packages-select dome_nav` clean; grep for
`dome_semantic` / `nav_manager` in dome_nav = none. Suite **231 pass**; the 4
`test_map_validation` failures are the known live-stack tests (need a running
robot), unrelated to this change.

## T07 — Explore action swap + top-level launch (composes TF33 T08 sub-stack)
**Status**: done (2026-07-31) — code + unit + node-introspection smoke; full sim bring-up pending a sim host

**Description**: **Owns the top-level launch (settled 2026-07-31).** Composes
via `bl.include`: pulls in the TF33 T08 sub-stack (OAK-D + slam + Nav2 +
explorer_manager + dome_semantic) and adds `dome_mission` on top as the
`/intent` front-end. No `/intent` wiring into the explorer anywhere — that is
dome_mission's alone (T04). `better_launch` per style guide. TF33 T08 is the
included sub-stack, not a second top-level launch.

**Also (moved from T06, 2026-07-31)**: swap `explorer_manager_node` from its
`/intent` subscription to an **`ExploreArea` action server** (goal=start,
cancel=stop, feedback=frontiers/area/current_goal, result=outcome), and wire
mission_node's ExploreArea **client** (replacing the T05 START_EXPLORE/
CANCEL_EXPLORE logging stub). This is the atomic point where explorer loses
`/intent` and the single-`/intent`-handler invariant (T04) is finally met, with
sim bring-up verifying it end-to-end. Adds the `dome_nav_msgs` dep to dome_nav.

**Done (2026-07-31)**:
- **Explorer action server**: `explorer_manager_node` drops `/intent`; exposes
  the `ExploreArea` action on `explore_area`. Blocking `execute_callback` +
  `MultiThreadedExecutor` + a `ReentrantCallbackGroup` (timer + action) so the
  1Hz tick advances the session while the callback publishes feedback
  (`frontiers_remaining` via optional `frontier_count` hook, `explored_area_m2`
  from known map cells, `current_goal`). Session start extracted to
  `start_session(map_name)`; the two DONE paths set `session_outcome`
  (EXPLORED_DONE vs NO_TARGETS_BLOCKED); cancel → STOPPED. `dome_nav_msgs` dep
  added to dome_nav.
- **Mission client**: `mission_node.start_explore` sends the `ExploreArea` goal
  (execute() is now a 4-way dispatch table); result outcome maps back to the FSM
  via `on_done`; `cancel_explore` cancels the goal. Server-not-ready → STOPPED.
- **Top-level launch**: `dome_mission/launch/mission_explore.launch.py`
  `bl.include`s dome_nav `robot_explore.launch.py` + adds `mission_node`.
  dome_semantic/OAK-D sub-stack omitted (TF33 uncoded) — noted in the launch.
- **Invariant met**: explorer no longer subscribes `/intent`; mission_node is
  the sole handler.

**Verification**: dome_nav 64 explorer tests + dome_mission 45 tests pass
(outcome mapping, feedback build, known-area, session transitions, action
accept/reject). Live smoke: booted `explorer_manager_node`, confirmed
`/explore_area` advertised (`dome_nav_msgs/action/ExploreArea`) and **no
`/intent` subscription**. **Not yet done**: full sim bring-up (drive a real
explore + go-to-label in gz) — needs a sim host; this Pi can't run gz+Nav2. Left
as the ROS2-runtime verification below.

**Test**: sim bring-up: `nav explore` via dome_mission drives exploration;
`nav go <label>` drives to a recorded target; assert exactly one `/intent`
subscriber. Marked ROS2-runtime.

## T08 — Docs, literate, current.md
**Status**: not done
**Description**: Update `02-doc/current.md` (F35 underway, new layering),
regenerate literate for every changed/removed dome_nav module (`nav_manager.py`,
`nav_manager_node.py`, `explorer_manager_node.py`), add dome_mission to package
lists in README/CLAUDE.md, note the layering diagram. Cross-check F33/TF33
reflect the T05 relocation.
**Test**: full suite green across dome_nav + dome_mission; `colcon build` clean.
