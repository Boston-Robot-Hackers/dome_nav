# F35 — dome_mission: Mission-Sequencing Layer

**Priority**: High
**Done:** no
**Tasks File Created:** yes (TF35)
**Tests Written:** no
**Test Passing:** no
**Description**: Extract high-level mission sequencing out of dome_nav into a
new neutral package `dome_mission`. dome_nav becomes navigation **primitives
only** (SLAM, plan, drive-to-pose, frontier-explore); dome_mission owns the
mission FSM and the `/intent` contract, orchestrating dome_nav + dome_semantic.
Introduced alongside F33 Phase A: the semantic-target consumer (label→pose
resolution) lands in dome_mission from the start, so dome_nav never gains a
dependency on `dome_semantic_msgs`.

Motivation: the three mission verbs are not peers. `explore` is a dome_nav
primitive, but `locate targets` (explore + semantic ingest, sequenced) and
`go to target` (label→pose from dome_semantic + drive-to-pose in dome_nav) are
cross-package orchestration that today is smeared into dome_nav
(`nav_manager` and `explorer_manager_node` both subscribe `/intent`). A mission
layer gives that logic one home and keeps every package below it a dumb,
reusable primitive.

## Settled decisions (2026-07-31, author)

- **Extract in Phase A**, not deferred to Phase C. The F33 typed-msg consumer
  (F33 T05, label→pose) is built in dome_mission from day one; dome_nav never
  consumes `SemanticTargetArray`. This reassigns F33 T05 — see F33 G9.
- **`dome_mission` owns `/intent`.** dome_control → `/intent` → dome_mission.
  dome_nav's `explorer_manager_node` and `nav_manager_node` **stop** subscribing
  `/intent` and instead expose primitives dome_mission calls. Exactly one
  `/intent` handler in the system (no two-handler race).
- **Neutral package.** dome_mission depends on dome_nav's primitive interface,
  `dome_semantic_msgs`, and TF. dome_nav depends on none of the above.

## Target layering

```
dome_control      dome_mission (NEW)        dome_nav        dome_semantic
  intents    →    behavior FSM        →    primitives   ←   object memory
 (voice/UI)       explore/locate/goto      explore/nav      label→pose source
```

## Scope

- New `dome_mission` package: mission FSM node subscribing `/intent`, driving
  the three behaviors. Pure/ROS split per dome_nav L0/L1 discipline — sequencing
  logic testable without a live graph.
- Three behaviors:
  - **explore** — start/stop dome_nav frontier exploration; done on no-frontiers.
  - **locate targets** — explore (or survey vantage points) while dome_semantic
    ingests; the stateful cross-package sequence (F33 Phase C's real home).
  - **go to target** — resolve label→pose from `SemanticTargetArray`
    (dome_semantic), send `NavigateToPose` to Nav2 via dome_nav.
- dome_nav primitive interface: how dome_mission commands explore start/stop and
  drive-to-pose (candidate: ROS actions `ExploreArea` + Nav2's existing
  `NavigateToPose`; decided in the task file). Removes `/intent` subscription
  and label-lookup from dome_nav.
- Move F33 T05 label→pose consumer here; `tools/nav_intent_check.py` retargeted
  to talk to dome_mission.

## Constraints

- Exactly one `/intent` handler (dome_mission). dome_nav nodes must not also
  subscribe `/intent` after this lands.
- dome_nav must not depend on `dome_semantic_msgs` or `dome_semantic`.
- No YAML patching; launch composition via `better_launch`.
- Resolves F33 open question G3 (where dwell/sequencing lives): in dome_mission,
  not the explorer node FSM, algorithm plugin, or Nav2 BT.

## Open questions

- dome_nav primitive interface shape: ROS actions vs services vs a thin
  command topic — settle in the F35 task file with the explore start/stop and
  drive-to-pose contract.
- Boundary vs the explorer node's existing watchdog/stuck FSM (analysis.md
  Part 1 node-watchdog tension): which stop/abort authority stays in dome_nav
  vs moves up to dome_mission.
- **Orchestrator: FSM vs behavior tree — Phase-B decision.** Phase A/C use a
  plain FSM (3 verbs, mostly sequential; BT is YAGNI). BT becomes a candidate at
  Phase B for reactive vision-aware behaviors (pause/dwell/confirm/resume,
  fallback/retry, viewpoint coverage). If adopted it is `py_trees_ros` (Python),
  a separate higher tree, **not** Nav2's internal C++ BT. The T01 ROS-action
  interface is chosen to keep this FSM→BT swap cheap (see TF35 T01 rationale).

## How to Demo

**Setup**: dome_mission + dome_nav primitives + dome_semantic running; a saved
or in-progress semantic map.

**Steps**:
1. `nav explore` → dome_mission drives dome_nav exploration, map grows.
2. `nav go can` → dome_mission resolves `can` → pose from the semantic map,
   commands dome_nav drive-to-pose; robot arrives.
3. Confirm dome_nav logs show no `/intent` subscription and no semantic-msg
   dependency.

**Expected output**: mission verbs orchestrated from one place; dome_nav a
dumb navigation primitive with no mission or semantic knowledge.
