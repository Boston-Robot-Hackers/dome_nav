# F26 — Academic Paper on Indoor-Survey Algorithms

**Priority**: Medium
**Done:** no
**Tasks File Created:** yes (TF26)
**Tests Written:** n/a (document deliverable)
**Test Passing:** n/a
**Description**: Write an academic paper on algorithms for comprehensively
surveying a complex indoor space. The paper is **algorithm-centric, not
ROS2-centric**: it surveys the literature on indoor exploration/survey algorithms,
then introduces, analyzes, and quantitatively evaluates two new algorithms
developed in this project. It is grounded in our experience and experiments with
the DOME robot, but ROS2 appears only in one small section, alongside a second
small section on our pluggable-algorithm architecture.

## Paper scope (per author, 2026-07-17)

- **Literature survey** — existing algorithms for comprehensive indoor surveying
  (frontier-based exploration, coverage/boustrophedon sweeps, next-best-view,
  learned exploration, etc.). Breadth over depth; positions our two contributions.
- **Two new algorithms** — introduced, analyzed, and evaluated with quantitative
  data. Confirmed by the author (2026-07-17):
  - F14 `preferred_goal_distance` — frontier selection by `min |d − preferred|`
    instead of nearest/farthest
  - F15 path novelty scoring — Bresenham unknown-cell count along the straight
    line to each candidate; re-ranks a top-N short-list
- **Quantitative data** — pulled from existing experiments/telemetry where it
  exists. Where it does not, the paper plan must include a **runbook**: exact,
  step-by-step instructions (commands, configs, metrics to record) for the author
  to generate the missing data.
- **Small ROS2 section** — how the algorithms are realized on ROS2 (Nav2,
  slam_toolbox, action servers); explicitly framed as one possible realization.
- **Small pluggable-architecture section** — the F12 algorithm registry + F23
  decoupling (intent-carrying `GoalDecision`, opaque hooks) that let us swap
  survey algorithms without touching the manager node.
- **Venue: arXiv / tech report** (author decision 2026-07-17) — no page-limit
  constraint; the plan's publishability assessment becomes an internal quality
  gate, not a venue fit analysis.

## Deliverables (in order)

1. **Paper plan** — working title, claim scope, section outline, evidence
   inventory mapping each claim to code/tests/data, experimental plan,
   publishability assessment. Same style as the oak-roboflow plan.
2. **Runbook** — the missing-data experiments, ready to execute.
3. **Draft** — sections written per the plan's writing phases.

## Background research (collected)

- `dome_vision/02-doc/paper_plan.md` — oak-roboflow/PASA paper plan; structural
  model and claim-hygiene rules.
- `dome_vision/02-doc/survey-exploration.md` — room-level survey spec (spiral /
  perimeter / boustrophedon, landmark estimator); related-design material.
- dome_nav: `02-doc/spec.md`, `02-doc/current.md` (sim exploration verified;
  real-robot modes unverified), F12/F23 (pluggable architecture), F14/F15 (the
  two candidate algorithms), `01-literate/` for algorithm detail.

## Constraints

- Claim hygiene per the oak-roboflow plan: every claim traceable to code, tests,
  or a measurement; quantified language only; hardware context on all numbers.
- State honestly what is sim-only vs. hardware-verified (real-robot Modes A/B/E
  have never been live-run as of this writing).
- Any quantitative claim without data today must be marked runbook-pending, not
  asserted.
- Paper artefacts live in dome_nav (this repo); exact location decided when the
  plan is written (dome_vision precedent: plan in `02-doc/`).

## How to Demo

**Setup**: Background docs above read; dome_nav sim exploration stack works
(`bl dome_nav sim_nav_full.launch.py --map_name <name> --world_name multi_room`).

**Steps**:
1. Paper plan reviewed with the author; two algorithms confirmed.
2. Runbook executed by the author; data collected.
3. Draft written; claims checked against the evidence inventory.

**Expected output**: paper plan + runbook + a draft that passes the plan's own
claim-hygiene check, ready for arXiv posting.
