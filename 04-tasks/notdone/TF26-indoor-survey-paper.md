# TF26 — Indoor-Survey Algorithms Paper for F26

Paper scope per F26: algorithm-centric survey of indoor-space surveying
algorithms; two new algorithms (F14 preferred-goal-distance, F15 path-novelty
scoring) introduced + analyzed + quantitatively evaluated; small ROS2 section;
small pluggable-architecture section; venue = arXiv / tech report. Tests are n/a
for all writing tasks (document deliverables, no code) — claim hygiene via the
T03 evidence inventory is the quality gate instead.

## T01 — Background synthesis
**Status**: not done
**Description**: Digest the background sources into the paper's evidence base:
`dome_vision/02-doc/paper_plan.md` (structural model, claim-hygiene rules),
`dome_vision/02-doc/survey-exploration.md` (room-survey spec), dome_nav
`02-doc/spec.md` + `02-doc/current.md` (what is sim-verified vs. unverified), and
the F14/F15 implementation detail in `01-literate/06-frontier_explorer.md`,
`01-literate/08-frontier_algorithm.md` + source. Output: a notes section in the
paper-plan doc listing every candidate claim and its evidence source.
**Test**: n/a (document); output is reviewed with the author as part of T03.

## T02 — Literature survey
**Status**: not done
**Description**: Structured review of published algorithms for comprehensive
indoor surveying/exploration: frontier-based exploration (Yamauchi et al.),
coverage/boustrophedon sweeps, next-best-view planning, receding-horizon NBV,
learned/R exploration. Build a comparison table (algorithm, mechanism,
guarantees, assumptions, hardware demands) that positions F14/F15. Every cited
work verified to exist — no hallucinated references.
**Test**: n/a (document); each reference checked against a real source before
inclusion.

## T03 — Paper plan
**Status**: not done
**Description**: Write the paper plan (same style as the oak-roboflow plan):
working title, claim scope (what the paper does and does NOT claim about
F14/F15), section-by-section outline, evidence inventory mapping every claim to
code/tests/telemetry, experimental plan (what data exists vs. what the runbook
must generate), and an internal quality-gate assessment (replacing venue-fit
analysis; venue is arXiv/tech report). Review with the author before T04/T05.
**Test**: n/a (document); gate = author sign-off.

## T04 — Runbook for missing quantitative data
**Status**: not done
**Description**: From the T03 experimental plan, produce a step-by-step runbook
of experiments the author executes to generate missing data. Expected entries
(detail settled in T03): F15 sim live-verification (TF15 T05: novelty on vs. off,
`novelty_score` in telemetry); F14/F15 A/B coverage runs (time-to-N%-coverage,
goals-attempted vs. reached, path length over the multi_room world, repeated
seeds); real-robot Mode E runs if hardware claims are made. Each entry: exact
commands, configs/params, duration, metrics to record, where to save the data.
**Test**: n/a (document); each runbook step must be executable as written —
checked against current launch files and params.

## T05 — Draft sections
**Status**: not done
**Description**: Write the draft per the T03 outline: algorithms (F14/F15,
formal description + analysis), evaluation (data from T04), literature
positioning, the small ROS2 section, the small pluggable-architecture section
(F12 registry + F23 decoupling), intro/related/conclusion. Enforce claim
hygiene: every claim traceable to the evidence inventory; quantitative claims
without T04 data marked pending, never asserted.
**Test**: n/a (document); final pass = claim-hygiene check against the T03
evidence inventory.
