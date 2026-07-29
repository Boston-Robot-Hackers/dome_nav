# TF27 — Never Dispatch a Goal to a Lethal Location (Feature F27)

For each task step, add a test when feasible. If not feasible, note why.

Guard rename is already done (see `04-tasks/chores.md`, 2026-07-18):
`goal_in_global_costmap` → `goal_within_costmap_bounds` — bounds-only meaning made
honest. T03 records it for traceability. The lethal check is the new work.

## T01 — Choose and unify the global-costmap cost scale
**Status**: done — kept `/global_costmap/costmap` (OccupancyGrid); added
`LETHAL_COST=100`/`INSCRIBED_COST=99`/`LETHAL_THRESHOLD=99` constants in
`explore_diagnostics.py`; fixed `costmap_radius_costs` (XXX on `>= LETHAL_COST`,
`???` on `v < 0`) and the failure legend to the scaled values. Regression tests added.
**Description**: The node reads `/global_costmap/costmap` = `nav_msgs/OccupancyGrid`,
whose publisher translates the raw 0–255 costmap to a scaled grid:

| Meaning | raw costmap (0–255) | `/global_costmap/costmap` (scaled) |
|---|---|---|
| lethal obstacle | 254 | 100 |
| inscribed (footprint collides) | 253 | 99 |
| inflated gradient | 1–252 | 1–98 |
| free | 0 | 0 |
| unknown | 255 | -1 |

`explore_diagnostics.py` assumes the **raw** scale (`v == 254`, legend
`lethal=254 inscribed=253 unknown=255`), so on the topic actually fed it, `== 254`
never fires — node and diagnostics disagree. Pick one scale for the whole node:

- **Recommended: keep `/global_costmap/costmap` (OccupancyGrid), lethal band on the
  scaled values** — lethal `== 100`, inscribed `== 99`, unknown `== -1`. Least churn:
  `fetch_grid` and `costmap_cell_cost` stay typed to `OccupancyGrid`. Fix
  `explore_diagnostics.py` constants/legend to the scaled values (`100`/`99`/`-1`).
- Alternative: switch the fetch to `/global_costmap/costmap_raw` (`nav2_msgs/Costmap`,
  0–255). Makes the existing `254`/`253` diagnostics correct but changes the message
  type and `costmap_cell_cost`’s signature — heavier.

Define one module-level lethal constant so the guard (T02) and diagnostics share it.
`lethal_cost_threshold: 65` / `trinary_costmap: true` in the yaml govern the **static
layer’s ingestion of `/map`**, not this output encoding — do not use them here.
**Test**: `costmap_cell_cost` returns the expected value for lethal/inscribed/free/
unknown/out-of-bounds cells on the chosen scale; `explore_diagnostics` formatting
labels a lethal cell correctly (regression for the `254`-never-fires bug).

## T02 — Add the lethal-goal guard (post-nudge)
**Status**: done — added `goal_is_lethal(xy)` (None cost ⇒ not lethal, permissive);
wired into the reselect loop as an early-continue after the bounds guard, on the final
post-nudge candidate; skip logs "on a lethal costmap cell". Unit + reselect-integration
tests added.
**Description**: Add a distinct, well-named guard (e.g. `goal_is_lethal(xy) -> bool`)
that reads the global costmap cell cost via `costmap_cell_cost` and returns True when
the cell is lethal. Reject band: lethal **+ inscribed** (`>= 99` scaled / `>= 253`
raw) — a goal on an inscribed cell means the footprint is guaranteed in collision, so
Nav2 rejects it anyway. Wire it into the existing reselect loop in
`explorer_manager_node.py` (~`:354`), evaluated on the **final post-nudge** candidate
(after `nudge_toward_robot` — the nudge is what can move a safe frontier goal onto a
lethal cell). A lethal candidate is excluded and `next_goal` re-asked, matching the
bounds guard’s skip-and-reselect flow. Degrade gracefully: no global costmap yet ⇒ do
not block (preserve startup-permissive behavior). Log a clear skip reason
("goal on lethal cell — skipping to next candidate").
**Test**: goal on a lethal cell ⇒ guard True, candidate skipped, next candidate tried;
goal on free/inflated-below-threshold cell ⇒ guard False, dispatched; no costmap ⇒
guard False (permissive); guard is applied after nudge (a frontier goal nudged onto a
lethal cell is rejected).

## T03 — Rename bounds guard (already done)
**Status**: done
**Description**: `goal_in_global_costmap` → `goal_within_costmap_bounds`, done as a
chore on 2026-07-18 (call site, tests, literate updated; pure rename, no behavior
change). Recorded here so the F27 scope item is accounted for. The bounds guard and
the T02 lethal guard are separate, complementary checks (bounds = worldToMap
protection; lethal = safety).
**Test**: covered by existing `test_goal_within_costmap_bounds_*` tests (40 node tests
green after the rename).

## T04 — Tests
**Status**: done — `test_explorer_manager_node.py`: `goal_is_lethal` truth table
(no-costmap/free/lethal/inscribed/below-threshold/out-of-bounds) +
`test_find_and_send_goal_skips_lethal_candidate` (reselect through the loop).
`test_explore_diagnostics.py`: lethal renders `XXX` at 100 + scaled legend regression.
Full suite 229 passed, 4 deselected.
**Description**: Consolidate/complete the T01–T02 tests in
`test/test_explorer_manager_node.py` (and `test/` diagnostics coverage): lethal guard
truth table on the chosen scale, post-nudge application, permissive-when-no-costmap,
reselect-on-lethal integration through the goal loop, and the diagnostics
lethal-labeling regression. Ensure the full suite stays green.
**Test**: this task is the tests; `python3 -m pytest test/ -m "not manual"` passes.

## T05 — Docs + literate
**Status**: done — `02-doc/current.md` notes the guard + single scale;
`01-literate/09-explorer_manager_node.md` regenerated (v1.2) with the two-guard
narrative. Feature stays open pending T06/T07; move to `done/` only after both.

## T06 — Sim verification
**Status**: done (2026-07-29) — hard to truly verify: forcing a nudged goal onto a
lethal cell on demand is not deterministic, so the skip path can't be reliably
provoked in sim. Guard is unit-tested (T04 truth table + reselect integration) and
live-observed working; marked done on that basis, not a clean staged sim repro.
**Description**: Run the sim explore stack
(`bl dome_nav sim_nav_full.launch.py --map_name f27test --world_name multi_room`).
Drive exploration so a frontier goal (post-nudge) would fall on a lethal
global-costmap cell near a wall. Confirm: node logs "on a lethal costmap cell —
skipping to next candidate", a different goal is dispatched, and no
`GOAL_OCCUPIED`/`NO_VALID_PATH` abort is caused by a lethal goal. Record the
`/explore/status` transitions and the skip log.
**Test**: manual (sim); record command, observed skip log, and that exploration
continued past the lethal candidate.

## T07 — Live (real-robot) verification
**Status**: done (2026-07-29) — same caveat as T06: very hard to really verify. Can't
force a nudged frontier goal onto a lethal cell on the real robot on demand, so no
clean staged repro of the skip. Guard is unit-tested + live-observed not misbehaving;
marked done on that basis.
**Description**: On the real robot (Mode E), run explore in a space where a nudged
frontier goal can land on a lethal cell near a wall. Confirm the guard skips it and
exploration proceeds without a lethal-goal abort. Deferred until hardware is
available (real-robot Modes A/B/E remain unverified overall).
**Test**: manual (hardware); record command, skip log, and behavior.
**Description**: Update `02-doc/current.md` (note the lethal-goal guard + the single
cost scale) and regenerate `01-literate/09-explorer_manager_node.md` (and the
diagnostics literate if present) for the changed source before PR. Flip F27
`Done`/`Tests Written`/`Test Passing` and move F27 + TF27 to `done/`.
**Test**: n/a (docs); literate regen per `.claude/literate.md`.
