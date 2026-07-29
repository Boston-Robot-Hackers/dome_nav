# TF05 Sensor-Only Rosbag Integration Test for Feature F05

Task file name matches F05. For each task step, add a test when feasible.

**Marker policy**: these are ROS2-runtime, launch-based integration tests — per the
style guide (ROS2-graph-dependent tests are separated from plain pytest). They run
under `launch_testing` via `colcon test`, marked so plain `pytest -m "not manual"`
skips them. "CI-runnable" (F05 constraint) means the colcon/launch_testing job, not
the pure-unit job. Record the mechanism in T02.

## T01 — Record and land the bag(s)
**Status**: not done
**Description**: Record a rosbag of `/scan` + `/odom` + `/tf` + `/tf_static` (2 min
drive) per the F05 recipe, from the real robot or the linorobot2 sim. Produce two
artifacts: (a) a mapping loop for Mode A / costmap build; optionally (b) a short
clip near a wall for the lethal-guard scenario (can reuse (a) if it passes a wall
close enough to raise a lethal band). Decide check-in vs document-only: if the bag
exceeds a sane git size (~tens of MB), keep it out of git (`test/bags/` gitignored)
and document exact record steps + where the file lives instead. Capture the saved
map (`~/.dome/slam_map.yaml`) that pairs with the bag for Modes B/nav.
**Test**: n/a (data artifact); verified usable by T02 (bag plays, topics appear).

## T02 — Bag-replay launch fixture + teardown
**Status**: not done
**Description**: A reusable `launch_testing` fixture that: starts the dome_nav nodes
under test (no dome_vision/dome_control), plays the bag with `ros2 bag play` (use
`--clock` + `use_sim_time:=true` so slam/AMCL/costmap consume bag time, not wall
time), waits for the graph to come up, and tears everything down deterministically
(kill bag play + nodes, no orphan `gz`/node processes — see current.md orphan
gotcha). Encapsulate the "wait for topic / TF within timeout" helper here so every
integration test reuses it.
**Test**: fixture smoke test — bag plays to end, `/scan` observed on the graph, all
processes reaped on teardown. This task's deliverable is the fixture + its smoke
test.

## T03 — Mode A map-build integration test
**Status**: not done
**Description**: `test/test_integration_map_build.py` — launch Mode A (slam) against
the bag via the T02 fixture. Assert `/map` (`nav_msgs/OccupancyGrid`) is published
within a timeout and `slam_status` reports "mapping". Confirms slam_toolbox digests
recorded sensors with no live robot.
**Test**: this task is the test.

## T04 — Mode B AMCL convergence test
**Status**: not done
**Description**: `test/test_integration_amcl.py` — launch Mode B against the bag with
the T01 saved map. Assert `map→odom` TF is available within a timeout and the AMCL
particle cloud converges (pose covariance drops below a threshold). Confirms
localization works on recorded sensors.
**Test**: this task is the test.

## T05 — Nav-readiness / NavigateToPose test
**Status**: not done
**Description**: `test/test_integration_nav.py` — with AMCL up on the bag, send a
`NavigateToPose` goal to a reachable pose and assert `nav_status` transitions
idle→navigating→done (or at minimum that Nav2 accepts the goal and produces a plan;
full drive-to-done may not be observable from a fixed bag pose — record which is
asserted and why). Confirms the Nav2 stack is wired and reachable.
**Test**: this task is the test.

## T06 — Lethal-guard + clearance firing against the real costmap
**Status**: not done
**Description**: `test/test_integration_lethal_guard.py` — the F27/F31 integration
gap. Replay the bag so the **real global costmap** builds, then drive the explorer
node against it. Two assertions: (1) feed a candidate goal on a known-lethal cell
near a mapped wall, assert `goal_is_lethal` returns True, the node logs the skip, and
a different candidate is dispatched (the real-costmap analog of the unit-level
`test_find_and_send_goal_skips_lethal_candidate`); (2) with F31 clearance on, assert
an off-wall candidate outranks a wall-hug candidate given the real costmap's
inflation. Pick the lethal cell from the built costmap at runtime (query
`/global_costmap/costmap`, find a cell `>= 99`) rather than hardcoding coordinates,
so the test survives map changes.
**Test**: this task is the test. Closes the "hard to really verify" caveat on TF27
T06/T07 and TF31 T07/T08 with a repeatable, robot-free check.

## T07 — CI wiring + docs + literate
**Status**: not done
**Description**: Register the integration tests in the colcon/launch_testing job,
keep them out of the plain-pytest path, and gitignore any test output dirs. Update
`02-doc/current.md` (F05 now has a task file + covers the heuristic-firing gap) and
`02-doc/notes.md` if the bag-replay recipe belongs in semi-permanent notes. No
Python module logic changes expected, so literate regen only if a source file is
touched.
**Test**: `colcon test --packages-select dome_nav` runs the integration suite;
`pytest -m "not manual"` still passes unchanged (integration tests excluded).
