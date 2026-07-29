# F05 — Sensor-Only Integration Test with Rosbag

**Priority**: Medium
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: Record a rosbag of `/scan` + `/odom` + `/tf` from the real robot
(or linorobot2 sim). Use it to verify dome_nav Mode A and Mode B work correctly
without dome_vision, dome_control, or a live robot. AMCL convergence and Nav2
readiness verified against recorded sensor data.

Second motivation (2026-07-29): the goal-scoring heuristics (F27 lethal-goal
guard, F31 clearance/novelty) are unit-tested for *correctness* over synthetic
grids, but nothing checks they *fire against a real Nav2 costmap*. A bag replay
builds the actual global costmap offline, so we can drive the explorer node against
it and assert the guard skips a lethal cell — the integration gap that made TF27
T06/T07 "hard to really verify." That scenario is added to scope below.

## Scope

- `test/bags/` — rosbag recorded from real robot or sim (checked in or documented
  how to record)
- `test/test_integration_map_build.py` — launch Mode A against bag, verify map
  appears on `/map` within timeout, verify slam_status = "mapping"
- `test/test_integration_amcl.py` — launch Mode B against bag with saved map,
  verify `map→odom` TF available within timeout, verify AMCL particle cloud
  converges (covariance drops below threshold)
- `test/test_integration_nav.py` — with AMCL running on bag, send `NavigateToPose`
  goal to a reachable pose, verify nav_status transitions idle→navigating→done
- `test/test_integration_lethal_guard.py` — replay bag so the real global costmap
  builds, feed the explorer node a candidate goal on a known-lethal cell (near a
  mapped wall), assert `goal_is_lethal` fires and the node skips + reselects. Same
  harness can assert F31 clearance ranks off-wall goals above wall-hug goals on the
  real costmap.

## Constraints

- No dome_vision node running
- No dome_control node running
- No live robot required — rosbag provides sensors
- Tests runnable in CI with `colcon test`

## How to Demo

**Setup**: saved map at `~/.dome/slam_map.yaml`, rosbag at `test/bags/house_loop.bag`.

**Steps**:
1. `colcon test --packages-select dome_nav`
2. Watch integration tests run bag playback + launch checks automatically

**Expected output**: all integration tests pass. dome_nav verified working on
sensor data alone with no other dome packages.

## How to record the bag

```bash
ros2 bag record /scan /odom /tf /tf_static -o test/bags/house_loop
```
Drive robot around room for ~2 minutes. Stop. Bag is ready.
