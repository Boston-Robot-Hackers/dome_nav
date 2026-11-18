# I13 Real-robot Nav2 params not migrated to Kilted

Raised from `dome_semantic`'s TF36.7 robot runs, 2026-11-13.

## Symptom

`robot_map.launch.py` loads `config/nav2_params_real.yaml`, and two things
break on Kilted:

- **`bt_navigator` dies at startup** with exit code −11 (segfault), so Nav2
  can't run navigation goals.
- **`/cmd_vel` has two types.** `collision_monitor` publishes
  `TwistStamped`, while `gendrv` subscribes to `Twist`. Nav2 commands never
  reach the motors. `ros2 bag record` logs `Topic '/cmd_vel' has more than
  one type associated` every 100 ms.

Teleop is unaffected, since it publishes plain `Twist` straight to `gendrv`.

## What has been checked

- `nav2_params_real.yaml:85` still sets `error_code_names`. Kilted replaced
  it with `error_code_name_prefixes`.
- No `enable_stamped_cmd_vel: false` in `nav2_params_real.yaml`.
- **The explore files were already fixed** (`nav2_params_explore_real.yaml`
  and `nav2_params_explore_sim.yaml` both carry the `# KILTED:` change and
  `enable_stamped_cmd_vel: false`). The fix came in with I01's commit; see
  `02-doc/current.md`.
- The same two settings are missing from
  `nav2_params_localization_real.yaml` (`error_code_names` at line 82) and
  `nav2_params_explore_real_mini.yaml` (line 93).

## Theory

These three files were missed when the explore files were migrated. Apply
the same two changes to them.

Not yet confirmed that `error_code_names` is what causes the segfault, as
opposed to only a startup error. Check after the change that
`bt_navigator` stays up.

## The SLAM params copy is out of step too (2026-11-17)

Found during `dome2`'s F05 experiments (TF05.1, E3).
`config/mapper_params_online_async.yaml` was copied from the Jazzy upstream
file. The robot now runs `slam_toolbox` **2.9.0** (Kilted).

### Comparison with Kilted's upstream file

Compared with `/opt/ros/kilted/share/slam_toolbox/config/mapper_params_online_async.yaml`,
ignoring comments:

- **No upstream key is missing** from the copy.
- **Five values changed on purpose**, all marked `# CHANGED`:
  `transform_publish_period`, `map_update_interval`, `max_laser_range`,
  `transform_timeout`, `tf_buffer_duration`.
- **`map_start_pose` / `map_start_at_dock` were added**, with a comment
  block explaining why. Kilted uses `map_start_at_dock`.
- **Three keys are ignored by Kilted**:
  - `message_queue_size: 40`. Marked `# ADDED`, meant to reduce dropped
    scans under load. **It has no effect on Kilted.**
  - `restamp_tf: false`. Unmarked.
  - `check_min_dist_and_heading_precisely: false`. Unmarked.

  None of the three names appears in the `async_slam_toolbox_node` binary
  or in the `slam_toolbox` libraries. `ros2 param get` on the running node
  returns "Parameter not set" for each. The two unmarked keys were probably
  in the Jazzy upstream file and dropped in Kilted.

### `slam_toolbox` doesn't always activate

On Kilted, `slam_toolbox` is a lifecycle node. In one of two starts of
`online_async_launch.py`, the reply to its configure request timed out
(`failed to send response to /slam_toolbox/change_state (timeout)`). The
launch file never sent activate, so the node stayed `inactive` with no
`/map` until `ros2 lifecycle set /slam_toolbox activate` was run by hand.
`robot.launch.py` (formerly `robot_map.launch.py`) includes the same file,
so it can stall the same way.

**Evidence so far (2026-11-17).** `~/.ros/log` has 31 `slam_toolbox` starts,
and 4 of them failed this way.

- **Each time,** the warning comes from Fast DDS (`rmw_response.cpp:164`),
  and `Activating` never follows.
- **Two of the four** came during 12-process SLAM + Nav2 launches.
- **One (E3)** had `slam_toolbox` starting alone, 21 s after the drivers.
- **12-process launches also succeeded,** so startup load raises the odds
  but is not needed.

**After the Kilted upgrade (2026-11-18).** Fast DDS was upgraded to 3.2.5
on 2026-11-17. Four starts of `bl dome_nav robot.launch.py --mode slam`
followed, with the drivers up.

- **All four reached `Activating`.** None logged the `change_state` timeout.
- **One run looked like a failure, but wasn't.** The errors the user saw
  came at shutdown, from `slam_manager`, after "Shutdown nodes and quit?"
  was answered (07:38:09). The run had already logged `Finished serializing
  Dataset` at 07:37:49. The errors were:
  - `Unable to start transition 7 from current state (12, 'shuttingdown')`
  - `Failed to publish log message to rosout`

  `slam_manager` tries a lifecycle transition while the launch is shutting it
  down. That is a separate, harmless problem.
- **Four starts can't show a fix.** The failure rate before was about 1 in
  10.

**Likely cause, unverified:** a DDS discovery race at startup. The reply and
the `transition_event` are sent before the launch process's reader has
matched. If so, the params rework will not fix it. No workaround yet: a
check-and-activate helper was rejected as a kludge.

### Decision: start over from upstream

**Replace the copy with Kilted's current upstream file, then re-add only the
changes that are shown to be needed.** Each change that comes back needs a
measured reason on the Pi, not just a carried-over comment. For each one,
run with and without it and compare:

- load
- EKF `/odom` rate and its largest gap
- dropped scans (`Message Filter dropping message`)
- `map → odom` publish rate
- the map itself

Candidates to re-test:

- `transform_publish_period`
- `map_update_interval`
- `max_laser_range`
- `transform_timeout`
- `tf_buffer_duration`
- `map_start_at_dock`

Record which `slam_toolbox` version the file was checked against in its
header.

The same approach likely applies to the Nav2 params files above.
