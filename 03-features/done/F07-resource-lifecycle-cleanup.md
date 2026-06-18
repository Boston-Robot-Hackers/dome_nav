# F07 — Resource Lifecycle and Cleanup

**Priority**: High
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes (unit; manual shutdown verify pending live stack)
**Test Passing:** yes (unit)
**Description**: Two resource-correctness fixes found in code review.

1. **Temp-file leak (I11)**: `utils.py` `yaml_override`/`yaml_patch_dict` write
   `NamedTemporaryFile(delete=False)` on every launch and never clean up. `/tmp`
   accumulates `.yaml` files indefinitely. Replace with content-hashed writes into a
   bounded cache dir under `dome_home()` — identical configs reuse one file, distinct
   configs get distinct files, total bounded by the number of distinct configs.

2. **slam_manager_node lifecycle (I01)**: the node is a plain `Node`; the shutdown
   save in `main()` fires an async service call after `rclpy.spin()` exits, so the done
   callback never runs and the map is silently not written. Convert `SlamManagerNode`
   to `LifecycleNode`: resources created in `on_configure`, save timer in `on_activate`,
   and a **synchronous** final save in `on_shutdown` (driven by `spin_until_future_complete`
   before `destroy_node`). main() drives configure → activate → spin → shutdown.

## How to Demo

**Setup**: built workspace, sourced.

**Steps**:
1. `colcon build --packages-select dome_nav && source install/setup.bash`
2. `python3 -m pytest src/dome_nav/test/ -m "not manual" -v` — all pass
3. Launch Mode A, build a partial map, Ctrl-C
4. `ls -la ~/.dome/slam_maps/` — basement1.posegraph/.data updated mtime
5. `ls ~/.dome/launch_cache/` — bounded set of yaml files, not growing per launch

**Expected output**: tests pass; map written on shutdown; no /tmp accumulation.

## Process Gate

Scope approved by user (I11 + lifecycle nodes). nav_manager lifecycle deferred — see TF07 T03.
