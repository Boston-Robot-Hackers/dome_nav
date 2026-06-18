# TF01b — Fix Map Lost on Shutdown (I01)

Companion to TF01 (done). Fixes async save_map bug found in code review.

## T01 — make shutdown save synchronous (I01)
**Status**: done — superseded by F07 T02. Fixed at depth by converting SlamManagerNode to
a LifecycleNode with a synchronous on_shutdown save (spin_until_future_complete), rather
than the band-aid below. Regression tests: test_shutdown_saves_when_map_ready,
test_shutdown_skips_save_when_no_map.
**Description**: slam_manager_node.py main() calls save_map() after rclpy.spin() exits. The async done callback (_on_save_done) never fires because no executor is spinning; destroy_node() is called immediately after. Map is silently not written on every clean shutdown.
Fix: spin until the future completes before destroying the node.
```python
future = node.save_map_future()   # returns the Future, not bool
if future:
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
node.destroy_node()
```
Or make save_map use a synchronous service call at shutdown.
**Test**: manual — Ctrl-C the node, verify .posegraph/.data files have updated mtime.
