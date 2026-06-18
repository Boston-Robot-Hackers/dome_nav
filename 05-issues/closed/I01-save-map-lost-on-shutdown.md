# I01 — Map lost on every clean shutdown

* **Symptom**: Ctrl-C exits rclpy.spin(). save_map() fires an async service call but no executor is spinning, so _on_save_done never runs. destroy_node() is called immediately after. Map is silently not written.
* **Location**: `slam_manager_node.py:88` in main() finally block
* **Tests done**: None — only observable by checking whether .posegraph/.data files update on shutdown
* **Fix**: Replace async call with `rclpy.spin_until_future_complete(node, future)` after calling save_map(), or make shutdown save synchronous via a blocking service call.
* **RESOLVED (2026-06-17, F07 T02)**: SlamManagerNode converted to LifecycleNode; on_shutdown does a synchronous save via spin_until_future_complete before destroy. Regression tests: test_shutdown_saves_when_map_ready, test_shutdown_skips_save_when_no_map. Live shutdown verify still pending (TF07 T04).
