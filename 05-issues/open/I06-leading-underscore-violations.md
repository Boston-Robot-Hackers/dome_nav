# I06 — Leading underscore prefix used throughout — violates MUST rule

* **Symptom**: codereview.md MUST: "No leading underscore prefix on methods, functions, instance variables, or other custom identifiers." Widespread violations across three files.
* **Locations**:
  - `nav_manager_node.py`: _manager, _goal_handle, _last_loc_status, _last_loc_score (instance vars); _on_goal_accepted, _on_goal_result, _publish_localization (methods)
  - `slam_manager_node.py`: _manager, _path (local var); _on_save_done (method)
  - `utils.py`: _deep_merge, _write_temp (module-level functions)
  - `test/test_map_validation.py:25`: MapListener._cb (instance method)
  - `test/test_nav_manager.py:153,166,173,194,207,209`: directly accesses node._goal_handle, calls node._on_goal_result, node._on_goal_accepted, node._on_save_done — must be updated when source is renamed
  - `test/test_slam_manager.py:88`: calls node._on_save_done directly
* **Tests done**: N/A — style violation
* **Fix**: Rename all to drop underscore prefix. Update all test files that access renamed members. Run full test suite after.
