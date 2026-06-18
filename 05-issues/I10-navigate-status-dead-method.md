# I10 — navigate_status() defined but never called from production code

* **Symptom**: NavManager.navigate_status() (nav_manager.py:54) is tested but nav_manager_node.py builds the same status strings inline (f"no_target:{label}", f"navigating:{label}") without calling this method. Dead in production. Also a DRY violation — the format is defined in two places.
* **Location**: `nav_manager.py:54-57`, `nav_manager_node.py:72-73`
* **Tests done**: test_navigate_status_no_target, test_navigate_status_with_target cover the method but no test catches the node bypassing it
* **Fix**: Make nav_manager_node.py call self._manager.navigate_status(label, target) instead of formatting strings inline. Removes the duplication and activates the tested method.
* **Status**: fixed — nav_manager_node.py now calls self.manager.navigate_status() in all three publish_status call sites. 61 tests pass.
