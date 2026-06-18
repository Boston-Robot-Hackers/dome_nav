# I02 — Non-list JSON on /targets/confirmed crashes navigation

* **Symptom**: on_targets() assigns json.loads() result directly to confirmed_targets with no isinstance check. If publisher sends a dict (e.g. {"targets": [...]}), confirmed_targets becomes a dict. Next find_nearest_confirmed iterates the dict, yielding string keys, then calls str.get("label") → AttributeError, crashing the ROS subscription callback.
* **Location**: `nav_manager.py:19` on_targets(), `nav_manager.py:35` find_nearest_confirmed()
* **Tests done**: None for non-list input
* **Fix**: After json.loads(), check `isinstance(result, list)`. If not, log warning and return False. Add regression test to test_nav_manager_pure.py.
