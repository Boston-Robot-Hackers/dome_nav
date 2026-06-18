# I03 — Non-dict JSON on /intent raises AttributeError

* **Symptom**: parse_intent() calls intent.get("action") immediately after json.loads() with no isinstance(intent, dict) check. If publisher sends [1,2,3], json.loads succeeds but list.get() raises AttributeError which escapes the ROS subscription callback.
* **Location**: `nav_manager.py:26` parse_intent()
* **Tests done**: None for non-dict input
* **Fix**: Add `if not isinstance(intent, dict): return None` after json.loads(). Add regression test to test_nav_manager_pure.py.
