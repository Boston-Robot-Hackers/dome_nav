# I04 — Missing xyz_world key silently sends robot to map origin

* **Symptom**: navigate_to_object uses target.get("xyz_world", [0.0, 0.0, 0.0]) with no warning. If a confirmed target lacks xyz_world, robot navigates to (0,0) — potentially a wall or dock — with no log indicating the key was absent.
* **Location**: `nav_manager_node.py:75`
* **Tests done**: None for missing xyz_world
* **Fix**: Check "xyz_world" in target first. If absent, log warning and publish no_target:label instead of navigating. Add regression test.
