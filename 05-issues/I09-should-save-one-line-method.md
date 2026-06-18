# I09 — should_save() is a 1-line non-property method — violates MUST rule

* **Symptom**: style_guide.md MUST: "Avoid 1-line or 2-line methods unless they are properties, protocol adapters, or improve naming." `should_save()` returns `self.map_ready` unchanged with no transformation, gating logic, or semantic improvement over direct access.
* **Location**: `slam_manager.py:19-20`
* **Tests done**: test_should_save_false_before_map, test_should_save_true_after_map cover it
* **Fix**: Remove method. Replace call site in `slam_manager_node.py:50` with `self.manager.map_ready` directly. Update tests that call should_save() to access map_ready instead.
