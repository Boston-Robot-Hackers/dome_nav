# I05 — Malformed intent silently dropped with no diagnostic

* **Symptom**: When parse_intent() returns None (bad JSON or unknown action), on_intent does a bare return with no log. on_targets() logs a warning on its equivalent failure — this gap is inconsistent. Violates codereview.md MUST: "On a problem, raise with context; never swallow."
* **Location**: `nav_manager_node.py:60` on_intent(), `nav_manager.py:28` parse_intent()
* **Tests done**: None
* **Fix**: Add `self.get_logger().warning(f"Ignoring unrecognized or malformed intent: {msg.data!r}")` in on_intent() before the return.
