# I07 — Localization score not clamped to 1.0 (plausible, not confirmed in practice)

* **Symptom**: Formula `max(0.0, 1.0 - worst / MAX_COV)` only clamps the low end. If a covariance diagonal value is negative, score exceeds 1.0 and gets published on /dome_nav/localization_score. AMCL should never produce negative variances, but formula is unguarded.
* **Location**: `nav_manager.py:49` check_localization()
* **Tests done**: No test covers negative covariance input
* **Fix**: Change to `min(1.0, max(0.0, 1.0 - worst / MAX_COV))`. Add test case for negative covariance input.
