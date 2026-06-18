# I11 — Launch temp-file leak in utils.py

* **Symptom**: yaml_override/yaml_patch_dict wrote NamedTemporaryFile(delete=False) on every launch and never cleaned up. /tmp accumulated 2–3 .yaml files per launch indefinitely.
* **Location**: `utils.py` (former `_write_temp`)
* **RESOLVED (2026-06-17, F07 T01)**: replaced with `write_config(data)` — content-addressed write into `~/.dome/launch_cache/<sha1>.yaml`. Identical configs reuse one file; bounded by number of distinct configs. Tests: test/test_utils_pure.py (5 tests incl. test_repeated_identical_writes_do_not_accumulate).
