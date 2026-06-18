# I08 — Test files missing required file header

* **Symptom**: codereview.md MUST: "File header includes module name, one-line description, Author: Pito Salas and Claude Code, and Open Source Under MIT license." Both test files have only a shebang line.
* **Locations**: `test/test_nav_manager_pure.py:1`, `test/test_slam_manager_pure.py:1`
* **Tests done**: N/A — missing metadata
* **Fix**: Add standard header block to both files.
