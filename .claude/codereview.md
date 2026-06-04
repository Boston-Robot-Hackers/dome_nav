# Code Review Checklist

Version: 3.0

Use this for Python source reviews. `MUST` items are blocking unless explicitly
waived in the task or PR notes. `SHOULD` items are expected defaults. `CONSIDER`
items are review prompts, not mechanical rules.

## Workflow
1. Confirm the work has a corresponding feature file and task file.
2. Review the target file against the `MUST` sections first.
3. Record material violations as tasks in the feature's task file.
4. Keep process changes and source changes reviewable; commit only when requested.
5. Run relevant tests and update task status.
6. Before opening a PR: regenerate the literate version of each changed Python
   source file by applying `.claude/literate.md` and saving the result to
   `01-literate/<module>.md`.

## Must Pass
- [ ] MUST: No code changes without a corresponding feature and task file
- [ ] MUST: No task file without a corresponding feature file
- [ ] MUST: Feature/task status is updated when work is completed or deferred
- [ ] MUST: When `02-doc/current.md` and `03-features/`, `04-tasks/`, `05-issues/` disagree, resolve or document the mismatch before relying on either
- [ ] MUST: No ROS2 imports (`rclpy`, `sensor_msgs`, etc.) in `oak_roboflow/`
- [ ] MUST: ROS2 code lives under `oak_roboflow_ros/oak_roboflow_ros/`
- [ ] MUST: No secrets, API keys, passwords, or tokens committed in code or config examples
- [ ] MUST: Sensitive values come from environment variables or local untracked config
- [ ] MUST: Logs and exceptions do not expose passwords, tokens, PII, or API-key-bearing URLs
- [ ] MUST: No side effects at module import time
- [ ] MUST: No mutable default arguments
- [ ] MUST: No bare `except Exception:` or silent `except X: pass`
- [ ] MUST: Validation happens at system boundaries: user input, config, hardware, ROS, or external APIs
- [ ] MUST: Bug fixes include regression tests unless the case is hardware-only or otherwise documented
- [ ] MUST: JSON map/tracker format changes preserve old saved files or include migration/default handling
- [ ] MUST: Lifecycle nodes stop worker threads, timers, publishers/subscribers, and OAK resources cleanly on deactivate/cleanup
- [ ] MUST: OAK-D, ROS2 runtime, robot firmware, and physical-motion tests are marked manual and separated from plain pytest tests

## Repo-Specific Checks
- [ ] SHOULD: New config fields are added to dataclasses, YAML parsing, examples where relevant, and tests
- [ ] SHOULD: New ROS parameters are declared, read, launch-overridable where useful, and covered by tests
- [ ] SHOULD: Config defaults are defined in one place whenever practical
- [ ] SHOULD: Dataclass fields are not hand-transcribed into YAML parsing or ROS parameter declaration when `dataclasses.fields()` can reasonably be used
- [ ] SHOULD: Launch files use `better_launch` (`@launch_this`, `bl.node`, `bl.group`, `bl.include`)
- [ ] SHOULD: ROS2 runtime deps are declared as `exec_depend` in `package.xml`, not in `pyproject.toml`
- [ ] SHOULD: Tests in `tests/` run with plain `pytest`; no `colcon test` dependency
- [ ] SHOULD: Optional topics and missing ROS graph dependencies do not crash the node
- [ ] SHOULD: Standalone library modules have no `main()` or `argparse`; config comes from YAML dataclasses
- [ ] SHOULD: Current package layout includes expected build descriptors and `resource/<package_name>` marker for active packages
- [ ] SHOULD: If F37 repo restructure is complete, standalone install path is `pip3 install -e standalone/ --break-system-packages`

## Geometry And Persistence
- [ ] MUST: Changes to depth, intrinsics, projection, TF, frame IDs, or xyz math include frame-convention tests
- [ ] SHOULD: Camera-frame and world-frame values are named distinctly enough to prevent frame confusion
- [ ] SHOULD: Units are explicit at boundaries: `m` vs `mm`, pixels vs normalized coordinates
- [ ] SHOULD: Serialization tests cover missing optional fields from older saved files
- [ ] SHOULD: Persisted IDs and labels keep stable meanings across versions

## Tests
- [ ] SHOULD: Every public method has at least one test when practical
- [ ] SHOULD: Edge cases are covered: empty inputs, `None` values, and boundary conditions
- [ ] SHOULD: Non-obvious behavior explained by a comment has a test encoding that expectation
- [ ] SHOULD: External APIs, hardware, filesystems, time, and ROS2 graph dependencies are mocked or isolated where practical
- [ ] SHOULD: Test output directories are ignored by git
- [ ] SHOULD: No commented-out tests
- [ ] SHOULD: Manual test notes include command, setup, expected observation, and actual result

## Imports And Packaging
- [ ] SHOULD: All imports are at the top of the file
- [ ] MUST: Project modules use absolute imports; no relative imports
- [ ] MUST: No unused imports or wildcard imports (`from module import *`)
- [ ] SHOULD: Imports are sorted consistently
- [ ] SHOULD: Dependencies are declared in the correct package metadata
- [ ] MUST: All .py files have shebang line `#!/usr/bin/env python3`

## Style Preferences
- [ ] MUST: File header includes module name, one-line description, `Author: Pito Salas and Claude Code`, and `Open Source Under MIT license`
- [ ] SHOULD: Double quotes throughout; single quotes only when required
- [ ] MUST: No `from __future__ import annotations`
- [ ] SHOULD: No `Optional[X]`; use `X | None`
- [ ] MUST: No leading underscore prefix on methods, functions, instance variables, or other custom identifiers
- [ ] MUST: Line length <= 88 chars unless a longer line is materially clearer
- [ ] SHOULD: Boolean variables/params are named `is_X`, `has_X`, or `can_X`
- [ ] SHOULD: No single-letter variables except loop counters `i`, `j`
- [ ] SHOULD: Use `is` / `is not` for comparisons with `None`, `True`, and `False`
- [ ] MUST: Use f-strings for string formatting
- [ ] SHOULD: Use `enumerate()` instead of manual counter variables when the index is needed

## Design Prompts
- [ ] MUST: Functions and methods stay <= 50 lines where practical
- [ ] MUST: Files stay near 300 lines where practical
- [ ] SHOULD: One class per file; dataclasses co-located with their constructing class are allowed
- [ ] MUST: Identifiers are short enough to read and intention-revealing
- [ ] MUST: Avoid if/else nesting more than 1 deep; extract helpers or return early when clearer
- [ ] MUST: Avoid if statements with more than 3 branches; use lookup tables or helpers when clearer
- [ ] MUST: Avoid 1-line or 2-line methods unless they are properties, protocol adapters, or improve naming
- [ ] MUST: Avoid simple wrappers unless they isolate a boundary, adapt a framework API, or preserve a public interface
- [ ] CONSIDER: Prefer <=3 behavioral arguments; use dataclasses/config objects when parameter groups travel together
- [ ] CONSIDER: Defaults are acceptable in dataclasses, config, CLI, launch args, and stable public APIs; avoid hidden behavioral defaults in internal logic
- [ ] MUST: Avoid throwaway temporaries unless they clarify meaning, avoid recomputation, or aid debugging
- [ ] CONSIDER: No god methods, feature envy, data clumps, or unrelated responsibilities in one class
- [ ] CONSIDER: Mutable state is necessary and class invariants are enforced
- [ ] CONSIDER: Public API is minimal; internal helpers are clearly separated from public interface
- [ ] CONSIDER: Repeated 3+ line blocks or repeated object construction patterns may need a helper

## Comments And Types
- [ ] SHOULD: Simple methods with obvious bodies have no docstring
- [ ] SHOULD: Complex methods may use multi-line docstrings explaining why the mechanism exists and any non-obvious preconditions or postconditions
- [ ] SHOULD: Comments explain why, not what
- [ ] SHOULD: No task/fix/caller references in comments
- [ ] SHOULD: Public method parameters have type annotations where the type is non-obvious
- [ ] SHOULD: Return type is annotated when callers would otherwise have to guess
- [ ] MUST: Prefer simple annotations where possible; use precise collection types when they prevent caller ambiguity

## Runtime Quality
- [ ] MUST: No unreachable code, commented-out code blocks, debug print statements, or breakpoints
- [ ] SHOULD: Use context managers (`with`) for file handles and resources that require cleanup
- [ ] SHOULD: Use logging for runtime errors; do not leave debug `print()` calls in library or ROS2 modules
- [ ] SHOULD: No blocking network, file I/O, model loading, or expensive setup inside per-frame inference loops
- [ ] SHOULD: Avoid unnecessary image copies, large allocations, and repeated object construction in hot paths
- [ ] SHOULD: Optional per-frame outputs skip work when there are no subscribers or consumers
- [ ] SHOULD: Instrumentation does not materially change the timing of the path being measured
- [ ] SHOULD: Use the concurrency model expected by the framework; isolate threads and make shutdown deterministic
- [ ] CONSIDER: Avoid nested comprehensions unless simple enough to read at a glance or there is a clear performance reason
- [ ] CONSIDER: No features not required by current spec
