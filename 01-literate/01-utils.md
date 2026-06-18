---
version: "2.0"
generated: "2026-06-17"
---

# utils.py — Shared Utilities for dome_nav

## Purpose

`utils.py` provides two services: locating the robot's home directory on disk (`dome_home`) and merging YAML configuration files at launch time (`yaml_override`, `yaml_patch_dict`). These are foundational; every launch file and node that needs config depends on them.

## The DOME_HOME Convention

Rather than hard-coding paths, all persistent state (maps, logs, calibration) lives under a single directory resolved at runtime:

```python
def dome_home() -> str:
    """Return DOME_HOME path, expanding ~ if needed."""
    return os.path.expanduser(os.environ.get("DOME_HOME", "~/.dome"))
```

The `DOME_HOME` environment variable lets CI and multi-robot deployments redirect state without touching code. The `~/.dome` default keeps a developer's workstation tidy.

## YAML Merging

ROS2 launch files often need to layer a robot-specific override on top of a shared base config. Two functions handle this:

**File-to-file merge** — reads both YAML files and merges them:

```python
def yaml_override(base_file: str, override_file: str) -> str:
    with open(base_file) as f:
        base_params = yaml.safe_load(f) or {}
    with open(override_file) as f:
        override_params = yaml.safe_load(f) or {}
    merged = _deep_merge(base_params, override_params)
    return write_config(merged)
```

**Dict-to-file merge** — useful when overrides are computed at launch time:

```python
def yaml_patch_dict(base_file: str, overrides: dict) -> str:
    with open(base_file) as f:
        base_params = yaml.safe_load(f) or {}
    merged = _deep_merge(base_params, overrides)
    return write_config(merged)
```

Both return a path to a config file on disk. The caller passes that path to a ROS2 node as a params file argument.

## Deep Merge Algorithm

Shallow merge (`dict.update`) would clobber entire nested namespaces. `_deep_merge` recurses into matching dict keys so that overriding a single deeply-nested parameter doesn't erase its siblings:

```python
def _deep_merge(base: dict, override: dict) -> dict:
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result
```

```mermaid
flowchart TD
    A[base dict] --> C[copy base]
    B[override dict] --> D{for each key}
    C --> D
    D -->|both are dicts| E[recurse _deep_merge]
    D -->|otherwise| F[override wins]
    E --> G[merged result]
    F --> G
```

## Content-Addressed Config Sink

The merged config must persist after the function returns — ROS2 nodes read it after the launch process writes it. An earlier version used `tempfile.NamedTemporaryFile(delete=False)`, which leaked one `/tmp` file on *every* launch since nothing ever deleted them (issue I11). The fix keys the file by a hash of its own content:

```python
def write_config(data: dict) -> str:
    cache_dir = os.path.join(dome_home(), "launch_cache")
    os.makedirs(cache_dir, exist_ok=True)
    blob = yaml.dump(data, default_flow_style=False, sort_keys=False)
    digest = hashlib.sha1(blob.encode()).hexdigest()[:16]
    path = os.path.join(cache_dir, f"{digest}.yaml")
    with open(path, "w") as f:
        f.write(blob)
    return path
```

Because the filename is derived from the rendered YAML, identical configs map to the same file and repeated launches overwrite rather than accumulate. The on-disk set is bounded by the number of *distinct* configs — a handful — instead of growing without limit. The cache lives under `DOME_HOME` so it travels with the rest of the robot's state and is easy to inspect or wipe.

## Potential Improvements

- **Cache eviction**: the launch cache is bounded by distinct configs but never pruned. A startup sweep of files older than N days would keep it tidy across many map/param variations.
- **Missing-file error messages**: `open(base_file)` raises a generic `FileNotFoundError`. A wrapper with a descriptive message pointing to the offending config path would help operators debug misconfigured launches.
- **`sort_keys=False`** preserves author intent in YAML output and, conveniently, keeps the content hash stable across runs — but it does make deterministic diffs against alphabetised configs harder.
