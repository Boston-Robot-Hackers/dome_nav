#!/usr/bin/env python3
# utils.py — shared launch utilities for dome_nav
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import hashlib
import os
import yaml


def dome_home() -> str:
    """Return DOME_HOME path, expanding ~ if needed."""
    return os.path.expanduser(os.environ.get("DOME_HOME", "~/.dome"))


def yaml_override(base_file: str, override_file: str) -> str:
    """Merge two YAML files, override_file taking precedence. Returns temp file path."""
    with open(base_file) as f:
        base_params = yaml.safe_load(f) or {}
    with open(override_file) as f:
        override_params = yaml.safe_load(f) or {}

    merged = deep_merge(base_params, override_params)
    return write_config(merged)


def yaml_patch_dict(base_file: str, overrides: dict) -> str:
    """Merge a dict of overrides into a YAML file. Returns temp file path."""
    with open(base_file) as f:
        base_params = yaml.safe_load(f) or {}

    merged = deep_merge(base_params, overrides)
    return write_config(merged)


def deep_merge(base: dict, override: dict) -> dict:
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def write_config(data: dict) -> str:
    """Write merged config to a content-addressed file under the DOME_HOME launch cache.

    Keyed by a hash of the rendered YAML so identical configs reuse one file and
    repeated launches do not accumulate temp files (the old NamedTemporaryFile
    approach leaked one file per launch into /tmp).
    """
    cache_dir = os.path.join(dome_home(), "launch_cache")
    os.makedirs(cache_dir, exist_ok=True)
    blob = yaml.dump(data, default_flow_style=False, sort_keys=False)
    digest = hashlib.sha1(blob.encode()).hexdigest()[:16]
    path = os.path.join(cache_dir, f"{digest}.yaml")
    with open(path, "w") as f:
        f.write(blob)
    return path
