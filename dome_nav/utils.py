#!/usr/bin/env python3
# utils.py — shared launch utilities for dome_nav
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
import tempfile
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

    merged = _deep_merge(base_params, override_params)
    return _write_temp(merged)


def yaml_patch_dict(base_file: str, overrides: dict) -> str:
    """Merge a dict of overrides into a YAML file. Returns temp file path."""
    with open(base_file) as f:
        base_params = yaml.safe_load(f) or {}

    merged = _deep_merge(base_params, overrides)
    return _write_temp(merged)


def _deep_merge(base: dict, override: dict) -> dict:
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _write_temp(data: dict) -> str:
    temp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(data, temp, default_flow_style=False, sort_keys=False)
    temp.close()
    return temp.name
