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


def available_worlds(worlds_dir: str) -> list[str]:
    """List installed Gazebo world names (without the .world extension)."""
    return sorted(
        f[: -len(".world")] for f in os.listdir(worlds_dir) if f.endswith(".world")
    )


def require_world_name(world_name: str, worlds_dir: str, usage: str) -> str:
    """Validate world_name against the worlds actually installed in worlds_dir.

    Raises ValueError naming the available choices when missing or unknown,
    rather than letting Gazebo fail later with an opaque "file not found".
    """
    choices = available_worlds(worlds_dir)
    if world_name not in choices:
        raise ValueError(
            f"world_name is required and must be one of {choices}"
            f" (got {world_name!r}): {usage}"
        )
    return world_name


# Each world was designed with a specific robot starting position in mind
# (e.g. multi_room.world uses a corner origin, simple_room.world a centered
# one) -- selecting a world should not also require remembering its spawn
# point by hand.
WORLD_SPAWN_XY: dict[str, tuple[float, float]] = {
    "simple_room": (-1.0, -1.0),
    "multi_room": (1.0, 1.0),
}


def world_spawn_xy(world_name: str) -> tuple[float, float]:
    """Return the designed robot spawn (x, y) for a known world name."""
    return WORLD_SPAWN_XY.get(world_name, (0.0, 0.0))


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
