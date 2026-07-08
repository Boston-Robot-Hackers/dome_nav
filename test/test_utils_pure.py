#!/usr/bin/env python3
# test_utils_pure.py — pure unit tests for dome_nav.utils config writing
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import pytest
import yaml
from dome_nav.utils import (
    available_worlds, require_world_name, world_spawn_xy, write_config,
)


def test_same_data_same_path(monkeypatch, tmp_path):
    monkeypatch.setenv("DOME_HOME", str(tmp_path))
    data = {"a": {"b": 1}}
    assert write_config(data) == write_config(data)


def test_different_data_different_path(monkeypatch, tmp_path):
    monkeypatch.setenv("DOME_HOME", str(tmp_path))
    assert write_config({"a": 1}) != write_config({"a": 2})


def test_content_round_trips(monkeypatch, tmp_path):
    monkeypatch.setenv("DOME_HOME", str(tmp_path))
    data = {"x": {"y": [1, 2, 3]}}
    path = write_config(data)
    with open(path) as f:
        assert yaml.safe_load(f) == data


def test_cache_dir_created(monkeypatch, tmp_path):
    monkeypatch.setenv("DOME_HOME", str(tmp_path))
    write_config({"k": "v"})
    assert (tmp_path / "launch_cache").is_dir()


# Encodes the bug this fix exists for: repeated identical launches must not
# accumulate files on disk (the old NamedTemporaryFile approach leaked one per call).
def test_repeated_identical_writes_do_not_accumulate(monkeypatch, tmp_path):
    monkeypatch.setenv("DOME_HOME", str(tmp_path))
    for _ in range(10):
        write_config({"same": "config"})
    files = list((tmp_path / "launch_cache").iterdir())
    assert len(files) == 1


# --- world selection ---

def make_worlds_dir(tmp_path, names):
    d = tmp_path / "worlds"
    d.mkdir()
    for name in names:
        (d / f"{name}.world").write_text("<sdf></sdf>")
    return str(d)


def test_available_worlds_lists_world_files_without_extension(tmp_path):
    d = make_worlds_dir(tmp_path, ["simple_room", "multi_room"])
    assert available_worlds(d) == ["multi_room", "simple_room"]


def test_available_worlds_ignores_non_world_files(tmp_path):
    d = make_worlds_dir(tmp_path, ["simple_room"])
    (tmp_path / "worlds" / "readme.txt").write_text("not a world")
    assert available_worlds(d) == ["simple_room"]


def test_require_world_name_accepts_known_choice(tmp_path):
    d = make_worlds_dir(tmp_path, ["simple_room", "multi_room"])
    assert require_world_name("multi_room", d, "usage") == "multi_room"


def test_require_world_name_rejects_empty(tmp_path):
    d = make_worlds_dir(tmp_path, ["simple_room"])
    with pytest.raises(ValueError, match="world_name is required"):
        require_world_name("", d, "usage")


def test_require_world_name_lists_choices_in_message(tmp_path):
    d = make_worlds_dir(tmp_path, ["simple_room", "multi_room"])
    pattern = r"multi_room.*simple_room|simple_room.*multi_room"
    with pytest.raises(ValueError, match=pattern):
        require_world_name("no_such_world", d, "usage")


def test_require_world_name_includes_usage_hint(tmp_path):
    d = make_worlds_dir(tmp_path, ["simple_room"])
    with pytest.raises(ValueError, match="bl dome_nav sim_robot.launch.py"):
        require_world_name(
            "", d, "bl dome_nav sim_robot.launch.py --world_name <name>"
        )


def test_world_spawn_xy_known_world():
    assert world_spawn_xy("multi_room") == (1.0, 1.0)
    assert world_spawn_xy("simple_room") == (-1.0, -1.0)


def test_world_spawn_xy_unknown_world_defaults_origin():
    assert world_spawn_xy("some_future_world") == (0.0, 0.0)


