#!/usr/bin/env python3
# test_utils_pure.py — pure unit tests for dome_nav.utils config writing
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import yaml
from dome_nav.utils import write_config


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
