#!/usr/bin/env python3
import pytest
from dome_nav.slam_manager import SlamManager


def make_manager(path="/tmp/test_map"):
    return SlamManager(path)


def test_initial_state_not_ready():
    m = make_manager()
    assert m.map_ready is False


def test_should_save_false_before_map():
    m = make_manager()
    assert m.should_save() is False


def test_on_map_received_sets_ready():
    m = make_manager()
    m.on_map_received()
    assert m.map_ready is True


def test_on_map_received_returns_mapping():
    m = make_manager()
    assert m.on_map_received() == "mapping"


def test_on_map_received_idempotent():
    m = make_manager()
    m.on_map_received()
    assert m.on_map_received() == "mapping"
    assert m.map_ready is True


def test_should_save_true_after_map():
    m = make_manager()
    m.on_map_received()
    assert m.should_save() is True


def test_ensure_map_dir_creates_parent(tmp_path):
    path = str(tmp_path / "subdir" / "slam_map")
    m = SlamManager(path)
    m.ensure_map_dir()
    assert (tmp_path / "subdir").exists()


def test_ensure_map_dir_no_parent_no_error():
    m = SlamManager("slam_map")  # no dirname
    m.ensure_map_dir()  # must not raise
