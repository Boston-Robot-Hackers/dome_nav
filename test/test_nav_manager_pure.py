#!/usr/bin/env python3
import json
import pytest
from dome_nav.nav_manager import NavManager


@pytest.fixture
def mgr():
    return NavManager()


# --- on_targets ---

def test_on_targets_valid_json(mgr):
    targets = [{"label": "chair", "xyz_world": [1.0, 2.0, 0.0]}]
    assert mgr.on_targets(json.dumps(targets)) is True
    assert mgr.confirmed_targets == targets


def test_on_targets_invalid_json(mgr):
    assert mgr.on_targets("not json") is False
    assert mgr.confirmed_targets == []


def test_on_targets_empty_list(mgr):
    assert mgr.on_targets("[]") is True
    assert mgr.confirmed_targets == []


# --- parse_intent ---

def test_parse_intent_go_to_object(mgr):
    payload = json.dumps({"action": "go_to_object", "label": "chair"})
    result = mgr.parse_intent(payload)
    assert result is not None
    action, intent = result
    assert action == "go_to_object"
    assert intent["label"] == "chair"


def test_parse_intent_cancel(mgr):
    payload = json.dumps({"action": "cancel_navigation"})
    result = mgr.parse_intent(payload)
    assert result is not None
    assert result[0] == "cancel_navigation"


def test_parse_intent_invalid_json(mgr):
    assert mgr.parse_intent("bad json") is None


def test_parse_intent_unknown_action(mgr):
    payload = json.dumps({"action": "fly_to_moon"})
    assert mgr.parse_intent(payload) is None


def test_parse_intent_missing_action(mgr):
    assert mgr.parse_intent(json.dumps({})) is None


# --- find_nearest_confirmed ---

def test_find_nearest_empty_targets(mgr):
    assert mgr.find_nearest_confirmed("chair", None) is None


def test_find_nearest_no_label_match(mgr):
    mgr.confirmed_targets = [{"label": "table", "xyz_world": [1.0, 0.0, 0.0]}]
    assert mgr.find_nearest_confirmed("chair", None) is None


def test_find_nearest_no_robot_xy_returns_first(mgr):
    mgr.confirmed_targets = [
        {"label": "cup", "xyz_world": [10.0, 0.0, 0.0]},
        {"label": "cup", "xyz_world": [1.0, 0.0, 0.0]},
    ]
    result = mgr.find_nearest_confirmed("cup", None)
    assert result["xyz_world"] == [10.0, 0.0, 0.0]


def test_find_nearest_returns_closest(mgr):
    mgr.confirmed_targets = [
        {"label": "chair", "xyz_world": [10.0, 0.0, 0.0]},
        {"label": "chair", "xyz_world": [1.0, 0.0, 0.0]},
        {"label": "chair", "xyz_world": [5.0, 0.0, 0.0]},
    ]
    result = mgr.find_nearest_confirmed("chair", (0.0, 0.0))
    assert result["xyz_world"] == [1.0, 0.0, 0.0]


def test_find_nearest_from_non_origin(mgr):
    mgr.confirmed_targets = [
        {"label": "box", "xyz_world": [0.0, 0.0, 0.0]},
        {"label": "box", "xyz_world": [8.0, 0.0, 0.0]},
    ]
    result = mgr.find_nearest_confirmed("box", (7.0, 0.0))
    assert result["xyz_world"] == [8.0, 0.0, 0.0]


# --- navigate_status ---

def test_navigate_status_no_target(mgr):
    assert mgr.navigate_status("chair", None) == "no_target:chair"


def test_navigate_status_with_target(mgr):
    target = {"label": "chair", "xyz_world": [1.0, 2.0, 0.0]}
    assert mgr.navigate_status("chair", target) == "navigating:chair"
