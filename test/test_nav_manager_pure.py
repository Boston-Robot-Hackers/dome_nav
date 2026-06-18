#!/usr/bin/env python3
# test_nav_manager_pure.py — pure unit tests for NavManager (no ROS2)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

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


def test_on_targets_dict_json_rejected(mgr):
    assert mgr.on_targets(json.dumps({"label": "chair"})) is False
    assert mgr.confirmed_targets == []


def test_on_targets_scalar_json_rejected(mgr):
    assert mgr.on_targets("42") is False
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


def test_parse_intent_list_json_rejected(mgr):
    assert mgr.parse_intent(json.dumps(["go_to_object", "chair"])) is None


def test_parse_intent_string_json_rejected(mgr):
    assert mgr.parse_intent(json.dumps("go_to_object")) is None


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


# --- check_localization ---

def _cov(x, y):
    cov = [0.0] * 36
    cov[0] = x
    cov[7] = y
    return cov


def test_check_localization_perfect(mgr):
    status, score = mgr.check_localization(_cov(0.0, 0.0))
    assert score == 1.0
    assert status == "converged"


def test_check_localization_at_threshold(mgr):
    status, score = mgr.check_localization(_cov(0.1, 0.05))
    assert abs(score - 0.9) < 1e-9
    assert status == "converged"


def test_check_localization_partial(mgr):
    status, score = mgr.check_localization(_cov(0.5, 0.3))
    assert abs(score - 0.5) < 1e-9
    assert status == "localizing"


def test_check_localization_lost(mgr):
    status, score = mgr.check_localization(_cov(1.0, 1.0))
    assert score == 0.0
    assert status == "localizing"


def test_check_localization_clamped(mgr):
    status, score = mgr.check_localization(_cov(2.0, 2.0))
    assert score == 0.0
    assert status == "localizing"


def test_check_localization_uses_max_of_two(mgr):
    status, score = mgr.check_localization(_cov(0.05, 0.8))
    assert abs(score - 0.2) < 1e-9
    assert status == "localizing"


def test_check_localization_negative_cov_clamped_to_1(mgr):
    status, score = mgr.check_localization(_cov(-1.0, -1.0))
    assert score == 1.0
    assert status == "converged"


# --- navigate_status ---

def test_navigate_status_no_target(mgr):
    assert mgr.navigate_status("chair", None) == "no_target:chair"


def test_navigate_status_with_target(mgr):
    target = {"label": "chair", "xyz_world": [1.0, 2.0, 0.0]}
    assert mgr.navigate_status("chair", target) == "navigating:chair"
