#!/usr/bin/env python3
# test_telemetry.py — pure unit tests for explore_telemetry filename logic
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import pytest
from datetime import datetime
from unittest.mock import patch
from dome_nav.explore_telemetry import build_telemetry_filename


FIXED_NOW = datetime(2026, 7, 10)


def _build(map_name, existing=(), now=FIXED_NOW):
    fake_home = "/fake"
    telemetry_dir = "/fake/.dome/telemetry"
    existing_paths = {f"{telemetry_dir}/{f}" for f in existing}
    with patch("dome_nav.explore_telemetry.os.path.expanduser", return_value=fake_home), \
         patch("dome_nav.explore_telemetry.os.path.exists", side_effect=lambda p: p in existing_paths):
        return build_telemetry_filename(map_name, now=now)


def test_basic_filename():
    assert _build("basement1") == "ebasement110-jul.json"


def test_date_format():
    assert _build("test", now=datetime(2026, 12, 1)) == "etest01-dec.json"


def test_invalid_chars_replaced():
    name = _build("my map/run")
    assert "/" not in name
    assert " " not in name
    assert name.startswith("e")
    assert name.endswith(".json")


def test_name_truncated_at_32_chars():
    name = _build("a" * 50)
    safe_part = name[1:name.index("10-jul")]
    assert len(safe_part) <= 32


def test_default_session_fallback():
    assert _build("session") == "esession10-jul.json"


def test_collision_adds_suffix():
    existing = ["etest10-jul.json"]
    assert _build("test", existing=existing) == "etest10-jul-2.json"


def test_collision_increments_to_three():
    existing = ["etest10-jul.json", "etest10-jul-2.json"]
    assert _build("test", existing=existing) == "etest10-jul-3.json"
