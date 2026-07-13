#!/usr/bin/env python3
# nav_manager.py — pure Python navigation logic: intent parsing, target selection, status
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import math


def is_valid_target(target) -> bool:
    # A target is usable if it is a dict carrying an xyz_world with at least two
    # numeric coordinates (x, y). bool is excluded — it is a subclass of int.
    if not isinstance(target, dict):
        return False
    xyz = target.get("xyz_world")
    return (
        isinstance(xyz, (list, tuple)) and len(xyz) >= 2
        and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in xyz[:2])
    )


class NavManager:
    MAX_COV = 1.0
    CONVERGED_THRESHOLD = 0.9

    def __init__(self):
        self.confirmed_targets: list[dict] = []

    def on_targets(self, json_str: str) -> bool:
        try:
            result = json.loads(json_str)
        except json.JSONDecodeError:
            return False
        if not isinstance(result, list):
            return False
        # Validate once, here at the boundary: keep only targets with a usable
        # xyz_world. Downstream code then trusts every stored target.
        self.confirmed_targets = [t for t in result if is_valid_target(t)]
        return True

    def parse_intent(self, json_str: str) -> tuple[str, dict] | None:
        try:
            intent = json.loads(json_str)
        except json.JSONDecodeError:
            return None
        if not isinstance(intent, dict):
            return None
        action = intent.get("name", "")
        if action not in ("navigation_go", "navigation_cancel"):
            return None
        return (action, intent)

    # target dicts: {"label": str, "xyz_world": [x, y, z], ...} — every stored
    # target has a valid xyz_world (validated in on_targets).
    # robot_xy None = no pose available; fall back to first match rather than blocking navigation
    def find_nearest_confirmed(self, label: str, robot_xy: tuple[float, float] | None) -> dict | None:
        matches = [t for t in self.confirmed_targets if t.get("label") == label]
        if not matches:
            return None
        if robot_xy is None:
            return matches[0]
        rx, ry = robot_xy

        def dist(target: dict) -> float:
            xyz = target["xyz_world"]
            return math.sqrt((xyz[0] - rx) ** 2 + (xyz[1] - ry) ** 2)

        return min(matches, key=dist)

    # covariance: 36-element row-major 6x6; [0]=xx, [7]=yy (meters²)
    # score = 1.0 fully converged, 0.0 fully lost; MAX_COV is the "lost" ceiling
    def check_localization(self, covariance: list[float]) -> tuple[str, float]:
        worst = max(covariance[0], covariance[7])
        score = min(1.0, max(0.0, 1.0 - worst / self.MAX_COV))
        status = "converged" if score >= self.CONVERGED_THRESHOLD else "localizing"
        return (status, score)

    def navigate_status(self, label: str, target: dict | None) -> str:
        if target is None:
            return f"no_target:{label}"
        return f"navigating:{label}"
