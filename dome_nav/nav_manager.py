#!/usr/bin/env python3
# nav_manager.py — pure Python navigation logic: intent parsing, target selection, status
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import math


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
        self.confirmed_targets = result
        return True

    def parse_intent(self, json_str: str) -> tuple[str, dict] | None:
        try:
            intent = json.loads(json_str)
        except json.JSONDecodeError:
            return None
        if not isinstance(intent, dict):
            return None
        action = intent.get("name", "")
        if action not in ("go_to_object", "cancel_navigation"):
            return None
        return (action, intent)

    def find_nearest_confirmed(self, label: str, robot_xy: tuple[float, float] | None) -> dict | None:
        matches = [t for t in self.confirmed_targets if t.get("label") == label]
        if not matches:
            return None
        if robot_xy is None:
            return matches[0]
        rx, ry = robot_xy

        def dist(target: dict) -> float:
            xyz = target.get("xyz_world", [0.0, 0.0, 0.0])
            return math.sqrt((xyz[0] - rx) ** 2 + (xyz[1] - ry) ** 2)

        return min(matches, key=dist)

    def check_localization(self, covariance: list[float]) -> tuple[str, float]:
        worst = max(covariance[0], covariance[7])
        score = min(1.0, max(0.0, 1.0 - worst / self.MAX_COV))
        status = "converged" if score >= self.CONVERGED_THRESHOLD else "localizing"
        return (status, score)

    def navigate_status(self, label: str, target: dict | None) -> str:
        if target is None:
            return f"no_target:{label}"
        return f"navigating:{label}"
