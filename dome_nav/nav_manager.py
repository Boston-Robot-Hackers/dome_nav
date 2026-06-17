#!/usr/bin/env python3
# nav_manager.py — pure Python navigation logic: intent parsing, target selection, status
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import math


class NavManager:
    def __init__(self):
        self.confirmed_targets: list[dict] = []

    def on_targets(self, json_str: str) -> bool:
        try:
            self.confirmed_targets = json.loads(json_str)
            return True
        except json.JSONDecodeError:
            return False

    def parse_intent(self, json_str: str) -> tuple[str, dict] | None:
        try:
            intent = json.loads(json_str)
        except json.JSONDecodeError:
            return None
        action = intent.get("action", "")
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

    def navigate_status(self, label: str, target: dict | None) -> str:
        if target is None:
            return f"no_target:{label}"
        return f"navigating:{label}"
