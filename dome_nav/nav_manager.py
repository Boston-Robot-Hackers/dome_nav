#!/usr/bin/env python3
# nav_manager.py — pure nav logic: intent parsing, target selection, status
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import math


def is_valid_target(target) -> bool:
    """True if target is a dict with an xyz_world of >=2 numeric (x, y) coords.

    bool is excluded — it is a subclass of int.
    """
    if not isinstance(target, dict):
        return False
    xyz = target.get("xyz_world")
    return (
        isinstance(xyz, (list, tuple)) and len(xyz) >= 2
        and all(
            isinstance(coord, (int, float)) and not isinstance(coord, bool)
            for coord in xyz[:2]
        )
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
        # Validate once at the boundary; stored targets are then trusted.
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

    def find_nearest_confirmed(
        self, label: str, robot_xy: tuple[float, float] | None
    ) -> dict | None:
        """Nearest label-matching confirmed target to robot_xy, or None if no match.

        Every stored target has a valid xyz_world (validated in on_targets). robot_xy
        None means no pose available; falls back to the first match rather than
        blocking navigation.
        """
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

    def check_localization(self, covariance: list[float]) -> tuple[str, float]:
        """Map AMCL covariance to a (status, score) pair.

        covariance is the 36-element row-major 6x6 pose covariance; [0]=xx, [7]=yy
        in meters². score is 1.0 fully converged, 0.0 fully lost; MAX_COV is the
        "lost" ceiling.
        """
        worst = max(covariance[0], covariance[7])
        score = min(1.0, max(0.0, 1.0 - worst / self.MAX_COV))
        status = "converged" if score >= self.CONVERGED_THRESHOLD else "localizing"
        return (status, score)

    def navigate_status(self, label: str, target: dict | None) -> str:
        if target is None:
            return f"no_target:{label}"
        return f"navigating:{label}"
