#!/usr/bin/env python3
# nav_intent_check.py — diagnostic: test intent-driven navigation on live stack
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
#
# What this does:
#   Reads the robot's current map-frame pose from /amcl_pose (requires AMCL converged).
#   Computes a target 50cm to the LEFT of the robot's current heading. Nav2's
#   RotationShimController rotates ~90° to face the target before driving, so both
#   rotation and translation are visible.
#   Publishes that target to /targets/confirmed and a go_to_object intent to /intent,
#   then waits for nav_status to transition through navigating:chair → done/failed.
#   Verifies the full intent→Nav2 pipeline end-to-end without dome_vision or dome_control.
#
# Usage:
#   python3 tools/nav_intent_check.py
#
# Requires: bl dome_nav robot_nav.launch.py running, AMCL converged (localization_score > 0.9)

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

LABEL = "chair"
GOAL_DIST_M = 0.50
NAV_TIMEOUT_SEC = 20.0


class NavIntentChecker(Node):
    def __init__(self):
        super().__init__("nav_intent_check")
        self.current_pose = None
        self.nav_statuses: list[str] = []

        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, amcl_qos
        )
        self.status_sub = self.create_subscription(
            String, "/dome_nav/nav_status", self.on_nav_status, 10
        )
        self.targets_pub = self.create_publisher(String, "/targets/confirmed", 10)
        self.intent_pub = self.create_publisher(String, "/intent", 10)

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def on_nav_status(self, msg: String):
        self.nav_statuses.append(msg.data)
        print(f"  nav_status: {msg.data}")

    def spin_for(self, secs: float):
        deadline = time.time() + secs
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_pose(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_pose is not None

    def current_yaw(self) -> float:
        q = self.current_pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def compute_target(self) -> dict:
        p = self.current_pose
        yaw = self.current_yaw()
        # Place target 90° left of robot heading — NavFn plans straight path,
        # RotationShimController must rotate ~90° (above 45° threshold) before driving
        left_yaw = yaw + math.pi / 2.0
        tx = p.position.x + GOAL_DIST_M * math.cos(left_yaw)
        ty = p.position.y + GOAL_DIST_M * math.sin(left_yaw)
        return {
            "label": LABEL,
            "xyz_world": [round(tx, 3), round(ty, 3), 0.0],
        }

    def send_target(self, target: dict):
        msg = String()
        msg.data = json.dumps([target])
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self.targets_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(3):
            self.targets_pub.publish(msg)
            self.spin_for(0.1)

    def send_intent(self):
        msg = String()
        msg.data = json.dumps({"name": "navigation_go", "source": "tool", "slots": {"label": LABEL}})
        # Wait until nav_manager's subscription is visible, then publish once
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self.intent_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.intent_pub.publish(msg)
        self.spin_for(0.2)

    def wait_for_status_prefix(self, prefix: str, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if any(s.startswith(prefix) for s in self.nav_statuses):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_for_terminal_status(self, timeout_sec: float) -> str | None:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            for s in self.nav_statuses:
                if s.startswith("done:") or s.startswith("failed:"):
                    return s
            rclpy.spin_once(self, timeout_sec=0.1)
        return None


def confirm_ready():
    print("\n" + "=" * 60)
    print("DIAGNOSTIC: Intent-driven navigation check")
    print("=" * 60)
    print("\nWhat will happen:")
    print("  1. Read current robot pose from /amcl_pose")
    print("  2. Compute target 50cm BEHIND robot's current heading")
    print("  3. Robot rotates ~90° left to face target, then drives 50cm")
    print("  4. Report nav_status transitions: navigating:chair → done/failed")
    print("\nWARNING: Ensure at least 1m clear space in ALL directions.")
    print("Requires: robot_nav.launch.py running, AMCL converged.\n")
    answer = input("Proceed? [y/n]: ").strip().lower()
    return answer == "y"


def main():
    if not confirm_ready():
        print("Aborted.")
        sys.exit(0)

    rclpy.init()
    node = NavIntentChecker()

    print("\n[1/4] Waiting for AMCL pose...")
    if not node.wait_for_pose(timeout_sec=10.0):
        print("FAIL: /amcl_pose not received within 10s — stack running and AMCL converged?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    p = node.current_pose
    yaw = node.current_yaw()
    print(f"  pose: x={p.position.x:.3f} y={p.position.y:.3f} yaw={math.degrees(yaw):.1f}°")

    target = node.compute_target()
    print(f"\n[2/4] Sending target 50cm to robot's left: {target['xyz_world']}")
    node.send_target(target)
    print(f"  targets subscribers found: {node.targets_pub.get_subscription_count()}")

    print("\n[3/4] Sending navigation_go intent...")
    print(f"  intent subscribers found: {node.intent_pub.get_subscription_count()}")
    node.send_intent()

    print("\n[4/4] Waiting for navigation to start...")
    if not node.wait_for_status_prefix(f"navigating:{LABEL}", timeout_sec=5.0):
        print(f"FAIL: nav_status never showed navigating:{LABEL}")
        print(f"  Statuses seen: {node.nav_statuses}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"  OK: navigation started. Waiting up to {NAV_TIMEOUT_SEC}s for completion...")
    terminal = node.wait_for_terminal_status(timeout_sec=NAV_TIMEOUT_SEC)
    if terminal is None:
        print(f"FAIL: navigation did not complete within {NAV_TIMEOUT_SEC}s")
        print(f"  Statuses seen: {node.nav_statuses}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"\nRESULT: {terminal}")
    print("PASS" if terminal.startswith("done:") else "WARN: navigation failed (path blocked?)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
