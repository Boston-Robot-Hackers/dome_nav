#!/usr/bin/env python3
# explore_manager_node.py — autonomous frontier exploration via Nav2
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import functools
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import tf2_ros

from dome_nav.explore_telemetry import TelemetryWriter
from dome_nav.frontier_explorer import (
    MapInfo,
    _frontier_diag,
    cell_to_world,
    find_frontier_clusters,
    nudge_toward_robot,
    pick_best_frontier,
)

XY = tuple[float, float]

GOAL_STATUS_NAMES = {4: "succeeded", 5: "canceled", 6: "aborted"}


class ExploreManagerNode(Node):
    # Clusters smaller than this are noise (wall edges, single-pixel artifacts).
    # Too low → robot chases tiny artifacts. Too high → misses tight doorway gaps.
    # Good range: 5–20 cells at 0.05 m/cell resolution.
    MIN_FRONTIER_SIZE = 10

    # Frontier cell is skipped if within this distance of any blacklisted point.
    # Must be large enough to cover centroid drift across map updates (~0.1–0.2 m)
    # but small enough not to block adjacent valid cells. 0.5 m works well in practice.
    BLACKLIST_RADIUS = 0.5

    # Robot must be at least this far from a frontier cell for it to be a valid goal.
    # Prevents goals Nav2 deems already-reached (xy_goal_tolerance=0.25 m in patch).
    # Must satisfy: MIN_FRONTIER_DIST > GOAL_INSET_M + xy_goal_tolerance.
    # At current values: 0.8 > 0.3 + 0.25 = 0.55 ✓. Raising this skips nearby frontiers.
    MIN_FRONTIER_DIST = 0.8

    # Nudge the frontier goal this far toward the robot before sending to Nav2.
    # Frontier cells sit at the known/unknown boundary — placing the goal exactly
    # there causes Nav2 worldToMap out-of-bounds errors. 0.3 m keeps the goal
    # comfortably inside the costmap. Must be < MIN_FRONTIER_DIST − xy_goal_tolerance.
    GOAL_INSET_M = 0.3

    # Timer frequency for the exploration loop. 2 Hz is responsive without
    # flooding the action server. Lower values (1 Hz) add latency between goals;
    # higher values (5 Hz) are unnecessary since Nav2 goals are async.
    EXPLORE_HZ = 2.0

    # How many consecutive ticks with no valid frontier before declaring done.
    # At 2 Hz this is 4 s of patience. Too low → quits while map is still updating.
    # Too high → long wait at end of a complete map. 8 ticks (4 s) works well.
    NO_FRONTIER_PATIENCE = 8

    # Cancel active goal after this many seconds to break Nav2 BT recovery loops.
    # Nav2's default BT runs spin + retry before aborting, which can take 60+ s.
    # 25 s is enough for Nav2 to reach most goals; shorter values cause false timeouts
    # on long traversals. Cancelled goal is blacklisted so the same spot is not retried.
    GOAL_TIMEOUT_S = 25.0

    def __init__(self):
        super().__init__("explore_manager_node")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, "/explore/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/explore/markers", 10)
        self.intent_sub = self.create_subscription(
            String, "/intent", self.on_intent, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, 10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.EXPLORE_HZ, self.explore_tick)

        self.declare_parameter("max_explore_radius", 0.0)
        self.declare_parameter("map_name", "unknown")
        self.max_explore_radius: float = self.get_parameter("max_explore_radius").value
        self.map_name: str = self.get_parameter("map_name").value
        self.telemetry = TelemetryWriter(self.map_name, self.get_logger().info)

        self.latest_map: OccupancyGrid | None = None
        self.latest_clusters: list[list[int]] = []
        self.latest_map_info: MapInfo | None = None
        self.reset_session()
        self.clear_active_goal()
        self.get_logger().info("ExploreManagerNode ready.")

    def on_map(self, msg: OccupancyGrid):
        # Called on every /map update from slam_toolbox (typically 1 Hz).
        self.latest_map = msg

    def on_intent(self, msg: String):
        # Called on every /intent message. Only acts on exploration_start (when
        # idle/done) and exploration_stop. All other intents are silently ignored.
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                f"Malformed intent JSON on /intent: {msg.data!r}"
            )
            return
        name = intent.get("name", "")
        if name == "exploration_start" and self.state in ("idle", "done"):
            self.reset_session()
            self.start_xy = self.robot_xy_in_map()
            self.state = "exploring"
            self.publish_status("exploring")
            r = (
                f", max_radius={self.max_explore_radius}m"
                if self.max_explore_radius > 0 else ""
            )
            self.get_logger().info(f"Exploration started{r}.")
            self.telemetry.write(
                "session_start", map_name=self.map_name,
                start_xy=list(self.start_xy) if self.start_xy else None,
                params={
                    "min_frontier_dist": self.MIN_FRONTIER_DIST,
                    "goal_inset": self.GOAL_INSET_M,
                    "timeout_s": self.GOAL_TIMEOUT_S,
                    "max_radius": self.max_explore_radius,
                },
            )
        elif name == "exploration_stop":
            self.stop_exploring("idle")

    def explore_tick(self):
        # Timer callback at EXPLORE_HZ. Skips if not exploring or a goal is active.
        # One goal at a time: next frontier only sent after previous goal completes.
        self.publish_status(self.state)
        self.publish_markers()
        if self.state != "exploring":
            return
        if self.has_active_goal:
            self.check_goal_timeout()
            return
        self.find_and_send_frontier()

    def check_goal_timeout(self):
        # Cancels the active goal if it has exceeded GOAL_TIMEOUT_S.
        if self.goal_start_time is None:
            return
        if (time.monotonic() - self.goal_start_time) <= self.GOAL_TIMEOUT_S:
            return
        elapsed = round(time.monotonic() - self.goal_start_time, 1)
        self.get_logger().warning(
            f"Goal timed out after {elapsed}s — cancelling and blacklisting."
        )
        self.telemetry.write(
            "goal_result", goal_num=self.goal_count, status="timeout",
            elapsed_s=elapsed, robot_xy=None, blacklisted=len(self.blacklist),
            goal_xy=(
                [round(self.current_goal_xy[0], 3), round(self.current_goal_xy[1], 3)]
                if self.current_goal_xy else None
            ),
        )
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.current_goal_centroid is not None:
            self.blacklist.add(self.current_goal_centroid)
        self.clear_active_goal()

    def find_and_send_frontier(self):
        # Core exploration step: scan latest map for frontier clusters, pick the
        # nearest valid one, nudge it inward, and send a Nav2 goal. If no valid
        # frontier exists for NO_FRONTIER_PATIENCE consecutive ticks, declares done.
        if self.latest_map is None:
            self.telemetry.write("no_frontier", reason="no_map")
            return
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            self.get_logger().warning("TF map→base_footprint unavailable — waiting.")
            self.telemetry.write("no_frontier", reason="no_tf")
            return
        m = self.latest_map
        info = MapInfo(
            width=m.info.width, height=m.info.height, resolution=m.info.resolution,
            origin_x=m.info.origin.position.x, origin_y=m.info.origin.position.y,
        )
        clusters = find_frontier_clusters(list(m.data), info)
        self.latest_clusters = clusters
        self.latest_map_info = info
        target = pick_best_frontier(
            clusters, info, robot_xy,
            self.MIN_FRONTIER_SIZE, self.blacklist, self.BLACKLIST_RADIUS,
            self.max_explore_radius, self.start_xy, self.MIN_FRONTIER_DIST,
        )
        if target is None:
            self.no_frontier_count += 1
            self.get_logger().info(
                f"No frontiers found "
                f"(tick {self.no_frontier_count}/{self.NO_FRONTIER_PATIENCE})."
            )
            diag = _frontier_diag(
                clusters, info, robot_xy,
                self.MIN_FRONTIER_SIZE, self.MIN_FRONTIER_DIST,
            )
            self.telemetry.write(
                "no_frontier", reason="filtered",
                tick=self.no_frontier_count,
                patience=self.NO_FRONTIER_PATIENCE,
                raw_clusters=len(clusters),
                blacklisted=len(self.blacklist),
                **diag,
            )
            if self.no_frontier_count >= self.NO_FRONTIER_PATIENCE:
                self.get_logger().info(
                    "Frontier patience exhausted — exploration done."
                )
                self.stop_exploring("done")
            return
        self.no_frontier_count = 0
        goal_xy = nudge_toward_robot(target, robot_xy, self.GOAL_INSET_M)
        self.send_nav_goal(goal_xy, centroid=target)

    def reset_session(self):
        # Resets per-session counters and blacklist.
        # Called on __init__ and exploration_start.
        self.state = "idle"
        self.blacklist: set[XY] = set()
        self.start_xy: XY | None = None
        self.no_frontier_count = 0
        self.goal_count = 0
        self.goals_reached = 0
        self.goals_failed = 0

    def clear_active_goal(self):
        # Resets all goal-tracking fields after a goal completes,
        # times out, or is cancelled.
        self.goal_handle = None
        self.has_active_goal = False
        self.goal_start_time = None
        self.current_goal_centroid = None
        self.current_goal_xy = None

    def send_nav_goal(self, xy: XY, centroid: XY):
        # Sends NavigateToPose goal (nudged xy) to Nav2. Centroid is stored
        # separately for blacklisting — the comparison must use the original centroid,
        # not the nudged goal, so blacklist lookups stay consistent.
        if not self.nav_client.server_is_ready():
            self.get_logger().warning("NavigateToPose server not ready — will retry.")
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = xy[0]
        goal.pose.pose.position.y = xy[1]
        goal.pose.pose.orientation.w = 1.0
        self.has_active_goal = True
        self.goal_start_time = time.monotonic()
        self.current_goal_centroid = centroid
        self.current_goal_xy = xy
        self.goal_count += 1
        robot_xy = self.robot_xy_in_map()
        dist = (
            math.sqrt((xy[0] - robot_xy[0]) ** 2 + (xy[1] - robot_xy[1]) ** 2)
            if robot_xy else -1.0
        )
        self.get_logger().info(
            f"Goal #{self.goal_count}: ({xy[0]:.2f},{xy[1]:.2f})"
            f" dist={dist:.2f}m blacklisted={len(self.blacklist)}"
        )
        self.telemetry.write(
            "goal_sent", goal_num=self.goal_count,
            goal_xy=[round(xy[0], 3), round(xy[1], 3)],
            frontier_xy=[round(centroid[0], 3), round(centroid[1], 3)],
            dist_m=round(dist, 3),
            robot_xy=[round(v, 3) for v in robot_xy] if robot_xy else None,
            blacklisted=len(self.blacklist),
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            functools.partial(self.on_goal_accepted, xy=xy, centroid=centroid)
        )

    def on_goal_accepted(self, future, xy: XY, centroid: XY):
        # Nav2 action callback: goal either accepted (attach result callback) or
        # rejected (blacklist centroid so we don't retry the same unreachable spot).
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning(
                f"Goal rejected at ({xy[0]:.2f}, {xy[1]:.2f}) — blacklisting centroid."
            )
            self.blacklist.add(centroid)
            self.has_active_goal = False
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self.on_goal_result, xy=xy, centroid=centroid)
        )

    def on_goal_result(self, future, xy: XY, centroid: XY):
        # Called when Nav2 finishes (success or failure). Always blacklists centroid
        # to prevent re-visiting the same frontier on the next tick.
        elapsed = (
            round(time.monotonic() - self.goal_start_time, 1)
            if self.goal_start_time else 0.0
        )
        self.clear_active_goal()
        result = future.result()
        robot_xy = self.robot_xy_in_map()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            status_name = "reached"
            self.get_logger().info(
                f"Goal #{self.goal_count} REACHED"
                f" ({xy[0]:.2f},{xy[1]:.2f}) in {elapsed}s."
            )
        else:
            self.goals_failed += 1
            status_name = GOAL_STATUS_NAMES.get(result.status, str(result.status))
            self.get_logger().warning(
                f"Goal #{self.goal_count} FAILED ({xy[0]:.2f},{xy[1]:.2f})"
                f" status={status_name} after {elapsed}s — blacklisting."
            )
        self.telemetry.write(
            "goal_result", goal_num=self.goal_count,
            goal_xy=[round(xy[0], 3), round(xy[1], 3)],
            status=status_name, elapsed_s=elapsed,
            robot_xy=[round(v, 3) for v in robot_xy] if robot_xy else None,
            blacklisted=len(self.blacklist),
        )
        # blacklist centroid (not nudged goal) so pick_best_frontier comparison is exact
        self.blacklist.add(centroid)

    def stop_exploring(self, new_state: str):
        # Cancels any active Nav2 goal and transitions state. Called by
        # exploration_stop intent (→ "idle") and patience exhaustion (→ "done").
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.has_active_goal = False
        self.state = new_state
        self.publish_status(new_state)
        self.get_logger().info(f"Exploration stopped → {new_state}.")
        self.telemetry.write(
            "session_end", reason=new_state, goals_sent=self.goal_count,
            reached=self.goals_reached, failed=self.goals_failed,
        )

    def robot_xy_in_map(self) -> XY | None:
        # Looks up map→base_footprint TF. Returns None if transform not yet available.
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            t = tf.transform.translation
            return (t.x, t.y)
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ):
            return None

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        is_exploring = self.state == "exploring"

        frontier_marker = Marker()
        frontier_marker.header.frame_id = "map"
        frontier_marker.header.stamp = now
        frontier_marker.ns = "frontiers"
        frontier_marker.id = 0
        frontier_marker.type = Marker.POINTS
        frontier_marker.action = Marker.ADD if is_exploring else Marker.DELETE
        frontier_marker.scale.x = 0.05
        frontier_marker.scale.y = 0.05
        frontier_marker.color.r = 1.0
        frontier_marker.color.g = 1.0
        frontier_marker.color.b = 0.0
        frontier_marker.color.a = 1.0
        if is_exploring and self.latest_map_info is not None:
            for cluster in self.latest_clusters:
                if len(cluster) >= self.MIN_FRONTIER_SIZE:
                    for idx in cluster:
                        wx, wy = cell_to_world(idx, self.latest_map_info)
                        p = Point()
                        p.x = wx
                        p.y = wy
                        frontier_marker.points.append(p)
        markers.markers.append(frontier_marker)

        blacklist_marker = Marker()
        blacklist_marker.header.frame_id = "map"
        blacklist_marker.header.stamp = now
        blacklist_marker.ns = "blacklist"
        blacklist_marker.id = 1
        blacklist_marker.type = Marker.POINTS
        blacklist_marker.action = Marker.ADD
        blacklist_marker.scale.x = 0.1
        blacklist_marker.scale.y = 0.1
        blacklist_marker.color.r = 1.0
        blacklist_marker.color.g = 0.0
        blacklist_marker.color.b = 0.0
        blacklist_marker.color.a = 1.0
        for bx, by in self.blacklist:
            p = Point()
            p.x = bx
            p.y = by
            blacklist_marker.points.append(p)
        markers.markers.append(blacklist_marker)

        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = now
        goal_marker.ns = "goal"
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        if is_exploring and self.current_goal_xy is not None:
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.current_goal_xy[0]
            goal_marker.pose.position.y = self.current_goal_xy[1]
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.2
            goal_marker.scale.y = 0.2
            goal_marker.scale.z = 0.2
            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0
            goal_marker.color.b = 1.0
            goal_marker.color.a = 1.0
        else:
            goal_marker.action = Marker.DELETE
        markers.markers.append(goal_marker)

        self.marker_pub.publish(markers)

    def publish_status(self, status: str):
        # Publishes JSON to /explore/status at 2Hz.
        robot_xy = self.robot_xy_in_map()
        data: dict = {
            "state": status,
            "reached": self.goals_reached,
            "failed": self.goals_failed,
        }
        if status == "exploring":
            data["goal_num"] = self.goal_count
            data["blacklisted"] = len(self.blacklist)
            data["no_frontier_ticks"] = self.no_frontier_count
            has_goal = self.current_goal_xy is not None and robot_xy is not None
            if has_goal:
                gx, gy = self.current_goal_xy
                dist = math.sqrt((gx - robot_xy[0]) ** 2 + (gy - robot_xy[1]) ** 2)
                data["goal_xy"] = [round(gx, 2), round(gy, 2)]
                data["dist_m"] = round(dist, 2)
                data["elapsed_s"] = (
                    round(time.monotonic() - self.goal_start_time, 1)
                    if self.goal_start_time else None
                )
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ExploreManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.telemetry.write(
            "session_end", reason="shutdown", goals_sent=node.goal_count,
            reached=node.goals_reached, failed=node.goals_failed,
        )
        node.telemetry.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
