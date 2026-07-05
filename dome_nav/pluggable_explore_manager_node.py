#!/usr/bin/env python3
# pluggable_explore_manager_node.py — autonomous frontier exploration via Nav2 (pluggable algorithm)
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import NavigateToPose
import tf2_ros

from dome_nav.explore_context import (
    ExplorationAlgorithm,
    ExplorationContext,
    ExploreParams,
)
from dome_nav.explore_telemetry import TelemetryWriter
from dome_nav.explore_markers import build_explore_markers
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.frontier_explorer import MapInfo

XY = tuple[float, float]

GOAL_STATUS_NAMES = {4: "succeeded", 5: "canceled", 6: "aborted"}


class PluggableExploreManagerNode(Node):
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

    # If the best frontier moves more than this many metres from the current goal,
    # cancel mid-flight and redirect. Accounts for the map updating during transit
    # (lidar reveals new cells along the path). Too small → constant churn; too
    # large → stale goals persist after the map changes significantly.
    REDIRECT_THRESHOLD = 1.5

    def __init__(self, algorithm: ExplorationAlgorithm | None = None):
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
        self.declare_parameter("max_frontier_dist", 3.0)
        self.declare_parameter("prefer_farthest", False)
        self.declare_parameter("min_frontier_size", 10)
        self.declare_parameter("map_name", "unknown")
        self.max_explore_radius: float = self.get_parameter("max_explore_radius").value
        self.max_frontier_dist: float = self.get_parameter("max_frontier_dist").value
        self.prefer_farthest: bool = self.get_parameter("prefer_farthest").value
        self.min_frontier_size: int = self.get_parameter("min_frontier_size").value
        self.map_name: str = self.get_parameter("map_name").value
        self.telemetry = TelemetryWriter(self.map_name, self.get_logger().info)

        self.params = ExploreParams(
            max_explore_radius=self.max_explore_radius,
            max_frontier_dist=self.max_frontier_dist,
            prefer_farthest=self.prefer_farthest,
            min_frontier_size=self.min_frontier_size,
        )
        self.algorithm = algorithm or FrontierAlgorithm()

        self.latest_map: OccupancyGrid | None = None
        self.latest_map_info: MapInfo | None = None
        self.redirect_map_stamp: tuple[int, int] | None = None
        self.redirect_cached_goal: XY | None = None
        self.reset_session()
        self.clear_active_goal()
        self.get_logger().info("PluggableExploreManagerNode ready.")

    def on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def on_intent(self, msg: String):
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
                f", max_radius={self.params.max_explore_radius}m"
                if self.params.max_explore_radius > 0 else ""
            )
            self.get_logger().info(f"Exploration started{r}.")
            self.telemetry.write(
                "session_start", map_name=self.map_name,
                start_xy=list(self.start_xy) if self.start_xy else None,
                params={
                    "min_frontier_dist": self.params.min_frontier_dist,
                    "max_frontier_dist": self.params.max_frontier_dist,
                    "goal_inset": self.params.goal_inset_m,
                    "timeout_s": self.GOAL_TIMEOUT_S,
                    "max_radius": self.params.max_explore_radius,
                    "prefer_farthest": self.params.prefer_farthest,
                    "min_frontier_size": self.params.min_frontier_size,
                },
            )
        elif name == "exploration_stop":
            self.stop_exploring("idle")

    def explore_tick(self):
        self.publish_status(self.state)
        self.publish_markers()
        if self.state != "exploring":
            return
        if self.has_active_goal:
            self.check_goal_timeout()
            self.check_goal_redirect()
            return
        self.find_and_send_frontier()

    def check_goal_timeout(self):
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

    def frontier_goal_for_current_map(self, robot_xy: XY) -> XY | None:
        m = self.latest_map
        stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
        if stamp == self.redirect_map_stamp:
            return self.redirect_cached_goal
        info = MapInfo(
            width=m.info.width, height=m.info.height, resolution=m.info.resolution,
            origin_x=m.info.origin.position.x, origin_y=m.info.origin.position.y,
        )
        ctx = ExplorationContext(
            map_data=list(m.data),
            map_info=info,
            robot_xy=robot_xy,
            blacklist=self.blacklist,
            start_xy=self.start_xy,
            params=self.params,
        )
        self.redirect_map_stamp = stamp
        self.redirect_cached_goal = self.algorithm.next_goal(ctx)
        return self.redirect_cached_goal

    def check_goal_redirect(self):
        # Re-evaluate the frontier each tick while in transit. If the best goal
        # has moved more than REDIRECT_THRESHOLD from the current goal (because
        # the map updated as the robot scanned along its path), cancel and redirect.
        # The intent is to opportunistically capture map updates that happen
        # mid-flight: the lidar keeps scanning during travel, so by the time the
        # robot would have arrived at a goal picked minutes ago, that frontier may
        # already be stale — a nearer, newly-revealed one is often better.
        #
        # slam_toolbox only republishes /map on its own map_update_interval (5s by
        # default), far slower than this 2Hz tick — recomputing frontier clustering
        # against an unchanged map is wasted work, so the result is memoized by map
        # stamp and reused until a genuinely new map arrives.
        #
        # Disabled under prefer_farthest: "best" there means "farthest from the
        # robot's current position," which flips sides as soon as the robot moves
        # at all toward either side — not because the map changed, but purely as
        # an artifact of the robot's own motion. Confirmed via telemetry
        # (~/.dome/telemetry/explore-boo1-20260705.jsonl) on 2026-07-05: the robot
        # ping-ponged between two points ~1.7m apart, redirected every ~10s,
        # 17 goals sent and zero reached in one session. Under nearest-first this
        # redirect is stable (moving toward the nearest frontier keeps it nearest,
        # or the map reveals it and a new nearby one takes over) — the instability
        # is specific to farthest-first's dependence on the robot's own position.
        if self.prefer_farthest:
            return
        no_active_redirect_target = (
            self.is_redirecting or self.latest_map is None
            or self.current_goal_xy is None
        )
        if no_active_redirect_target:
            return
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            return
        new_goal = self.frontier_goal_for_current_map(robot_xy)
        if new_goal is None:
            return
        dist = math.sqrt(
            (new_goal[0] - self.current_goal_xy[0]) ** 2
            + (new_goal[1] - self.current_goal_xy[1]) ** 2
        )
        if dist < self.REDIRECT_THRESHOLD:
            return
        self.get_logger().info(
            f"Redirect: best frontier moved {dist:.2f}m from current goal "
            f"— cancelling #{self.goal_count} and redirecting."
        )
        self.telemetry.write(
            "redirect", goal_num=self.goal_count,
            old_goal_xy=[round(self.current_goal_xy[0], 3),
                         round(self.current_goal_xy[1], 3)],
            new_goal_xy=[round(new_goal[0], 3), round(new_goal[1], 3)],
            shift_m=round(dist, 3),
        )
        self.is_redirecting = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def find_and_send_frontier(self):
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
        self.latest_map_info = info
        ctx = ExplorationContext(
            map_data=list(m.data),
            map_info=info,
            robot_xy=robot_xy,
            blacklist=self.blacklist,
            start_xy=self.start_xy,
            params=self.params,
        )
        goal_xy = self.algorithm.next_goal(ctx)
        if goal_xy is None:
            self.no_frontier_count += 1
            self.get_logger().info(
                f"No frontiers found "
                f"(tick {self.no_frontier_count}/{self.NO_FRONTIER_PATIENCE})."
            )
            diag = self.algorithm.latest_diag or {}
            self.telemetry.write(
                "no_frontier", reason="filtered",
                tick=self.no_frontier_count,
                patience=self.NO_FRONTIER_PATIENCE,
                raw_clusters=len(self.algorithm.latest_clusters),
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
        # The algorithm already nudges; centroid is the pre-nudge cluster cell.
        # For blacklisting we need the raw frontier cell, not the nudged goal.
        # FrontierAlgorithm stores that as the pick_best_frontier return value
        # before nudging; we recover it from the clusters for blacklist consistency.
        # However, the original node passed centroid=target (the raw pick_best_frontier
        # result) separately. Here we pass goal_xy as both — the blacklist comparison
        # still works because the same nudged point is stored and checked each time.
        self.send_nav_goal(goal_xy, centroid=goal_xy)

    def reset_session(self):
        self.state = "idle"
        self.blacklist: set[XY] = set()
        self.start_xy: XY | None = None
        self.no_frontier_count = 0
        self.goal_count = 0
        self.goals_reached = 0
        self.goals_failed = 0

    def clear_active_goal(self):
        self.goal_handle = None
        self.has_active_goal = False
        self.goal_start_time = None
        self.current_goal_centroid = None
        self.current_goal_xy = None
        self.is_redirecting = False

    def send_nav_goal(self, xy: XY, centroid: XY):
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
        elapsed = (
            round(time.monotonic() - self.goal_start_time, 1)
            if self.goal_start_time else 0.0
        )
        if self.is_redirecting:
            # Cancelled for a better frontier — not a failure, do not blacklist.
            self.clear_active_goal()
            return
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
        self.blacklist.add(centroid)

    def stop_exploring(self, new_state: str):
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
        markers = build_explore_markers(
            now=self.get_clock().now().to_msg(),
            is_exploring=self.state == "exploring",
            clusters=self.algorithm.latest_clusters,
            min_frontier_size=self.params.min_frontier_size,
            map_info=self.latest_map_info,
            blacklist=self.blacklist,
            goal_xy=self.current_goal_xy,
        )
        self.marker_pub.publish(markers)

    def publish_status(self, status: str):
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
    node = PluggableExploreManagerNode()
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
