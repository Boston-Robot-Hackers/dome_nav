#!/usr/bin/env python3
# pluggable_explore_manager_node.py — Nav2 frontier exploration, pluggable algorithm
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
from dome_nav.frontier_explorer import MapInfo, cell_to_world

XY = tuple[float, float]

GOAL_STATUS_NAMES = {4: "succeeded", 5: "canceled", 6: "aborted"}

# ComputePathToPose error codes (200 range)
# FollowPath error codes (100 range)
NAV2_ERROR_CODES = {
    0: "NONE",
    100: "FOLLOW/UNKNOWN", 101: "FOLLOW/INVALID_CONTROLLER", 102: "FOLLOW/TF_ERROR",
    103: "FOLLOW/INVALID_PATH", 104: "FOLLOW/PATIENCE_EXCEEDED",
    105: "FOLLOW/FAILED_TO_MAKE_PROGRESS", 106: "FOLLOW/NO_VALID_CONTROL",
    107: "FOLLOW/CONTROLLER_TIMED_OUT",
    200: "PLAN/UNKNOWN", 201: "PLAN/INVALID_PLANNER", 202: "PLAN/TF_ERROR",
    203: "PLAN/START_OUTSIDE_MAP", 204: "PLAN/GOAL_OUTSIDE_MAP",
    205: "PLAN/START_OCCUPIED", 206: "PLAN/GOAL_OCCUPIED",
    207: "PLAN/TIMEOUT", 208: "PLAN/NO_VALID_PATH",
}


class PluggableExploreManagerNode(Node):
    # Timer frequency for the exploration loop. 2 Hz is responsive without
    # flooding the action server. Lower values (1 Hz) add latency between goals;
    # higher values (5 Hz) are unnecessary since Nav2 goals are async.
    EXPLORE_HZ = 2.0

    # How many consecutive ticks with no valid frontier before declaring done.
    # At 2 Hz this is 7 s of patience. Too low → quits while map is still updating.
    # Too high → long wait at end of a complete map. Must exceed slam_toolbox's own
    # map_update_interval (5 s default, not overridden) -- 8 ticks (4 s) was shorter
    # than that, so patience could run out before /map had refreshed even once.
    # 14 ticks (7 s) gives one full 5 s interval plus margin.
    NO_FRONTIER_PATIENCE = 14

    # Cancel active goal after this many seconds to break Nav2 BT recovery loops.
    # Nav2's default BT runs spin + retry before aborting, which can take 60+ s.
    # 25 s is enough for Nav2 to reach most goals; shorter values cause false timeouts
    # on long traversals. Cancelled goal is blacklisted so the same spot is not retried.
    GOAL_TIMEOUT_S = 25.0

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
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_global_costmap, 1
        )
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_local_costmap, 1
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.EXPLORE_HZ, self.explore_tick)

        self.declare_parameter("max_explore_radius", 0.0)
        self.declare_parameter("max_frontier_dist", 15.0)
        # Default matches ExploreParams.min_frontier_dist (real-robot 1.0 m sent-goal
        # floor after the 0.3 m goal_inset); sim launch files lower it.
        self.declare_parameter("min_frontier_dist", 1.3)
        self.declare_parameter("prefer_farthest", False)
        self.declare_parameter("min_frontier_size", 10)
        self.declare_parameter("map_name", "unknown")
        self.max_explore_radius: float = self.get_parameter("max_explore_radius").value
        self.max_frontier_dist: float = self.get_parameter("max_frontier_dist").value
        self.min_frontier_dist: float = self.get_parameter("min_frontier_dist").value
        self.prefer_farthest: bool = self.get_parameter("prefer_farthest").value
        self.min_frontier_size: int = self.get_parameter("min_frontier_size").value
        self.map_name: str = self.get_parameter("map_name").value
        self.telemetry = TelemetryWriter(self.get_logger().info)

        self.params = ExploreParams(
            max_explore_radius=self.max_explore_radius,
            max_frontier_dist=self.max_frontier_dist,
            min_frontier_dist=self.min_frontier_dist,
            prefer_farthest=self.prefer_farthest,
            min_frontier_size=self.min_frontier_size,
        )
        self.algorithm = algorithm or FrontierAlgorithm()

        self.latest_map: OccupancyGrid | None = None
        self.latest_map_info: MapInfo | None = None
        self.latest_global_costmap: OccupancyGrid | None = None
        self.latest_local_costmap: OccupancyGrid | None = None
        self.paused_on_failure = False
        self.reset_session()
        self.clear_active_goal()
        self.get_logger().info("PluggableExploreManagerNode ready.")

    def on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def on_global_costmap(self, msg: OccupancyGrid):
        self.latest_global_costmap = msg

    def on_local_costmap(self, msg: OccupancyGrid):
        self.latest_local_costmap = msg

    def costmap_cell_cost(self, costmap: OccupancyGrid | None, xy: XY) -> int | None:
        if costmap is None:
            return None
        info = costmap.info
        col = int((xy[0] - info.origin.position.x) / info.resolution)
        row = int((xy[1] - info.origin.position.y) / info.resolution)
        if col < 0 or col >= info.width or row < 0 or row >= info.height:
            return None
        return costmap.data[row * info.width + col]

    def dump_frontier_exhaustion(self, robot_xy: XY | None):
        sep = "=" * 60
        clusters = self.algorithm.latest_clusters
        info = self.latest_map_info
        lines = [
            sep,
            f"FRONTIER EXHAUSTION — {len(clusters)} raw clusters,"
            f" patience={self.NO_FRONTIER_PATIENCE}",
            f"  filters: min_size={self.params.min_frontier_size}"
            f"  min_dist={self.params.min_frontier_dist}m"
            f"  max_dist={self.params.max_frontier_dist}m"
            f"  blacklisted={len(self.blacklist)}",
            f"  robot_xy: {robot_xy}",
            "",
        ]
        if info is None or not clusters:
            lines.append("  (no map info or no clusters)")
        else:
            rx, ry = robot_xy if robot_xy else (0.0, 0.0)
            bl = self.blacklist
            br = self.params.blacklist_radius
            for i, cl in enumerate(clusters):
                cx = sum(cell_to_world(idx, info)[0] for idx in cl) / len(cl)
                cy = sum(cell_to_world(idx, info)[1] for idx in cl) / len(cl)
                centroid_dist = (
                    math.sqrt((cx - rx)**2 + (cy - ry)**2) if robot_xy else -1.0
                )
                too_small = len(cl) < self.params.min_frontier_size
                # nearest non-blacklisted cell distance
                min_cell_dist = float("inf")
                for cell_idx in cl:
                    wx, wy = cell_to_world(cell_idx, info)
                    if any(math.sqrt((wx-bx)**2+(wy-by)**2) < br for bx, by in bl):
                        continue
                    d = math.sqrt((wx-rx)**2+(wy-ry)**2) if robot_xy else 0.0
                    min_cell_dist = min(min_cell_dist, d)
                min_dist = self.params.min_frontier_dist
                max_dist = self.params.max_frontier_dist
                min_size = self.params.min_frontier_size
                reasons = []
                if too_small:
                    reasons.append(f"too_small({len(cl)}<{min_size})")
                if min_cell_dist == float("inf"):
                    reasons.append("all_blacklisted")
                elif min_dist > 0 and min_cell_dist < min_dist:
                    reasons.append(f"too_close({min_cell_dist:.2f}<{min_dist})")
                elif max_dist > 0 and min_cell_dist > max_dist:
                    reasons.append(f"too_far({min_cell_dist:.2f}>{max_dist})")
                status = "SKIP:" + ",".join(reasons) if reasons else "OK"
                min_str = (
                    "inf" if min_cell_dist == float("inf") else f"{min_cell_dist:.2f}m"
                )
                lines.append(
                    f"  [{i:2d}] centroid=({cx:.2f},{cy:.2f})"
                    f"  size={len(cl):4d}"
                    f"  centroid_dist={centroid_dist:.2f}m"
                    f"  nearest_cell={min_str}"
                    f"  {status}"
                )
        lines.append(sep)
        print("\n".join(lines), flush=True)

    def costmap_radius_costs(
        self, costmap: OccupancyGrid | None, xy: XY, radius_cells: int = 4
    ) -> str:
        if costmap is None:
            return "n/a"
        info = costmap.info
        cx = int((xy[0] - info.origin.position.x) / info.resolution)
        cy = int((xy[1] - info.origin.position.y) / info.resolution)
        costs = []
        for dr in range(-radius_cells, radius_cells + 1):
            row = []
            for dc in range(-radius_cells, radius_cells + 1):
                col, r = cx + dc, cy + dr
                if 0 <= col < info.width and 0 <= r < info.height:
                    v = costmap.data[r * info.width + col]
                    if v == 254:
                        row.append("XXX")
                    elif v == 255:
                        row.append("???")
                    elif v < 0:
                        row.append("???")
                    elif dc == 0 and dr == 0:
                        row.append(f"[{v:3d}]")
                    else:
                        row.append(f"{v:4d}")
                else:
                    row.append("    ")
            costs.append(" ".join(row))
        return "\n      ".join(costs)

    def dump_failure_diagnostics(
        self, goal_xy: XY, robot_xy: XY | None, status: str, elapsed: float,
        nav2_error_code: int = 0, nav2_error_msg: str = "",
    ):
        sep = "=" * 60
        error_name = NAV2_ERROR_CODES.get(nav2_error_code, f"code={nav2_error_code}")
        lines = [
            sep,
            f"NAV FAILURE: goal #{self.goal_count}  status={status}  elapsed={elapsed}s",
            (
                f"  nav2 error: {error_name}  ({nav2_error_msg})"
                if nav2_error_msg else f"  nav2 error: {error_name}"
            ),
            f"  goal_xy  : ({goal_xy[0]:.3f}, {goal_xy[1]:.3f})",
        ]
        if robot_xy:
            dist = math.sqrt((goal_xy[0]-robot_xy[0])**2 + (goal_xy[1]-robot_xy[1])**2)
            lines.append(f"  robot_xy : ({robot_xy[0]:.3f}, {robot_xy[1]:.3f})  dist={dist:.2f}m")
        else:
            lines.append("  robot_xy : unavailable")

        costmaps = [
            ("global", self.latest_global_costmap),
            ("local", self.latest_local_costmap),
        ]
        for label, cm in costmaps:
            gc = self.costmap_cell_cost(cm, goal_xy)
            rc = self.costmap_cell_cost(cm, robot_xy) if robot_xy else None
            lines.append(
                f"  {label:6s} costmap @ goal={gc!s:>4}  @ robot={rc!s:>4}"
                f"  (lethal=254 inscribed=253 unknown=255)"
            )
            lines.append(f"  {label:6s} costmap 4-cell radius around GOAL (XXX=lethal ???=unknown):")
            lines.append(f"      {self.costmap_radius_costs(cm, goal_xy, 4)}")
            if robot_xy:
                lines.append(f"  {label:6s} costmap 4-cell radius around ROBOT:")
                lines.append(f"      {self.costmap_radius_costs(cm, robot_xy, 4)}")

        lines.append(f"  blacklist: {len(self.blacklist)} entries")
        if self.blacklist:
            entries = "  ".join(f"({x:.2f},{y:.2f})" for x, y in sorted(self.blacklist))
            lines.append(f"    {entries}")

        clusters = getattr(self.algorithm, "latest_clusters", [])
        lines.append(f"  frontiers: {len(clusters)} clusters available")
        info = self.latest_map_info
        for i, cl in enumerate(clusters[:10]):
            if info is not None and cl:
                xs = [cell_to_world(idx, info)[0] for idx in cl]
                ys = [cell_to_world(idx, info)[1] for idx in cl]
                cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
                lines.append(f"    [{i}] centroid=({cx:.2f},{cy:.2f}) size={len(cl)}")
            else:
                lines.append(f"    [{i}] size={len(cl)} (no map info)")
        if len(clusters) > 10:
            lines.append(f"    ... and {len(clusters)-10} more")

        lines.append(sep)
        resume_cmd = (
            "To resume: ros2 topic pub --once /intent "
            "std_msgs/msg/String "
            "'{data: \"{\\\"name\\\": \\\"exploration_resume\\\"}\"}'"
        )
        lines.append(resume_cmd)
        lines.append(sep)
        print("\n".join(lines), flush=True)
        self.paused_on_failure = True

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
        elif name == "exploration_resume":
            if self.paused_on_failure:
                self.paused_on_failure = False
                self.get_logger().info("Resumed by exploration_resume intent.")

    def explore_tick(self):
        self.publish_status(self.state)
        self.publish_markers()
        if self.state != "exploring":
            return
        if self.paused_on_failure:
            return
        if self.has_active_goal:
            # The frontier choice is only reconsidered when the current goal
            # finishes (reached, aborted, or timed out), not mid-flight.
            self.check_goal_timeout()
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
                self.dump_frontier_exhaustion(robot_xy)
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
            self.dump_failure_diagnostics(xy, self.robot_xy_in_map(), "rejected", 0.0)
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
            if result.status == GoalStatus.STATUS_ABORTED:
                self.dump_failure_diagnostics(
                    xy, robot_xy, status_name, elapsed,
                    nav2_error_code=result.result.error_code,
                    nav2_error_msg=result.result.error_msg,
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
