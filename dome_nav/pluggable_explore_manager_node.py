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
from dome_nav.explore_diagnostics import (
    costmap_cell_cost,
    format_failure_diagnostics,
    format_frontier_exhaustion,
)
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

    # Max frontiers to try in one tick when a candidate goal maps outside the
    # global costmap. Each rejected goal is excluded and next_goal is re-asked, so
    # a run of edge goals near the growing map boundary can't wedge the tick.
    MAX_GOAL_ATTEMPTS = 8

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
        self.declare_parameter("preferred_goal_distance", 1.0)
        self.declare_parameter("prefer_farthest", False)  # deprecated
        self.declare_parameter("min_frontier_size", 10)
        self.declare_parameter("frontier_buffer_cells", 2)
        self.declare_parameter("map_name", "unknown")
        self.max_explore_radius: float = self.get_parameter("max_explore_radius").value
        self.max_frontier_dist: float = self.get_parameter("max_frontier_dist").value
        self.min_frontier_dist: float = self.get_parameter("min_frontier_dist").value
        self.min_frontier_size: int = self.get_parameter("min_frontier_size").value
        self.frontier_buffer_cells: int = self.get_parameter(
            "frontier_buffer_cells"
        ).value
        self.map_name: str = self.get_parameter("map_name").value

        preferred_goal_distance: float = self.get_parameter("preferred_goal_distance").value
        prefer_farthest_val: bool = self.get_parameter("prefer_farthest").value
        if prefer_farthest_val:
            effective_max = self.max_frontier_dist if self.max_frontier_dist > 0.0 else 1000.0
            preferred_goal_distance = effective_max
            self.get_logger().warning(
                "prefer_farthest is deprecated; use preferred_goal_distance instead. "
                f"Mapping prefer_farthest=True to preferred_goal_distance={effective_max}"
            )
        self.preferred_goal_distance: float = preferred_goal_distance

        self.telemetry = TelemetryWriter(self.get_logger().info, map_name=self.map_name)

        self.params = ExploreParams(
            max_explore_radius=self.max_explore_radius,
            max_frontier_dist=self.max_frontier_dist,
            min_frontier_dist=self.min_frontier_dist,
            preferred_goal_distance=self.preferred_goal_distance,
            min_frontier_size=self.min_frontier_size,
            frontier_buffer_cells=self.frontier_buffer_cells,
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

    def goal_in_global_costmap(self, xy: XY) -> bool:
        # True if xy maps inside the current global costmap extent. When no
        # global costmap has been received yet, returns True so startup is not
        # blocked. Guards against dispatching frontier goals outside the costmap,
        # which the planner rejects with a worldToMap failure -> PLAN/NO_VALID_PATH
        # (the SLAM /map the frontier detector reads can extend past the costmap).
        if self.latest_global_costmap is None:
            return True
        return costmap_cell_cost(self.latest_global_costmap, xy) is not None

    def dump_frontier_exhaustion(self, robot_xy: XY):
        self.get_logger().info(format_frontier_exhaustion(
            self.algorithm.latest_clusters, self.latest_map_info, robot_xy,
            self.params, self.blacklist, self.NO_FRONTIER_PATIENCE,
        ))

    def dump_failure_diagnostics(
        self, goal_xy: XY, robot_xy: XY | None, status: str, elapsed: float,
        nav2_error_code: int = 0, nav2_error_msg: str = "",
    ):
        self.get_logger().warning(format_failure_diagnostics(
            goal_xy, robot_xy, status, elapsed, self.goal_count,
            self.latest_global_costmap, self.latest_local_costmap, self.blacklist,
            self.algorithm.latest_clusters, self.latest_map_info,
            nav2_error_code, nav2_error_msg,
        ))
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
                    "preferred_goal_distance": self.params.preferred_goal_distance,
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
        if self.current_goal_xy is not None:
            self.blacklist.add(self.current_goal_xy)
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
        map_data = list(m.data)
        # Ask the algorithm for a goal; if the candidate maps outside the global
        # costmap the planner would reject it (worldToMap failure), so exclude it
        # and re-ask for the next-best frontier. rejected is local to this tick —
        # next tick re-evaluates fresh in case the costmap has since grown.
        rejected: set[XY] = set()
        goal_xy = None
        for _ in range(self.MAX_GOAL_ATTEMPTS):
            ctx = ExplorationContext(
                map_data=map_data,
                map_info=info,
                robot_xy=robot_xy,
                blacklist=self.blacklist | rejected,
                start_xy=self.start_xy,
                params=self.params,
            )
            candidate = self.algorithm.next_goal(ctx)
            if candidate is None:
                break
            if self.goal_in_global_costmap(candidate):
                goal_xy = candidate
                break
            self.get_logger().warning(
                f"Frontier goal ({candidate[0]:.3f}, {candidate[1]:.3f}) is "
                "outside the global costmap — skipping to next frontier."
            )
            rejected.add(candidate)
        if goal_xy is None:
            self.handle_no_frontier(robot_xy)
            return
        self.no_frontier_count = 0
        self.send_nav_goal(goal_xy)

    def handle_no_frontier(self, robot_xy: XY):
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
            self.get_logger().info("Frontier patience exhausted — exploration done.")
            self.dump_frontier_exhaustion(robot_xy)
            self.stop_exploring("done")

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
        self.current_goal_xy = None

    def send_nav_goal(self, xy: XY):
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
            dist_m=round(dist, 3),
            robot_xy=[round(v, 3) for v in robot_xy] if robot_xy else None,
            blacklisted=len(self.blacklist),
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            functools.partial(self.on_goal_accepted, xy=xy)
        )

    def on_goal_accepted(self, future, xy: XY):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning(
                f"Goal rejected at ({xy[0]:.2f}, {xy[1]:.2f}) — blacklisting."
            )
            self.blacklist.add(xy)
            self.has_active_goal = False
            self.dump_failure_diagnostics(xy, self.robot_xy_in_map(), "rejected", 0.0)
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self.on_goal_result, xy=xy)
        )

    def on_goal_result(self, future, xy: XY):
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
        self.blacklist.add(xy)

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
