#!/usr/bin/env python3
# explorer_manager_node.py — Nav2 exploration session manager, pluggable algorithm
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import functools
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
)
from rclpy.wait_for_message import wait_for_message
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
    GoalOutcome,
    MapInfo,
    RenderContext,
)
from dome_nav.explore_telemetry import TelemetryWriter
from dome_nav.explore_diagnostics import (
    LETHAL_THRESHOLD,
    costmap_cell_cost,
    format_failure_diagnostics,
)
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.hello_world_algorithm import HelloWorldAlgorithm

XY = tuple[float, float]

# Registry of selectable exploration algorithms. The node instantiates the class
# named by the `explore_algorithm` ROS param; tests can still inject any
# ExplorationAlgorithm via the constructor.
DEFAULT_ALGORITHM = "frontier"
ALGORITHM_REGISTRY: dict[str, type[ExplorationAlgorithm]] = {
    "frontier": FrontierAlgorithm,
    "hello": HelloWorldAlgorithm,
}

GOAL_STATUS_NAMES = {5: "canceled", 6: "aborted"}


def dist(xy_a: XY, xy_b: XY) -> float:
    return math.sqrt((xy_a[0] - xy_b[0]) ** 2 + (xy_a[1] - xy_b[1]) ** 2)


def rounded(xy: XY | None) -> list[float] | None:
    """Telemetry wire format: 3-decimal coordinate list, None when pose unknown."""
    return [round(xy[0], 3), round(xy[1], 3)] if xy is not None else None


class ExplorerManagerNode(Node):
    # Timer frequency for the exploration loop.
    EXPLORE_HZ = 1.0

    # How many consecutive ticks the algorithm reports no usable goal (blocked)
    # before the node gives up and declares the session done.
    NO_TARGET_PATIENCE = 14

    # Cancel active goal after this many seconds to break Nav2 BT recovery loops.
    GOAL_TIMEOUT_S = 25.0

    # No-progress goal-abandon timeout, set above Nav2's progress_checker (10s) so
    # its BT recovery runs before the explorer cancels. Progress = dist dropped by
    # STUCK_PROGRESS_EPS or robot moved STUCK_MOVE_EPS.
    STUCK_T_S = 20.0
    STUCK_MOVE_EPS = 0.05
    STUCK_PROGRESS_EPS = 0.10

    # Max goal candidates per tick when a candidate maps outside the global costmap,
    # so a run of edge goals can't wedge the tick.
    MAX_GOAL_ATTEMPTS = 8

    # Same-pose (within STUCK_MOVE_EPS) stuck failures before declaring the robot
    # wedged and stopping; reselecting goals can't fix a wedged pose.
    WEDGED_STUCK_LIMIT = 3

    def __init__(self, algorithm: ExplorationAlgorithm | None = None):
        super().__init__("explore_manager_node")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, "/explore/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/explore/markers", 10)
        self.intent_sub = self.create_subscription(
            String, "/intent", self.on_intent, 10
        )
        # Grids fetched on demand (fetch_grid) while exploring, not held as standing
        # subs (idle deserialize burned 10-20% CPU on the Pi). Must match the latched
        # publishers' QoS for wait_for_message to return the last grid.
        self.map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        # TF listener runs only while exploring: /tf at ~40Hz costs ~8% CPU to
        # deserialize for a 1Hz pose. Started in exploration_start, torn down in stop.
        self.tf_buffer: tf2_ros.Buffer | None = None
        self.tf_listener: tf2_ros.TransformListener | None = None
        self.create_timer(1.0 / self.EXPLORE_HZ, self.explore_tick)

        # Shared params only; algorithm-specific tuning is declared by the
        # algorithm itself (declare_params below).
        self.declare_parameter("explore_algorithm", DEFAULT_ALGORITHM)
        self.declare_parameter("max_explore_radius", 0.0)
        self.declare_parameter("preferred_goal_distance", 1.0)
        self.declare_parameter("map_name", "unknown")
        self.map_name: str = self.get_parameter("map_name").value

        self.telemetry = TelemetryWriter(self.get_logger().info, map_name=self.map_name)

        self.params = ExploreParams(
            max_explore_radius=self.get_parameter("max_explore_radius").value,
            preferred_goal_distance=self.get_parameter("preferred_goal_distance").value,
        )
        if algorithm is not None:
            self.algorithm = algorithm
        else:
            chosen = self.get_parameter("explore_algorithm").value
            if chosen not in ALGORITHM_REGISTRY:
                self.get_logger().warning(
                    f"Unknown explore_algorithm '{chosen}'; "
                    f"falling back to '{DEFAULT_ALGORITHM}'."
                )
                chosen = DEFAULT_ALGORITHM
            self.algorithm = ALGORITHM_REGISTRY[chosen]()
        self.algorithm.declare_params(self)  # algorithm declares its own ROS params

        self.latest_map: OccupancyGrid | None = None
        self.latest_map_info: MapInfo | None = None
        self.latest_global_costmap: OccupancyGrid | None = None
        self.latest_local_costmap: OccupancyGrid | None = None
        self.paused_on_failure = False
        self.reset_session()
        self.clear_active_goal()
        self.get_logger().info("ExplorerManagerNode ready.")

    def fetch_grid(self, topic: str) -> OccupancyGrid | None:
        """On-demand latest grid from the latched publisher, or None if none arrives.

        Deserializes here only, never while idle. Briefly blocks the executor up to
        time_to_wait (~ms).
        """
        ok, msg = wait_for_message(
            OccupancyGrid, self, topic,
            qos_profile=self.map_qos, time_to_wait=1.0,
        )
        return msg if ok else None

    def goal_within_costmap_bounds(self, xy: XY) -> bool:
        """True if xy maps inside the global costmap (None costmap ⇒ True).

        Goals outside it fail worldToMap -> NO_VALID_PATH (the SLAM /map can extend
        past the costmap); None costmap returns True so startup isn't blocked.
        """
        if self.latest_global_costmap is None:
            return True
        return costmap_cell_cost(self.latest_global_costmap, xy) is not None

    def goal_is_lethal(self, xy: XY) -> bool:
        """True if xy sits on a lethal/inscribed costmap cell (Nav2 would reject it).

        None cost (no costmap / out of bounds) is not lethal; bounds are checked by
        goal_within_costmap_bounds.
        """
        cost = costmap_cell_cost(self.latest_global_costmap, xy)
        is_lethal = cost is not None and cost >= LETHAL_THRESHOLD
        if is_lethal:
            self.get_logger().warning(
                f"Goal ({xy[0]:.3f}, {xy[1]:.3f}) on lethal costmap cell "
                f"(cost={cost})."
            )
        return is_lethal

    def render_context(self, robot_xy: XY | None = None) -> RenderContext:
        return RenderContext(
            now=self.get_clock().now().to_msg(),
            is_exploring=self.state == "exploring",
            map_info=self.latest_map_info,
            robot_xy=robot_xy if robot_xy is not None else self.robot_xy_in_map(),
            blacklist=self.blacklist,
            goal_xy=self.current_goal_xy,
            params=self.params,
            patience=self.NO_TARGET_PATIENCE,
        )

    def call_hook(self, hook: str, *args, default=None):
        """Call optional hook: absent -> default, present -> called opaquely."""
        fn = getattr(self.algorithm, hook, None)
        return fn(*args) if fn is not None else default

    def dump_exhaustion(self, robot_xy: XY):
        report = self.call_hook("exhaustion_report", self.render_context(robot_xy))
        if report is not None:
            self.get_logger().info(report)

    def session_start_params(self) -> dict:
        """Node shared params merged with the algorithm's opaque session_params."""
        params: dict = {
            "timeout_s": self.GOAL_TIMEOUT_S,
            "max_radius": self.params.max_explore_radius,
            "preferred_goal_distance": self.params.preferred_goal_distance,
        }
        params.update(self.call_hook("session_params", default={}))
        return params

    def dump_failure_diagnostics(
        self, goal_xy: XY, robot_xy: XY | None, status: str, elapsed: float,
        nav2_error_code: int = 0, nav2_error_msg: str = "",
    ):
        self.latest_local_costmap = self.fetch_grid("/local_costmap/costmap")
        report = self.call_hook("failure_report", self.render_context(robot_xy))
        self.get_logger().warning(format_failure_diagnostics(
            goal_xy, robot_xy, status, elapsed, self.goal_count,
            self.latest_global_costmap, self.latest_local_costmap, self.blacklist,
            nav2_error_code, nav2_error_msg, algorithm_report=report,
        ))
        # Dump diagnostics but do not halt: while frontiers remain the tick loop
        # blacklists the failed goal and reselects; NO_TARGETS_BLOCKED patience is
        # the real stop. Flag/resume intent kept for manual pause paths.
        self.paused_on_failure = False

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
            self.start_tf()
            # start_xy stays None here: the fresh TF buffer is still empty, so
            # it is captured on the first tick where map->base_footprint resolves
            # (see explore_tick).
            self.state = "exploring"
            self.publish_status("exploring")
            radius_note = (
                f", max_radius={self.params.max_explore_radius}m"
                if self.params.max_explore_radius > 0 else ""
            )
            self.get_logger().info(f"Exploration started{radius_note}.")
            self.telemetry.write(
                "session_start", map_name=self.map_name, start_xy=None,
                params=self.session_start_params(),
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
        if self.start_xy is None:
            # Deferred from exploration_start: TF buffer was empty then.
            self.start_xy = self.robot_xy_in_map()
        if self.has_active_goal:
            # The next goal is only reconsidered when the current goal finishes
            # (reached, aborted, timed out, or abandoned for no progress).
            self.check_stuck()
            if self.has_active_goal:
                self.check_goal_timeout()
            return
        # Fetch grids on demand only when about to pick a goal — keeps the idle
        # node free of standing grid subscriptions (the CPU sink).
        self.latest_map = self.fetch_grid("/map")
        self.latest_global_costmap = self.fetch_grid("/global_costmap/costmap")
        self.find_and_send_goal()

    def check_stuck(self):
        # Abandon a no-progress goal before GOAL_TIMEOUT_S; blacklisting also
        # suppresses its blacklist_radius neighborhood so reselection avoids the wall.
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None or self.current_goal_xy is None:
            return
        goal_dist = dist(robot_xy, self.current_goal_xy)
        moved = (
            dist(robot_xy, self.last_progress_xy)
            if self.last_progress_xy is not None else 0.0
        )
        if (self.best_dist_to_goal is None
                or goal_dist < self.best_dist_to_goal - self.STUCK_PROGRESS_EPS
                or moved > self.STUCK_MOVE_EPS):
            self.best_dist_to_goal = (
                goal_dist if self.best_dist_to_goal is None
                else min(self.best_dist_to_goal, goal_dist)
            )
            self.last_progress_xy = robot_xy
            self.last_progress_time = time.monotonic()
            return
        if (time.monotonic() - self.last_progress_time) <= self.STUCK_T_S:
            return
        elapsed = round(time.monotonic() - self.goal_start_time, 1)
        self.get_logger().warning(
            f"No progress for {self.STUCK_T_S}s — abandoning goal, blacklisting."
        )
        self.goals_failed += 1
        self.write_goal_result(self.current_goal_xy, robot_xy, "stuck", elapsed)
        self.abandon_active_goal()
        self.note_stuck(robot_xy)

    def note_stuck(self, robot_xy: XY):
        # Wedge detector: stuck failures from the same pose mean the robot can't
        # move, so stop cleanly. Streak resets on a new stuck pose or a reached goal.
        same_pose = (
            self.stuck_streak_xy is not None
            and dist(robot_xy, self.stuck_streak_xy) <= self.STUCK_MOVE_EPS
        )
        self.stuck_streak = self.stuck_streak + 1 if same_pose else 1
        self.stuck_streak_xy = robot_xy
        if self.stuck_streak < self.WEDGED_STUCK_LIMIT:
            return
        self.get_logger().error(
            f"Robot wedged: {self.stuck_streak} consecutive stuck goals from "
            f"({robot_xy[0]:.2f}, {robot_xy[1]:.2f}) — stopping exploration."
        )
        self.telemetry.write(
            "wedged", robot_xy=rounded(robot_xy), stuck_streak=self.stuck_streak,
            goals_sent=self.goal_count,
        )
        self.dump_exhaustion(robot_xy)
        self.stop_exploring("idle")

    def check_goal_timeout(self):
        # Caller guarantees an active goal (goal_start_time seeded in send_nav_goal).
        if (time.monotonic() - self.goal_start_time) <= self.GOAL_TIMEOUT_S:
            return
        elapsed = round(time.monotonic() - self.goal_start_time, 1)
        self.get_logger().warning(
            f"Goal timed out after {elapsed}s — cancelling and blacklisting."
        )
        self.goals_failed += 1
        self.write_goal_result(self.current_goal_xy, None, "timeout", elapsed)
        self.abandon_active_goal()

    def find_and_send_goal(self):
        if self.latest_map is None:
            self.telemetry.write("no_frontier", reason="no_map")
            return
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            self.get_logger().warning("TF map→base_footprint unavailable — waiting.")
            self.telemetry.write("no_frontier", reason="no_tf")
            return
        grid = self.latest_map
        info = MapInfo(
            width=grid.info.width, height=grid.info.height,
            resolution=grid.info.resolution,
            origin_x=grid.info.origin.position.x,
            origin_y=grid.info.origin.position.y,
        )
        self.latest_map_info = info
        # grid.data (array.array) is passed uncopied: every consumer is read-only
        # indexing/iteration, and a full-map list() copy per tick is pure waste.
        map_data = grid.data
        # Ask for a goal, re-asking past candidates the planner would reject (outside
        # costmap or lethal/inscribed). Checks run on the post-nudge candidate;
        # rejected is per-tick so a grown/cleared costmap re-evaluates fresh.
        rejected: set[XY] = set()
        goal_xy = None
        decision = None
        for _ in range(self.MAX_GOAL_ATTEMPTS):
            ctx = ExplorationContext(
                map_data=map_data,
                map_info=info,
                robot_xy=robot_xy,
                blacklist=self.blacklist | rejected,
                start_xy=self.start_xy,
                params=self.params,
            )
            decision = self.algorithm.next_goal(ctx)
            if decision.outcome is not GoalOutcome.NEW_GOAL:
                break
            candidate = decision.xy
            if not self.goal_within_costmap_bounds(candidate):
                self.get_logger().warning(
                    f"Goal candidate ({candidate[0]:.3f}, {candidate[1]:.3f}) is "
                    "outside the global costmap — skipping to next candidate."
                )
                rejected.add(candidate)
                continue
            if self.goal_is_lethal(candidate):
                rejected.add(candidate)
                continue
            goal_xy = candidate
            break
        if goal_xy is not None:
            self.no_target_count = 0
            self.send_nav_goal(goal_xy)
            return
        # No goal: EXPLORED_DONE ends the session; anything else debounces.
        if decision is not None and decision.outcome is GoalOutcome.EXPLORED_DONE:
            self.get_logger().info("Algorithm reports exploration complete.")
            self.dump_exhaustion(robot_xy)
            self.stop_exploring("done")
            return
        self.handle_no_target(robot_xy)

    def handle_no_target(self, robot_xy: XY):
        self.no_target_count += 1
        self.get_logger().info(
            f"No usable goal this tick "
            f"(tick {self.no_target_count}/{self.NO_TARGET_PATIENCE})."
        )
        # "no_frontier" kept as a telemetry wire contract; rename is a migration.
        self.telemetry.write(
            "no_frontier", reason="filtered",
            tick=self.no_target_count,
            patience=self.NO_TARGET_PATIENCE,
            blacklisted=len(self.blacklist),
            **self.call_hook("telemetry_extra", default={}),
        )
        if self.no_target_count >= self.NO_TARGET_PATIENCE:
            # Blocked, not done (done comes via EXPLORED_DONE). Clear the blacklist
            # once in case the growing map reopened stale entries; then give up.
            if not self.blacklist_cleared_once:
                self.get_logger().info(
                    "Blocked: targets exist but all filtered — "
                    "clearing blacklist once and retrying."
                )
                self.blacklist.clear()
                self.blacklist_cleared_once = True
                self.no_target_count = 0
                return
            self.get_logger().info("No-target patience exhausted — exploration done.")
            self.dump_exhaustion(robot_xy)
            self.stop_exploring("done")

    def reset_session(self):
        self.state = "idle"
        self.blacklist: set[XY] = set()
        self.start_xy: XY | None = None
        self.no_target_count = 0
        self.blacklist_cleared_once = False
        self.goal_count = 0
        self.goals_reached = 0
        self.goals_failed = 0
        self.stuck_streak = 0
        self.stuck_streak_xy: XY | None = None

    def clear_active_goal(self):
        self.goal_handle = None
        self.has_active_goal = False
        self.goal_start_time = None
        self.current_goal_xy = None
        # No-progress tracking (see check_stuck); set fresh in send_nav_goal.
        self.best_dist_to_goal = None
        self.last_progress_xy = None
        self.last_progress_time = None

    def abandon_active_goal(self):
        """Cancel + blacklist the active goal; shared by the stuck/timeout paths."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.current_goal_xy is not None:
            self.blacklist.add(self.current_goal_xy)
        self.clear_active_goal()

    def write_goal_result(self, xy: XY | None, robot_xy: XY | None,
                          status: str, elapsed: float):
        self.telemetry.write(
            "goal_result", goal_num=self.goal_count, goal_xy=rounded(xy),
            status=status, elapsed_s=elapsed, robot_xy=rounded(robot_xy),
            blacklisted=len(self.blacklist),
        )

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
        goal_dist = dist(xy, robot_xy) if robot_xy is not None else -1.0
        # Seed no-progress tracking for check_stuck.
        self.best_dist_to_goal = goal_dist if goal_dist >= 0.0 else None
        self.last_progress_xy = robot_xy
        self.last_progress_time = time.monotonic()
        self.get_logger().info(
            f"Goal #{self.goal_count}: ({xy[0]:.2f},{xy[1]:.2f})"
            f" dist={goal_dist:.2f}m blacklisted={len(self.blacklist)}"
        )
        # telemetry_extra rides along so per-goal algorithm state (e.g.
        # novelty_score) is visible on the goals themselves, not only on
        # no_frontier ticks.
        self.telemetry.write(
            "goal_sent", goal_num=self.goal_count, goal_xy=rounded(xy),
            dist_m=round(goal_dist, 3), robot_xy=rounded(robot_xy),
            blacklisted=len(self.blacklist),
            **self.call_hook("telemetry_extra", default={}),
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
            # Full clear, not just has_active_goal: a dangling current_goal_xy /
            # goal_start_time showed the dead goal in status and markers until
            # the next tick overwrote it.
            self.clear_active_goal()
            self.dump_failure_diagnostics(xy, self.robot_xy_in_map(), "rejected", 0.0)
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self.on_goal_result, xy=xy)
        )

    def on_goal_result(self, future, xy: XY):
        # Ignore stale callbacks. A goal canceled by the stuck/timeout watchdog or by
        # exploration_stop has already cleared its state; its late result must not run
        # against None goal_start_time or against a superseding goal's state.
        if (not self.has_active_goal or xy != self.current_goal_xy
                or self.goal_start_time is None):
            return
        elapsed = round(time.monotonic() - self.goal_start_time, 1)
        self.clear_active_goal()
        result = future.result()
        robot_xy = self.robot_xy_in_map()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            self.stuck_streak = 0
            self.stuck_streak_xy = None
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
            # Blacklist failures only: blacklisting reached goals killed live
            # frontier cells around each success and ended sessions early.
            self.blacklist.add(xy)
        self.write_goal_result(xy, robot_xy, status_name, elapsed)

    def stop_exploring(self, new_state: str):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.has_active_goal = False
        self.state = new_state
        self.stop_tf()
        self.publish_status(new_state)
        self.get_logger().info(f"Exploration stopped → {new_state}.")
        self.telemetry.write(
            "session_end", reason=new_state, goals_sent=self.goal_count,
            reached=self.goals_reached, failed=self.goals_failed,
        )

    def start_tf(self):
        # Create the TF listener on demand (see __init__ note). Buffer needs a
        # moment to fill; first lookups may return None and retry next tick.
        if self.tf_listener is not None:
            return
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def stop_tf(self):
        # Tear down the listener so an idle node deserializes no /tf. Destroy the
        # subscriptions it registered on this node (attr names guarded for distro
        # differences), then drop the buffer.
        if self.tf_listener is None:
            return
        for attr in ("tf_sub", "tf_static_sub"):
            sub = getattr(self.tf_listener, attr, None)
            if sub is not None:
                self.destroy_subscription(sub)
        self.tf_listener = None
        self.tf_buffer = None

    def robot_xy_in_map(self) -> XY | None:
        if self.tf_buffer is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            trans = tf.transform.translation
            return (trans.x, trans.y)
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ):
            return None

    def publish_markers(self):
        """Optional opaque hook: publish the algorithm's MarkerArray verbatim."""
        markers = self.call_hook("render_markers", self.render_context())
        if markers is not None:
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
            # "no_frontier_ticks" kept as a status wire contract; rename is a migration.
            data["no_frontier_ticks"] = self.no_target_count
            has_goal = self.current_goal_xy is not None and robot_xy is not None
            if has_goal:
                gx, gy = self.current_goal_xy
                data["goal_xy"] = [round(gx, 2), round(gy, 2)]
                data["dist_m"] = round(dist(self.current_goal_xy, robot_xy), 2)
                data["elapsed_s"] = (
                    round(time.monotonic() - self.goal_start_time, 1)
                    if self.goal_start_time else None
                )
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ExplorerManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop_exploring owns session_end for normally-ended sessions; only an
        # interrupted active session needs the shutdown record here.
        if node.state == "exploring":
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
