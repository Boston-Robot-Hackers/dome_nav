#!/usr/bin/env python3
# explore_diagnostics.py — pure formatters for exploration failure/exhaustion dumps
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
from nav_msgs.msg import OccupancyGrid

from dome_nav.explore_context import MapInfo
from dome_nav.frontier_params import FrontierTuning
from dome_nav.frontier_explorer import cell_to_world

XY = tuple[float, float]
SEP = "=" * 60

# /global_costmap/costmap and /local_costmap/costmap are nav_msgs/OccupancyGrid:
# Nav2 scales the raw 0-255 costmap onto this topic (lethal 254->100,
# inscribed 253->99, unknown 255->-1). All cost reads here are on that scaled grid.
LETHAL_COST = 100
INSCRIBED_COST = 99
# A goal on a lethal or inscribed cell puts the footprint in guaranteed collision,
# so the planner rejects it — treat both as "do not send".
LETHAL_THRESHOLD = INSCRIBED_COST

# ComputePathToPose error codes (200 range), FollowPath error codes (100 range).
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


def cluster_centroid(cluster: list[int], info: MapInfo) -> XY:
    xs = [cell_to_world(idx, info)[0] for idx in cluster]
    ys = [cell_to_world(idx, info)[1] for idx in cluster]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def costmap_cell_cost(costmap: OccupancyGrid | None, xy: XY) -> int | None:
    if costmap is None:
        return None
    info = costmap.info
    col = int((xy[0] - info.origin.position.x) / info.resolution)
    row = int((xy[1] - info.origin.position.y) / info.resolution)
    if col < 0 or col >= info.width or row < 0 or row >= info.height:
        return None
    return costmap.data[row * info.width + col]


def costmap_radius_costs(
    costmap: OccupancyGrid | None, xy: XY, radius_cells: int = 4
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
            if not (0 <= col < info.width and 0 <= r < info.height):
                row.append("    ")
                continue
            v = costmap.data[r * info.width + col]
            if v >= LETHAL_COST:
                row.append("XXX")
            elif v < 0:
                row.append("???")
            elif dc == 0 and dr == 0:
                row.append(f"[{v:3d}]")
            else:
                row.append(f"{v:4d}")
        costs.append(" ".join(row))
    return "\n      ".join(costs)


def format_frontier_exhaustion(
    clusters: list[list[int]],
    info: MapInfo,
    robot_xy: XY,
    params: FrontierTuning,
    blacklist: set[XY],
    patience: int,
) -> str:
    lines = [
        SEP,
        f"FRONTIER EXHAUSTION — {len(clusters)} raw clusters, patience={patience}",
        f"  filters: min_size={params.min_frontier_size}"
        f"  min_dist={params.min_frontier_dist}m"
        f"  max_dist={params.max_frontier_dist}m"
        f"  blacklisted={len(blacklist)}",
        f"  robot_xy: {robot_xy}",
        "",
    ]
    if not clusters:
        lines.append("  (no clusters)")
    else:
        for i, cl in enumerate(clusters):
            lines.append(exhaustion_cluster_line(i, cl, info, robot_xy, params, blacklist))
    lines.append(SEP)
    return "\n".join(lines)


def exhaustion_cluster_line(
    i: int, cl: list[int], info: MapInfo, robot_xy: XY,
    params: FrontierTuning, blacklist: set[XY],
) -> str:
    br = params.blacklist_radius
    cx, cy = cluster_centroid(cl, info)
    centroid_dist = math.dist((cx, cy), robot_xy)
    cell_dists = [
        math.dist(cell_to_world(idx, info), robot_xy)
        for idx in cl
        if not any(math.dist(cell_to_world(idx, info), bl) < br for bl in blacklist)
    ]
    min_cell_dist = min(cell_dists, default=float("inf"))
    min_str = "inf" if min_cell_dist == float("inf") else f"{min_cell_dist:.2f}m"
    status = exhaustion_reason(len(cl), min_cell_dist, params)
    return (
        f"  [{i:2d}] centroid=({cx:.2f},{cy:.2f})  size={len(cl):4d}"
        f"  centroid_dist={centroid_dist:.2f}m  nearest_cell={min_str}  {status}"
    )


def exhaustion_reason(size: int, min_cell_dist: float, params: FrontierTuning) -> str:
    reasons = []
    if size < params.min_frontier_size:
        reasons.append(f"too_small({size}<{params.min_frontier_size})")
    if min_cell_dist == float("inf"):
        reasons.append("all_blacklisted")
    elif params.min_frontier_dist > 0 and min_cell_dist < params.min_frontier_dist:
        reasons.append(f"too_close({min_cell_dist:.2f}<{params.min_frontier_dist})")
    elif params.max_frontier_dist > 0 and min_cell_dist > params.max_frontier_dist:
        reasons.append(f"too_far({min_cell_dist:.2f}>{params.max_frontier_dist})")
    return "SKIP:" + ",".join(reasons) if reasons else "OK"


def format_failure_diagnostics(
    goal_xy: XY, robot_xy: XY | None, status: str, elapsed: float, goal_count: int,
    global_costmap: OccupancyGrid | None, local_costmap: OccupancyGrid | None,
    blacklist: set[XY], nav2_error_code: int = 0, nav2_error_msg: str = "",
    algorithm_report: str | None = None,
) -> str:
    # Node-general failure dump. Anything algorithm-specific (e.g. frontier
    # clusters) arrives pre-rendered in algorithm_report and is appended blindly.
    error_name = NAV2_ERROR_CODES.get(nav2_error_code, f"code={nav2_error_code}")
    lines = [
        SEP,
        f"NAV FAILURE: goal #{goal_count}  status={status}  elapsed={elapsed}s",
        (
            f"  nav2 error: {error_name}  ({nav2_error_msg})"
            if nav2_error_msg else f"  nav2 error: {error_name}"
        ),
        f"  goal_xy  : ({goal_xy[0]:.3f}, {goal_xy[1]:.3f})",
    ]
    if robot_xy:
        dist = math.sqrt((goal_xy[0] - robot_xy[0]) ** 2 + (goal_xy[1] - robot_xy[1]) ** 2)
        lines.append(f"  robot_xy : ({robot_xy[0]:.3f}, {robot_xy[1]:.3f})  dist={dist:.2f}m")
    else:
        lines.append("  robot_xy : unavailable")

    for label, cm in (("global", global_costmap), ("local", local_costmap)):
        lines.extend(failure_costmap_lines(label, cm, goal_xy, robot_xy))

    lines.append(f"  blacklist: {len(blacklist)} entries")
    if blacklist:
        entries = "  ".join(f"({x:.2f},{y:.2f})" for x, y in sorted(blacklist))
        lines.append(f"    {entries}")

    if algorithm_report:
        lines.append(algorithm_report)

    lines.append(SEP)
    lines.append(
        "To resume: ros2 topic pub --once /intent std_msgs/msg/String "
        "'{data: \"{\\\"name\\\": \\\"exploration_resume\\\"}\"}'"
    )
    lines.append(SEP)
    return "\n".join(lines)


def format_cluster_summary(
    clusters: list[list[int]], info: MapInfo | None
) -> str:
    # Frontier-owned block for the failure dump: the available raw clusters.
    lines = [f"  frontiers: {len(clusters)} clusters available"]
    for i, cl in enumerate(clusters[:10]):
        if info is not None and cl:
            cx, cy = cluster_centroid(cl, info)
            lines.append(f"    [{i}] centroid=({cx:.2f},{cy:.2f}) size={len(cl)}")
        else:
            lines.append(f"    [{i}] size={len(cl)} (no map info)")
    if len(clusters) > 10:
        lines.append(f"    ... and {len(clusters) - 10} more")
    return "\n".join(lines)


def failure_costmap_lines(
    label: str, cm: OccupancyGrid | None, goal_xy: XY, robot_xy: XY | None
) -> list[str]:
    gc = costmap_cell_cost(cm, goal_xy)
    rc = costmap_cell_cost(cm, robot_xy) if robot_xy else None
    lines = [
        f"  {label:6s} costmap @ goal={gc!s:>4}  @ robot={rc!s:>4}"
        f"  (lethal={LETHAL_COST} inscribed={INSCRIBED_COST} unknown=-1)",
        f"  {label:6s} costmap 4-cell radius around GOAL (XXX=lethal ???=unknown):",
        f"      {costmap_radius_costs(cm, goal_xy, 4)}",
    ]
    if robot_xy:
        lines.append(f"  {label:6s} costmap 4-cell radius around ROBOT:")
        lines.append(f"      {costmap_radius_costs(cm, robot_xy, 4)}")
    return lines
