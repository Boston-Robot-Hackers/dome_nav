#!/usr/bin/env python3
# algo_demo.py — interactive CLI visualization of FrontierAlgorithm on small maps
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
#
# Usage:
#   python3 tools/algo_demo.py [--map room|corridor|ring|maze] [--inset 0.3]
#                              [--min-size 1] [--min-dist 0.0] [--auto]
#
# Map legend:
#   .  free      #  occupied    ?  unknown
#   A-Z frontier cluster (each letter = distinct cluster, color-coded)
#   R  robot     G  goal (nudged)    B  blacklisted

import argparse
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dome_nav.explore_context import ExplorationContext, ExploreParams
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.frontier_explorer import MapInfo, cell_to_world, pick_best_frontier

# ---------------------------------------------------------------------------
# ANSI color helpers
# ---------------------------------------------------------------------------

RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"


def fg256(n: int) -> str:
    return f"\033[38;5;{n}m"


def colored(text: str, code: str) -> str:
    return f"{code}{text}{RESET}"


C_UNKNOWN = DIM + "\033[37m"       # dim white
C_FREE = "\033[90m"                 # dark gray
C_WALL = BOLD + "\033[37m"          # bright white bold
C_ROBOT = BOLD + "\033[96m"         # bright cyan bold
C_GOAL = BOLD + "\033[93m"          # bright yellow bold
C_BLACKLIST = "\033[91m"            # red
C_TARGET = BOLD + fg256(213)        # bright pink — raw pick_best_frontier cell

# Distinct 256-color palette for up to 8 clusters; cycles if more
CLUSTER_COLORS = [
    fg256(226),   # bright yellow
    fg256(51),    # bright cyan
    fg256(213),   # pink
    fg256(118),   # bright green
    fg256(208),   # orange
    fg256(171),   # purple
    fg256(123),   # sky blue
    fg256(154),   # yellow-green
]

CLUSTER_LABELS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"

# ---------------------------------------------------------------------------
# Built-in maps  (rows top→bottom, cols left→right; ?=unknown, 0=free, #=wall, R=robot start)
# ---------------------------------------------------------------------------

MAPS: dict[str, list[str]] = {
    "room": [
        "???????????",
        "?000000000?",
        "?000000000?",
        "?000000000?",
        "?000R00000?",
        "?000000000?",
        "?000000000?",
        "???????????",
    ],
    "corridor": [
        "???????????",
        "?000000000?",
        "###########",
        "?000R00000?",
        "###########",
        "?000000000?",
        "???????????",
    ],
    "ring": [
        "?????????????",
        "?00000000000?",
        "?00???????00?",
        "?00?00000?00?",
        "?00?0R000?00?",
        "?00?00000?00?",
        "?00???????00?",
        "?00000000000?",
        "?????????????",
    ],
    "maze": [
        "?????????????",
        "?0000?000000?",
        "?0##0?0####0?",
        "?0#R0000000??",
        "?0##0?0####0?",
        "?0000?000000?",
        "?############",
        "?000000000000",
        "?????????????",
    ],
    "large": [
        # 30x30 — three rooms (top-left, top-right, bottom) joined by corridors.
        # Vertical wall at col 14, rows 1-20; door at rows 10-11.
        # Horizontal wall at row 14; doors at cols 6-7 (left) and cols 20-21 (right).
        "??????????????????????????????" ,
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?000000R000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?#####00############00#######?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000#00000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "?0000000000000000000000000000?",
        "??????????????????????????????",
    ],
}

CELL_FREE = 0
CELL_OCC = 100
CELL_UNK = -1
PATIENCE = 6


def parse_map(rows: list[str]) -> tuple[list[int], MapInfo]:
    height = len(rows)
    width = max(len(r) for r in rows)
    data: list[int] = []
    for row in rows:
        for ch in row.ljust(width):
            if ch in ("0", "R"):
                data.append(CELL_FREE)
            elif ch == "#":
                data.append(CELL_OCC)
            else:
                data.append(CELL_UNK)
    info = MapInfo(width=width, height=height, resolution=1.0,
                   origin_x=0.0, origin_y=0.0)
    return data, info


def find_robot_start(rows: list[str]) -> tuple[float, float]:
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            if ch == "R":
                return (c + 0.5, r + 0.5)
    return (len(rows[0]) / 2.0, len(rows) / 2.0)


def world_to_cell(xy: tuple[float, float], info: MapInfo) -> tuple[int, int]:
    col = int((xy[0] - info.origin_x) / info.resolution)
    row = int((xy[1] - info.origin_y) / info.resolution)
    return (row, col)


def build_cluster_index(
    clusters: list[list[int]], min_frontier_size: int
) -> dict[int, int]:
    # Returns {cell_idx: cluster_number} for large clusters only.
    result: dict[int, int] = {}
    cluster_num = 0
    for cluster in clusters:
        if len(cluster) >= min_frontier_size:
            for idx in cluster:
                result[idx] = cluster_num
            cluster_num += 1
    return result


def render(
    data: list[int],
    info: MapInfo,
    robot_xy: tuple[float, float],
    goal_xy: tuple[float, float] | None,
    target_xy: tuple[float, float] | None,
    blacklist: set[tuple[float, float]],
    clusters: list[list[int]],
    min_frontier_size: int,
) -> str:
    cluster_index = build_cluster_index(clusters, min_frontier_size)
    bl_cells = {world_to_cell(bxy, info) for bxy in blacklist}
    robot_rc = world_to_cell(robot_xy, info)
    goal_rc = world_to_cell(goal_xy, info) if goal_xy else None
    target_rc = world_to_cell(target_xy, info) if target_xy else None

    lines = []
    for r in range(info.height):
        row_parts = []
        for c in range(info.width):
            idx = r * info.width + c
            rc = (r, c)
            if rc == robot_rc:
                row_parts.append(colored("R", C_ROBOT))
            elif rc == target_rc and rc == goal_rc:
                row_parts.append(colored("T", C_TARGET))   # same cell: T wins
            elif rc == target_rc:
                row_parts.append(colored("T", C_TARGET))
            elif rc == goal_rc:
                row_parts.append(colored("G", C_GOAL))
            elif rc in bl_cells:
                row_parts.append(colored("B", C_BLACKLIST))
            elif idx in cluster_index:
                cn = cluster_index[idx]
                color = CLUSTER_COLORS[cn % len(CLUSTER_COLORS)]
                label = CLUSTER_LABELS[cn % len(CLUSTER_LABELS)]
                row_parts.append(colored(label, BOLD + color))
            elif data[idx] == CELL_FREE:
                row_parts.append(colored(".", C_FREE))
            elif data[idx] == CELL_OCC:
                row_parts.append(colored("#", C_WALL))
            else:
                row_parts.append(colored("?", C_UNKNOWN))
        lines.append(" ".join(row_parts))
    return "\n".join(lines)


def cluster_legend(clusters: list[list[int]], min_frontier_size: int) -> str:
    large = [c for c in clusters if len(c) >= min_frontier_size]
    if not large:
        return ""
    parts = []
    for i, cluster in enumerate(large):
        color = CLUSTER_COLORS[i % len(CLUSTER_COLORS)]
        label = CLUSTER_LABELS[i % len(CLUSTER_LABELS)]
        parts.append(f"{BOLD}{color}{label}{RESET}={len(cluster)}cells")
    return "  clusters: " + "  ".join(parts)


def uncover_around_robot(
    data: list[int], info: MapInfo, robot_xy: tuple[float, float], radius: float
) -> list[int]:
    data = list(data)
    rx, ry = robot_xy
    for idx in range(info.width * info.height):
        if data[idx] != CELL_UNK:
            continue
        wx, wy = cell_to_world(idx, info)
        if math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2) <= radius:
            data[idx] = CELL_FREE
    return data


def main():
    parser = argparse.ArgumentParser(description="FrontierAlgorithm interactive demo")
    parser.add_argument("--map", choices=list(MAPS.keys()), default="room",
                        help="Map to explore (default: room)")
    parser.add_argument("--inset", type=float, default=0.3,
                        help="goal_inset_m (default: 0.3)")
    parser.add_argument("--min-size", type=int, default=1,
                        help="min_frontier_size (default: 1)")
    parser.add_argument("--min-dist", type=float, default=0.0,
                        help="min_frontier_dist (default: 0.0)")
    parser.add_argument("--sensor-radius", type=float, default=2.5,
                        help="Lidar reveal radius in cells (default: 2.5)")
    parser.add_argument("--auto", action="store_true",
                        help="Run without pausing between steps")
    args = parser.parse_args()

    rows = MAPS[args.map]
    data, info = parse_map(rows)
    robot_xy = find_robot_start(rows)
    data = uncover_around_robot(data, info, robot_xy, args.sensor_radius)

    params = ExploreParams(
        min_frontier_size=args.min_size,
        min_frontier_dist=args.min_dist,
        goal_inset_m=args.inset,
    )
    algo = FrontierAlgorithm()
    blacklist: set[tuple[float, float]] = set()
    no_frontier_count = 0
    step = 0

    compact_legend = (
        f"{colored('R', C_ROBOT)}=robot "
        f"{colored('T', C_TARGET)}=target(nearest frontier cell) "
        f"{colored('G', C_GOAL)}=goal(T nudged {args.inset}m toward R) "
        f"{colored('A', BOLD + CLUSTER_COLORS[0])}=cluster "
        f"{colored('B', C_BLACKLIST)}=blacklisted "
        f"{colored('.', C_FREE)}=free "
        f"{colored('#', C_WALL)}=wall "
        f"{colored('?', C_UNKNOWN)}=unknown"
    )

    print(f"\nMap: {args.map}  size: {info.width}x{info.height}  "
          f"min_size={params.min_frontier_size}  "
          f"min_dist={params.min_frontier_dist}  inset={params.goal_inset_m}\n")

    while True:
        ctx = ExplorationContext(
            map_data=data,
            map_info=info,
            robot_xy=robot_xy,
            blacklist=blacklist,
            start_xy=None,
            params=params,
        )
        goal_xy = algo.next_goal(ctx)
        target_xy = pick_best_frontier(
            algo.latest_clusters, info, robot_xy,
            min_size=params.min_frontier_size,
            blacklist=blacklist,
            blacklist_radius=params.blacklist_radius,
            min_dist=params.min_frontier_dist,
        ) if goal_xy is not None else None

        print(compact_legend)
        print(render(data, info, robot_xy, goal_xy, target_xy, blacklist,
                     algo.latest_clusters, params.min_frontier_size))

        if goal_xy is None:
            no_frontier_count += 1
            diag = algo.latest_diag or {}
            print(f"\nStep {step}: no frontier "
                  f"({no_frontier_count}/{PATIENCE})  diag={diag}")
            if no_frontier_count >= PATIENCE:
                print("\nExploration complete — no frontiers remain.")
                break
        else:
            no_frontier_count = 0
            dist = math.sqrt((goal_xy[0] - robot_xy[0]) ** 2
                             + (goal_xy[1] - robot_xy[1]) ** 2)
            legend = cluster_legend(algo.latest_clusters, params.min_frontier_size)
            print(f"\nStep {step}: goal=({goal_xy[0]:.2f},{goal_xy[1]:.2f})"
                  f"  dist={dist:.2f}  blacklisted={len(blacklist)}{legend}")
            robot_xy = goal_xy
            data = uncover_around_robot(data, info, robot_xy, args.sensor_radius)
            blacklist.add(goal_xy)

        step += 1
        if not args.auto:
            try:
                input("\nPress Enter for next step (Ctrl-C to quit)...")
            except KeyboardInterrupt:
                print("\nAborted.")
                break
        print()


if __name__ == "__main__":
    main()
