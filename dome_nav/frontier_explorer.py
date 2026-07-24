#!/usr/bin/env python3
# frontier_explorer.py — pure Python frontier detection from OccupancyGrid data
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from dome_nav.explore_context import MapInfo

if TYPE_CHECKING:
    from dome_nav.frontier_params import FrontierTuning

# /map (SLAM OccupancyGrid) convention: -1 unknown, 0 free, 100 occupied. Cells at
# or above this are walls (clearance sources); unknown and free are passable fill.
OCCUPIED_THRESHOLD = 65
SQRT2 = math.sqrt(2.0)


def find_frontier_clusters(
    data: Sequence[int], info: MapInfo, buffer_cells: int = 2
) -> list[list[int]]:
    """Cluster frontier cells, `buffer_cells` known-cell rings inside the unknown
    boundary. Returns flat cell indices per cluster (see 01-literate/06 for the
    buffer-ring rationale and the worldToMap-seam reason the default is 2)."""
    width, height = info.width, info.height

    def neighbors4(idx: int):
        r, c = divmod(idx, width)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width:
                yield nr * width + nc

    def neighbors8(idx: int):
        r, c = divmod(idx, width)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width:
                    yield nr * width + nc

    # Boundary ring (depth 0): free cells directly adjacent to unknown.
    boundary: set[int] = set()
    for idx in range(width * height):
        if data[idx] != 0:
            continue
        for nb in neighbors4(idx):
            if data[nb] == -1:
                boundary.add(idx)
                break

    # Walk buffer_cells free-cell rings inward; last ring reached is the frontier.
    claimed: set[int] = set(boundary)
    ring: set[int] = boundary
    is_frontier: set[int] = boundary
    for _ in range(buffer_cells):
        next_ring: set[int] = set()
        for idx in ring:
            for nb in neighbors4(idx):
                if data[nb] == 0 and nb not in claimed:
                    next_ring.add(nb)
        claimed |= next_ring
        ring = next_ring
        is_frontier = next_ring

    visited: set[int] = set()
    clusters: list[list[int]] = []
    for seed in is_frontier:
        if seed in visited:
            continue
        cluster: list[int] = []
        stack = [seed]
        while stack:
            cell = stack.pop()
            if cell in visited or cell not in is_frontier:
                continue
            visited.add(cell)
            cluster.append(cell)
            for nb in neighbors8(cell):
                if nb not in visited and nb in is_frontier:
                    stack.append(nb)
        clusters.append(cluster)
    return clusters


def cell_to_world(idx: int, info: MapInfo) -> tuple[float, float]:
    r, c = divmod(idx, info.width)
    x = info.origin_x + (c + 0.5) * info.resolution
    y = info.origin_y + (r + 0.5) * info.resolution
    return (x, y)


def world_to_cell(xy: tuple[float, float], info: MapInfo) -> tuple[int, int]:
    """Inverse of cell_to_world: world meters -> (row, col) grid indices."""
    # math.floor, not int(): int() truncates toward zero, so a point just left
    # of the origin would land in cell 0 (in-bounds) instead of -1.
    col = math.floor((xy[0] - info.origin_x) / info.resolution)
    row = math.floor((xy[1] - info.origin_y) / info.resolution)
    return (row, col)


def bresenham_cells(r0: int, c0: int, r1: int, c1: int):
    """Yield integer (row, col) cells on the raster line (r0,c0)->(r1,c1), inclusive."""
    dc = abs(c1 - c0)
    dr = -abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc + dr
    while True:
        yield (r0, c0)
        if r0 == r1 and c0 == c1:
            return
        e2 = 2 * err
        if e2 >= dr:
            err += dr
            c0 += sc
        if e2 <= dc:
            err += dc
            r0 += sr


def path_novelty_score(
    start_xy: tuple[float, float], end_xy: tuple[float, float],
    data: Sequence[int], info: MapInfo,
) -> int:
    """Count unknown cells on the straight line start_xy -> end_xy.

    More unknown cells crossed = more new territory revealed by travelling there.
    Out-of-bounds cells are skipped, not counted. Pure integer-cell arithmetic.
    """
    r0, c0 = world_to_cell(start_xy, info)
    r1, c1 = world_to_cell(end_xy, info)
    score = 0
    for row, col in bresenham_cells(r0, c0, r1, c1):
        if 0 <= row < info.height and 0 <= col < info.width:
            if data[row * info.width + col] == -1:
                score += 1
    return score


def clearance_field(data: Sequence[int], info: MapInfo) -> list[float]:
    """Cells-to-nearest-occupied for every grid cell.

    Multi-source relaxation BFS from every wall cell (data >= OCCUPIED_THRESHOLD),
    8-connected, diagonal step SQRT2. Occupied cells are 0.0; a cell keeps inf only
    when no wall is reachable from it (e.g. a map with no occupied cell at all) —
    callers treat inf as "maximally open". Returns one distance per flat cell index.
    """
    width, height = info.width, info.height
    dist = [float("inf")] * (width * height)
    queue: deque[int] = deque()
    for idx, value in enumerate(data):
        if value >= OCCUPIED_THRESHOLD:
            dist[idx] = 0.0
            queue.append(idx)
    steps = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
    )
    while queue:
        idx = queue.popleft()
        row, col = divmod(idx, width)
        base = dist[idx]
        for dr, dc, step in steps:
            nr, nc = row + dr, col + dc
            if 0 <= nr < height and 0 <= nc < width:
                nidx = nr * width + nc
                nd = base + step
                if nd < dist[nidx]:
                    dist[nidx] = nd
                    queue.append(nidx)
    return dist


# --- F31 goal-scoring pipeline (see 01-literate/06 for the design rationale) ---


@dataclass
class CellCtx:
    """One candidate cell's inputs for filters/scorers; shared fields are per-tick."""
    world_xy: tuple[float, float]        # goal point in world meters
    cell_index: int                      # flat offset into map_data
    dist_to_robot_m: float               # euclidean today; path distance when F30 lands
    clearance_cells: float               # to nearest occupied cell; inf until F31 T04
    map_data: Sequence[int]              # OccupancyGrid: -1 unknown, 0 free, >=lethal occupied
    map_info: MapInfo
    robot_world_xy: tuple[float, float]
    start_world_xy: tuple[float, float] | None
    tuning: "FrontierTuning"
    blacklist: set[tuple[float, float]]  # excluded goal points (post-nudge world xy)


Filter = Callable[[CellCtx], bool]        # True = keep
Scorer = Callable[[CellCtx], float]       # raw cost, lower = better
WeightedScorer = tuple[float, Scorer]     # weight 0.0 disables in place


@dataclass
class Registry:
    """Filters and scorers driving selection; populated per tick by build_registry."""
    cluster_filters: list[Callable[[list[int]], bool]] = field(default_factory=list)
    cell_filters: list[Filter] = field(default_factory=list)
    scorers: list[WeightedScorer] = field(default_factory=list)


def minmax_normalize(values: list[float]) -> list[float]:
    """Scale values to [0, 1]; all-equal input returns zeros.

    All-equal (incl. a single survivor, or an all-inf clearance column when the map
    has no walls) means the scorer does not discriminate, so it must not sway the
    weighted sum -> zeros. Compares lo == hi rather than the span so inf stays
    finite-safe.
    """
    lo, hi = min(values), max(values)
    if lo == hi:
        return [0.0] * len(values)
    return [(v - lo) / (hi - lo) for v in values]


def select_cell(
    cells: list[CellCtx],
    cell_filters: list[Filter],
    scorers: list[WeightedScorer],
) -> tuple[float, float] | None:
    """Return the world_xy of the min weighted-cost cell that passes all filters.

    Applies every cell filter (keep on all True), then for each scorer normalizes
    its raw costs across the survivors and sums the weighted normalized columns;
    the lowest total wins. Normalization is per-scorer across survivors (cross-cell)
    so it lives here, not in the scorers. Returns None when no cell survives.
    """
    survivors = [c for c in cells if all(f(c) for f in cell_filters)]
    if not survivors:
        return None
    columns = [
        (weight, minmax_normalize([scorer(c) for c in survivors]))
        for weight, scorer in scorers
    ]
    best_xy: tuple[float, float] | None = None
    best_cost = float("inf")
    for i, cell in enumerate(survivors):
        cost = sum(weight * col[i] for weight, col in columns)
        if cost < best_cost:
            best_cost = cost
            best_xy = cell.world_xy
    return best_xy


def keep_off_blacklist(ctx: CellCtx) -> bool:
    br = ctx.tuning.blacklist_radius
    wx, wy = ctx.world_xy
    return not any(
        math.sqrt((wx - bx) ** 2 + (wy - by) ** 2) < br for bx, by in ctx.blacklist
    )


def keep_within_dist_range(ctx: CellCtx) -> bool:
    dist_m = ctx.dist_to_robot_m
    lo, hi = ctx.tuning.min_frontier_dist, ctx.tuning.max_frontier_dist
    return not (lo > 0.0 and dist_m < lo) and not (hi > 0.0 and dist_m > hi)


def score_distance_to_preferred(ctx: CellCtx) -> float:
    """Cost = distance of the post-nudge goal from preferred_goal_distance."""
    # The nudge pulls the goal goal_inset_m closer, so score d - inset, not raw d.
    reach = ctx.dist_to_robot_m - ctx.tuning.goal_inset_m
    return abs(reach - ctx.tuning.preferred_goal_distance)


def score_novelty(ctx: CellCtx) -> float:
    """Cost rewarding paths that cross more unknown cells (more new territory)."""
    # Negate the unknown-cell count so higher novelty = lower cost.
    return -float(path_novelty_score(
        ctx.robot_world_xy, ctx.world_xy, ctx.map_data, ctx.map_info
    ))


def score_clearance_bonus(ctx: CellCtx) -> float:
    """Cost rewarding open cells (more clearance = lower cost)."""
    # Negate clearance so more-open = lower cost. inf (no walls) normalizes to 0
    # alongside its peers, so it never dominates.
    return -ctx.clearance_cells


def keep_clearance_floor(ctx: CellCtx) -> bool:
    """Keep cells whose clearance >= inscribed radius + margin (reachability floor)."""
    # Kept low so corridors (max clearance ~ half the width) still yield goals; the
    # bonus does the "prefer open" work, not this.
    floor_m = ctx.tuning.robot_radius + ctx.tuning.clearance_margin_m
    return ctx.clearance_cells * ctx.map_info.resolution >= floor_m


def make_min_size_filter(tuning: "FrontierTuning") -> Callable[[list[int]], bool]:
    return lambda cluster: len(cluster) >= tuning.min_frontier_size


def make_max_radius_filter(
    tuning: "FrontierTuning", info: MapInfo, start_xy: tuple[float, float] | None
) -> Callable[[list[int]], bool]:
    return lambda cluster: not cluster_outside_radius(
        cluster, info, start_xy, tuning.max_explore_radius
    )


def build_registry(
    tuning: "FrontierTuning", info: MapInfo, start_xy: tuple[float, float] | None
) -> Registry:
    """Build the per-tick filter/scorer Registry that drives selection.

    Cell filters and scorers read CellCtx; cluster filters close over the per-tick
    tuning/info/start. Novelty and clearance tenants are added only when enabled.
    """
    scorers: list[WeightedScorer] = [(tuning.w_distance, score_distance_to_preferred)]
    if tuning.use_novelty_scoring:
        scorers.append((tuning.w_novelty, score_novelty))
    cell_filters: list[Filter] = [keep_off_blacklist, keep_within_dist_range]
    if tuning.w_clearance > 0.0:
        cell_filters.append(keep_clearance_floor)
        scorers.append((tuning.w_clearance, score_clearance_bonus))
    return Registry(
        cluster_filters=[
            make_min_size_filter(tuning),
            make_max_radius_filter(tuning, info, start_xy),
        ],
        cell_filters=cell_filters,
        scorers=scorers,
    )


def cell_contexts(
    clusters: list[list[int]],
    data: Sequence[int],
    info: MapInfo,
    robot_xy: tuple[float, float],
    tuning: "FrontierTuning",
    blacklist: set[tuple[float, float]],
    start_xy: tuple[float, float] | None,
    cluster_filters: list[Callable[[list[int]], bool]],
    clearance: Sequence[float] | None = None,
) -> list[CellCtx]:
    """Flatten clusters passing cluster_filters into per-cell CellCtx (order preserved).

    clearance is the per-tick field, or None when clearance scoring is off, in which
    case each cell's clearance_cells is inf.
    """
    rx, ry = robot_xy
    contexts: list[CellCtx] = []
    for cluster in clusters:
        if not all(f(cluster) for f in cluster_filters):
            continue
        for idx in cluster:
            wx, wy = cell_to_world(idx, info)
            clr = clearance[idx] if clearance is not None else float("inf")
            contexts.append(CellCtx(
                world_xy=(wx, wy),
                cell_index=idx,
                dist_to_robot_m=math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2),
                clearance_cells=clr,
                map_data=data,
                map_info=info,
                robot_world_xy=robot_xy,
                start_world_xy=start_xy,
                tuning=tuning,
                blacklist=blacklist,
            ))
    return contexts


# Per-cell (not centroid) selection avoids the ring-cluster problem; blacklist
# is per-cell. See 01-literate/06.

def pick_best_frontier(
    clusters: list[list[int]],
    info: MapInfo,
    robot_xy: tuple[float, float],
    params: "FrontierTuning",
    blacklist: set[tuple[float, float]] | None = None,
    start_xy: tuple[float, float] | None = None,
    data: Sequence[int] = (),
) -> tuple[float, float] | None:
    """Pick the best frontier goal world_xy via the F31 pipeline, or None.

    Builds the per-tick registry, computes the clearance field only when clearance
    scoring is active, flattens surviving clusters into per-cell contexts, and
    returns the min weighted-scorer cell. With only the distance scorer this
    reproduces the pre-F31 preferred-distance selection. data is read only by the
    grid-based scorers (novelty, clearance); pass () when none are enabled.
    """
    registry = build_registry(params, info, start_xy)
    use_clearance = params.w_clearance > 0.0 and bool(data)
    clearance = clearance_field(data, info) if use_clearance else None
    cells = cell_contexts(
        clusters, data, info, robot_xy, params, blacklist or set(), start_xy,
        registry.cluster_filters, clearance,
    )
    return select_cell(cells, registry.cell_filters, registry.scorers)


def cluster_outside_radius(
    cluster: list[int], info: MapInfo, start_xy: tuple[float, float] | None,
    max_radius: float,
) -> bool:
    if max_radius <= 0.0 or start_xy is None:
        return False
    cx = sum(cell_to_world(i, info)[0] for i in cluster) / len(cluster)
    cy = sum(cell_to_world(i, info)[1] for i in cluster) / len(cluster)
    return math.sqrt((cx - start_xy[0]) ** 2 + (cy - start_xy[1]) ** 2) > max_radius


def frontier_diag(
    clusters: list[list[int]],
    info: MapInfo,
    robot_xy: tuple[float, float],
    min_size: int,
    min_dist: float,
    max_dist: float = 0.0,
) -> dict:
    """Return filter-stage cluster counts for telemetry (too_small/large/out-of-range).

    A cheap extra pass, only called when pick_best_frontier returns None, so
    normal-path performance is unaffected.
    """
    rx, ry = robot_xy
    too_small = sum(1 for c in clusters if len(c) < min_size)
    large = [c for c in clusters if len(c) >= min_size]
    all_out_of_range = 0
    for cluster in large:
        if all(
            cell_out_of_range(cell_to_world(i, info), robot_xy, min_dist, max_dist)
            for i in cluster
        ):
            all_out_of_range += 1
    return {
        "too_small": too_small,
        "large_clusters": len(large),
        "all_cells_out_of_range": all_out_of_range,
    }


def cell_out_of_range(
    cell_xy: tuple[float, float],
    robot_xy: tuple[float, float],
    min_dist: float,
    max_dist: float,
) -> bool:
    dist_m = math.sqrt(
        (cell_xy[0] - robot_xy[0]) ** 2 + (cell_xy[1] - robot_xy[1]) ** 2
    )
    return (min_dist > 0.0 and dist_m < min_dist) or (
        max_dist > 0.0 and dist_m > max_dist
    )


def nudge_toward_robot(
    xy: tuple[float, float], robot_xy: tuple[float, float], inset_m: float
) -> tuple[float, float]:
    """Pull xy toward robot_xy by inset_m; returns xy unchanged if closer than inset_m.

    Keeps the nav goal inside the costmap boundary rather than on the unknown-cell
    edge, which avoids Nav2 worldToMap errors.
    """
    dx = robot_xy[0] - xy[0]
    dy = robot_xy[1] - xy[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < inset_m:
        return xy
    scale = inset_m / dist
    return (xy[0] + dx * scale, xy[1] + dy * scale)
