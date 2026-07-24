#!/usr/bin/env python3
# explore_markers.py — RViz marker construction for frontier exploration
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from dome_nav.explore_context import MapInfo
from dome_nav.frontier_explorer import cell_to_world


def new_marker(now: Time, ns: str, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = now
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    return marker


def add_point(marker: Marker, x: float, y: float) -> None:
    point = Point()
    point.x = x
    point.y = y
    marker.points.append(point)


def build_explore_markers(
    now: Time,
    is_exploring: bool,
    clusters: list[list[int]],
    min_frontier_size: int,
    map_info: MapInfo | None,
    blacklist: set[tuple[float, float]],
    goal_xy: tuple[float, float] | None,
) -> MarkerArray:
    """Build the frontiers/blacklist/goal marker array for /explore/markers."""
    markers = MarkerArray()
    markers.markers.append(
        build_frontier_marker(now, is_exploring, clusters, min_frontier_size, map_info)
    )
    markers.markers.append(build_blacklist_marker(now, blacklist))
    markers.markers.append(build_goal_marker(now, is_exploring, goal_xy))
    return markers


def build_frontier_marker(
    now: Time,
    is_exploring: bool,
    clusters: list[list[int]],
    min_frontier_size: int,
    map_info: MapInfo | None,
) -> Marker:
    marker = new_marker(now, "frontiers", 0, Marker.POINTS)
    marker.action = Marker.ADD if is_exploring else Marker.DELETE
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    if is_exploring and map_info is not None:
        for cluster in clusters:
            if len(cluster) >= min_frontier_size:
                for idx in cluster:
                    wx, wy = cell_to_world(idx, map_info)
                    add_point(marker, wx, wy)
    return marker


def build_blacklist_marker(
    now: Time,
    blacklist: set[tuple[float, float]],
) -> Marker:
    marker = new_marker(now, "blacklist", 1, Marker.POINTS)
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    for bx, by in blacklist:
        add_point(marker, bx, by)
    return marker


def build_goal_marker(
    now: Time,
    is_exploring: bool,
    goal_xy: tuple[float, float] | None,
) -> Marker:
    marker = new_marker(now, "goal", 2, Marker.SPHERE)
    if is_exploring and goal_xy is not None:
        marker.action = Marker.ADD
        marker.pose.position.x = goal_xy[0]
        marker.pose.position.y = goal_xy[1]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
    else:
        marker.action = Marker.DELETE
    return marker
