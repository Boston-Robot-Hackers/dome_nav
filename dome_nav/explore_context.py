#!/usr/bin/env python3
# explore_context.py — data types and protocol for pluggable exploration algorithms
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol

from builtin_interfaces.msg import Time


@dataclass
class MapInfo:
    """Occupancy-grid geometry; lives in the shared contract module, not a strategy."""
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


class GoalOutcome(Enum):
    """Names why next_goal returned no goal, so the node needn't peek at internals."""
    NEW_GOAL = auto()
    NO_TARGETS_BLOCKED = auto()  # targets exist but all filtered/blacklisted
    EXPLORED_DONE = auto()       # algorithm finished — end the session


@dataclass(frozen=True)
class GoalDecision:
    outcome: GoalOutcome
    xy: tuple[float, float] | None = None

    @classmethod
    def new_goal(cls, xy: tuple[float, float]) -> "GoalDecision":
        return cls(GoalOutcome.NEW_GOAL, xy)

    @classmethod
    def blocked(cls) -> "GoalDecision":
        return cls(GoalOutcome.NO_TARGETS_BLOCKED)

    @classmethod
    def done(cls) -> "GoalDecision":
        return cls(GoalOutcome.EXPLORED_DONE)


@dataclass
class ExploreParams:
    """Shared/session tuning the node owns.

    Strategy-specific tuning lives in the algorithm (e.g. frontier_params.
    FrontierParams). blacklist_radius is here because the node's own reselection
    policy uses it.
    """
    max_explore_radius: float = 0.0
    blacklist_radius: float = 0.5
    preferred_goal_distance: float = 1.0


@dataclass
class ExplorationContext:
    """Read-only view; the node passes the OccupancyGrid's array.array uncopied."""
    map_data: Sequence[int]
    map_info: MapInfo
    robot_xy: tuple[float, float]
    blacklist: set[tuple[float, float]]
    start_xy: tuple[float, float] | None
    params: ExploreParams


@dataclass
class RenderContext:
    """Session state handed to an algorithm's optional viz/diagnostic hooks.

    The node treats whatever the hook returns as opaque.
    """
    now: Time
    is_exploring: bool
    map_info: MapInfo | None
    robot_xy: tuple[float, float] | None
    blacklist: set[tuple[float, float]]
    goal_xy: tuple[float, float] | None
    params: ExploreParams
    patience: int  # node's no-target debounce threshold, for report labels


class ExplorationAlgorithm(Protocol):
    """Pluggable exploration strategy contract.

    next_goal is the only required method. These optional hooks are called via
    getattr and treated as opaque; a plugin omits any it doesn't need:
        render_markers(rc) -> MarkerArray | None
        exhaustion_report(rc) -> str | None
        failure_report(rc) -> str | None
        telemetry_extra() -> dict
        session_params() -> dict
    """
    def next_goal(self, ctx: ExplorationContext) -> GoalDecision: ...

    def declare_params(self, node) -> None:
        """Optional: declare/read the algorithm's ROS params in the node namespace.

        Must be node-declared to be yaml/launch-settable. No-op if the plugin has none.
        """
        ...
