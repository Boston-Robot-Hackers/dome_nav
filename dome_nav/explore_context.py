#!/usr/bin/env python3
# explore_context.py — data types and protocol for pluggable exploration algorithms
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol

from builtin_interfaces.msg import Time


@dataclass
class MapInfo:
    # Occupancy-grid geometry. Strategy-neutral — lives here in the shared contract
    # module so the manager node and any algorithm can use it without depending on
    # a specific strategy's module.
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


class GoalOutcome(Enum):
    # What an algorithm decided this tick. NEW_GOAL carries an (x, y); the other
    # two carry no goal and name *why* there is none, so the node no longer has to
    # peek at algorithm internals to tell "blocked right now" from "finished".
    NEW_GOAL = auto()
    NO_TARGETS_BLOCKED = auto()  # targets exist but all filtered/blacklisted
    EXPLORED_DONE = auto()       # algorithm is finished — end the session


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
    # Shared/session tuning owned by the manager node — the small set meaningful to
    # any exploration strategy, not just frontier. Frontier-only tuning moved to
    # frontier_params.FrontierParams, which the frontier algorithm owns and
    # self-declares (F23 T03). blacklist_radius stays here because the node's own
    # blacklist/reselection policy uses it.
    max_explore_radius: float = 0.0
    blacklist_radius: float = 0.5
    preferred_goal_distance: float = 1.0


@dataclass
class ExplorationContext:
    map_data: list[int]
    map_info: MapInfo
    robot_xy: tuple[float, float]
    blacklist: set[tuple[float, float]]
    start_xy: tuple[float, float] | None
    params: ExploreParams


@dataclass
class RenderContext:
    # Node-owned session state handed to an algorithm's optional visualization /
    # diagnostics hooks. Everything here is general session state (no frontier
    # concepts); the node fills it and treats whatever the hook returns — a
    # MarkerArray, a report string, a telemetry dict — as opaque.
    now: Time
    is_exploring: bool
    map_info: MapInfo | None
    robot_xy: tuple[float, float] | None
    blacklist: set[tuple[float, float]]
    goal_xy: tuple[float, float] | None
    params: ExploreParams
    patience: int  # node's no-target debounce threshold, for report labels


class ExplorationAlgorithm(Protocol):
    # Required surface: turn a context into a decision. Nothing else is mandatory.
    #
    # Visualization and diagnostics are OPTIONAL hooks the node calls via getattr
    # and treats as opaque (see explorer_manager_node). An algorithm may implement
    # any subset:
    #   render_markers(rc: RenderContext) -> MarkerArray | None
    #   exhaustion_report(rc: RenderContext) -> str | None
    #   failure_report(rc: RenderContext) -> str | None
    #   telemetry_extra() -> dict
    #   session_params() -> dict
    # A plugin with no markers or diagnostics simply omits them.
    def next_goal(self, ctx: ExplorationContext) -> GoalDecision: ...

    def declare_params(self, node) -> None:
        # Optional hook the node calls once at construction so an algorithm can
        # declare and read its own ROS parameters in the node's namespace (ROS
        # params must be node-declared to be yaml/launch-settable). Algorithms
        # with no tuning of their own leave this as a no-op.
        ...
