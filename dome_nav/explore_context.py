#!/usr/bin/env python3
# explore_context.py — data types and protocol for pluggable exploration algorithms
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from collections.abc import Sequence
from dataclasses import dataclass, field, fields
from enum import Enum, auto
from typing import Protocol

from builtin_interfaces.msg import Time
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor


def tuning_field(
    default, desc: str, important: bool, dynamic: bool,
    min_value: float | None = None,
):
    """One tunable field: default value plus its ROS declaration metadata.

    ros_description becomes the ParameterDescriptor (ros2 param describe); units
    are stated in the text. ros_important / ros_dynamic are documentation-only
    markers for docs and tuning tooling — nothing reads them in code yet, and
    today every param is read once at startup (dynamic = intended mid-run lever
    once a parameter-event callback lands). ros_min adds a FloatingPointRange so
    ROS itself rejects out-of-range values.
    """
    metadata = {
        "ros_description": desc,
        "ros_important": important,
        "ros_dynamic": dynamic,
    }
    if min_value is not None:
        metadata["ros_min"] = min_value
    return field(default=default, metadata=metadata)


# ROS infers the parameter type from the default's Python type, so the allowed
# field types are pinned deliberately rather than inferred from whatever lands
# in the dataclass.
DECLARABLE_TYPES = (bool, int, float)


def ros_descriptor_for(field_def) -> ParameterDescriptor:
    descriptor = ParameterDescriptor(
        description=field_def.metadata["ros_description"],
    )
    if "ros_min" in field_def.metadata:
        descriptor.floating_point_range = [
            FloatingPointRange(
                from_value=float(field_def.metadata["ros_min"]),
                to_value=float("inf"),
                step=0.0,
            )
        ]
    return descriptor


def declare_dataclass_params(node, params_cls):
    """Declare every field of a tuning dataclass as a ROS param and read it back.

    Each field must be a plain bool/int/float whose default matches its
    annotation and must carry the tuning_field() metadata; violations raise
    loudly at startup rather than declaring a mistyped param.
    """
    values = {}
    for field_def in fields(params_cls):
        is_declarable = (
            field_def.type in DECLARABLE_TYPES
            and isinstance(field_def.default, field_def.type)
        )
        if not is_declarable:
            raise TypeError(
                f"{params_cls.__name__}.{field_def.name}: only plain "
                f"bool/int/float fields with matching defaults are "
                f"declarable, got {field_def.type!r}"
            )
        node.declare_parameter(
            field_def.name, field_def.default, ros_descriptor_for(field_def),
        )
        values[field_def.name] = node.get_parameter(field_def.name).value
    return params_cls(**values)


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

    Ownership rule (F34 T03): a field is shared iff the node itself reads it for
    its own policy — radius gating (max_explore_radius) or blacklist reselection
    (blacklist_radius). Tuning only an algorithm's scorer reads lives in that
    algorithm (e.g. frontier_params.FrontierParams.preferred_goal_distance).
    Declared on the node via declare_dataclass_params — adding a field here
    auto-declares and auto-reads with no node edits.
    """
    max_explore_radius: float = tuning_field(
        0.0, "Maximum exploration radius (m) from the start pose; 0 = unlimited",
        important=True, dynamic=False)
    blacklist_radius: float = tuning_field(
        0.5,
        "Radius (m) around a failed goal suppressed from reselection; "
        "must exceed goal_inset_m",
        important=True, dynamic=False)


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
