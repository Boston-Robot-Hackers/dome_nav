#!/usr/bin/env python3
# frontier_params.py — frontier-algorithm-owned tuning params and ROS declaration
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dataclasses import dataclass, fields

from dome_nav.explore_context import (
    ExploreParams,
    declare_dataclass_params,
    tuning_field,
)


@dataclass
class FrontierParams:
    """Frontier-only tuning, owned and self-declared by FrontierAlgorithm.

    Single source of truth (F34): declaration, read-back, and merge are driven
    by dataclasses.fields(), and FrontierTuning inherits this class — adding a
    field here auto-declares, auto-reads, and auto-merges with no other edits.
    """
    min_frontier_size: int = tuning_field(
        15, "Minimum cells in a frontier cluster to be a candidate",
        important=True, dynamic=False)
    min_frontier_dist: float = tuning_field(
        1.3, "Minimum goal distance (m) from the robot",
        important=True, dynamic=False)
    max_frontier_dist: float = tuning_field(
        0.0, "Maximum goal distance (m); 0 = unlimited",
        important=True, dynamic=False)
    goal_inset_m: float = tuning_field(
        0.3, "Distance (m) to nudge the goal inward from the frontier cells",
        important=True, dynamic=False)
    frontier_buffer_cells: int = tuning_field(
        2, "Known-cell rings inside the unknown boundary",
        important=False, dynamic=False)
    use_novelty_scoring: bool = tuning_field(
        False, "Opt-in: adds the novelty scorer (F31 pipeline)",
        important=False, dynamic=True)
    # F31 scorer weights (all scorers normalized [0,1] per cycle, so comparable).
    w_distance: float = tuning_field(
        1.0, "Distance-to-preferred scorer weight",
        important=False, dynamic=True, min_value=0.0)
    w_novelty: float = tuning_field(
        1.0, "Novelty scorer weight (only when use_novelty_scoring)",
        important=False, dynamic=True, min_value=0.0)
    w_clearance: float = tuning_field(
        1.0, "Clearance scorer weight; 0 disables F31 clearance",
        important=True, dynamic=True, min_value=0.0)
    robot_radius: float = tuning_field(
        0.17, "R_inscribed (m) for the clearance floor; match the Nav2 footprint",
        important=True, dynamic=False)
    clearance_margin_m: float = tuning_field(
        0.05, "Clearance floor = robot_radius + this (m); keep small",
        important=True, dynamic=False)
    preferred_goal_distance: float = tuning_field(
        1.0, "Preferred goal distance (m) from the robot; scorer pulls toward it",
        important=True, dynamic=True)


@dataclass
class FrontierTuning(FrontierParams):
    """FrontierParams plus the shared ExploreParams fields, merged per tick.

    Inherits FrontierParams so a new frontier knob appears here with zero
    edits (TF34 T01); merge_tuning overlays the two shared fields explicitly.
    """
    blacklist_radius: float = 0.5
    max_explore_radius: float = 0.0


def merge_tuning(shared: ExploreParams, frontier: FrontierParams) -> FrontierTuning:
    """Merge shared and frontier params into one per-tick FrontierTuning.

    Enforces blacklist_radius > goal_inset_m at this boundary: exclusion coords are
    stored post-nudge but filtered against raw cells goal_inset_m apart, so a radius
    at or below the inset would let an excluded cell reselect every tick.
    """
    if shared.blacklist_radius <= frontier.goal_inset_m:
        raise ValueError(
            f"blacklist_radius ({shared.blacklist_radius}) must exceed "
            f"goal_inset_m ({frontier.goal_inset_m}); exclusion is stored "
            "post-nudge but filtered against raw cells."
        )
    return FrontierTuning(
        **{
            field_def.name: getattr(frontier, field_def.name)
            for field_def in fields(frontier)
        },
        blacklist_radius=shared.blacklist_radius,
        max_explore_radius=shared.max_explore_radius,
    )


def declare_frontier_params(node) -> FrontierParams:
    """Declare the frontier tuning as ROS params in the node's namespace and read back.

    Keeps every frontier param name out of the node itself.
    """
    return declare_dataclass_params(node, FrontierParams)
