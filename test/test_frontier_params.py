#!/usr/bin/env python3
# test_frontier_params.py — TF34 T01: fields()-driven declare/read + merge dedup
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dataclasses import field, fields, make_dataclass
from types import SimpleNamespace

import pytest

from dome_nav.explore_context import ExploreParams, declare_dataclass_params
from dome_nav.frontier_params import (
    FrontierParams,
    FrontierTuning,
    declare_frontier_params,
    merge_tuning,
)


class FakeNode:
    """Minimal declare/get node: records declarations, serves the default back."""

    def __init__(self):
        self.declared = {}  # name -> (default, descriptor)

    def declare_parameter(self, name, default, descriptor=None):
        self.declared[name] = (default, descriptor)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.declared[name][0])


# --- declare/read round-trip ---

def test_declare_round_trips_every_field():
    node = FakeNode()
    params = declare_frontier_params(node)
    assert params == FrontierParams()
    assert set(node.declared) == {fd.name for fd in fields(FrontierParams)}


def test_shared_params_round_trip_through_generic_declare():
    # The node's shared ExploreParams use the same machinery as the frontier side.
    node = FakeNode()
    params = declare_dataclass_params(node, ExploreParams)
    assert params == ExploreParams()
    assert set(node.declared) == {fd.name for fd in fields(ExploreParams)}


def test_every_field_carries_ros_metadata():
    # Doc surface can't silently rot: description + both doc-only flags required.
    for params_cls in (ExploreParams, FrontierParams):
        for fd in fields(params_cls):
            desc = fd.metadata.get("ros_description")
            assert isinstance(desc, str) and desc.strip(), fd.name
            assert isinstance(fd.metadata.get("ros_important"), bool), fd.name
            assert isinstance(fd.metadata.get("ros_dynamic"), bool), fd.name


def test_descriptions_reach_the_descriptor():
    node = FakeNode()
    declare_frontier_params(node)
    for name, (_, descriptor) in node.declared.items():
        assert descriptor.description.strip(), name


def test_scorer_weights_reject_negatives_via_declared_range():
    node = FakeNode()
    declare_frontier_params(node)
    for name in ("w_distance", "w_novelty", "w_clearance"):
        (r,) = node.declared[name][1].floating_point_range
        assert r.from_value == 0.0
        assert r.to_value == float("inf")
    # Non-weight fields carry no range.
    assert not node.declared["min_frontier_size"][1].floating_point_range
    assert not node.declared["use_novelty_scoring"][1].floating_point_range


def test_declare_rejects_non_scalar_field_type():
    bad = make_dataclass(
        "BadParams",
        [("xs", list, field(default_factory=list))],
    )
    with pytest.raises(TypeError, match="xs"):
        declare_dataclass_params(FakeNode(), bad)


# --- merge: pass-through is fields()-driven, shared fields overlaid ---

def test_tuning_fields_are_frontier_plus_shared_overlay():
    # The structural proof that a new FrontierParams field flows into
    # FrontierTuning and merge_tuning with zero edits outside the dataclass.
    frontier_names = {fd.name for fd in fields(FrontierParams)}
    tuning_names = {fd.name for fd in fields(FrontierTuning)}
    assert tuning_names == frontier_names | {
        "blacklist_radius", "max_explore_radius",
    }


def test_merge_tuning_matches_handwritten_reference():
    # Field-for-field fixture: the fields()-driven merge must produce exactly
    # what the pre-refactor hand-listed merge produced (override case).
    shared = ExploreParams(
        max_explore_radius=3.0, blacklist_radius=0.6,
    )
    frontier = FrontierParams(
        min_frontier_size=7, min_frontier_dist=0.4, max_frontier_dist=9.0,
        goal_inset_m=0.25, frontier_buffer_cells=3,
        use_novelty_scoring=True,
        w_distance=2.0, w_novelty=3.0, w_clearance=4.0,
        robot_radius=0.2, clearance_margin_m=0.07,
        preferred_goal_distance=2.0,
    )
    tuning = merge_tuning(shared, frontier)
    assert tuning.min_frontier_size == 7
    assert tuning.min_frontier_dist == 0.4
    assert tuning.max_frontier_dist == 9.0
    assert tuning.goal_inset_m == 0.25
    assert tuning.frontier_buffer_cells == 3
    assert tuning.use_novelty_scoring is True
    assert tuning.w_distance == 2.0
    assert tuning.w_novelty == 3.0
    assert tuning.w_clearance == 4.0
    assert tuning.robot_radius == 0.2
    assert tuning.clearance_margin_m == 0.07
    assert tuning.blacklist_radius == 0.6
    assert tuning.max_explore_radius == 3.0
    assert tuning.preferred_goal_distance == 2.0


def test_merge_tuning_defaults_match_frontier_params():
    tuning = merge_tuning(ExploreParams(), FrontierParams())
    for fd in fields(FrontierParams):
        assert getattr(tuning, fd.name) == getattr(FrontierParams(), fd.name)


def test_merge_tuning_rejects_out_of_bounds_radius_override():
    # T02 regression: a blacklist_radius override at or below goal_inset_m must
    # still trip the boundary validation (exclusion would reselect every tick).
    with pytest.raises(ValueError, match="blacklist_radius"):
        merge_tuning(ExploreParams(blacklist_radius=0.2), FrontierParams())


# --- zero-edit proof: a subclassed temp field declares and reads back ---

def test_new_field_round_trips_with_zero_edits():
    temp_params = make_dataclass(
        "TempParams",
        [("temp_knob", float, field(default=0.5, metadata={
            "ros_description": "temporary knob proving zero-edit declaration",
            "ros_important": False,
            "ros_dynamic": False,
        }))],
        bases=(FrontierParams,),
    )
    node = FakeNode()
    params = declare_dataclass_params(node, temp_params)
    assert params.temp_knob == 0.5
    assert params.min_frontier_size == 15
    assert "temp_knob" in node.declared
