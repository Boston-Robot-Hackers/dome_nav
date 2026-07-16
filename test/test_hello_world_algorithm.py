#!/usr/bin/env python3
# test_hello_world_algorithm.py — unit tests for HelloWorldAlgorithm (pure Python)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dome_nav.explore_context import (
    ExplorationContext, ExploreParams, GoalOutcome,
)
from dome_nav.frontier_explorer import MapInfo
from dome_nav.hello_world_algorithm import HelloWorldAlgorithm


def make_ctx(robot_xy=(0.0, 0.0), step=1.0):
    return ExplorationContext(
        map_data=[],
        map_info=MapInfo(width=0, height=0, resolution=1.0,
                         origin_x=0.0, origin_y=0.0),
        robot_xy=robot_xy,
        blacklist=set(),
        start_xy=None,
        params=ExploreParams(preferred_goal_distance=step),
    )


# --- first call: NEW_GOAL one step ahead in map +x ---

def test_first_call_new_goal():
    algo = HelloWorldAlgorithm()
    decision = algo.next_goal(make_ctx(robot_xy=(2.0, 5.0), step=1.5))
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert decision.xy == (3.5, 5.0)


# --- second call: EXPLORED_DONE immediately, no NO_FRONTIER_PATIENCE wait ---

def test_second_call_done():
    algo = HelloWorldAlgorithm()
    algo.next_goal(make_ctx())
    decision = algo.next_goal(make_ctx())
    assert decision.outcome is GoalOutcome.EXPLORED_DONE
    assert decision.xy is None


def test_stays_done_after_completion():
    algo = HelloWorldAlgorithm()
    algo.next_goal(make_ctx())
    for _ in range(3):
        assert algo.next_goal(make_ctx()).outcome is GoalOutcome.EXPLORED_DONE
