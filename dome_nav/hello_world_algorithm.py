#!/usr/bin/env python3
# hello_world_algorithm.py — minimal reference exploration algorithm
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

# Minimal reference algorithm and template for writing a new one. Only next_goal
# is required (see explore_context.py); the optional viz/diag hooks are omitted.

from dome_nav.explore_context import ExplorationContext, GoalDecision


class HelloWorldAlgorithm:
    """Emits ONE goal a step ahead of the robot (map +x), then done. Ignores the map."""

    def __init__(self):
        self.emitted = False
        # Own step distance (F34 T03): preferred_goal_distance moved to
        # FrontierParams, so this algorithm no longer reads it off ctx.params.
        self.step_distance_m = 1.0

    def declare_params(self, node):
        node.declare_parameter("preferred_goal_distance", self.step_distance_m)
        self.step_distance_m = node.get_parameter("preferred_goal_distance").value

    def next_goal(self, ctx: ExplorationContext) -> GoalDecision:
        if self.emitted:
            return GoalDecision.done()
        self.emitted = True
        # step_distance_m metres straight ahead; heading ignored.
        rx, ry = ctx.robot_xy
        return GoalDecision.new_goal((rx + self.step_distance_m, ry))
