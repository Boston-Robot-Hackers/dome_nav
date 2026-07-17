#!/usr/bin/env python3
# hello_world_algorithm.py — minimal reference exploration algorithm
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

# The smallest thing that plugs into the F12 pluggable seam and still drives the
# robot. Read this as the template for writing a new exploration algorithm.
#
# The contract (see explore_context.py -> ExplorationAlgorithm protocol):
#   - implement next_goal(ctx) -> GoalDecision
#       GoalDecision.new_goal((x, y)) to send a world-frame goal;
#       GoalDecision.done() when finished (ends the session immediately);
#       GoalDecision.blocked() when targets exist but none are usable this tick.
#   - that is the ONLY required method. Visualization and diagnostics are
#     optional hooks (render_markers, exhaustion_report, failure_report,
#     telemetry_extra); an algorithm with nothing to show, like this one, simply
#     omits them — no faked cluster state required.
#
# The node calls next_goal once (or a few times per tick if a goal maps outside
# the global costmap) whenever it has NO active goal. It pursues each returned
# goal to completion before asking again. GoalDecision.done() ends exploration
# straight away — no NO_TARGET_PATIENCE wait.

from dome_nav.explore_context import ExplorationContext, GoalDecision


class HelloWorldAlgorithm:
    # Emits ONE goal a fixed step ahead of the robot (map +x), then declares done
    # forever after. Ignores the map entirely — no frontier detection, no scan,
    # no costmap reasoning. Purely a wiring demonstration.

    def __init__(self):
        # Session state: have we already handed out our one goal?
        self.emitted = False

    def declare_params(self, node):
        # No tuning of its own — needs only the shared params the node declares.
        pass

    def next_goal(self, ctx: ExplorationContext) -> GoalDecision:
        # Already sent our single goal -> we are finished. EXPLORED_DONE ends the
        # session immediately; no NO_TARGET_PATIENCE wait.
        if self.emitted:
            return GoalDecision.done()
        self.emitted = True
        # One goal, preferred_goal_distance metres straight ahead in the map +x
        # direction. Heading is ignored (the node sends orientation w=1.0).
        rx, ry = ctx.robot_xy
        step = ctx.params.preferred_goal_distance
        return GoalDecision.new_goal((rx + step, ry))
