---
version: "1.0"
generated: "2026-07-17"
---

# Hello World Algorithm

`hello_world_algorithm.py` is the smallest plugin that still exercises the
`ExplorationAlgorithm` protocol end to end. It ignores the map and drives the
robot one fixed step forward in the map +x direction.

## Purpose

A reference plugin serves two roles:

1. **Template.** New algorithm authors can copy it and replace the decision
   logic without first understanding frontier clustering.
2. **Seam test.** If the manager can run this trivial plugin, the plugin boundary
   is real. If the manager only works with `FrontierAlgorithm`, the boundary is
   leaky.

## State

The algorithm has a single boolean of internal state:

```python
def __init__(self):
    self.emitted = False
```

`emitted` tracks whether the one goal has already been handed out.

## Decision logic

```python
def next_goal(self, ctx: ExplorationContext) -> GoalDecision:
    if self.emitted:
        return GoalDecision.done()
    self.emitted = True
    rx, ry = ctx.robot_xy
    return GoalDecision.new_goal((rx + ctx.params.preferred_goal_distance, ry))
```

- First call: return a goal `preferred_goal_distance` meters ahead in map +x.
- Subsequent calls: return `EXPLORED_DONE`, which ends the session immediately.

The algorithm uses `preferred_goal_distance` as a step size, not as a preference
toward a particular frontier distance. That is acceptable for a wiring demo, but
a more descriptive parameter name would be clearer for this specific plugin.

## No optional hooks

`HelloWorldAlgorithm` implements only the required `next_goal` and the optional
`declare_params` no-op. It has no markers, diagnostics, or extra telemetry. The
node's `getattr`-based hook calls silently skip it.

```python
def declare_params(self, node):
    pass  # no tuning of its own
```

## How it differs from FrontierAlgorithm

| Aspect | FrontierAlgorithm | HelloWorldAlgorithm |
|--------|-------------------|---------------------|
| Map use | Reads every cell | Ignores |
| Done condition | No frontier cells | One goal emitted |
| Hooks | All implemented | None |
| State | Clusters, diag, params | Single boolean |

## Observations for improvement

- The parameter name `preferred_goal_distance` is semantically odd here. For a
  step-goal plugin, `step_distance` would be clearer.
- There is currently no runtime selector in the manager, so this plugin is only
  reachable through constructor injection in tests. Adding an
  `explore_algorithm` ROS param would let users run it from launch.
- A second hello-world variant that publishes its own marker (for example, a
  single arrow showing the chosen direction) would be a good illustration of the
  optional `render_markers` hook.
