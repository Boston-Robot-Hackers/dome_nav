---
version: "1.3"
generated: "2026-07-08"
---

# nav_manager.py — Pure Navigation Logic

`NavManager` is the brain of point-to-point navigation (Mode B: "go to the
chair") with none of the ROS plumbing. It parses intents, remembers the set of
known targets, picks the nearest matching one, judges whether localization has
converged, and formats status strings. Every method takes plain data and returns
plain data, which is the whole point: the ROS node (`nav_manager_node.py`) is a
thin adapter around this, and this class is exhaustively unit-testable without a
running graph.

## Remembering targets

Targets come from the vision system as a JSON array of dicts. `on_targets`
validates and stores them, refusing anything that isn't a list:

```python
def on_targets(self, json_str: str) -> bool:
    try:
        result = json.loads(json_str)
    except json.JSONDecodeError:
        return False
    if not isinstance(result, list):
        return False
    self.confirmed_targets = result
    return True
```

The boolean return lets the node log a warning on bad input rather than crash —
validation at the boundary, then trust downstream. Each target dict is expected
to look like `{"label": str, "xyz_world": [x, y, z], ...}`.

## Parsing intents

`parse_intent` is the counterpart on the command side. It accepts only the two
navigation intents and rejects everything else (including non-dict JSON) by
returning `None`:

```python
def parse_intent(self, json_str: str) -> tuple[str, dict] | None:
    ...
    action = intent.get("name", "")
    if action not in ("navigation_go", "navigation_cancel"):
        return None
    return (action, intent)
```

Note it reads the `"name"` key — the dome_control intent contract. (This was
once `"action"`, a mismatch that silently dropped every command; see the F09
history.) Returning the whole intent dict alongside the action lets the caller
pull `slots.label` without re-parsing.

## Choosing the nearest match

Given a label, `find_nearest_confirmed` returns the closest target of that label
to the robot — with a deliberate fallback:

```python
def find_nearest_confirmed(self, label, robot_xy):
    matches = [t for t in self.confirmed_targets if t.get("label") == label]
    if not matches:
        return None
    if robot_xy is None:
        return matches[0]
    rx, ry = robot_xy
    def dist(target):
        xyz = target.get("xyz_world", [0.0, 0.0, 0.0])
        return math.sqrt((xyz[0] - rx) ** 2 + (xyz[1] - ry) ** 2)
    return min(matches, key=dist)
```

The `robot_xy is None` branch is the interesting design decision: if no pose is
available, rather than *block* navigation, it falls back to the first match.
Better to drive toward *a* chair than to refuse because localization hiccuped.

## Judging localization

`check_localization` turns an AMCL covariance matrix into a human-friendly
(status, score) pair. The covariance is a 36-element row-major 6×6; only the
`xx` (index 0) and `yy` (index 7) diagonal terms matter for a 2D base:

```python
def check_localization(self, covariance):
    worst = max(covariance[0], covariance[7])
    score = min(1.0, max(0.0, 1.0 - worst / self.MAX_COV))
    status = "converged" if score >= self.CONVERGED_THRESHOLD else "localizing"
    return (status, score)
```

Score is `1 − worst/MAX_COV`, clamped to `[0, 1]`: 1.0 is fully converged, 0.0 is
lost. `MAX_COV = 1.0` is the "lost" ceiling and `CONVERGED_THRESHOLD = 0.9` is
the cutoff for calling it good. The clamp matters — without it a covariance above
`MAX_COV` would produce a negative "score." (This clamp was issue I07.)

```python
def navigate_status(self, label, target):
    return f"no_target:{label}" if target is None else f"navigating:{label}"
```

`navigate_status` is just the string contract the node publishes.

## Observations / possible improvements

- **`MAX_COV = 1.0` is a magic ceiling.** It works, but a real covariance can
  exceed 1.0 m² when badly lost, and the clamp then just pins the score at 0.
  Whether 1.0 is the right normalizer is a tuning question worth revisiting on
  hardware.
- **Target dicts are duck-typed with `.get(..., default)` everywhere.** Forgiving,
  but a malformed target (missing `xyz_world`) silently sorts as if at the
  origin. If the vision contract is stable, validating target shape once in
  `on_targets` would catch that at the boundary.
- **`find_nearest_confirmed` recomputes distances on every call.** Negligible for
  a handful of targets; irrelevant unless the target list grows large.
