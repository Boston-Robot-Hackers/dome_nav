---
version: "1.0"
generated: "2026-06-25"
---

# ExploreTelemetry — JSONL Session Logger

## Overview

`explore_telemetry.py` is a thin file-writing helper extracted from
`explore_manager_node` to keep telemetry concerns out of the node. It opens a
JSONL file per exploration session and writes one event per line.

## File Naming

Each session gets its own file named after the map and start time:

```
~/.dome/telemetry/<map_name>_<YYYYMMDD_HHMMSS>.jsonl
```

This makes it easy to correlate a telemetry file with the map it was built
from and to `tail -f` the latest session.

## Event Format

Each line is a JSON object with `event`, `ts` (monotonic seconds), and
event-specific fields:

```jsonl
{"event": "session_start", "ts": 100.1, "map_name": "basement", "start_xy": [0.0, 0.0], "params": {...}}
{"event": "goal_sent",     "ts": 102.3, "goal_num": 1, "goal_xy": [1.2, 3.4], "dist_m": 2.1, ...}
{"event": "goal_result",   "ts": 106.8, "goal_num": 1, "status": "reached", "elapsed_s": 4.5, ...}
{"event": "no_frontier",   "ts": 110.0, "tick": 1, "patience": 8, "blacklisted": 3}
{"event": "session_end",   "ts": 112.0, "reason": "done", "goals_sent": 5, "reached": 4, "failed": 1}
```

`ts` uses `time.monotonic()` — wall-clock-relative, not absolute. Sufficient
for computing elapsed times within a session; not suitable for cross-session
comparison.

## Usage

```python
telemetry = TelemetryWriter(map_name, self.get_logger().info)
telemetry.write("goal_sent", goal_num=1, goal_xy=[1.2, 3.4])
telemetry.close()
```

## Observations

- `time.monotonic()` resets between process restarts — timestamps are only
  meaningful within one session. If cross-session wall time is needed, switch
  to `time.time()` or include an ISO timestamp field.
- `flush()` on every write adds latency but ensures no events are lost if the
  node crashes. For high-frequency logging, batch writes and flush on a timer.
