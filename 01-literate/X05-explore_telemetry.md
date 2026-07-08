---
version: "1.2"
generated: "2026-07-08"
---

# explore_telemetry.py — Append-Only Session Logging

Exploration is hard to debug live: goals come and go asynchronously, the map
changes underneath you, and failures happen minutes into a run. `TelemetryWriter`
exists so that after any session you can reconstruct exactly what happened from a
flat, greppable log. It is deliberately tiny — one class, three methods — and has
no ROS dependency, so the node can log without any special infrastructure.

## One file per map per day, append mode

```python
class TelemetryWriter:
    def __init__(self, map_name: str, log_fn):
        telemetry_dir = os.path.join(os.path.expanduser("~"), ".dome", "telemetry")
        os.makedirs(telemetry_dir, exist_ok=True)
        date = time.strftime("%Y%m%d")
        path = os.path.join(telemetry_dir, f"explore-{map_name}-{date}.jsonl")
        self.file = open(path, "a")
        log_fn(f"Telemetry: {path}")
```

The filename encodes both the map name and the date, and the file is opened in
**append** mode. That choice matters: multiple exploration sessions against the
same map on the same day accumulate into one file rather than clobbering each
other, so you can compare runs. The constructor also takes a `log_fn` (the node
passes `self.get_logger().info`) purely so it can announce the path without
importing a logger — a small dependency-injection touch that keeps the module
ROS-free.

## JSON Lines, flushed every write

```python
def write(self, event: str, **kwargs):
    row = {"event": event, "ts": round(time.monotonic(), 3), **kwargs}
    self.file.write(json.dumps(row) + "\n")
    self.file.flush()
```

Each record is one JSON object on its own line (the JSONL format), which is ideal
for this: you can `tail -f` it live, `grep` for an event type, or parse it
line-by-line without loading the whole file. Every row automatically carries an
`event` tag and a monotonic timestamp; the caller supplies everything else as
keyword arguments, so the schema is whatever the node decides per event
(`goal_sent`, `goal_result`, `no_frontier`, `session_start`, `session_end`).

The **flush on every write** is the deliberate reliability choice. Exploration
runs often end with a `kill` or a crash, and an unflushed buffer would lose
exactly the records that explain the ending. Flushing trades a little throughput
(negligible at a few events per second) for the guarantee that what happened is
on disk the instant it happens.

```python
def close(self):
    self.file.close()
```

`close()` is called from the node's shutdown path, after a final `session_end`
record is written — so even Ctrl-C leaves a well-formed log with a terminating
event.

## Observations / possible improvements

- **`monotonic()` timestamps aren't wall-clock.** They're perfect for measuring
  durations within a run but can't be correlated to ROS log timestamps or
  `/clock` after the fact. Adding a wall-clock or sim-time field per row would
  make cross-referencing with Nav2 logs easier.
- **No schema/versioning.** Analysis scripts key off field names that the node
  can change freely. A `schema` or writer-version field would let downstream
  tooling adapt across format changes.
- **Flush-per-write** is the right default here, but if event rates ever climb
  (e.g. per-tick logging) a periodic flush would cost less while keeping most of
  the crash-safety.
