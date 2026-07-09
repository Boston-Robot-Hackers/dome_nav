---
version: "1.3"
generated: "2026-07-09"
---

# explore_telemetry.py — Sequential Per-Run Session Logging

Exploration is hard to debug live: goals come and go asynchronously, the map
changes underneath you, and failures happen minutes into a run. `TelemetryWriter`
exists so that after any session you can reconstruct exactly what happened from a
flat, greppable log. It is deliberately tiny — one class, three methods — and has
no ROS dependency, so the node can log without any special infrastructure.

## One file per run, sequential numbering

```python
class TelemetryWriter:
    def __init__(self, log_fn):
        telemetry_dir = os.path.join(os.path.expanduser("~"), ".dome", "telemetry")
        os.makedirs(telemetry_dir, exist_ok=True)
        next_n = next_run_number(telemetry_dir)
        path = os.path.join(telemetry_dir, f"exp-{next_n:04d}.json")
        self.file = open(path, "w")
        log_fn(f"Telemetry: {path}")
```

Each exploration run gets its own numbered file (`exp-0001.json`, `exp-0002.json`,
…). The constructor scans the telemetry directory for existing `exp-NNNN.json`
files and picks the next unused number. Opening in **write** mode means each run
starts fresh — no accumulation across sessions in the same file. This makes
post-hoc analysis simpler: one file = one run. The constructor also takes a
`log_fn` (the node passes `self.get_logger().info`) purely so it can announce
the path without importing a logger, keeping this module ROS-free.

The numbering logic lives in a module-level helper:

```python
def next_run_number(telemetry_dir: str) -> int:
    pattern = re.compile(r"^exp-(\d{4})\.json$")
    nums = [
        int(m.group(1))
        for f in os.listdir(telemetry_dir)
        if (m := pattern.match(f))
    ]
    return (max(nums) + 1) if nums else 1
```

The walrus-operator assignment `(m := pattern.match(f))` keeps the list
comprehension single-pass: match, filter, and extract in one expression.

## JSON Lines, flushed every write

```python
def write(self, event: str, **kwargs):
    row = {"event": event, "ts": round(time.monotonic(), 3), **kwargs}
    self.file.write(json.dumps(row) + "\n")
    self.file.flush()
```

Each record is one JSON object on its own line (JSONL format), ideal for this
use case: `tail -f`, `grep`, or line-by-line parsing without loading the whole
file. Every row automatically carries an `event` tag and a monotonic timestamp;
the caller supplies everything else as keyword arguments (`goal_sent`,
`goal_result`, `no_frontier`, `session_start`, `session_end`).

The **flush on every write** is the deliberate reliability choice. Exploration
runs often end with a `kill` or a crash, and an unflushed buffer would lose
exactly the records that explain the ending.

```python
def close(self):
    self.file.close()
```

`close()` is called from the node's shutdown path, after a final `session_end`
record is written — so even Ctrl-C leaves a well-formed log with a terminating
event.

## Observations / possible improvements

- **`monotonic()` timestamps aren't wall-clock.** They can't be correlated to
  ROS log timestamps or `/clock` after the fact. A wall-clock field per row
  would help cross-referencing with Nav2 logs.
- **No schema/versioning.** Analysis scripts key off field names the node can
  change freely. A `schema` or writer-version field would let downstream tooling
  adapt across format changes.
- **Sequential numbering wraps at 9999.** The four-digit format overflows silently
  after 9999 runs. Negligible in practice but worth noting.
