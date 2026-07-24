---
version: "1.0"
generated: "2026-07-24"
---

# Appendix — `explore_telemetry.py`

A small, self-contained JSONL telemetry writer for exploration sessions. It is an
appendix because there is nothing algorithmically deep here: it opens a file and
appends one JSON object per line per event. But it is the record the
`explorer_manager_node` leaves behind, and the analysis notebooks read, so it is
worth documenting the two decisions that give it its shape.

## One event per line

Telemetry is written as **JSON Lines** — one self-contained JSON object per line —
rather than a single JSON array. This is the right format for append-only event
logs: a run that crashes mid-session still leaves a valid, parseable file up to
the last flushed line, whereas a truncated JSON array is unparseable. Every row
gets its event name and a monotonic timestamp merged with the caller's fields:

```python
def write(self, event: str, **kwargs):
    row = {"event": event, "ts": round(time.monotonic(), 3), **kwargs}
    self.file.write(json.dumps(row) + "\n")
    self.file.flush()
```

The `flush()` after every write is deliberate: it trades a little throughput for
the guarantee that a killed process loses at most the in-flight line. For a 1 Hz
exploration loop the cost is irrelevant and the durability is worth it.

`time.monotonic()` (not wall-clock) is used so timestamps are immune to NTP steps
and system-clock changes during a run — you can always compute a correct *elapsed*
between two rows.

## Collision-safe filenames

Each session opens a fresh file named for the map and the date, and the naming
guards against overwriting an earlier run from the same day. It sanitizes the map
name to filesystem-safe characters, caps its length, and appends `-2`, `-3`, …
until it finds an unused name:

```python
def build_telemetry_filename(map_name, now=None) -> str:
    date_str = (now or datetime.now()).strftime("%d-%b").lower()
    safe_name = re.sub(r"[^a-zA-Z0-9_-]", "_", map_name)[:32]
    base = f"e{safe_name}{date_str}.json"
    if not os.path.exists(os.path.join(telemetry_dir(), base)):
        return base
    suffix = 2
    while True:
        candidate = f"e{safe_name}{date_str}-{suffix}.json"
        if not os.path.exists(os.path.join(telemetry_dir(), candidate)):
            return candidate
        suffix += 1
```

Files land under `~/.dome/telemetry/`. Factoring `now` as an injectable parameter
(defaulting to `datetime.now()`) is a small testability win — the collision logic
can be unit-tested deterministically.

## Observations and possible improvements

- **The date format has no year.** `e<map>24-jul.json` collides across years
  unless the `-N` suffix logic happens to save you; an ISO `YYYY-MM-DD` stamp
  would sort correctly and never alias.
- **`flush()` per line, no `fsync`.** Data survives a process kill but not a power
  loss (it may sit in the OS page cache). For a Pi that can lose power abruptly, a
  periodic `os.fsync` would harden the log — at a real throughput cost, so only if
  power-loss durability matters.
- **No context manager.** The writer is opened in a constructor and closed by an
  explicit `close()` in the node's `finally`. Making it a context manager (or
  registering `atexit`) would make a leaked file handle harder to cause.
- **The `.json` extension is a slight misnomer** — the content is JSONL, not JSON.
  `.jsonl` would signal the format to tooling.
