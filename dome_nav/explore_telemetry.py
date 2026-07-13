#!/usr/bin/env python3
# explore_telemetry.py — JSONL telemetry writer for exploration sessions
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import os
import re
import time
from datetime import datetime


class TelemetryWriter:
    def __init__(self, log_fn, map_name: str = "session"):
        telemetry_dir = os.path.join(os.path.expanduser("~"), ".dome", "telemetry")
        os.makedirs(telemetry_dir, exist_ok=True)
        path = os.path.join(telemetry_dir, build_telemetry_filename(map_name))
        self.file = open(path, "w")
        log_fn(f"Telemetry: {path}")

    def write(self, event: str, **kwargs):
        row = {"event": event, "ts": round(time.monotonic(), 3), **kwargs}
        self.file.write(json.dumps(row) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


def build_telemetry_filename(map_name: str, now: datetime | None = None) -> str:
    if now is None:
        now = datetime.now()
    date_str = now.strftime("%d-%b").lower()
    safe_name = re.sub(r"[^a-zA-Z0-9_-]", "_", map_name)[:32]
    base = f"e{safe_name}{date_str}.json"
    telemetry_dir = os.path.join(os.path.expanduser("~"), ".dome", "telemetry")
    if not os.path.exists(os.path.join(telemetry_dir, base)):
        return base
    n = 2
    while True:
        candidate = f"e{safe_name}{date_str}-{n}.json"
        if not os.path.exists(os.path.join(telemetry_dir, candidate)):
            return candidate
        n += 1
