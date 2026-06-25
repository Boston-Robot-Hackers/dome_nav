#!/usr/bin/env python3
# explore_telemetry.py — JSONL telemetry writer for exploration sessions
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import os
import time


class TelemetryWriter:
    def __init__(self, map_name: str, log_fn):
        telemetry_dir = os.path.join(os.path.expanduser("~"), ".dome", "telemetry")
        os.makedirs(telemetry_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(telemetry_dir, f"{map_name}_{ts}.jsonl")
        self.file = open(path, "w")
        log_fn(f"Telemetry: {path}")

    def write(self, event: str, **kwargs):
        row = {"event": event, "ts": round(time.monotonic(), 3), **kwargs}
        self.file.write(json.dumps(row) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()
