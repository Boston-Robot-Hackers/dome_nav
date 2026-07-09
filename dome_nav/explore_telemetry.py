#!/usr/bin/env python3
# explore_telemetry.py — JSONL telemetry writer for exploration sessions
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import os
import re
import time


class TelemetryWriter:
    def __init__(self, log_fn):
        telemetry_dir = os.path.join(os.path.expanduser("~"), ".dome", "telemetry")
        os.makedirs(telemetry_dir, exist_ok=True)
        next_n = next_run_number(telemetry_dir)
        path = os.path.join(telemetry_dir, f"exp-{next_n:04d}.json")
        self.file = open(path, "w")
        log_fn(f"Telemetry: {path}")

    def write(self, event: str, **kwargs):
        row = {"event": event, "ts": round(time.monotonic(), 3), **kwargs}
        self.file.write(json.dumps(row) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


def next_run_number(telemetry_dir: str) -> int:
    pattern = re.compile(r"^exp-(\d{4})\.json$")
    nums = [
        int(m.group(1))
        for f in os.listdir(telemetry_dir)
        if (m := pattern.match(f))
    ]
    return (max(nums) + 1) if nums else 1
