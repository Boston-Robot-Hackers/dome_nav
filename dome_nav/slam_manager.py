#!/usr/bin/env python3
# slam_manager.py — pure Python SLAM state: map readiness, save gating, path setup
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os


class SlamManager:
    def __init__(self, map_persist_path: str):
        self.map_persist_path = map_persist_path
        self.map_ready = False

    def on_map_received(self) -> str:
        if not self.map_ready:
            self.map_ready = True
        return "mapping"

    def should_save(self) -> bool:
        return self.map_ready

    def ensure_map_dir(self):
        parent = os.path.dirname(self.map_persist_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
