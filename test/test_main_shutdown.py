#!/usr/bin/env python3
# test_main_shutdown — node main() functions exit cleanly when a launch is stopped
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
"""Regression test: stopping a launch must not print a traceback.

On SIGTERM or Ctrl-C, rclpy's signal handler shuts the context down and
`rclpy.spin` raises `ExternalShutdownException`. Each `main()` must treat
that as a normal stop and still destroy its node and shut rclpy down.
`rclpy` and the node class are replaced, so no ROS graph is needed.
"""

import importlib
from unittest.mock import Mock

import pytest
from rclpy.executors import ExternalShutdownException

NODE_MAINS = [
    ("dome_nav.explorer_manager_node", "ExplorerManagerNode"),
    ("dome_nav.slam_manager_node", "SlamManagerNode"),
]


@pytest.mark.parametrize("module_name, node_class_name", NODE_MAINS)
@pytest.mark.parametrize("stop", [ExternalShutdownException, KeyboardInterrupt])
def test_main_exits_cleanly_on_stop(monkeypatch, module_name, node_class_name, stop):
    module = importlib.import_module(module_name)
    node = Mock()
    monkeypatch.setattr(module, node_class_name, Mock(return_value=node))
    if hasattr(module, "MultiThreadedExecutor"):
        monkeypatch.setattr(module, "MultiThreadedExecutor", Mock())
    monkeypatch.setattr(module.rclpy, "init", Mock(), raising=False)
    monkeypatch.setattr(module.rclpy, "spin", Mock(side_effect=stop), raising=False)
    try_shutdown = Mock()
    monkeypatch.setattr(module.rclpy, "try_shutdown", try_shutdown, raising=False)

    module.main()

    node.destroy_node.assert_called_once()
    try_shutdown.assert_called_once()
