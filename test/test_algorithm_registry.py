#!/usr/bin/env python3
# test_algorithm_registry.py — pure-Python tests for the algorithm registry.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dome_nav.explorer_manager_node import (
    DEFAULT_ALGORITHM,
    ALGORITHM_REGISTRY,
)
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.hello_world_algorithm import HelloWorldAlgorithm


def test_registry_contains_frontier_and_hello():
    assert "frontier" in ALGORITHM_REGISTRY
    assert "hello" in ALGORITHM_REGISTRY
    assert ALGORITHM_REGISTRY["frontier"] is FrontierAlgorithm
    assert ALGORITHM_REGISTRY["hello"] is HelloWorldAlgorithm


def test_default_is_frontier():
    assert DEFAULT_ALGORITHM == "frontier"
    assert ALGORITHM_REGISTRY[DEFAULT_ALGORITHM] is FrontierAlgorithm
