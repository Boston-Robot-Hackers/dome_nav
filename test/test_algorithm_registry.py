#!/usr/bin/env python3
# test_algorithm_registry.py — pure-Python tests for the algorithm registry.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dome_nav.explorer_manager_node import (
    DEFAULT_ALGORITHM,
    ALGORITHM_REGISTRY,
    resolve_algorithm,
)
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.hello_world_algorithm import HelloWorldAlgorithm


def test_registry_contains_frontier_and_hello():
    assert "frontier" in ALGORITHM_REGISTRY
    assert "hello" in ALGORITHM_REGISTRY
    assert ALGORITHM_REGISTRY["frontier"] is FrontierAlgorithm
    assert ALGORITHM_REGISTRY["hello"] is HelloWorldAlgorithm


def test_resolve_frontier():
    cls = resolve_algorithm("frontier")
    assert cls is FrontierAlgorithm


def test_resolve_hello():
    cls = resolve_algorithm("hello")
    assert cls is HelloWorldAlgorithm


def test_resolve_unknown_falls_back_to_default():
    cls = resolve_algorithm("not_a_real_algorithm")
    assert cls is ALGORITHM_REGISTRY[DEFAULT_ALGORITHM]
    assert cls is FrontierAlgorithm


def test_default_is_frontier():
    assert DEFAULT_ALGORITHM == "frontier"
