"""Tests for the Python scheduler layer. Requires the built route_engine."""
import math
import os
import shutil
from pathlib import Path

import pytest

from scheduler import Scheduler, threat_to_demand, ENGINE

ENGINE_AVAILABLE = Path(ENGINE).exists()
skip_no_engine = pytest.mark.skipif(
    not ENGINE_AVAILABLE, reason="route_engine not built")


def test_threat_to_demand_monotonic():
    assert threat_to_demand(1) <= threat_to_demand(5) <= threat_to_demand(10)
    assert threat_to_demand(10) == 5.0


def test_default_fleet_has_three_drones():
    s = Scheduler()
    assert len(s.fleet) == 3


def test_demand_table_sorted_desc():
    s = Scheduler()
    rows = s.demand_table()
    assert rows, "expected zones in the dataset"
    scores = [r["threat_score"] for r in rows]
    assert scores == sorted(scores, reverse=True)


@skip_no_engine
def test_patrol_exact_not_longer_than_heuristic():
    s = Scheduler()
    exact = s.patrol_circuit(exact=True)
    approx = s.patrol_circuit(exact=False)
    assert exact["optimal"] is True
    assert exact["length"] <= approx["length"] + 1e-6


@skip_no_engine
def test_dispatch_returns_reachable_drone():
    s = Scheduler()
    res = s.dispatch("Z7")
    assert res["path"][-1] == "Z7"
    assert res["travel_hours"] > 0


@skip_no_engine
def test_allocate_flow_nonnegative():
    s = Scheduler()
    a = s.allocate()
    assert a["max_flow"] >= 0


@skip_no_engine
def test_relay_backbone_connected():
    s = Scheduler()
    r = s.relay_backbone()
    assert r["connected"] is True
