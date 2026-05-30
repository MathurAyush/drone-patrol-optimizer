"""Tests for the OR-Tools routing layer. Run: pytest routing/tests -q"""
import math
from pathlib import Path

import pytest

from routing.vrp import VrpProblem, solve_vrp

REPO = Path(__file__).resolve().parents[2]
ENGINE = REPO / "build" / "route_engine"
GIS_GRAPH = REPO / "datasets" / "zones_gis.json"
HAVE_ENGINE = ENGINE.exists() and GIS_GRAPH.exists()
needs_engine = pytest.mark.skipif(not HAVE_ENGINE,
                                  reason="build engine + GIS graph first")


def _square_matrix():
    # 4 points on a unit square (perimeter optimum = 4).
    import math as m
    d = m.sqrt(2.0)
    return [[0, 1, d, 1], [1, 0, 1, d], [d, 1, 0, 1], [1, d, 1, 0]]


def test_single_vehicle_visits_all():
    sol = solve_vrp(VrpProblem(matrix_km=_square_matrix(), num_vehicles=1,
                               depot=0, time_limit_s=2))
    assert sol.solved
    assert len(sol.routes) == 1
    # depot + 3 stops + return = 5 entries, all 4 nodes covered
    assert set(sol.routes[0].stops) == {0, 1, 2, 3}
    assert sol.total_distance_km == pytest.approx(4.0, abs=1e-6)
    assert not sol.dropped


def test_two_vehicles_share_the_work():
    sol = solve_vrp(VrpProblem(matrix_km=_square_matrix(), num_vehicles=2,
                               depot=0, time_limit_s=2))
    assert sol.solved
    # every non-depot node served exactly once across all routes
    served = [s for r in sol.routes for s in r.stops if s != 0]
    assert sorted(set(served)) == [1, 2, 3]


def test_range_limit_forces_drop():
    # Two far clusters; a tiny range can't reach the far node -> dropped.
    M = [[0, 1, 100], [1, 0, 100], [100, 100, 0]]
    sol = solve_vrp(VrpProblem(matrix_km=M, num_vehicles=1, depot=0,
                               vehicle_range_km=5.0, drop_penalty_km=1000.0,
                               time_limit_s=2))
    assert sol.solved
    assert 2 in sol.dropped          # the unreachable node is dropped
    assert 2 not in [s for r in sol.routes for s in r.stops]


def test_capacity_dimension_respected():
    # 4 customers each demand 1; capacity 2 per vehicle, 2 vehicles -> ok.
    M = [[0, 2, 3, 4, 5], [2, 0, 2, 3, 4], [3, 2, 0, 2, 3],
         [4, 3, 2, 0, 2], [5, 4, 3, 2, 0]]
    sol = solve_vrp(VrpProblem(matrix_km=M, num_vehicles=2, depot=0,
                               demands=[0, 1, 1, 1, 1], vehicle_capacity=2,
                               time_limit_s=2))
    assert sol.solved
    for r in sol.routes:
        assert (len(r.stops) - 2) <= 2   # at most capacity stops per route


@needs_engine
def test_matrix_from_engine_is_square_complete():
    from routing.matrix import matrix_from_engine
    ids, M = matrix_from_engine(str(GIS_GRAPH))
    n = len(ids)
    assert all(len(row) == n for row in M)
    assert all(M[i][i] == 0 for i in range(n))


@needs_engine
def test_ortools_matches_held_karp_optimum():
    import json, subprocess
    from routing.matrix import matrix_from_engine
    ids, M = matrix_from_engine(str(GIS_GRAPH))
    hk = json.loads(subprocess.run(
        [str(ENGINE), "tsp", "--graph", str(GIS_GRAPH)],
        input='{"method":"held_karp"}', capture_output=True, text=True).stdout)["length"]
    sol = solve_vrp(VrpProblem(matrix_km=M, num_vehicles=1, depot=0, time_limit_s=5))
    # OR-Tools single-vehicle should reach the exact optimum on this size.
    assert sol.total_distance_km == pytest.approx(hk, rel=0.01)
