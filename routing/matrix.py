"""
matrix.py
=========
Distance-matrix sources for the VRP solver.

  matrix_from_engine() — pulls the terrain-aware metric closure from the C++
      `route_engine matrix` command, so OR-Tools routes over exactly the costs
      the rest of the system uses (consistency between single- and multi-drone).

  terrain_matrix()      — builds a complete NxN matrix directly over the DEM for
      an arbitrary set of survey waypoints. Used for scalability work where the
      sparse feature graph has too few nodes to stress the solver. Drones fly
      point-to-point over open terrain, so direct climb-aware cost is the right
      model here.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
ENGINE = Path(os.environ.get("ROUTE_ENGINE", str(REPO / "build" / "route_engine")))


def matrix_from_engine(graph_path: str, nodes: list[str] | None = None
                       ) -> tuple[list[str], list[list[float]]]:
    payload = {"nodes": nodes} if nodes else {}
    proc = subprocess.run(
        [str(ENGINE), "matrix", "--graph", graph_path],
        input=json.dumps(payload), capture_output=True, text=True)
    if proc.returncode != 0 and not proc.stdout.strip():
        raise RuntimeError(f"engine matrix failed: {proc.stderr}")
    d = json.loads(proc.stdout)
    if not d.get("complete", False):
        raise RuntimeError("matrix incomplete: node set not mutually reachable")
    return d["ids"], d["matrix_km"]


def terrain_matrix(waypoints: list[dict], terrain) -> list[list[float]]:
    """Complete climb-aware cost matrix (km) between survey waypoints.

    waypoints: list of {"id","lon","lat"}; terrain: gis.terrain.Terrain
    """
    n = len(waypoints)
    m = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            a, b = waypoints[i], waypoints[j]
            c = terrain.climb_cost_km(a["lon"], a["lat"], b["lon"], b["lat"])
            m[i][j] = c["effective_km"]
    return m


def random_waypoints(n: int, bounds: tuple[float, float, float, float],
                     seed: int = 1947) -> list[dict]:
    """n survey waypoints uniformly within (west, south, east, north)."""
    west, south, east, north = bounds
    rng = np.random.default_rng(seed)
    return [{"id": f"W{i}",
             "lon": float(rng.uniform(west, east)),
             "lat": float(rng.uniform(south, north))}
            for i in range(n)]
