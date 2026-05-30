"""
scheduler.py
============
The orchestration layer. It does NOT re-implement the algorithms — it shells out
to the compiled C++ `route_engine` over the JSON command contract
(docs/ARCHITECTURE.md section 3.3) and adds operational policy on top:

  * threat-score -> patrol-hour demand conversion
  * choosing which engine command answers which operational question
  * assembling a fleet definition to pass to the engine

Keeping the algorithms in C++ and the policy in Python is the whole point of the
split: the policy here is readable and easy to change; the compute is fast.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# Resolve the engine + dataset relative to the repo root, overridable by env.
REPO_ROOT = Path(__file__).resolve().parents[1]
# Make top-level packages (routing/, gis/) importable regardless of cwd.
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
ENGINE = os.environ.get("ROUTE_ENGINE", str(REPO_ROOT / "build" / "route_engine"))
DEFAULT_GRAPH = os.environ.get("ZONES_JSON", str(REPO_ROOT / "datasets" / "zones.json"))


class EngineError(RuntimeError):
    pass


def call_engine(command: str, payload: dict[str, Any] | None = None,
                graph: str = DEFAULT_GRAPH) -> dict[str, Any]:
    """Invoke route_engine <command> with JSON params on stdin."""
    if not Path(ENGINE).exists():
        raise EngineError(
            f"route_engine not found at {ENGINE!r}. Build it first:\n"
            f"  cmake -S . -B build && cmake --build build")
    proc = subprocess.run(
        [ENGINE, command, "--graph", graph],
        input=json.dumps(payload or {}),
        capture_output=True, text=True,
    )
    if proc.returncode != 0 and not proc.stdout.strip():
        raise EngineError(f"engine failed ({proc.returncode}): {proc.stderr.strip()}")
    try:
        out = json.loads(proc.stdout)
    except json.JSONDecodeError as exc:
        raise EngineError(f"bad engine output: {exc}\n{proc.stdout[:500]}") from exc
    if isinstance(out, dict) and "error" in out:
        raise EngineError(out["error"])
    return out


# --------------------------------------------------------------------------
# Threat -> demand policy
# --------------------------------------------------------------------------
def threat_to_demand(threat_score: float) -> float:
    """Convert a 1..10 threat score into a patrol-hour demand.

    Mirrors the C++ zoneDemand() so Python and the engine agree. The policy is
    deliberately simple and lives in one place; tune it here.
    """
    return math.ceil(threat_score) * 0.5


@dataclass
class Drone:
    id: str
    position: str
    home_base: str = ""
    battery_pct: float = 100.0
    speed_kmh: float = 60.0
    flight_hours_left: float = 6.0

    def to_payload(self) -> dict[str, Any]:
        return {
            "id": self.id, "position": self.position,
            "home_base": self.home_base or self.position,
            "battery_pct": self.battery_pct, "speed_kmh": self.speed_kmh,
            "flight_hours_left": self.flight_hours_left,
        }


@dataclass
class Scheduler:
    graph: str = DEFAULT_GRAPH
    fleet: list[Drone] = field(default_factory=list)

    def __post_init__(self):
        if not self.fleet:
            self.fleet = self._default_fleet()

    def _default_fleet(self) -> list[Drone]:
        g = json.loads(Path(self.graph).read_text())
        bases = [n["id"] for n in g["nodes"] if n["kind"] == "BASE"]
        b0 = bases[0] if bases else ""
        b1 = bases[1] if len(bases) > 1 else b0
        return [
            Drone("D1", b0, b0, 100, 60, 6.0),
            Drone("D2", b1, b1, 100, 60, 6.0),
            Drone("D3", b0, b0, 95, 55, 5.0),
        ]

    def _fleet_payload(self) -> dict[str, Any]:
        return {"drones": [d.to_payload() for d in self.fleet]}

    # ---- operational queries -------------------------------------------
    def patrol_circuit(self, exact: bool = True) -> dict[str, Any]:
        """Routine night patrol: TSP over all checkpoints."""
        method = "held_karp" if exact else "nearest_neighbour"
        return call_engine("tsp", {"method": method}, self.graph)

    def dispatch(self, target_zone: str, method: str = "astar") -> dict[str, Any]:
        """Single threat alert: fastest drone + path to the target.

        We ask the engine for each drone's shortest path, then pick the one with
        the smallest travel time = path_cost / speed.
        """
        best = None
        for d in self.fleet:
            res = call_engine("shortest_path",
                              {"from": d.position, "to": target_zone,
                               "method": method}, self.graph)
            if not res.get("found"):
                continue
            travel_h = res["cost"] / max(d.speed_kmh, 1e-6)
            cand = {"drone": d.id, "from": d.position, "travel_hours": travel_h,
                    "cost_km": res["cost"], "path": res["path"],
                    "nodes_expanded": res.get("nodes_expanded")}
            if best is None or travel_h < best["travel_hours"]:
                best = cand
        if best is None:
            raise EngineError(f"no drone can reach {target_zone}")
        return best

    def allocate(self) -> dict[str, Any]:
        """Multiple alerts: max-flow flight-hour allocation across zones."""
        return call_engine("max_flow", self._fleet_payload(), self.graph)

    def relay_backbone(self, failed_relay: str = "") -> dict[str, Any]:
        """Minimum-cost relay network (optionally after a tower failure)."""
        payload: dict[str, Any] = {"with_backup": True}
        if failed_relay:
            payload["failed_relay"] = failed_relay
        return call_engine("mst", payload, self.graph)

    def multi_drone_patrol(self, num_drones: int = 3,
                           range_km: float | None = None,
                           time_limit_s: int = 3) -> dict[str, Any]:
        """Fleet routing (VRP) via OR-Tools over the terrain-aware matrix.

        Unlike patrol_circuit() (single-drone exact TSP, <=20 stops), this splits
        the work across a fleet with per-drone range limits and scales to
        hundreds of stops. Lazily imports the routing layer so the rest of the
        scheduler works even without OR-Tools installed.
        """
        from routing.matrix import matrix_from_engine
        from routing.vrp import VrpProblem, solve_vrp
        ids, matrix = matrix_from_engine(self.graph)
        prob = VrpProblem(matrix_km=matrix, num_vehicles=num_drones, depot=0,
                          vehicle_range_km=range_km, drop_penalty_km=1000.0,
                          time_limit_s=time_limit_s)
        sol = solve_vrp(prob)
        return {
            "num_drones": num_drones,
            "range_km": range_km,
            "solved": sol.solved,
            "total_distance_km": sol.total_distance_km,
            "routes": [{"drone": r.vehicle, "distance_km": r.distance_km,
                        "stops": [ids[s] for s in r.stops]} for r in sol.routes],
            "dropped": [ids[i] for i in sol.dropped],
        }

    def demand_table(self) -> list[dict[str, Any]]:
        """Threat -> demand conversion for every zone (for the dashboard)."""
        g = json.loads(Path(self.graph).read_text())
        rows = []
        for n in g["nodes"]:
            if n["kind"] == "ZONE":
                ts = n.get("threat_score", 0.0)
                rows.append({"zone": n["id"], "threat_score": ts,
                             "demand_hours": threat_to_demand(ts)})
        return sorted(rows, key=lambda r: -r["threat_score"])


if __name__ == "__main__":
    s = Scheduler()
    print("Demand table:")
    for r in s.demand_table():
        print(f"  {r['zone']:4s} threat={r['threat_score']:.1f} "
              f"demand={r['demand_hours']:.1f}h")
    print("\nMax-flow allocation:")
    a = s.allocate()
    print(f"  max coverage = {a['max_flow']} flight-hours")
    print(f"  bottleneck edges = {len(a['min_cut_edges'])}")
