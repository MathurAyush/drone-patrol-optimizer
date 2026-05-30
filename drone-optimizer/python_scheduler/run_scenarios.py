"""
run_scenarios.py
================
The simulation engine. Reads datasets/scenarios.json, runs each scenario by
invoking the Scheduler (which drives the C++ engine), prints a readable report,
and writes a JSON timeline per scenario into logs/ for the dashboard to replay.

Run:  python python_scheduler/run_scenarios.py [scenario_id ...]
      (no args = run all six)
"""

from __future__ import annotations

import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from scheduler import Scheduler, EngineError

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = REPO_ROOT / "datasets" / "scenarios.json"
LOGS = REPO_ROOT / "logs"


def banner(text: str) -> None:
    line = "=" * 64
    print(f"\n{line}\n{text}\n{line}")


def run_patrol(s: Scheduler, p: dict) -> dict:
    exact = s.patrol_circuit(exact=True)
    approx = s.patrol_circuit(exact=False)
    gap = (approx["length"] - exact["length"]) / exact["length"] * 100
    print(f"  exact (Held-Karp) : {exact['length']:.2f} km over {len(exact['tour'])} stops")
    print(f"  approx (NN)       : {approx['length']:.2f} km  (+{gap:.1f}% gap)")
    print(f"  tour: {' -> '.join(exact['tour'])} -> {exact['tour'][0]}")
    return {"exact": exact, "approx": approx, "gap_pct": gap}


def run_dispatch(s: Scheduler, p: dict) -> dict:
    zone = p.get("zone", "Z1")
    a = s.dispatch(zone, method="astar")
    d = s.dispatch(zone, method="dijkstra")
    print(f"  target {zone}: nearest drone {a['drone']} from {a['from']}")
    print(f"  distance {a['cost_km']:.2f} km, ETA {a['travel_hours']*60:.1f} min")
    print(f"  A* expanded {a['nodes_expanded']} nodes vs Dijkstra {d['nodes_expanded']} "
          f"(same path cost {a['cost_km']:.2f} == {d['cost_km']:.2f})")
    return {"astar": a, "dijkstra": d}


def run_allocate(s: Scheduler, p: dict) -> dict:
    a = s.allocate()
    print(f"  max coverage = {a['max_flow']} flight-hours")
    short = [z for z in a["zone_coverage"] if z["shortfall"] > 1e-6]
    print(f"  fully covered: {len(a['zone_coverage'])-len(short)}/{len(a['zone_coverage'])} zones")
    for z in short:
        print(f"    SHORTFALL {z['zone']} (threat {z['threat_score']}): "
              f"{z['covered']:.1f}/{z['demand']:.1f}h")
    print(f"  bottleneck (min-cut) edges: {len(a['min_cut_edges'])}")
    return a


def run_relay(s: Scheduler, p: dict) -> dict:
    failed = p.get("failed", "")
    base = s.relay_backbone()
    after = s.relay_backbone(failed_relay=failed) if failed else base
    print(f"  baseline backbone: cost {base['total_cost']:.1f}, "
          f"{len(base['mst_edges'])} links, {len(base['critical_links'])} critical")
    if failed:
        print(f"  after {failed} fails: connected={after['connected']}, "
              f"cost {after['total_cost']:.1f}, components {after['components']}")
    return {"baseline": base, "after_failure": after, "failed": failed}


def run_reroute(s: Scheduler, p: dict) -> dict:
    zone = p.get("zone", "Z8")
    disp = s.dispatch(zone, method="astar")
    # NN reroute: insert the new threat into the checkpoint set and re-solve fast
    approx = s.patrol_circuit(exact=False)
    print(f"  new threat {zone}: A* dispatch {disp['drone']} "
          f"({disp['cost_km']:.2f} km, expanded {disp['nodes_expanded']})")
    print(f"  NN reroute circuit length: {approx['length']:.2f} km "
          f"(O(n^2), suitable for real-time)")
    return {"dispatch": disp, "reroute": approx}


def run_fleet(s: Scheduler, p: dict) -> dict:
    n = int(p.get("num_drones", 3))
    rng = p.get("range_km")
    res = s.multi_drone_patrol(num_drones=n, range_km=rng, time_limit_s=3)
    single = s.patrol_circuit(exact=True)
    print(f"  fleet of {n} drones (range {rng} km): "
          f"{len(res['routes'])} routes, total {res['total_distance_km']:.1f} km")
    for r in res["routes"]:
        print(f"    drone {r['drone']}: {r['distance_km']:.1f} km, "
              f"{len(r['stops'])-2} stops")
    if res["dropped"]:
        print(f"    dropped (out of range): {res['dropped']}")
    print(f"  vs single-drone exact circuit: {single['length']:.1f} km "
          f"(one drone flies the whole route)")
    return {"fleet": res, "single_drone_km": single["length"]}


def run_stress(s: Scheduler, p: dict) -> dict:
    t0 = time.perf_counter()
    out = {}
    out["patrol"] = s.patrol_circuit(exact=True)
    out["allocate"] = s.allocate()
    out["relay"] = s.relay_backbone()
    zones = [z["zone"] for z in s.demand_table()]
    out["dispatches"] = {z: s.dispatch(z, method="astar")["cost_km"] for z in zones}
    dt = (time.perf_counter() - t0) * 1000
    print(f"  ran patrol + allocate + relay + {len(zones)} dispatches in {dt:.0f} ms")
    print(f"  patrol {out['patrol']['length']:.1f} km · "
          f"coverage {out['allocate']['max_flow']}h · "
          f"backbone {out['relay']['total_cost']:.0f}")
    out["elapsed_ms"] = dt
    return out


RUNNERS = {
    "patrol": run_patrol, "dispatch": run_dispatch, "allocate": run_allocate,
    "relay": run_relay, "reroute": run_reroute, "stress": run_stress,
    "fleet_vrp": run_fleet,
}


def main(argv: list[str]) -> int:
    spec = json.loads(SCENARIOS.read_text())["scenarios"]
    wanted = {int(a) for a in argv} if argv else {sc["id"] for sc in spec}
    LOGS.mkdir(exist_ok=True)
    s = Scheduler()

    summary = []
    for sc in spec:
        if sc["id"] not in wanted:
            continue
        banner(f"SCENARIO {sc['id']} — {sc['name']}")
        print(f"  {sc['description']}")
        try:
            result = RUNNERS[sc["type"]](s, sc.get("params", {}))
            status = "ok"
        except EngineError as e:
            print(f"  ERROR: {e}")
            result, status = {"error": str(e)}, "error"
        timeline = {
            "scenario_id": sc["id"], "name": sc["name"], "type": sc["type"],
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "status": status, "result": result,
        }
        out = LOGS / f"scenario_{sc['id']}.json"
        out.write_text(json.dumps(timeline, indent=2))
        summary.append((sc["id"], sc["name"], status))

    banner("SUMMARY")
    for sid, name, status in summary:
        mark = "PASS" if status == "ok" else "FAIL"
        print(f"  [{mark}] scenario {sid}: {name}")
    print(f"\n  timelines written to {LOGS}/")
    return 0 if all(st == "ok" for _, _, st in summary) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
