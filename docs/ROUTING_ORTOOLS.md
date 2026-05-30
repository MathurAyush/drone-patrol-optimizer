# Fleet Routing with OR-Tools

This is the production-scale routing layer. The C++ `route_engine` provides an
*exact* single-drone circuit (Held-Karp) — provably optimal, but capped at ~20
stops because its state space is `O(n^2 · 2^n)`. Real deployments have a *fleet*
and *hundreds* of stops. This layer solves that with Google OR-Tools.

---

## 1. What it adds over Held-Karp

| | Held-Karp (C++ engine) | OR-Tools VRP (this layer) |
|---|---|---|
| drones | 1 | many |
| stops | ≤ ~20 (exact) | hundreds (metaheuristic) |
| per-drone range | — | yes (battery limit) |
| capacity / demand | — | yes |
| time windows | — | yes |
| droppable stops | — | yes (penalty) |
| guarantee | exact optimum | optimal on small n, near-optimal at scale |

Both consume the **same terrain-aware cost matrix**, so a fleet plan is
consistent with single-drone dispatch.

---

## 2. How costs stay consistent

OR-Tools routes over the distance matrix from the C++ engine:

```
route_engine matrix --graph datasets/zones_gis.json   # terrain-aware NxN
```

`matrix_from_engine()` calls this; `solve_vrp()` consumes it. So multi-drone
routing respects the very same geodesic + terrain-climb costs as everything
else. For scalability work beyond the sparse feature graph, `terrain_matrix()`
builds a complete matrix directly over the DEM for arbitrary waypoints.

---

## 3. The solver

`routing/vrp.py` builds an OR-Tools `RoutingModel`:

- **arc cost** — terrain distance, scaled to integer metres
- **range** — a *Distance* dimension capped at each drone's km range
- **capacity** — a *Capacity* dimension over per-node demand
- **time windows** — a *Time* dimension (travel = distance/speed + service), each
  stop constrained to its `[earliest, latest]`
- **droppable stops** — `AddDisjunction` per node with a penalty, so an
  under-sized fleet yields a *partial* plan instead of INFEASIBLE
- **search** — `PATH_CHEAPEST_ARC` construction + `GUIDED_LOCAL_SEARCH`
  refinement under a wall-clock budget

```python
from routing.vrp import VrpProblem, solve_vrp
from routing.matrix import matrix_from_engine

ids, M = matrix_from_engine("datasets/zones_gis.json")
sol = solve_vrp(VrpProblem(matrix_km=M, num_vehicles=3, depot=0,
                           vehicle_range_km=60.0, drop_penalty_km=500.0,
                           time_limit_s=5))
for r in sol.routes:
    print(f"drone {r.vehicle}: {r.distance_km:.1f} km", [ids[s] for s in r.stops])
```

Via the scheduler / API:

```bash
ZONES_JSON=$PWD/datasets/zones_gis.json python3 python_scheduler/api_server.py
curl "http://127.0.0.1:5000/api/fleet_routes?drones=3&range_km=60"
```

---

## 4. Validation and scalability (measured)

`python -m routing.benchmark`:

**Validation** — single-vehicle OR-Tools vs exact Held-Karp on the GIS
checkpoints: `Held-Karp = 93.49 km, OR-Tools = 93.49 km, gap = 0.00%`. The
scalable solver reproduces the proven optimum, so it is trustworthy.

**Scalability** — terrain-aware multi-drone VRP, 4 drones, 3 s budget:

| stops | matrix build | solve | total km | served | Held-Karp |
|------:|-------------:|------:|---------:|-------:|-----------|
| 20 | 0.01 s | 3 s | 112 | 19/19 | feasible |
| 50 | 0.04 s | 3 s | 174 | 49/49 | 2^50 states (impossible) |
| 100 | 0.18 s | 3 s | 265 | 99/99 | 2^100 states |
| 200 | 0.71 s | 3 s | 370 | 199/199 | 2^200 states |

(Numbers depend on the random waypoint seed and the machine; reproduce with the
benchmark.) The point: OR-Tools stays in seconds where Held-Karp is physically
impossible past ~20 stops.

---

## 5. Honest limitations

- At scale OR-Tools is a **metaheuristic**: near-optimal, not provably optimal.
  Longer `time_limit_s` tightens the gap. Use Held-Karp when n ≤ ~18 and you need
  a certificate of optimality.
- Time windows here use a constant cruise speed; real wind/altitude effects on
  speed are not yet modelled in the time dimension (the cost dimension already
  carries terrain climb energy).
- Battery is modelled as a distance cap; a full energy model would couple
  payload, climb, and speed. The hook (`vehicle_range_km`, demand dimension) is
  in place to extend.
