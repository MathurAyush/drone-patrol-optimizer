"""
vrp.py
======
Production-grade fleet routing on top of Google OR-Tools.

Where Held-Karp gives the *exact* single-drone circuit but dies past ~20 stops
(its state space is O(2^n)), this module solves the realistic problem:

    * MANY drones, not one (Vehicle Routing Problem, not TSP)
    * each with a finite flight RANGE (battery)
    * each zone with a service DEMAND and the fleet a CAPACITY
    * optional TIME WINDOWS per stop (survey must happen within a window)
    * stops may be DROPPED at a penalty when the fleet cannot cover everything

It scales to hundreds of stops in seconds using a cheapest-arc construction
heuristic refined by guided local search — the same engine behind Google Maps
fleet routing. Arc costs come from the terrain-aware distance matrix produced by
the C++ engine (`route_engine matrix`) or the GIS layer, so plans respect real
geography.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ortools.constraint_solver import routing_enums_pb2, pywrapcp


@dataclass
class VrpProblem:
    matrix_km: list[list[float]]          # complete NxN cost matrix (km)
    num_vehicles: int
    depot: int = 0
    vehicle_range_km: float | None = None  # per-drone max route distance
    demands: list[int] | None = None       # per-node service demand
    vehicle_capacity: int | None = None    # per-drone capacity (same units)
    time_windows: list[tuple[float, float]] | None = None  # (earliest, latest) min
    service_time_min: float = 5.0
    speed_kmh: float = 55.0
    drop_penalty_km: float | None = None    # allow dropping a stop at this cost
    first_solution: str = "PATH_CHEAPEST_ARC"
    metaheuristic: str = "GUIDED_LOCAL_SEARCH"
    time_limit_s: int = 5


@dataclass
class VrpRoute:
    vehicle: int
    stops: list[int]            # node indices in visit order (depot .. depot)
    distance_km: float


@dataclass
class VrpSolution:
    routes: list[VrpRoute]
    dropped: list[int] = field(default_factory=list)
    total_distance_km: float = 0.0
    solved: bool = False
    objective: float = 0.0


_SCALE = 1000  # km -> integer metres for the solver


def solve_vrp(p: VrpProblem) -> VrpSolution:
    n = len(p.matrix_km)
    mgr = pywrapcp.RoutingIndexManager(n, p.num_vehicles, p.depot)
    routing = pywrapcp.RoutingModel(mgr)

    # --- arc cost: terrain-aware distance, integer metres ---
    dist = [[int(round(c * _SCALE)) for c in row] for row in p.matrix_km]

    def dist_cb(i, j):
        return dist[mgr.IndexToNode(i)][mgr.IndexToNode(j)]

    transit = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)

    # --- per-drone range (battery) as a Distance dimension ---
    if p.vehicle_range_km is not None:
        routing.AddDimension(
            transit, 0, int(p.vehicle_range_km * _SCALE), True, "Distance")

    # --- fleet capacity / per-zone demand ---
    if p.demands is not None and p.vehicle_capacity is not None:
        def demand_cb(i):
            return p.demands[mgr.IndexToNode(i)]
        dem = routing.RegisterUnaryTransitCallback(demand_cb)
        routing.AddDimensionWithVehicleCapacity(
            dem, 0, [p.vehicle_capacity] * p.num_vehicles, True, "Capacity")

    # --- time windows ---
    if p.time_windows is not None:
        def time_cb(i, j):
            a, b = mgr.IndexToNode(i), mgr.IndexToNode(j)
            travel = p.matrix_km[a][b] / max(p.speed_kmh, 1e-6) * 60.0  # minutes
            return int(round(travel + p.service_time_min))
        tcb = routing.RegisterTransitCallback(time_cb)
        horizon = int(max(w[1] for w in p.time_windows) + 600)
        routing.AddDimension(tcb, horizon, horizon, False, "Time")
        time_dim = routing.GetDimensionOrDie("Time")
        for node, (lo, hi) in enumerate(p.time_windows):
            if node == p.depot:
                continue
            idx = mgr.NodeToIndex(node)
            time_dim.CumulVar(idx).SetRange(int(lo), int(hi))

    # --- allow dropping stops at a penalty (else INFEASIBLE if fleet too small) ---
    if p.drop_penalty_km is not None:
        penalty = int(p.drop_penalty_km * _SCALE)
        for node in range(n):
            if node == p.depot:
                continue
            routing.AddDisjunction([mgr.NodeToIndex(node)], penalty)

    # --- search strategy ---
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = getattr(
        routing_enums_pb2.FirstSolutionStrategy, p.first_solution)
    params.local_search_metaheuristic = getattr(
        routing_enums_pb2.LocalSearchMetaheuristic, p.metaheuristic)
    params.time_limit.FromSeconds(p.time_limit_s)

    sol = routing.SolveWithParameters(params)
    if sol is None:
        return VrpSolution(routes=[], solved=False)

    out = VrpSolution(routes=[], solved=True, objective=sol.ObjectiveValue() / _SCALE)
    visited = set()
    for v in range(p.num_vehicles):
        idx = routing.Start(v)
        stops, route_m = [], 0
        while not routing.IsEnd(idx):
            node = mgr.IndexToNode(idx)
            stops.append(node)
            visited.add(node)
            nxt = sol.Value(routing.NextVar(idx))
            route_m += routing.GetArcCostForVehicle(idx, nxt, v)
            idx = nxt
        stops.append(mgr.IndexToNode(idx))  # return to depot
        if len(stops) > 2:                   # skip empty vehicles
            out.routes.append(VrpRoute(v, stops, route_m / _SCALE))
            out.total_distance_km += route_m / _SCALE

    out.dropped = [i for i in range(n) if i not in visited and i != p.depot]
    return out
