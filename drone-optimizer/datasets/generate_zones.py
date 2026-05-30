#!/usr/bin/env python3
"""
generate_zones.py
=================
Deterministically generates the unified zone graph (zones.json) for the
Smart Drone Patrol Route Optimizer.

The graph is a SINGLE unified model that all four algorithms project from:

    node kinds : COMMAND, BASE, ZONE, CHECKPOINT, RELAY
    edge kinds : FLIGHT_PATH  (used by Dijkstra / A* / TSP / MaxFlow demand side)
                 RADIO_LINK   (used by Kruskal MST relay backbone)

Edges are created by spatial proximity (drones and radios both have finite
range), then connectivity is repaired so that:
    * the FLIGHT subgraph (BASE/ZONE/CHECKPOINT/COMMAND) is connected
    * the RADIO  subgraph (RELAY/COMMAND) is connected

A fixed RNG seed makes the dataset reproducible (research artifacts must be).

Coordinate frame: planar (x, y) in kilometres over a ~44 x 32 km region.
The southern edge (low y) is the secure rear; the northern edge (high y)
is the forward/border line where threat is concentrated.
"""

from __future__ import annotations
import json
import math
import random
from dataclasses import dataclass, field, asdict
from pathlib import Path

SEED = 1947
RNG = random.Random(SEED)

FLIGHT_RANGE_KM = 12.0   # max single-hop flight edge candidate
RADIO_RANGE_KM = 9.0     # max single-hop radio link candidate
FLIGHT_K = 4             # connect each flight node to ~k nearest in range
RADIO_K = 4              # connect each relay to ~k nearest in range
NO_FLY_FRACTION = 0.10   # fraction of candidate flight edges flagged no-fly


@dataclass
class Node:
    id: str
    kind: str            # COMMAND | BASE | ZONE | CHECKPOINT | RELAY
    x: float
    y: float
    label: str
    # kind-specific attributes (only the relevant ones are emitted)
    threat_score: float | None = None      # ZONE: 1..10, drives patrol demand
    activation_cost: float | None = None    # RELAY: cost to power on a tower
    speed_kmh: float | None = None          # BASE: nominal drone cruise speed


@dataclass
class Edge:
    u: str
    v: str
    kind: str            # FLIGHT_PATH | RADIO_LINK
    distance_km: float
    # FLIGHT_PATH attrs
    wind_resistance: float | None = None    # 0..0.6 multiplier on cost
    no_fly: bool | None = None
    # RADIO_LINK attrs
    link_cost: float | None = None          # cost to establish/maintain link


def dist(a: Node, b: Node) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


# --------------------------------------------------------------------------
# 1. Node placement
# --------------------------------------------------------------------------
def build_nodes() -> list[Node]:
    nodes: list[Node] = []

    # Command center: secure rear, centre-south. Also the flow SINK.
    nodes.append(Node("CMD", "COMMAND", 22.0, 3.0, "Command Post"))

    # Two drone bases flanking the rear. 3 drones launch from these.
    nodes.append(Node("BASE_A", "BASE", 9.0, 5.5, "Base Alpha", speed_kmh=60.0))
    nodes.append(Node("BASE_B", "BASE", 35.0, 5.5, "Base Bravo", speed_kmh=60.0))

    # 8 patrol zones across the mid/forward band (y in ~16..28).
    zone_pts = [
        (6, 18), (14, 22), (21, 26), (28, 23),
        (36, 19), (12, 27), (30, 28), (40, 25),
    ]
    for i, (x, y) in enumerate(zone_pts, start=1):
        threat = round(RNG.uniform(2.0, 9.5), 1)
        nodes.append(Node(f"Z{i}", "ZONE", float(x), float(y),
                          f"Zone {i}", threat_score=threat))

    # 10 checkpoints: the routine night-patrol circuit waypoints (TSP).
    cp_pts = [
        (10, 12), (18, 14), (25, 13), (32, 12), (38, 14),
        (8, 24), (20, 30), (27, 19), (34, 27), (42, 18),
    ]
    for i, (x, y) in enumerate(cp_pts, start=1):
        nodes.append(Node(f"C{i}", "CHECKPOINT", float(x), float(y),
                          f"Checkpoint {i}"))

    # 32 relay towers scattered to give radio coverage across the region.
    # Placed on a jittered grid so the radio graph is connectable.
    relay_id = 1
    for gx in range(4, 44, 6):       # x: 4,10,16,22,28,34,40  (7)
        for gy in range(4, 32, 6):   # y: 4,10,16,22,28         (5)
            if relay_id > 32:
                break
            jx = RNG.uniform(-2.2, 2.2)
            jy = RNG.uniform(-2.2, 2.2)
            cost = round(RNG.uniform(40.0, 120.0), 1)
            nodes.append(Node(f"R{relay_id}", "RELAY",
                              round(gx + jx, 2), round(gy + jy, 2),
                              f"Relay {relay_id}", activation_cost=cost))
            relay_id += 1

    return nodes


# --------------------------------------------------------------------------
# 2. Edge construction (proximity-based + connectivity repair)
# --------------------------------------------------------------------------
class UnionFind:
    def __init__(self, items):
        self.p = {i: i for i in items}

    def find(self, a):
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb
            return True
        return False


def k_nearest_edges(nodes, kinds, k, max_range):
    """Candidate undirected edges between nodes of the given kinds."""
    pool = [n for n in nodes if n.kind in kinds]
    seen = set()
    edges = []
    for a in pool:
        ranked = sorted(
            (n for n in pool if n.id != a.id),
            key=lambda n: dist(a, n),
        )
        added = 0
        for b in ranked:
            d = dist(a, b)
            if d > max_range or added >= k:
                if added >= k:
                    break
                continue
            key = frozenset((a.id, b.id))
            if key in seen:
                added += 1
                continue
            seen.add(key)
            edges.append((a, b, d))
            added += 1
    return edges, pool


def ensure_connected(edges, pool, max_extra_range=1e9):
    """Add the shortest cross-component edges until pool is one component."""
    uf = UnionFind([n.id for n in pool])
    for a, b, _ in edges:
        uf.union(a.id, b.id)
    # candidate bridging edges sorted by distance
    bridges = []
    for i, a in enumerate(pool):
        for b in pool[i + 1:]:
            bridges.append((dist(a, b), a, b))
    bridges.sort(key=lambda t: t[0])
    existing = {frozenset((a.id, b.id)) for a, b, _ in edges}
    for d, a, b in bridges:
        if uf.find(a.id) != uf.find(b.id):
            key = frozenset((a.id, b.id))
            if key not in existing and d <= max_extra_range:
                edges.append((a, b, d))
                existing.add(key)
                uf.union(a.id, b.id)
    return edges


def build_edges(nodes):
    by_id = {n.id: n for n in nodes}

    # --- FLIGHT graph: BASE, ZONE, CHECKPOINT, COMMAND ---
    flight_kinds = {"BASE", "ZONE", "CHECKPOINT", "COMMAND"}
    f_raw, f_pool = k_nearest_edges(nodes, flight_kinds, FLIGHT_K, FLIGHT_RANGE_KM)
    f_raw = ensure_connected(f_raw, f_pool)

    flight_edges = []
    n_candidates = len(f_raw)
    no_fly_count = int(n_candidates * NO_FLY_FRACTION)
    no_fly_idx = set(RNG.sample(range(n_candidates), no_fly_count))
    for idx, (a, b, d) in enumerate(f_raw):
        wind = round(RNG.uniform(0.0, 0.6), 3)
        nf = idx in no_fly_idx
        flight_edges.append(Edge(a.id, b.id, "FLIGHT_PATH",
                                 round(d, 3), wind_resistance=wind, no_fly=nf))

    # Guarantee the flight graph is still connected when no-fly edges are
    # removed (no-fly edges are unusable, so connectivity must survive without
    # them). If not, clear no_fly on the cheapest reconnecting edges.
    usable = [(by_id[e.u], by_id[e.v], e.distance_km)
              for e in flight_edges if not e.no_fly]
    uf = UnionFind([n.id for n in f_pool])
    for a, b, _ in usable:
        uf.union(a.id, b.id)
    for e in sorted(flight_edges, key=lambda e: e.distance_km):
        if not e.no_fly:
            continue
        if uf.find(e.u) != uf.find(e.v):
            e.no_fly = False          # re-open a cheap path to keep reachability
            uf.union(e.u, e.v)

    # --- RADIO graph: RELAY + COMMAND ---
    radio_kinds = {"RELAY", "COMMAND"}
    r_raw, r_pool = k_nearest_edges(nodes, radio_kinds, RADIO_K, RADIO_RANGE_KM)
    r_raw = ensure_connected(r_raw, r_pool)

    radio_edges = []
    for a, b, d in r_raw:
        # link cost rises with distance and with rough terrain (random proxy)
        terrain = RNG.uniform(0.8, 1.6)
        link_cost = round(d * terrain * 2.5, 2)
        radio_edges.append(Edge(a.id, b.id, "RADIO_LINK",
                                round(d, 3), link_cost=link_cost))

    return flight_edges, radio_edges


# --------------------------------------------------------------------------
# 3. Serialisation + validation
# --------------------------------------------------------------------------
def node_to_json(n: Node) -> dict:
    out = {"id": n.id, "kind": n.kind, "x": n.x, "y": n.y, "label": n.label}
    if n.threat_score is not None:
        out["threat_score"] = n.threat_score
    if n.activation_cost is not None:
        out["activation_cost"] = n.activation_cost
    if n.speed_kmh is not None:
        out["speed_kmh"] = n.speed_kmh
    return out


def edge_to_json(e: Edge) -> dict:
    out = {"u": e.u, "v": e.v, "kind": e.kind, "distance_km": e.distance_km}
    if e.kind == "FLIGHT_PATH":
        out["wind_resistance"] = e.wind_resistance
        out["no_fly"] = e.no_fly
        # effective traversal cost (km-equivalent) used by Dijkstra/A*/TSP
        out["effective_cost"] = round(e.distance_km * (1.0 + e.wind_resistance), 3)
    else:
        out["link_cost"] = e.link_cost
    return out


def validate(nodes, flight_edges, radio_edges):
    by_id = {n.id: n for n in nodes}

    def connected(node_ids, edge_pairs):
        if not node_ids:
            return True
        uf = UnionFind(node_ids)
        for u, v in edge_pairs:
            uf.union(u, v)
        roots = {uf.find(i) for i in node_ids}
        return len(roots) == 1

    flight_nodes = [n.id for n in nodes
                    if n.kind in {"BASE", "ZONE", "CHECKPOINT", "COMMAND"}]
    usable_flight = [(e.u, e.v) for e in flight_edges if not e.no_fly]
    radio_nodes = [n.id for n in nodes if n.kind in {"RELAY", "COMMAND"}]
    radio_pairs = [(e.u, e.v) for e in radio_edges]

    checks = {
        "flight_connected_usable": connected(flight_nodes, usable_flight),
        "radio_connected": connected(radio_nodes, radio_pairs),
        "all_edge_endpoints_exist": all(
            e.u in by_id and e.v in by_id
            for e in flight_edges + radio_edges
        ),
        "checkpoint_count": sum(1 for n in nodes if n.kind == "CHECKPOINT"),
        "zone_count": sum(1 for n in nodes if n.kind == "ZONE"),
        "relay_count": sum(1 for n in nodes if n.kind == "RELAY"),
    }
    return checks


def main():
    nodes = build_nodes()
    flight_edges, radio_edges = build_edges(nodes)
    checks = validate(nodes, flight_edges, radio_edges)

    doc = {
        "meta": {
            "name": "border-sector-7",
            "description": "Synthetic border-sector zone graph for the "
                           "Smart Drone Patrol Route Optimizer.",
            "seed": SEED,
            "units": "kilometres",
            "region_extent_km": {"x": 44, "y": 32},
            "fleet": {
                "drones": 3,
                "bases": ["BASE_A", "BASE_B"],
                "flow_sink": "CMD",
            },
            "counts": {
                "nodes": len(nodes),
                "flight_edges": len(flight_edges),
                "radio_edges": len(radio_edges),
            },
        },
        "nodes": [node_to_json(n) for n in nodes],
        "edges": [edge_to_json(e) for e in flight_edges]
                 + [edge_to_json(e) for e in radio_edges],
    }

    out_path = Path(__file__).with_name("zones.json")
    out_path.write_text(json.dumps(doc, indent=2))

    print("Validation:")
    for k, v in checks.items():
        print(f"  {k:30s} : {v}")
    print(f"\nWrote {out_path} "
          f"({len(nodes)} nodes, "
          f"{len(flight_edges)} flight edges, "
          f"{len(radio_edges)} radio edges)")


if __name__ == "__main__":
    main()
