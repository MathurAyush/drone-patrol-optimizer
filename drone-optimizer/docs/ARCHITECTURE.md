# Smart Drone Patrol Route Optimizer — System Architecture (Phase 1)

> Status: **Phase 1 of 10 — Architecture + Graph Modeling**
> Target quality bar: deployable research prototype (autonomous-systems R&D).

This document defines *what we are building and how the pieces fit together*,
before any algorithm is implemented. Phases 2–10 fill in the components defined
here. Nothing in this document depends on a specific algorithm's internals — it
defines the **contracts** between modules so each can be built and tested in
isolation.

---

## 1. Problem framing

A sector (border region / campus / city district) is decomposed into discrete
**patrol zones**, **checkpoints**, **drone bases**, a **command post**, and a
field of **relay towers**. A fleet of 3 drones must:

1. Fly a routine circuit covering every checkpoint and return to base.
2. Respond to threat alerts by dispatching the *fastest-reaching* drone.
3. Stay in radio contact with command through a relay backbone.
4. Spread limited flight-hours across zones in proportion to threat.

These four duties map cleanly onto four classical graph problems. The central
design idea of the whole system is:

> **One unified graph, four algorithm-specific projections.**

We do **not** maintain four separate graphs. We maintain one typed graph and
expose *views* of it. This avoids data duplication, keeps a single source of
truth, and makes the "why this algorithm" reasoning explicit because each
algorithm consumes a precisely defined slice of the same world.

---

## 2. The unified graph model

### 2.1 Nodes

| kind | role | key attributes |
|------|------|----------------|
| `COMMAND` | command post; **sink** of the flow network; anchor of the relay backbone | `x, y` |
| `BASE` | drone home; launch/recharge point; **source side** of the flow network | `x, y, speed_kmh` |
| `ZONE` | monitored patrol area carrying a threat score | `x, y, threat_score (1–10)` |
| `CHECKPOINT` | waypoint on the routine TSP circuit | `x, y` |
| `RELAY` | radio relay tower (may be ON or OFF) | `x, y, activation_cost` |

Every node has planar coordinates `(x, y)` in km. Coordinates are not decoration:
they are what makes the **A\*** heuristic admissible (straight-line distance is a
provable lower bound on travel distance) and what lets the dashboard render the
sector to scale.

### 2.2 Edges (typed)

There are two physically different kinds of connection, so edges are typed:

| kind | connects | weight semantics | consumed by |
|------|----------|------------------|-------------|
| `FLIGHT_PATH` | BASE/ZONE/CHECKPOINT/COMMAND | `effective_cost = distance_km × (1 + wind_resistance)` | Dijkstra, A\*, TSP, MaxFlow (demand side) |
| `RADIO_LINK` | RELAY/COMMAND | `link_cost` (distance × terrain factor) | Kruskal MST |

`FLIGHT_PATH` edges additionally carry a `no_fly` flag (restricted airspace /
terrain obstacle). A no-fly edge exists in the data but is **invisible to the
flight projection** — this is how we model obstacles without deleting topology.

Why fold wind into a single `effective_cost` rather than passing two numbers to
the routing algorithms? Because Dijkstra/A\*/TSP are defined over a *scalar*
edge weight. Pre-collapsing `(distance, wind)` into one cost keeps the algorithm
code generic and reusable; if we later want a different cost policy (e.g. weight
battery drain too), we change one projection function, not five algorithms.

### 2.3 Projections (the key abstraction)

A **projection** is a read-only view of the unified graph that selects a node
subset, an edge type, and a weight function. The C++ `Graph` class produces them
on demand:

```
              ┌──────────────────────── unified Graph ───────────────────────┐
              │  nodes: COMMAND BASE ZONE CHECKPOINT RELAY                     │
              │  edges: FLIGHT_PATH (dist, wind, no_fly)  RADIO_LINK (link_cost)│
              └───────────────────────────────────────────────────────────────┘
                 │              │                  │                  │
   flightProjection()   flightProjection()   flowNetwork()      relayProjection()
   weight=eff_cost      weight=eff_cost      capacity model      weight=link_cost
   drop no_fly          drop no_fly          (transformed)       relays + CMD only
                 │              │                  │                  │
            Dijkstra / A*     Held-Karp /      Edmonds-Karp        Kruskal MST
            (Phase 3)         NN-TSP (Ph 4)    (Phase 5)           (Phase 6)
```

| projection | nodes kept | edges kept | weight | used for |
|------------|-----------|-----------|--------|----------|
| `flightProjection()` | BASE, ZONE, CHECKPOINT, COMMAND | `FLIGHT_PATH` with `no_fly == false` | `effective_cost` | Dijkstra, A\*, TSP |
| `relayProjection()` | RELAY, COMMAND | `RADIO_LINK` | `link_cost` | Kruskal MST + bridge analysis |
| `flowNetwork()` | derived super-source → bases → zones → COMMAND | synthetic capacity edges | capacities (flight-hours / demand) | Edmonds-Karp |

The flow network is **not** a sub-slice — it is a *transformation* (Section 4.3),
so it gets its own builder rather than a filter.

---

## 3. Component architecture

Five components, three languages, deliberately decoupled by **JSON contracts**.

```
 datasets/zones.json
        │ (graph definition)
        ▼
┌───────────────────────────┐     stdin/stdout JSON      ┌──────────────────────────┐
│  C++17 ROUTING ENGINE      │ ◀───────────────────────▶ │  PYTHON SCHEDULER (Flask)  │
│  (route_engine CLI)        │   command + args / result │                            │
│                            │                            │  threat → demand convert   │
│  graph/      Graph, proj.  │                            │  drone↔zone assignment     │
│  algorithms/ Dijkstra,A*,  │                            │  TSP retrigger on event    │
│              TSP,MaxFlow,  │                            │  REST API for dashboard    │
│              MST           │                            └────────────┬──────────────┘
│  fleet/      DroneFleet     │                                         │ HTTP/JSON
│  simulation/ scenarios     │                                         ▼
│  utils/      Logger, JsonIO│                            ┌──────────────────────────┐
└───────────────────────────┘                            │  HTML5/JS DASHBOARD        │
        │ JSON results, logs/                             │  live map, TSP animation,  │
        ▼                                                 │  shortest-path overlay,    │
   logs/*.json                                            │  flow bar chart, MST + cut │
                                                          └──────────────────────────┘
```

### 3.1 Why this language split

- **C++17 for the engine.** Held-Karp is `O(n² · 2ⁿ)` in both time and memory;
  Edmonds-Karp does repeated BFS over residual graphs. These are tight inner
  loops where bitmask DP, contiguous memory, and no GC matter. C++ is the right
  tool for the compute core.
- **Python for the scheduler/API.** Orchestration, threat→demand policy, and a
  REST surface are I/O-bound glue. Python's expressiveness and Flask's
  simplicity win here; performance is irrelevant for this layer.
- **HTML5/JS for the dashboard.** Visualization belongs in the browser; canvas
  animation of the TSP circuit and live overlays are natural there.

### 3.2 Why a CLI + JSON boundary (not pybind11)

The engine compiles to a single `route_engine` executable invoked as
`route_engine <command> < input.json > output.json`. The scheduler calls it via
`subprocess`. Rationale:

- **Language-agnostic & testable.** Each side is tested independently against
  the JSON contract; the engine is usable without Python at all.
- **Crash isolation.** A bug in the C++ core cannot take down the API process.
- **Simplicity over micro-performance.** Per-request marshalling cost is
  negligible next to the algorithms themselves.

A pybind11 in-process binding is documented as a future optimization path (it
removes process-spawn overhead) but is intentionally out of scope for the
prototype: the decoupling is worth more than the milliseconds.

### 3.3 The JSON contract (engine commands)

Each command takes the graph plus command-specific parameters and returns a
structured result. Defined now so Phases 3–9 implement against a fixed shape:

| command | input params | output |
|---------|-------------|--------|
| `shortest_path` | `from, to, method: dijkstra\|astar` | `path[], cost, nodes_expanded` |
| `tsp` | `nodes[], method: held_karp\|nearest_neighbour, start` | `tour[], length, optimal: bool` |
| `max_flow` | `capacities, demands` | `max_flow, allocation{}, min_cut_edges[]` |
| `mst` | (uses relay projection) | `mst_edges[], total_cost, bridges[], backup{}` |

`nodes_expanded` is returned so Phase 3 can *empirically* show A\* expanding
fewer nodes than Dijkstra — the learning-mode payoff is built into the contract.

---

## 4. Algorithm → graph mapping (the "why this algorithm")

### 4.1 MST — Kruskal (Phase 6)
**On:** `relayProjection()` (relays + command, `RADIO_LINK`, weight `link_cost`).
**Question:** the minimum-cost set of radio links that keeps every relay (and
thus every drone via the nearest relay) connected to command.
**Why Kruskal over Prim:** the relay graph is *sparse* (proximity-limited links),
and Kruskal's edge-sorting + union-find is natural for sparse graphs and gives us
**bridge/critical-link** detection almost for free (an MST edge whose removal
disconnects the tree is a single point of failure). Prim would work but Kruskal
makes the "which link is critical" analysis cleaner.
**Extensions:** identify bridges; recompute a **backup MST** with a failed relay
removed to show graceful degradation.

### 4.2 Shortest path — Dijkstra + A\* (Phase 3)
**On:** `flightProjection()` (no-fly edges excluded, weight `effective_cost`).
**Question:** when a threat fires at zone X, which drone reaches it fastest and
by what exact path.
**Why both:** Dijkstra is the exact, heuristic-free baseline — correct on any
non-negative graph. A\* adds the straight-line (Euclidean) heuristic, which is
**admissible** because effective cost ≥ geometric distance, so A\* stays optimal
while expanding far fewer nodes on a spatial map. We implement both and report
`nodes_expanded` to demonstrate the speedup empirically. Use Dijkstra when there
is no usable heuristic or you need all-targets distances; use A\* for
point-to-point queries on a map with coordinates (our exact situation).

### 4.3 Max flow / min cut — Edmonds-Karp (Phase 5)
**On:** a **transformed** flow network, not a slice:
```
  SUPER_SOURCE ──cap = drone flight-hours──▶ each BASE/drone
       drone ──cap = reachability──▶ ZONE        (a drone can serve a zone)
       ZONE  ──cap = patrol_demand──▶ COMMAND     (demand = f(threat_score))
```
**Question:** the maximum total patrol coverage achievable given limited
flight-hours, and the bottleneck that caps it.
**Why Edmonds-Karp over plain Ford-Fulkerson:** Edmonds-Karp uses **BFS** to find
shortest augmenting paths, giving a guaranteed `O(V·E²)` bound independent of
capacity magnitudes (plain Ford-Fulkerson can loop badly on awkward capacities).
For a prototype with modest node counts this is more than fast enough and easy to
prove correct. **Min cut** (the saturated edges separating source from sink)
names the limiting sector — i.e. *where to request reinforcement*.

### 4.4 TSP — Held-Karp + Nearest-Neighbour (Phase 4)
**On:** `flightProjection()` restricted to the 10 checkpoints (+ base) with a
metric-closure of shortest paths between them.
**Question:** the minimum-distance closed circuit visiting every checkpoint.
**Why both:** Held-Karp is the exact DP, `O(n²·2ⁿ)` — tractable to ~n≤20 and
correct, used for the planned nightly circuit. Nearest-Neighbour is an `O(n²)`
greedy approximation used for **real-time rerouting** when a new threat mutates
the checkpoint set mid-flight and we cannot afford exponential recompute. The
tradeoff — provable optimality vs. responsiveness — is exactly the operational
choice between "plan the night patrol" and "react now."

---

## 5. Fleet model (Phase 2)

`DroneAgent`: `id, position (node or interpolated x,y), battery%, speed_kmh,
state ∈ {PATROLLING, EN_ROUTE, RETURNING, CHARGING}, assigned_target`.
`DroneFleet`: owns the 3 agents, answers "nearest available drone to zone X"
(by running the routing engine per candidate), and tracks battery/flight-hours
that feed the flow network's source capacities.

---

## 6. Simulation engine (Phase 9) — the six scenarios

| # | scenario | exercises |
|---|----------|-----------|
| 1 | Routine patrol | TSP circuit (Held-Karp) |
| 2 | Single threat alert | Dijkstra dispatch |
| 3 | Multiple simultaneous alerts | Max-flow allocation |
| 4 | Relay tower failure | Backup MST + bridge report |
| 5 | New threat mid-patrol | A\* reroute + NN-TSP |
| 6 | Full fleet stress test | all of the above under load |

The `ScenarioRunner` reads `datasets/scenarios.json`, drives an event queue, and
emits a JSON timeline to `logs/` that the dashboard replays.

---

## 7. Build, test, CI

- **C++:** CMake (`CMakeLists.txt`), C++17, warnings-as-errors. A single
  `route_engine` target + a `tests` target.
- **Python:** `requirements.txt` (Flask, pytest); `pytest` for the scheduler.
- **Tests:** every algorithm gets unit tests with hand-checkable small graphs
  *and* a known-answer case (e.g. MST total cost on a fixed graph, TSP optimal
  on a square). Phase 3+ each ship their tests.
- **CI:** `.github/workflows/ci.yml` — build C++, run C++ tests, run pytest, on
  every push/PR. The repo is GitHub-ready from Phase 2 onward.

---

## 8. Complexity summary (reference)

| algorithm | time | space | our use |
|-----------|------|-------|---------|
| Kruskal MST | `O(E log E)` | `O(V)` | relay backbone (sparse) |
| Dijkstra (binary heap) | `O((V+E) log V)` | `O(V)` | exact dispatch |
| A\* | `O(E)` worst, ≪ in practice | `O(V)` | dispatch on map |
| Edmonds-Karp | `O(V·E²)` | `O(V+E)` | flight-hour allocation |
| Held-Karp TSP | `O(n²·2ⁿ)` | `O(n·2ⁿ)` | exact night circuit (n≤~20) |
| Nearest-Neighbour TSP | `O(n²)` | `O(n)` | real-time reroute |

---

## 9. The dataset: `border-sector-7`

Generated by `datasets/generate_zones.py` (seeded, reproducible) and validated
on build. Current instance:

- **53 nodes:** 1 command · 2 bases · 8 zones · 10 checkpoints · 32 relays
- **45 flight-path edges**, **72 radio-link edges**
- Connectivity **proven** at generation time: the flight subgraph stays
  connected even after removing all no-fly edges, and the radio subgraph is
  fully connected (so an MST spanning every relay exists).

Regenerate or rescale with `python3 datasets/generate_zones.py`.

---

## 10. What Phase 1 fixes for everything downstream

1. The five node kinds and two edge kinds — the single source of truth.
2. The **projection** abstraction — each algorithm's exact input is defined.
3. The **JSON command contract** — engine and scheduler can be built in parallel.
4. The CLI/JSON process boundary — language split with clean isolation.
5. A concrete, validated dataset to test against from day one.

Phase 2 implements `graph/` (Node, Edge, Graph, the projection methods) and the
`fleet/` classes against these contracts.
