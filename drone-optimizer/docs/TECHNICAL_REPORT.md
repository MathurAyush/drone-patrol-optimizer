# Technical Report — Smart Drone Patrol Route Optimizer

**Classification of the work:** graph-algorithm research prototype for autonomous
multi-agent patrol planning on a synthetic sector graph. Three-language system
(C++17 engine, Python scheduler/API, HTML5/JS dashboard) exercising four
classical algorithm families against one unified graph model.

All numbers below are measured on the bundled `border-sector-7` dataset
(53 nodes: 1 command, 2 bases, 8 zones, 10 checkpoints, 32 relays; 45 flight
edges, 72 radio edges) and are reproducible with the commands in section 7.

---

## 1. Problem and approach

A sector is decomposed into patrol zones, checkpoints, drone bases, a command
post, and relay towers. The fleet must (1) fly a routine circuit, (2) respond to
threats fastest, (3) stay in radio contact, and (4) spread limited flight-hours
by threat. Each duty is a classical graph problem. The architectural spine is
**one typed graph, projected per algorithm** — detailed in `ARCHITECTURE.md`.
This report focuses on the algorithms, their justification, complexity, and
measured behaviour.

---

## 2. Algorithm 1 — Shortest path (Dijkstra + A\*)

**Problem.** Threat at zone X; which drone reaches it fastest and by what path,
respecting no-fly airspace.

**Why these.** Dijkstra is the exact, heuristic-free baseline, correct on any
non-negative graph. A\* adds the straight-line (Euclidean) heuristic. Because
`effective_cost = distance × (1 + wind) ≥ distance`, straight-line distance never
overestimates the true remaining cost — the heuristic is **admissible**, so A\*
returns the same optimal cost as Dijkstra while expanding fewer nodes.

**Complexity.** Dijkstra with a binary heap: `O((V+E) log V)` time, `O(V)` space.
A\*: same worst case, but far fewer expansions in practice on spatial graphs.

**Measured (BASE_A → Z7).** Both return cost **41.65 km**. Node expansions:
Dijkstra **15**, A\* **6** — a 2.5× reduction in frontier work for an identical
answer. This is the empirical payoff of the heuristic and is surfaced in the
engine's `nodes_expanded` field and the dashboard readout.

**Design tradeoff.** Use Dijkstra when there is no usable heuristic or you need
distances to many targets at once; use A\* for point-to-point queries on a graph
with coordinates — exactly the dispatch case.

---

## 3. Algorithm 2 — TSP (Held-Karp + Nearest-Neighbour)

**Problem.** Routine night patrol: visit all 10 checkpoints and return to base
at minimum distance. The flight graph is sparse, so we first build the **metric
closure** of the checkpoint set via Dijkstra (a complete distance matrix where
each entry is a real shortest-path cost), then solve TSP on that matrix. Each
tour leg is expanded back into its true multi-hop flight path for rendering.

**Why both.** Held-Karp is the exact dynamic program: `O(n² · 2ⁿ)` time,
`O(n · 2ⁿ)` space, tractable to roughly n ≤ 20. Nearest-Neighbour is an `O(n²)`
greedy heuristic for real-time rerouting when the checkpoint set mutates
mid-flight and an exponential recompute is unaffordable.

**Measured (11-node circuit: base + 10 checkpoints).**

| method | length | optimal | time |
|--------|--------|---------|------|
| Held-Karp | **151.11 km** | yes | ~3.5 ms |
| Nearest-Neighbour | 170.47 km | no | ~3.4 ms |

The heuristic's tour is **+12.8%** longer than optimal here. At this size both
are fast; the gap is the point — it quantifies what you give up for the `O(n²)`
guarantee. As n grows past ~18–20 Held-Karp becomes intractable and NN (or a
2-opt refinement) is the only real-time option.

**Design tradeoff.** Plan the nightly circuit with Held-Karp offline; reroute
with NN online. The same engine exposes both via the `method` parameter.

---

## 4. Algorithm 3 — Max flow / Min cut (Edmonds-Karp)

**Problem.** Limited drone flight-hours, several zones demanding patrol by threat
score. Maximize total coverage; identify the bottleneck.

**Model (transformation, not a slice).**
```
SUPER_SRC --(flight_hours)--> drone --(reachable)--> zone --(demand)--> CMD
```
`demand(zone) = ceil(threat_score) × 0.5` hours. A drone connects to a zone only
if a flight path exists. Max flow = maximum achievable coverage; the min cut =
the saturated edges that cap it.

**Why Edmonds-Karp.** It is Ford-Fulkerson with **BFS-chosen shortest augmenting
paths**, giving an `O(V · E²)` bound independent of capacity magnitudes (plain
Ford-Fulkerson can behave badly on adversarial capacities). For prototype-scale
networks it is fast and easy to prove correct.

**Complexity.** `O(V · E²)` time, `O(V + E)` space.

**Measured.** Max coverage = **17.0 flight-hours**; **5 of 8** zones fully
covered; **3** min-cut edges. The shortfall zones (e.g. Z7, Z8) are precisely
where the model says to request reinforcement — the operational meaning of the
min cut. Runtime ~2.6 ms.

**Validation.** The unit suite checks Edmonds-Karp against the classic CLRS
network (known max flow **23**) and confirms min-cut capacity equals max flow.

---

## 5. Algorithm 4 — MST (Kruskal) + critical links

**Problem.** 32 relay towers; activating all is expensive. Find the minimum-cost
set of radio links keeping every relay connected to command, and flag single
points of failure.

**Why Kruskal.** The relay graph is sparse (proximity-limited links). Kruskal's
edge-sort + union-find suits sparse graphs and pairs naturally with **bridge
detection** (DFS low-link) for critical-link analysis. A **backup MST** is
recomputed with a relay removed to demonstrate graceful degradation.

**Complexity.** Kruskal `O(E log E)` time, `O(V)` space. Bridges `O(V + E)`.

**Measured.** Backbone cost **429.9** over **32** links, **0 critical links** —
the dense relay field is fully 2-edge-connected, so no single link failure
disconnects command (a desirable resilience property). Simulating relay **R5**
failing recomputes a still-connected backbone at cost **415.3** (one component).
Runtime ~2.6 ms.

**Design note.** Zero bridges is a property of this dataset's density, not a bug;
a sparser relay layout (raise `RADIO_RANGE_KM` constraints or thin the grid in
`generate_zones.py`) will expose bridges that the engine then highlights in red.

---

## 6. System integration and the six scenarios

The Python scheduler converts threat to demand, selects which engine command
answers which question, and assembles the fleet; it never re-implements an
algorithm. The Flask API exposes one endpoint per question and serves the
dashboard. The simulation engine runs all six scenarios and writes replayable
JSON timelines.

| # | scenario | result on `border-sector-7` |
|---|----------|------------------------------|
| 1 | Routine patrol | exact 151.11 km vs NN 170.47 km (+12.8%) |
| 2 | Single threat alert | D1 → Z1, 18.05 km; A\* 3 vs Dijkstra 3 expansions |
| 3 | Multiple alerts | coverage 17 h, 5/8 zones, 3 bottleneck edges |
| 4 | Relay failure | backbone 429.9 → 415.3 after R5, still connected |
| 5 | New threat mid-patrol | A\* dispatch + NN reroute path returned |
| 6 | Stress test | full pipeline + 8 dispatches in ~68 ms |

All six report **PASS**.

---

## 7. Reproducing every number

```bash
# 1. Build the engine and run unit tests (13 cases, 35 assertions)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
ctest --test-dir build --output-on-failure

# 2. Regenerate + validate the dataset
python3 datasets/generate_zones.py

# 3. Exercise the engine directly
echo '{"from":"BASE_A","to":"Z7","method":"astar"}' | ./build/route_engine shortest_path
echo '{"method":"held_karp"}' | ./build/route_engine tsp
echo '{}' | ./build/route_engine max_flow
echo '{"with_backup":true}' | ./build/route_engine mst

# 4. Python layer + all six scenarios
pip install -r python_scheduler/requirements.txt
pytest python_scheduler/test_scheduler.py -q
python3 python_scheduler/run_scenarios.py

# 5. Live dashboard
python3 python_scheduler/api_server.py   # then open http://127.0.0.1:5000/
```

---

## 8. Complexity summary

| algorithm | time | space | role |
|-----------|------|-------|------|
| Dijkstra | `O((V+E) log V)` | `O(V)` | exact dispatch / metric closure |
| A\* | `O((V+E) log V)` worst, ≪ typical | `O(V)` | dispatch on the map |
| Held-Karp TSP | `O(n² · 2ⁿ)` | `O(n · 2ⁿ)` | exact night circuit (n ≤ ~20) |
| Nearest-Neighbour | `O(n²)` | `O(n)` | real-time reroute |
| Edmonds-Karp | `O(V · E²)` | `O(V+E)` | flight-hour allocation |
| Kruskal MST | `O(E log E)` | `O(V)` | relay backbone |
| Bridge finding | `O(V + E)` | `O(V)` | critical relay links |

---

## 9. Limitations and honest scope

- The dataset is **synthetic** and the physics (wind, terrain, battery) are
  simplified proxies. The contribution is the algorithmic framework and its
  integration, not a validated flight model.
- Held-Karp is exponential; beyond ~20 checkpoints the system must fall back to
  NN or a metaheuristic (2-opt / Lin-Kernighan) — noted but not implemented.
- The CLI/JSON boundary spawns a process per query. Fine for a prototype; a
  pybind11 in-process binding is the documented optimization path.
- The flow model grants each reachable drone capacity equal to a zone's full
  demand; richer per-drone hour budgeting is a natural extension.

These are deliberate prototype boundaries, called out so results are not
overclaimed.
