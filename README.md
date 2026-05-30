# Smart Drone Patrol Route Optimizer

[![CI](https://github.com/USER/drone-optimizer/actions/workflows/ci.yml/badge.svg)](https://github.com/USER/drone-optimizer/actions/workflows/ci.yml)

An autonomous multi-drone fleet patrol planner built on four classical graph
algorithms, wrapped in a fleet simulation and a live tactical dashboard. The
"drones" are abstract monitoring/relay agents on a synthetic sector graph — the
project is a graph-algorithms research prototype in the same family as logistics
routing, telecom backbone planning, and search-and-rescue dispatch.

| Duty | Algorithm | Why |
|------|-----------|-----|
| Minimum relay radio backbone (+ critical links, backup) | **Kruskal MST** + bridges | sparse graph, free critical-link analysis |
| Fastest-drone threat dispatch | **Dijkstra + A\*** | exact baseline + admissible heuristic speedup |
| Flight-hour allocation across zones | **Edmonds-Karp** max flow / min cut | capacity-independent bound, names the bottleneck |
| Routine patrol circuit | **Held-Karp** TSP + **nearest-neighbour** | exact offline, fast online reroute |
| Multi-drone fleet routing at scale | **OR-Tools VRP** | hundreds of stops, range + time-window constraints |

**Stack:** C++17 routing engine · Python (Flask) scheduler + REST API · HTML5/JS canvas dashboard.

---

## Quick start

```bash
# 1. Build the C++ engine (requires CMake >= 3.15, a C++17 compiler)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# 2. Run the unit tests (13 cases / 35 assertions)
ctest --test-dir build --output-on-failure

# 3. Python scheduler + API
pip install -r python_scheduler/requirements.txt
python3 python_scheduler/run_scenarios.py          # run all 6 scenarios
python3 python_scheduler/api_server.py             # serve dashboard at :5000
```

Open <http://127.0.0.1:5000/> for the live tactical console. (The dashboard also
renders standalone with an embedded sample graph if the API is offline.)

### Run on real GIS terrain data

The system can route over **real geography** — ellipsoidal distances, elevation
from a GeoTIFF DEM, terrain climb-energy cost, restricted-airspace polygons, and
radio line-of-sight. See [`docs/GIS_PIPELINE.md`](docs/GIS_PIPELINE.md).

```bash
# build the real-coordinate sample (georeferenced DEM + features + no-fly zones)
python3 gis/generate_terrain.py

# ingest GIS -> engine-ready graph
python3 -m gis.pipeline --out datasets/zones_gis.json

# run the whole stack on the terrain graph
ZONES_JSON=$PWD/datasets/zones_gis.json python3 python_scheduler/run_scenarios.py
ZONES_JSON=$PWD/datasets/zones_gis.json python3 python_scheduler/api_server.py
```

Swap `gis/data/dem.tif` for a real Copernicus GLO-30 / SRTM tile (and the
features / no-fly files for authoritative data) with no code change.

### Scale to a fleet (OR-Tools VRP)

Held-Karp is exact but single-drone and caps at ~20 stops. The OR-Tools layer
routes a **multi-drone fleet over hundreds of stops** with per-drone range,
capacity, and time-window constraints — validated to match Held-Karp's optimum
on small instances. See [`docs/ROUTING_ORTOOLS.md`](docs/ROUTING_ORTOOLS.md).

```bash
python -m routing.benchmark          # validation vs exact + scalability table
# 3-drone fleet with 60 km range each, over the terrain matrix:
ZONES_JSON=$PWD/datasets/zones_gis.json python3 -c \
 "import sys; sys.path.insert(0,'python_scheduler'); from scheduler import Scheduler; \
  import json; print(json.dumps(Scheduler().multi_drone_patrol(3, 60.0), indent=2))"
```

### Use the engine directly

The engine is a standalone CLI reading JSON params on stdin:

```bash
echo '{"from":"BASE_A","to":"Z1","method":"astar"}' | ./build/route_engine shortest_path
echo '{"method":"held_karp"}'                        | ./build/route_engine tsp
echo '{}'                                            | ./build/route_engine max_flow
echo '{"with_backup":true}'                          | ./build/route_engine mst
```

---

## Architecture in one idea

**One unified typed graph, projected per algorithm.** A single graph (5 node
kinds, 2 edge kinds) is the source of truth; each algorithm consumes a read-only
*projection* of it (node subset + edge type + weight function). The C++ engine
and Python scheduler talk over a JSON command contract across a process
boundary, which keeps the compute core fast and crash-isolated from the API.

Full design: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).
Measured performance + algorithm justification: [`docs/TECHNICAL_REPORT.md`](docs/TECHNICAL_REPORT.md).

---

## Layout

```
src/
  graph/        Types, Graph, projections (flight / relay / flow)
  algorithms/   ShortestPath (Dijkstra, A*), Tsp (Held-Karp, NN),
                MaxFlow (Edmonds-Karp), Mst (Kruskal + bridges), MetricClosure
  fleet/        DroneAgent, DroneFleet, FlowBuilder
  utils/        JSON I/O + logging
  main.cpp      route_engine CLI
gis/            REAL GIS INGESTION
  geodesy.py        WGS84 geodesic distance + UTM projection
  terrain.py        DEM elevation, climb-energy cost, radio line-of-sight
  airspace.py       no-fly polygon crossing tests (shapely)
  ingest.py         load + validate GeoJSON features, build projector
  graph_builder.py  real GIS -> engine graph (terrain cost, LOS, no-fly)
  pipeline.py       CLI entry point
  generate_terrain.py  builds the real-coordinate sample (DEM + features)
  data/             dem.tif, features.geojson, nofly.geojson
  tests/            pytest suite (geodesy, terrain, airspace, schema)
routing/         FLEET ROUTING (OR-Tools VRP)
  vrp.py            multi-drone VRP: range, capacity, time windows, drops
  matrix.py         terrain-aware cost matrix (from C++ engine or DEM)
  benchmark.py      validation vs Held-Karp + scalability table
  tests/            pytest suite
python_scheduler/
  scheduler.py      threat->demand, dispatch, engine orchestration
  api_server.py     Flask REST API + serves the dashboard
  run_scenarios.py  simulation engine (6 scenarios -> logs/)
  test_scheduler.py pytest suite
dashboard/index.html  live canvas tactical console
datasets/
  generate_zones.py  seeded synthetic graph generator
  zones.json         synthetic graph
  zones_gis.json     graph built from real GIS data
  scenarios.json     the 6 simulation scenarios
tests/test_main.cpp  C++ unit tests (doctest)
docs/                ARCHITECTURE.md, TECHNICAL_REPORT.md, GIS_PIPELINE.md
third_party/         vendored single headers (nlohmann/json, doctest)
.github/workflows/ci.yml   build + test C++ and Python on every push
```

---

## Scenarios

| # | scenario | exercises |
|---|----------|-----------|
| 1 | Routine patrol | Held-Karp TSP vs NN |
| 2 | Single threat alert | A\* / Dijkstra dispatch |
| 3 | Multiple simultaneous alerts | max-flow allocation + min-cut |
| 4 | Relay tower failure | backup MST + critical links |
| 5 | New threat mid-patrol | A\* reroute + NN |
| 6 | Full fleet stress test | the whole pipeline |

`python3 python_scheduler/run_scenarios.py` runs them all and writes replayable
timelines to `logs/`.

---

## License

MIT — see [LICENSE](LICENSE).
