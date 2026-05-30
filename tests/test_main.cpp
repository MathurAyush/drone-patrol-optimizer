#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "graph/Graph.hpp"
#include "algorithms/ShortestPath.hpp"
#include "algorithms/Tsp.hpp"
#include "algorithms/MaxFlow.hpp"
#include "algorithms/Mst.hpp"
#include "algorithms/MetricClosure.hpp"
#include "fleet/Fleet.hpp"
#include "fleet/FlowBuilder.hpp"
#include "utils/Util.hpp"

#include <cmath>
#include <string>

using namespace drone;

// A tiny hand-checkable graph used across several tests.
//   square of side 1, with a diagonal:
//   A(0,0) B(1,0)
//   D(0,1) C(1,1)
static const char* kSquareJson = R"JSON({
  "nodes": [
    {"id":"A","kind":"BASE","x":0,"y":0,"speed_kmh":60},
    {"id":"B","kind":"CHECKPOINT","x":1,"y":0},
    {"id":"C","kind":"CHECKPOINT","x":1,"y":1},
    {"id":"D","kind":"CHECKPOINT","x":0,"y":1}
  ],
  "edges": [
    {"u":"A","v":"B","kind":"FLIGHT_PATH","distance_km":1,"effective_cost":1,"no_fly":false},
    {"u":"B","v":"C","kind":"FLIGHT_PATH","distance_km":1,"effective_cost":1,"no_fly":false},
    {"u":"C","v":"D","kind":"FLIGHT_PATH","distance_km":1,"effective_cost":1,"no_fly":false},
    {"u":"D","v":"A","kind":"FLIGHT_PATH","distance_km":1,"effective_cost":1,"no_fly":false},
    {"u":"A","v":"C","kind":"FLIGHT_PATH","distance_km":1.41421356,"effective_cost":1.41421356,"no_fly":false}
  ]
})JSON";

TEST_CASE("graph loads and projects") {
    Graph g = Graph::fromJsonString(kSquareJson);
    CHECK(g.nodes().size() == 4);
    Projection f = g.flightProjection();
    CHECK(f.size() == 4);
    CHECK(f.has("A"));
}

TEST_CASE("no-fly edges are excluded from the flight projection") {
    std::string j = R"({"nodes":[
      {"id":"A","kind":"BASE","x":0,"y":0},
      {"id":"B","kind":"ZONE","x":2,"y":0,"threat_score":5}],
      "edges":[{"u":"A","v":"B","kind":"FLIGHT_PATH","distance_km":2,"effective_cost":2,"no_fly":true}]})";
    Graph g = Graph::fromJsonString(j);
    Projection f = g.flightProjection();
    PathResult r = dijkstra(f, "A", "B");
    CHECK_FALSE(r.found);  // only edge is no-fly => unreachable
}

TEST_CASE("dijkstra finds the shortest path on the square") {
    Graph g = Graph::fromJsonString(kSquareJson);
    Projection f = g.flightProjection();
    PathResult r = dijkstra(f, "A", "C");
    REQUIRE(r.found);
    CHECK(r.cost == doctest::Approx(1.41421356));  // diagonal beats 2-hop
    CHECK(r.path.front() == "A");
    CHECK(r.path.back() == "C");
}

TEST_CASE("A* agrees with Dijkstra on cost and is admissible") {
    Graph g = Graph::fromJsonString(kSquareJson);
    Projection f = g.flightProjection();
    auto h = euclideanHeuristic(g, f, "C");
    PathResult a = astar(f, "A", "C", h);
    PathResult d = dijkstra(f, "A", "C");
    REQUIRE(a.found);
    CHECK(a.cost == doctest::Approx(d.cost));
    // A* should never expand more nodes than Dijkstra here.
    CHECK(a.nodes_expanded <= d.nodes_expanded);
}

TEST_CASE("held-karp optimal on the unit square is 4.0") {
    // Optimal Hamiltonian cycle of a unit square (no diagonal) = perimeter = 4.
    Matrix d = {
        {0, 1, std::sqrt(2.0), 1},
        {1, 0, 1, std::sqrt(2.0)},
        {std::sqrt(2.0), 1, 0, 1},
        {1, std::sqrt(2.0), 1, 0}};
    TspResult t = heldKarp(d, 0);
    CHECK(t.optimal);
    CHECK(t.length == doctest::Approx(4.0));
    CHECK(t.tour.size() == 4);
}

TEST_CASE("nearest-neighbour is feasible and >= optimal") {
    Matrix d = {
        {0, 1, std::sqrt(2.0), 1},
        {1, 0, 1, std::sqrt(2.0)},
        {std::sqrt(2.0), 1, 0, 1},
        {1, std::sqrt(2.0), 1, 0}};
    TspResult opt = heldKarp(d, 0);
    TspResult nn = nearestNeighbour(d, 0);
    CHECK_FALSE(nn.optimal);
    CHECK(nn.tour.size() == 4);
    CHECK(nn.length >= opt.length - 1e-9);  // heuristic never beats exact
}

TEST_CASE("metric closure builds a complete matrix on a sparse graph") {
    Graph g = Graph::fromJsonString(kSquareJson);
    Projection f = g.flightProjection();
    ClosureResult c = metricClosure(f, {"A", "B", "C", "D"});
    CHECK(c.complete);
    CHECK(c.dist[0][2] == doctest::Approx(1.41421356));  // A->C via diagonal
}

TEST_CASE("edmonds-karp classic max flow") {
    // Classic CLRS-style network, known max flow = 23.
    std::vector<std::string> nodes = {"s","a","b","c","d","t"};
    std::vector<FlowArc> arcs = {
        {"s","a",16},{"s","b",13},
        {"a","b",10},{"b","a",4},
        {"a","c",12},{"c","b",9},
        {"b","d",14},{"d","c",7},
        {"c","t",20},{"d","t",4}};
    MaxFlowResult r = edmondsKarp(nodes, arcs, "s", "t");
    CHECK(r.max_flow == doctest::Approx(23.0));
    CHECK_FALSE(r.min_cut_edges.empty());
    // min-cut capacity equals max flow
    double cut = 0;
    for (auto& e : r.min_cut_edges) cut += e.cap;
    CHECK(cut == doctest::Approx(23.0));
}

TEST_CASE("max flow is bounded by the bottleneck (min cut meaning)") {
    // s -> m (cap 5) -> t (cap 100): bottleneck is the 5 edge.
    std::vector<std::string> nodes = {"s","m","t"};
    std::vector<FlowArc> arcs = {{"s","m",5},{"m","t",100}};
    MaxFlowResult r = edmondsKarp(nodes, arcs, "s", "t");
    CHECK(r.max_flow == doctest::Approx(5.0));
    REQUIRE(r.min_cut_edges.size() == 1);
    CHECK(r.min_cut_edges[0].from == "s");
}

TEST_CASE("kruskal MST total cost on the square") {
    // relay-style test: reuse square as a generic weighted graph via relay proj
    std::string j = R"({"nodes":[
      {"id":"R1","kind":"RELAY","x":0,"y":0,"activation_cost":10},
      {"id":"R2","kind":"RELAY","x":1,"y":0,"activation_cost":10},
      {"id":"R3","kind":"RELAY","x":1,"y":1,"activation_cost":10},
      {"id":"CMD","kind":"COMMAND","x":0,"y":1}],
      "edges":[
        {"u":"R1","v":"R2","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R2","v":"R3","kind":"RADIO_LINK","distance_km":1,"link_cost":2},
        {"u":"R3","v":"CMD","kind":"RADIO_LINK","distance_km":1,"link_cost":3},
        {"u":"CMD","v":"R1","kind":"RADIO_LINK","distance_km":1,"link_cost":4},
        {"u":"R1","v":"R3","kind":"RADIO_LINK","distance_km":1.4,"link_cost":5}]})";
    Graph g = Graph::fromJsonString(j);
    MstResult m = kruskal(g.relayProjection());
    CHECK(m.connected);
    CHECK(m.edges.size() == 3);          // n-1 edges for 4 nodes
    CHECK(m.total_cost == doctest::Approx(1 + 2 + 3));  // cheapest spanning set
}

TEST_CASE("bridge detection finds the single point of failure") {
    // Two triangles joined by one bridge edge R3-R4.
    std::string j = R"({"nodes":[
      {"id":"R1","kind":"RELAY","x":0,"y":0},
      {"id":"R2","kind":"RELAY","x":1,"y":0},
      {"id":"R3","kind":"RELAY","x":2,"y":0},
      {"id":"R4","kind":"RELAY","x":3,"y":0},
      {"id":"R5","kind":"RELAY","x":4,"y":0},
      {"id":"CMD","kind":"COMMAND","x":5,"y":0}],
      "edges":[
        {"u":"R1","v":"R2","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R2","v":"R3","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R3","v":"R1","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R3","v":"R4","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R4","v":"R5","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R5","v":"CMD","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"CMD","v":"R4","kind":"RADIO_LINK","distance_km":1,"link_cost":1}]})";
    Graph g = Graph::fromJsonString(j);
    auto br = bridges(g.relayProjection());
    // R3-R4 is the only bridge (the two triangles are internally redundant).
    bool found = false;
    for (auto& e : br) {
        bool match = (e.u == "R3" && e.v == "R4") || (e.u == "R4" && e.v == "R3");
        if (match) found = true;
    }
    CHECK(found);
}

TEST_CASE("backup MST recovers after a relay failure") {
    std::string j = R"({"nodes":[
      {"id":"R1","kind":"RELAY","x":0,"y":0},
      {"id":"R2","kind":"RELAY","x":1,"y":0},
      {"id":"R3","kind":"RELAY","x":2,"y":0},
      {"id":"CMD","kind":"COMMAND","x":1,"y":1}],
      "edges":[
        {"u":"R1","v":"R2","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R2","v":"R3","kind":"RADIO_LINK","distance_km":1,"link_cost":1},
        {"u":"R1","v":"CMD","kind":"RADIO_LINK","distance_km":1,"link_cost":2},
        {"u":"R3","v":"CMD","kind":"RADIO_LINK","distance_km":1,"link_cost":2}]})";
    Graph g = Graph::fromJsonString(j);
    MstResult full = kruskal(g.relayProjection());
    CHECK(full.connected);
    // Remove R2; remaining R1,R3,CMD must still span via CMD.
    MstResult backup = kruskal(g.relayProjection("R2"));
    CHECK(backup.connected);
    CHECK(backup.edges.size() == 2);
}

TEST_CASE("fleet picks the nearest available drone") {
    Graph g = Graph::fromJsonString(kSquareJson);
    Projection f = g.flightProjection();
    DroneFleet fleet;
    fleet.add({"D1","A","A",100,60,6,DroneState::Patrolling,""});
    fleet.add({"D2","C","C",100,60,6,DroneState::Patrolling,""});
    double t = 0; PathResult p;
    int idx = fleet.nearestAvailable(f, "B", t, p);
    REQUIRE(idx >= 0);
    // Both A->B and C->B cost 1; either is fine, but a valid drone is chosen.
    CHECK(p.found);
    CHECK(p.path.back() == "B");
}
