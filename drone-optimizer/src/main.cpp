// route_engine — the C++ compute core.
// Usage:  route_engine <command> [--graph FILE]   (reads params JSON from stdin,
//                                                   writes result JSON to stdout)
// Commands: shortest_path | tsp | max_flow | mst | fleet_state
//
// The JSON command contract is defined in docs/ARCHITECTURE.md section 3.3.

#include "graph/Graph.hpp"
#include "algorithms/ShortestPath.hpp"
#include "algorithms/Tsp.hpp"
#include "algorithms/MaxFlow.hpp"
#include "algorithms/Mst.hpp"
#include "algorithms/MetricClosure.hpp"
#include "fleet/Fleet.hpp"
#include "fleet/FlowBuilder.hpp"
#include "utils/Util.hpp"
#include "nlohmann/json.hpp"

#include <string>
#include <vector>
#include <iostream>

using nlohmann::json;
using namespace drone;

namespace {

std::string argValue(int argc, char** argv, const std::string& flag,
                     const std::string& def) {
    for (int i = 1; i < argc - 1; ++i)
        if (flag == argv[i]) return argv[i + 1];
    return def;
}

DroneFleet fleetFromJson(const json& j, const Graph& g) {
    DroneFleet fleet;
    if (!j.contains("drones")) {
        // default 3-drone fleet at the two bases
        auto bases = g.nodesOfKind(NodeKind::Base);
        std::string b0 = bases.empty() ? "" : bases[0];
        std::string b1 = bases.size() > 1 ? bases[1] : b0;
        fleet.add({"D1", b0, b0, 100, 60, 6.0, DroneState::Patrolling, ""});
        fleet.add({"D2", b1, b1, 100, 60, 6.0, DroneState::Patrolling, ""});
        fleet.add({"D3", b0, b0, 95, 55, 5.0, DroneState::Patrolling, ""});
        return fleet;
    }
    for (const auto& jd : j.at("drones")) {
        DroneAgent d;
        d.id = jd.at("id").get<std::string>();
        d.position = jd.at("position").get<std::string>();
        d.home_base = jd.value("home_base", d.position);
        d.battery_pct = jd.value("battery_pct", 100.0);
        d.speed_kmh = jd.value("speed_kmh", 60.0);
        d.flight_hours_left = jd.value("flight_hours_left", 6.0);
        fleet.add(d);
    }
    return fleet;
}

json doShortestPath(const Graph& g, const json& in) {
    std::string from = in.at("from").get<std::string>();
    std::string to   = in.at("to").get<std::string>();
    std::string method = in.value("method", "astar");
    Projection flight = g.flightProjection();

    PathResult r;
    if (method == "dijkstra") {
        r = dijkstra(flight, from, to);
    } else {
        auto h = euclideanHeuristic(g, flight, to);
        r = astar(flight, from, to, h);
    }
    json out;
    out["method"] = method;
    out["from"] = from;
    out["to"] = to;
    out["found"] = r.found;
    out["cost"] = r.cost;
    out["nodes_expanded"] = r.nodes_expanded;
    out["path"] = r.path;

    // Also run the other method to report the expansion comparison.
    if (method == "astar") {
        auto d = dijkstra(flight, from, to);
        out["compare"] = {{"dijkstra_nodes_expanded", d.nodes_expanded},
                          {"astar_nodes_expanded", r.nodes_expanded}};
    }
    return out;
}

json doTsp(const Graph& g, const json& in) {
    Projection flight = g.flightProjection();
    std::vector<std::string> subset;
    if (in.contains("nodes")) {
        subset = in.at("nodes").get<std::vector<std::string>>();
    } else {
        subset = g.nodesOfKind(NodeKind::Checkpoint);
        // include a base as the start/depot
        auto bases = g.nodesOfKind(NodeKind::Base);
        if (!bases.empty()) subset.insert(subset.begin(), bases[0]);
    }
    std::string method = in.value("method", "held_karp");
    std::string startId = in.value("start", subset.empty() ? "" : subset[0]);

    ClosureResult c = metricClosure(flight, subset);
    int start = 0;
    for (int i = 0; i < (int)c.ids.size(); ++i)
        if (c.ids[i] == startId) start = i;

    json out;
    out["method"] = method;
    out["complete_closure"] = c.complete;
    if (!c.complete) {
        out["error"] = "checkpoint set not mutually reachable on flight graph";
        return out;
    }

    TspResult t;
    if (method == "nearest_neighbour")
        t = nearestNeighbour(c.dist, start);
    else
        t = heldKarp(c.dist, start);

    std::vector<std::string> tourIds;
    for (int idx : t.tour) tourIds.push_back(c.ids[idx]);
    out["tour"] = tourIds;
    out["length"] = t.length;
    out["optimal"] = t.optimal;

    // Expand each tour leg into its real multi-hop flight path for rendering.
    json legs = json::array();
    for (size_t i = 0; i < t.tour.size(); ++i) {
        int a = t.tour[i];
        int b = t.tour[(i + 1) % t.tour.size()];
        legs.push_back({{"from", c.ids[a]}, {"to", c.ids[b]},
                        {"path", c.expandedPath[a][b]}, {"cost", c.dist[a][b]}});
    }
    out["legs"] = legs;
    return out;
}

// Emit the terrain-aware metric-closure distance matrix for a node subset.
// This is the bridge to the OR-Tools VRP layer: it routes over EXACTLY the same
// shortest-path costs (geodesic + terrain climb) the rest of the engine uses,
// so multi-drone plans stay consistent with single-path dispatch.
json doMatrix(const Graph& g, const json& in) {
    Projection flight = g.flightProjection();
    std::vector<std::string> subset;
    if (in.contains("nodes")) {
        subset = in.at("nodes").get<std::vector<std::string>>();
    } else {
        auto bases = g.nodesOfKind(NodeKind::Base);
        if (!bases.empty()) subset.push_back(bases[0]);
        for (const auto& c : g.nodesOfKind(NodeKind::Checkpoint))
            subset.push_back(c);
    }
    ClosureResult c = metricClosure(flight, subset);
    json out;
    out["ids"] = c.ids;
    out["complete"] = c.complete;
    json m = json::array();
    for (const auto& row : c.dist) m.push_back(row);
    out["matrix_km"] = m;
    return out;
}

json doMaxFlow(const Graph& g, const json& in) {
    DroneFleet fleet = fleetFromJson(in, g);
    double reach = in.value("reach_range_km", 1e9);
    FlowBuild fb = buildFlowNetwork(g, fleet, reach);
    MaxFlowResult r = edmondsKarp(fb.nodes, fb.arcs, fb.source, fb.sink);

    json out;
    out["max_flow"] = r.max_flow;
    out["source"] = fb.source;
    out["sink"] = fb.sink;

    json alloc = json::array();
    for (const auto& f : r.flows)
        alloc.push_back({{"from", f.from}, {"to", f.to},
                         {"flow", f.flow}, {"cap", f.cap}});
    out["allocation"] = alloc;

    json cut = json::array();
    for (const auto& e : r.min_cut_edges)
        cut.push_back({{"from", e.from}, {"to", e.to}, {"cap", e.cap}});
    out["min_cut_edges"] = cut;

    // Per-zone coverage summary (demand vs delivered).
    json zoneCov = json::array();
    auto zones = g.nodesOfKind(NodeKind::Zone);
    for (const auto& z : zones) {
        double demand = 0, delivered = 0;
        for (const auto& f : r.flows)
            if (f.to == "CMD" && f.from == z) { delivered = f.flow; demand = f.cap; }
        // demand edge may not carry flow; recover cap from builder
        for (const auto& a : fb.arcs)
            if (a.from == z && a.to == fb.sink) demand = a.cap;
        const Node* zn = g.node(z);
        zoneCov.push_back({{"zone", z},
                           {"threat_score", zn->threat_score.value_or(0.0)},
                           {"demand", demand},
                           {"covered", delivered},
                           {"shortfall", demand - delivered}});
    }
    out["zone_coverage"] = zoneCov;
    return out;
}

json doMst(const Graph& g, const json& in) {
    std::string failed = in.value("failed_relay", "");
    Projection relay = g.relayProjection(failed);
    MstResult m = kruskal(relay);
    auto br = bridges(relay);

    json out;
    out["failed_relay"] = failed;
    out["connected"] = m.connected;
    out["components"] = m.components;
    out["total_cost"] = m.total_cost;

    json edges = json::array();
    for (const auto& e : m.edges)
        edges.push_back({{"u", e.u}, {"v", e.v}, {"weight", e.weight}});
    out["mst_edges"] = edges;

    json bridgeArr = json::array();
    for (const auto& e : br)
        bridgeArr.push_back({{"u", e.u}, {"v", e.v}});
    out["critical_links"] = bridgeArr;

    // Backup analysis: if the first critical link's endpoint relay fails,
    // recompute and report the new cost (graceful degradation).
    if (in.value("with_backup", false) && !br.empty()) {
        std::string victim = br.front().v;  // a relay endpoint of a bridge
        if (g.node(victim) && g.node(victim)->kind == NodeKind::Relay) {
            Projection relay2 = g.relayProjection(victim);
            MstResult m2 = kruskal(relay2);
            out["backup"] = {{"removed", victim},
                             {"connected", m2.connected},
                             {"total_cost", m2.total_cost},
                             {"components", m2.components}};
        }
    }
    return out;
}

json doFleetState(const Graph& g, const json& in) {
    DroneFleet fleet = fleetFromJson(in, g);
    Projection flight = g.flightProjection();
    json out = json::array();
    for (const auto& d : fleet.drones()) {
        double x = 0, y = 0;
        g.coords(d.position, x, y);
        out.push_back({{"id", d.id}, {"position", d.position},
                       {"x", x}, {"y", y},
                       {"battery_pct", d.battery_pct},
                       {"state", droneStateToString(d.state)}});
    }
    return {{"drones", out}};
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: route_engine <command> [--graph FILE]\n"
                     "commands: shortest_path tsp max_flow mst fleet_state\n";
        return 2;
    }
    std::string command = argv[1];
    std::string graphPath = argValue(argc, argv, "--graph",
                                     "datasets/zones.json");
    try {
        Graph g = Graph::fromJsonString(readFile(graphPath));
        std::string stdinText = readStdin();
        json in = stdinText.empty() ? json::object() : json::parse(stdinText);

        json out;
        if (command == "shortest_path")   out = doShortestPath(g, in);
        else if (command == "tsp")        out = doTsp(g, in);
        else if (command == "max_flow")   out = doMaxFlow(g, in);
        else if (command == "mst")        out = doMst(g, in);
        else if (command == "matrix")     out = doMatrix(g, in);
        else if (command == "fleet_state")out = doFleetState(g, in);
        else { std::cerr << "unknown command: " << command << "\n"; return 2; }

        std::cout << out.dump(2) << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        std::cout << json({{"error", e.what()}}).dump() << std::endl;
        return 1;
    }
}
