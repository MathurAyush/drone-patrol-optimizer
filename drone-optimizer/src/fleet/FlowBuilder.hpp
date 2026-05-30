#ifndef DRONE_FLOW_BUILDER_HPP
#define DRONE_FLOW_BUILDER_HPP

#include "graph/Graph.hpp"
#include "algorithms/MaxFlow.hpp"
#include "algorithms/ShortestPath.hpp"
#include "fleet/Fleet.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace drone {

// Transforms the world into a flow network (ARCHITECTURE.md 4.3):
//   SUPER_SRC --(flight_hours)--> drone --(reachable)--> zone --(demand)--> CMD
// demand(zone) = ceil(threat_score) scaled to flight-hours; a drone connects to
// a zone only if it can fly there (shortest path exists within range).
struct FlowBuild {
    std::vector<std::string> nodes;
    std::vector<FlowArc>     arcs;
    std::string              source = "SUPER_SRC";
    std::string              sink = "CMD";
};

inline double zoneDemand(double threat_score) {
    // Map threat 1..10 to a patrol-hour demand. Higher threat => more hours.
    return std::ceil(threat_score) * 0.5;  // hours
}

inline FlowBuild buildFlowNetwork(const Graph& g, const DroneFleet& fleet,
                                  double reachRangeKm = 1e9) {
    FlowBuild fb;
    Projection flight = g.flightProjection();
    fb.nodes.push_back(fb.source);
    fb.nodes.push_back(fb.sink);

    for (auto& d : fleet.drones()) {
        fb.nodes.push_back(d.id);
        fb.arcs.push_back({fb.source, d.id, d.flight_hours_left});
    }

    auto zones = g.nodesOfKind(NodeKind::Zone);
    for (const auto& z : zones) {
        fb.nodes.push_back(z);
        const Node* zn = g.node(z);
        double demand = zoneDemand(zn->threat_score.value_or(1.0));
        fb.arcs.push_back({z, fb.sink, demand});
        // Each drone that can reach this zone supplies coverage capacity equal
        // to the zone's demand (a single drone could fully cover it).
        for (auto& d : fleet.drones()) {
            PathResult r = dijkstra(flight, d.position, z);
            if (r.found && r.cost <= reachRangeKm)
                fb.arcs.push_back({d.id, z, demand});
        }
    }
    return fb;
}

}  // namespace drone

#endif  // DRONE_FLOW_BUILDER_HPP
