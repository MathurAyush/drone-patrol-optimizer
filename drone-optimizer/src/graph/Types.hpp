#ifndef DRONE_GRAPH_TYPES_HPP
#define DRONE_GRAPH_TYPES_HPP

#include <string>
#include <optional>

namespace drone {

// The five node kinds of the unified zone graph (see docs/ARCHITECTURE.md).
enum class NodeKind { Command, Base, Zone, Checkpoint, Relay, Unknown };

NodeKind nodeKindFromString(const std::string& s);
std::string nodeKindToString(NodeKind k);

struct Node {
    std::string id;
    NodeKind    kind = NodeKind::Unknown;
    double      x = 0.0;
    double      y = 0.0;
    std::string label;
    // kind-specific, optional
    std::optional<double> threat_score;     // ZONE
    std::optional<double> activation_cost;  // RELAY
    std::optional<double> speed_kmh;        // BASE
};

// Two physically different edge kinds.
enum class EdgeKind { FlightPath, RadioLink, Unknown };

EdgeKind edgeKindFromString(const std::string& s);
std::string edgeKindToString(EdgeKind k);

struct Edge {
    std::string u;
    std::string v;
    EdgeKind    kind = EdgeKind::Unknown;
    double      distance_km = 0.0;
    // FLIGHT_PATH
    std::optional<double> wind_resistance;
    std::optional<bool>   no_fly;
    std::optional<double> effective_cost;
    // RADIO_LINK
    std::optional<double> link_cost;
};

}  // namespace drone

#endif  // DRONE_GRAPH_TYPES_HPP
