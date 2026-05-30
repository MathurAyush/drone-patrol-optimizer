#ifndef DRONE_GRAPH_HPP
#define DRONE_GRAPH_HPP

#include "Types.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace drone {

// A weighted adjacency-list view produced by projecting the unified graph.
// Node indices are local to the projection; idToIndex / indexToId translate.
struct Projection {
    struct Arc {
        int    to;       // local index of the neighbour
        double weight;
    };
    std::vector<std::vector<Arc>> adj;          // adj[localIndex] -> arcs
    std::vector<std::string>      indexToId;    // local index -> node id
    std::unordered_map<std::string, int> idToIndex;

    int size() const { return static_cast<int>(adj.size()); }
    bool has(const std::string& id) const { return idToIndex.count(id) > 0; }
    int index(const std::string& id) const;    // -1 if absent
};

// The unified zone graph: single source of truth, projected on demand.
class Graph {
public:
    // Build from a parsed zones.json document (nlohmann::json passed as string
    // to keep the header free of the json dependency — see Graph.cpp).
    static Graph fromJsonString(const std::string& jsonText);

    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<Edge>& edges() const { return edges_; }
    const Node* node(const std::string& id) const;

    std::vector<std::string> nodesOfKind(NodeKind k) const;

    // Projections (see ARCHITECTURE.md section 2.3).
    // Flight projection: BASE/ZONE/CHECKPOINT/COMMAND over FLIGHT_PATH edges
    // with no_fly == false; weight = effective_cost.
    Projection flightProjection() const;

    // Relay projection: RELAY + COMMAND over RADIO_LINK edges; weight = link_cost.
    // If excludeNode is non-empty, that node (a failed relay) is omitted.
    Projection relayProjection(const std::string& excludeNode = "") const;

    // Coordinates for the A* heuristic.
    bool coords(const std::string& id, double& x, double& y) const;

private:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::unordered_map<std::string, int> idIndex_;  // id -> index into nodes_
};

}  // namespace drone

#endif  // DRONE_GRAPH_HPP
