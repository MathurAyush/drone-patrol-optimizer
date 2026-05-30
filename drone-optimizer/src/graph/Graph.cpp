#include "graph/Graph.hpp"
#include "nlohmann/json.hpp"

#include <set>
#include <stdexcept>

using nlohmann::json;

namespace drone {

NodeKind nodeKindFromString(const std::string& s) {
    if (s == "COMMAND")    return NodeKind::Command;
    if (s == "BASE")       return NodeKind::Base;
    if (s == "ZONE")       return NodeKind::Zone;
    if (s == "CHECKPOINT") return NodeKind::Checkpoint;
    if (s == "RELAY")      return NodeKind::Relay;
    return NodeKind::Unknown;
}
std::string nodeKindToString(NodeKind k) {
    switch (k) {
        case NodeKind::Command:    return "COMMAND";
        case NodeKind::Base:       return "BASE";
        case NodeKind::Zone:       return "ZONE";
        case NodeKind::Checkpoint: return "CHECKPOINT";
        case NodeKind::Relay:      return "RELAY";
        default:                   return "UNKNOWN";
    }
}
EdgeKind edgeKindFromString(const std::string& s) {
    if (s == "FLIGHT_PATH") return EdgeKind::FlightPath;
    if (s == "RADIO_LINK")  return EdgeKind::RadioLink;
    return EdgeKind::Unknown;
}
std::string edgeKindToString(EdgeKind k) {
    switch (k) {
        case EdgeKind::FlightPath: return "FLIGHT_PATH";
        case EdgeKind::RadioLink:  return "RADIO_LINK";
        default:                   return "UNKNOWN";
    }
}

int Projection::index(const std::string& id) const {
    auto it = idToIndex.find(id);
    return it == idToIndex.end() ? -1 : it->second;
}

Graph Graph::fromJsonString(const std::string& jsonText) {
    json doc = json::parse(jsonText);
    Graph g;

    for (const auto& jn : doc.at("nodes")) {
        Node n;
        n.id    = jn.at("id").get<std::string>();
        n.kind  = nodeKindFromString(jn.at("kind").get<std::string>());
        n.x     = jn.at("x").get<double>();
        n.y     = jn.at("y").get<double>();
        n.label = jn.value("label", n.id);
        if (jn.contains("threat_score"))    n.threat_score    = jn["threat_score"].get<double>();
        if (jn.contains("activation_cost")) n.activation_cost = jn["activation_cost"].get<double>();
        if (jn.contains("speed_kmh"))       n.speed_kmh       = jn["speed_kmh"].get<double>();
        g.idIndex_[n.id] = static_cast<int>(g.nodes_.size());
        g.nodes_.push_back(std::move(n));
    }

    for (const auto& je : doc.at("edges")) {
        Edge e;
        e.u           = je.at("u").get<std::string>();
        e.v           = je.at("v").get<std::string>();
        e.kind        = edgeKindFromString(je.at("kind").get<std::string>());
        e.distance_km = je.value("distance_km", 0.0);
        if (je.contains("wind_resistance")) e.wind_resistance = je["wind_resistance"].get<double>();
        if (je.contains("no_fly"))          e.no_fly          = je["no_fly"].get<bool>();
        if (je.contains("effective_cost"))  e.effective_cost  = je["effective_cost"].get<double>();
        if (je.contains("link_cost"))       e.link_cost       = je["link_cost"].get<double>();
        g.edges_.push_back(std::move(e));
    }
    return g;
}

const Node* Graph::node(const std::string& id) const {
    auto it = idIndex_.find(id);
    return it == idIndex_.end() ? nullptr : &nodes_[it->second];
}

std::vector<std::string> Graph::nodesOfKind(NodeKind k) const {
    std::vector<std::string> out;
    for (const auto& n : nodes_)
        if (n.kind == k) out.push_back(n.id);
    return out;
}

bool Graph::coords(const std::string& id, double& x, double& y) const {
    const Node* n = node(id);
    if (!n) return false;
    x = n->x;
    y = n->y;
    return true;
}

// --- helper: build a projection from a node predicate + edge predicate ---
namespace {
template <typename NodePred, typename EdgePred, typename Weight>
Projection buildProjection(const std::vector<Node>& nodes,
                           const std::vector<Edge>& edges,
                           NodePred keepNode, EdgePred keepEdge, Weight weight) {
    Projection p;
    for (const auto& n : nodes) {
        if (keepNode(n)) {
            p.idToIndex[n.id] = static_cast<int>(p.indexToId.size());
            p.indexToId.push_back(n.id);
        }
    }
    p.adj.resize(p.indexToId.size());
    for (const auto& e : edges) {
        if (!keepEdge(e)) continue;
        auto iu = p.idToIndex.find(e.u);
        auto iv = p.idToIndex.find(e.v);
        if (iu == p.idToIndex.end() || iv == p.idToIndex.end()) continue;
        double w = weight(e);
        p.adj[iu->second].push_back({iv->second, w});
        p.adj[iv->second].push_back({iu->second, w});  // undirected
    }
    return p;
}
}  // namespace

Projection Graph::flightProjection() const {
    auto keepNode = [](const Node& n) {
        return n.kind == NodeKind::Base || n.kind == NodeKind::Zone ||
               n.kind == NodeKind::Checkpoint || n.kind == NodeKind::Command;
    };
    auto keepEdge = [](const Edge& e) {
        return e.kind == EdgeKind::FlightPath && !(e.no_fly && *e.no_fly);
    };
    auto weight = [](const Edge& e) {
        return e.effective_cost.value_or(e.distance_km);
    };
    return buildProjection(nodes_, edges_, keepNode, keepEdge, weight);
}

Projection Graph::relayProjection(const std::string& excludeNode) const {
    auto keepNode = [&](const Node& n) {
        if (n.id == excludeNode) return false;
        return n.kind == NodeKind::Relay || n.kind == NodeKind::Command;
    };
    auto keepEdge = [&](const Edge& e) {
        if (e.kind != EdgeKind::RadioLink) return false;
        return e.u != excludeNode && e.v != excludeNode;
    };
    auto weight = [](const Edge& e) {
        return e.link_cost.value_or(e.distance_km);
    };
    return buildProjection(nodes_, edges_, keepNode, keepEdge, weight);
}

}  // namespace drone
