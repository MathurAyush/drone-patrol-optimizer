#ifndef DRONE_MAXFLOW_HPP
#define DRONE_MAXFLOW_HPP

#include <string>
#include <vector>
#include <unordered_map>

namespace drone {

// A directed capacitated arc in the flow network.
struct FlowArc {
    std::string from;
    std::string to;
    double      cap = 0.0;
};

struct FlowEdgeResult {
    std::string from;
    std::string to;
    double      flow = 0.0;
    double      cap = 0.0;
};

struct MaxFlowResult {
    double                       max_flow = 0.0;
    std::vector<FlowEdgeResult>  flows;          // flow on each original arc
    std::vector<FlowEdgeResult>  min_cut_edges;  // saturated arcs crossing the cut
    std::vector<std::string>     source_side;    // nodes reachable from source in residual
};

// Edmonds-Karp max flow: BFS-augmenting Ford-Fulkerson, O(V * E^2),
// bound independent of capacity magnitudes. Also returns the min cut
// (by the max-flow min-cut theorem: arcs from the source-reachable residual
// set to the rest, which are exactly the saturated bottleneck arcs).
MaxFlowResult edmondsKarp(const std::vector<std::string>& nodes,
                          const std::vector<FlowArc>& arcs,
                          const std::string& source,
                          const std::string& sink);

}  // namespace drone

#endif  // DRONE_MAXFLOW_HPP
