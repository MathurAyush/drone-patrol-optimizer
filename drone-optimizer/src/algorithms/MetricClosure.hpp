#ifndef DRONE_METRIC_CLOSURE_HPP
#define DRONE_METRIC_CLOSURE_HPP

#include "graph/Graph.hpp"
#include "algorithms/Tsp.hpp"
#include "algorithms/ShortestPath.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace drone {

// The metric closure of a node subset over a sparse projection: a complete
// distance matrix where entry (i,j) is the shortest-path cost between subset
// node i and j. TSP requires a complete graph; real flight graphs are sparse,
// so we close them via Dijkstra. We also keep the expanded paths so a tour
// edge can be rendered as the real multi-hop route on the dashboard.
struct ClosureResult {
    std::vector<std::string> ids;     // subset node ids, in matrix order
    Matrix                   dist;    // complete distance matrix
    // expandedPath[i][j] = full node-id path from ids[i] to ids[j]
    std::vector<std::vector<std::vector<std::string>>> expandedPath;
    bool complete = false;            // false if any pair was unreachable
};

inline ClosureResult metricClosure(const Projection& p,
                                    const std::vector<std::string>& subset) {
    ClosureResult c;
    c.ids = subset;
    int m = static_cast<int>(subset.size());
    c.dist.assign(m, std::vector<double>(m, 0.0));
    c.expandedPath.assign(m, std::vector<std::vector<std::string>>(m));
    c.complete = true;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) {
            if (i == j) continue;
            PathResult r = dijkstra(p, subset[i], subset[j]);
            if (!r.found) { c.complete = false; continue; }
            c.dist[i][j] = r.cost;
            c.expandedPath[i][j] = r.path;
        }
    }
    return c;
}

}  // namespace drone

#endif  // DRONE_METRIC_CLOSURE_HPP
