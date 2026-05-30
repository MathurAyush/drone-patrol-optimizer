#ifndef DRONE_MST_HPP
#define DRONE_MST_HPP

#include "graph/Graph.hpp"
#include <string>
#include <vector>

namespace drone {

struct MstEdge {
    std::string u;
    std::string v;
    double      weight = 0.0;
};

struct MstResult {
    bool                 connected = false;  // false if graph is disconnected
    std::vector<MstEdge> edges;              // the spanning tree
    double               total_cost = 0.0;
    int                  components = 0;     // number of connected components
};

// Kruskal's MST on a projection. Time O(E log E) with union-find.
// Used on the relay projection to find the minimum-cost radio backbone.
MstResult kruskal(const Projection& p);

// Critical links = bridges of the relay graph: edges whose removal disconnects
// the graph. These are single points of failure with no redundant path.
// Time O(V + E) via DFS low-link. Returned as undirected id pairs.
std::vector<MstEdge> bridges(const Projection& p);

}  // namespace drone

#endif  // DRONE_MST_HPP
