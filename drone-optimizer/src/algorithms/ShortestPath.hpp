#ifndef DRONE_SHORTEST_PATH_HPP
#define DRONE_SHORTEST_PATH_HPP

#include "graph/Graph.hpp"
#include <string>
#include <vector>
#include <functional>

namespace drone {

struct PathResult {
    bool                     found = false;
    std::vector<std::string> path;            // node ids, source..target
    double                   cost = 0.0;
    int                      nodes_expanded = 0;  // popped from the frontier
};

// Exact shortest path on a non-negative weighted projection.
// O((V+E) log V) with a binary heap.
PathResult dijkstra(const Projection& p,
                    const std::string& from,
                    const std::string& to);

// A heuristic estimates the remaining cost from a local node index to the goal.
// It must be admissible (never overestimate) for A* to stay optimal.
using Heuristic = std::function<double(int localIndex)>;

// A* — Dijkstra guided by an admissible heuristic. Expands far fewer nodes than
// Dijkstra on spatially-structured graphs.
PathResult astar(const Projection& p,
                 const std::string& from,
                 const std::string& to,
                 const Heuristic& h);

// Convenience: straight-line (Euclidean) heuristic factory for the flight
// projection. Distance is a provable lower bound on effective cost
// (effective_cost = distance * (1 + wind) >= distance), so it is admissible.
Heuristic euclideanHeuristic(const Graph& g, const Projection& p,
                             const std::string& goal);

}  // namespace drone

#endif  // DRONE_SHORTEST_PATH_HPP
