#include "algorithms/ShortestPath.hpp"

#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <memory>

namespace drone {

namespace {
constexpr double INF = std::numeric_limits<double>::infinity();

PathResult reconstruct(const Projection& p, const std::vector<int>& prev,
                       int src, int dst, double cost, int expanded) {
    PathResult r;
    r.found = true;
    r.cost = cost;
    r.nodes_expanded = expanded;
    std::vector<int> rev;
    for (int cur = dst; cur != -1; cur = prev[cur]) {
        rev.push_back(cur);
        if (cur == src) break;
    }
    std::reverse(rev.begin(), rev.end());
    for (int idx : rev) r.path.push_back(p.indexToId[idx]);
    return r;
}
}  // namespace

PathResult dijkstra(const Projection& p, const std::string& from,
                    const std::string& to) {
    int src = p.index(from), dst = p.index(to);
    if (src < 0 || dst < 0) return {};

    std::vector<double> dist(p.size(), INF);
    std::vector<int>    prev(p.size(), -1);
    std::vector<char>   done(p.size(), 0);
    using QE = std::pair<double, int>;  // (dist, node)
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;

    dist[src] = 0.0;
    pq.push({0.0, src});
    int expanded = 0;

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (done[u]) continue;
        done[u] = 1;
        ++expanded;
        if (u == dst) return reconstruct(p, prev, src, dst, dist[dst], expanded);
        for (const auto& a : p.adj[u]) {
            double nd = d + a.weight;
            if (nd < dist[a.to]) {
                dist[a.to] = nd;
                prev[a.to] = u;
                pq.push({nd, a.to});
            }
        }
    }
    return {};  // unreachable
}

PathResult astar(const Projection& p, const std::string& from,
                 const std::string& to, const Heuristic& h) {
    int src = p.index(from), dst = p.index(to);
    if (src < 0 || dst < 0) return {};

    std::vector<double> g(p.size(), INF);
    std::vector<int>    prev(p.size(), -1);
    std::vector<char>   done(p.size(), 0);
    using QE = std::pair<double, int>;  // (f = g + h, node)
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;

    g[src] = 0.0;
    pq.push({h(src), src});
    int expanded = 0;

    while (!pq.empty()) {
        auto [f, u] = pq.top();
        pq.pop();
        if (done[u]) continue;
        done[u] = 1;
        ++expanded;
        if (u == dst) return reconstruct(p, prev, src, dst, g[dst], expanded);
        for (const auto& a : p.adj[u]) {
            double ng = g[u] + a.weight;
            if (ng < g[a.to]) {
                g[a.to] = ng;
                prev[a.to] = u;
                pq.push({ng + h(a.to), a.to});
            }
        }
    }
    return {};
}

Heuristic euclideanHeuristic(const Graph& gph, const Projection& p,
                             const std::string& goal) {
    double gx = 0, gy = 0;
    gph.coords(goal, gx, gy);
    // Precompute coordinates per local index for speed.
    auto xs = std::make_shared<std::vector<double>>(p.size());
    auto ys = std::make_shared<std::vector<double>>(p.size());
    for (int i = 0; i < p.size(); ++i) {
        double x = 0, y = 0;
        gph.coords(p.indexToId[i], x, y);
        (*xs)[i] = x;
        (*ys)[i] = y;
    }
    return [xs, ys, gx, gy](int i) {
        double dx = (*xs)[i] - gx, dy = (*ys)[i] - gy;
        return std::sqrt(dx * dx + dy * dy);
    };
}

}  // namespace drone
