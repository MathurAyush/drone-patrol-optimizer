#include "algorithms/MaxFlow.hpp"

#include <queue>
#include <limits>
#include <algorithm>

namespace drone {

namespace {
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double EPS = 1e-9;
}

MaxFlowResult edmondsKarp(const std::vector<std::string>& nodes,
                          const std::vector<FlowArc>& arcs,
                          const std::string& source,
                          const std::string& sink) {
    MaxFlowResult result;

    std::unordered_map<std::string, int> idx;
    for (const auto& id : nodes) {
        if (!idx.count(id)) idx[id] = static_cast<int>(idx.size());
    }
    auto ensure = [&](const std::string& id) {
        auto it = idx.find(id);
        if (it != idx.end()) return it->second;
        int v = static_cast<int>(idx.size());
        idx[id] = v;
        return v;
    };
    int s = ensure(source), t = ensure(sink);

    // Residual graph as edge list with paired reverse edges.
    struct REdge { int to; double cap; int rev; };
    int n0 = static_cast<int>(idx.size());
    for (const auto& a : arcs) { ensure(a.from); ensure(a.to); }
    int n = static_cast<int>(idx.size());
    (void)n0;

    std::vector<std::vector<REdge>> g(n);
    // Track original arcs so we can report their final flow.
    struct OrigRef { int u, ei; double cap; std::string from, to; };
    std::vector<OrigRef> originals;

    auto addEdge = [&](int u, int v, double cap) -> int {
        g[u].push_back({v, cap, static_cast<int>(g[v].size())});
        g[v].push_back({u, 0.0, static_cast<int>(g[u].size()) - 1});
        return static_cast<int>(g[u].size()) - 1;
    };

    for (const auto& a : arcs) {
        int u = idx[a.from], v = idx[a.to];
        int ei = addEdge(u, v, a.cap);
        originals.push_back({u, ei, a.cap, a.from, a.to});
    }

    std::vector<std::string> indexToId(n);
    for (const auto& kv : idx) indexToId[kv.second] = kv.first;

    // BFS to find a shortest augmenting path; augment; repeat.
    double total = 0.0;
    while (true) {
        std::vector<int> parentNode(n, -1), parentEdge(n, -1);
        parentNode[s] = s;
        std::queue<int> q;
        q.push(s);
        while (!q.empty() && parentNode[t] == -1) {
            int u = q.front();
            q.pop();
            for (int i = 0; i < static_cast<int>(g[u].size()); ++i) {
                const REdge& e = g[u][i];
                if (e.cap > EPS && parentNode[e.to] == -1) {
                    parentNode[e.to] = u;
                    parentEdge[e.to] = i;
                    q.push(e.to);
                }
            }
        }
        if (parentNode[t] == -1) break;  // no augmenting path

        // Bottleneck along the path.
        double bottleneck = INF;
        for (int v = t; v != s; v = parentNode[v]) {
            const REdge& e = g[parentNode[v]][parentEdge[v]];
            bottleneck = std::min(bottleneck, e.cap);
        }
        for (int v = t; v != s; v = parentNode[v]) {
            REdge& e = g[parentNode[v]][parentEdge[v]];
            e.cap -= bottleneck;
            g[v][e.rev].cap += bottleneck;
        }
        total += bottleneck;
    }
    result.max_flow = total;

    // Final flow on each original arc = capacity - residual capacity.
    for (const auto& o : originals) {
        double flow = o.cap - g[o.u][o.ei].cap;
        if (flow > EPS)
            result.flows.push_back({o.from, o.to, flow, o.cap});
    }

    // Min cut: nodes reachable from s in the residual graph.
    std::vector<char> reach(n, 0);
    std::queue<int> q;
    q.push(s);
    reach[s] = 1;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (const REdge& e : g[u]) {
            if (e.cap > EPS && !reach[e.to]) {
                reach[e.to] = 1;
                q.push(e.to);
            }
        }
    }
    for (int i = 0; i < n; ++i)
        if (reach[i]) result.source_side.push_back(indexToId[i]);

    for (const auto& a : arcs) {
        int u = idx[a.from], v = idx[a.to];
        if (reach[u] && !reach[v])  // crosses the cut, source-side -> sink-side
            result.min_cut_edges.push_back({a.from, a.to, a.cap, a.cap});
    }
    return result;
}

}  // namespace drone
