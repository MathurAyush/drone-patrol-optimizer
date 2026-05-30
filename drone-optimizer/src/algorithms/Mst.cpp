#include "algorithms/Mst.hpp"

#include <algorithm>
#include <numeric>
#include <vector>

namespace drone {

namespace {
struct DSU {
    std::vector<int> p, r;
    explicit DSU(int n) : p(n), r(n, 0) { std::iota(p.begin(), p.end(), 0); }
    int find(int x) { return p[x] == x ? x : p[x] = find(p[x]); }
    bool unite(int a, int b) {
        a = find(a); b = find(b);
        if (a == b) return false;
        if (r[a] < r[b]) std::swap(a, b);
        p[b] = a;
        if (r[a] == r[b]) ++r[a];
        return true;
    }
};
}  // namespace

MstResult kruskal(const Projection& p) {
    MstResult res;
    int n = p.size();
    if (n == 0) { res.connected = true; res.components = 0; return res; }

    // Collect undirected edges once (projection adjacency stores both directions).
    struct E { int u, v; double w; };
    std::vector<E> edges;
    for (int u = 0; u < n; ++u)
        for (const auto& a : p.adj[u])
            if (u < a.to) edges.push_back({u, a.to, a.weight});

    std::sort(edges.begin(), edges.end(),
              [](const E& a, const E& b) { return a.w < b.w; });

    DSU dsu(n);
    int used = 0;
    for (const auto& e : edges) {
        if (dsu.unite(e.u, e.v)) {
            res.edges.push_back({p.indexToId[e.u], p.indexToId[e.v], e.w});
            res.total_cost += e.w;
            if (++used == n - 1) break;
        }
    }

    int comps = 0;
    for (int i = 0; i < n; ++i)
        if (dsu.find(i) == i) ++comps;
    res.components = comps;
    res.connected = (comps == 1);
    return res;
}

std::vector<MstEdge> bridges(const Projection& p) {
    int n = p.size();
    std::vector<MstEdge> out;
    std::vector<int> disc(n, -1), low(n, 0);
    int timer = 0;

    // Iterative DFS to avoid stack overflow on large graphs.
    // We track the parent edge index to skip the immediate back-edge.
    std::vector<int> parent(n, -1);
    std::vector<size_t> it(n, 0);

    for (int s = 0; s < n; ++s) {
        if (disc[s] != -1) continue;
        std::vector<int> stack = {s};
        disc[s] = low[s] = timer++;
        while (!stack.empty()) {
            int u = stack.back();
            if (it[u] < p.adj[u].size()) {
                int v = p.adj[u][it[u]].to;
                ++it[u];
                if (v == parent[u]) continue;          // skip edge to parent
                if (disc[v] == -1) {
                    parent[v] = u;
                    disc[v] = low[v] = timer++;
                    stack.push_back(v);
                } else {
                    low[u] = std::min(low[u], disc[v]);  // back edge
                }
            } else {
                stack.pop_back();
                int par = parent[u];
                if (par != -1) {
                    low[par] = std::min(low[par], low[u]);
                    if (low[u] > disc[par])
                        out.push_back({p.indexToId[par], p.indexToId[u], 0.0});
                }
            }
        }
    }
    return out;
}

}  // namespace drone
