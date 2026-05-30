#include "algorithms/Tsp.hpp"

#include <limits>
#include <algorithm>
#include <stdexcept>

namespace drone {

namespace {
constexpr double INF = std::numeric_limits<double>::infinity();
}

TspResult heldKarp(const Matrix& dist, int start) {
    int n = static_cast<int>(dist.size());
    TspResult res;
    if (n == 0) return res;
    if (n == 1) { res.tour = {start}; res.optimal = true; return res; }
    if (n > 24)
        throw std::invalid_argument("heldKarp: n too large (n>24), use nearestNeighbour");

    const uint32_t FULL = (1u << n) - 1;
    // dp[mask][j]: min cost of a path that starts at `start`, visits exactly the
    // set `mask` (which includes start and j), and ends at j.
    std::vector<std::vector<double>> dp(1u << n, std::vector<double>(n, INF));
    std::vector<std::vector<int>>    par(1u << n, std::vector<int>(n, -1));

    dp[1u << start][start] = 0.0;

    for (uint32_t mask = 0; mask <= FULL; ++mask) {
        if (!(mask & (1u << start))) continue;  // must contain start
        for (int j = 0; j < n; ++j) {
            if (dp[mask][j] == INF) continue;
            for (int k = 0; k < n; ++k) {
                if (mask & (1u << k)) continue;       // k already visited
                uint32_t nm = mask | (1u << k);
                double nd = dp[mask][j] + dist[j][k];
                if (nd < dp[nm][k]) {
                    dp[nm][k] = nd;
                    par[nm][k] = j;
                }
            }
        }
    }

    // Close the tour back to start.
    double best = INF;
    int last = -1;
    for (int j = 0; j < n; ++j) {
        if (j == start) continue;
        double total = dp[FULL][j] + dist[j][start];
        if (total < best) { best = total; last = j; }
    }

    // Reconstruct.
    std::vector<int> rev;
    uint32_t mask = FULL;
    int cur = last;
    while (cur != -1) {
        rev.push_back(cur);
        int p = par[mask][cur];
        mask &= ~(1u << cur);
        cur = p;
    }
    std::reverse(rev.begin(), rev.end());
    res.tour = rev;
    res.length = best;
    res.optimal = true;
    return res;
}

TspResult nearestNeighbour(const Matrix& dist, int start) {
    int n = static_cast<int>(dist.size());
    TspResult res;
    if (n == 0) return res;

    std::vector<char> visited(n, 0);
    res.tour.push_back(start);
    visited[start] = 1;
    int cur = start;
    double total = 0.0;

    for (int step = 1; step < n; ++step) {
        int next = -1;
        double bestD = INF;
        for (int j = 0; j < n; ++j) {
            if (!visited[j] && dist[cur][j] < bestD) {
                bestD = dist[cur][j];
                next = j;
            }
        }
        if (next == -1) break;
        visited[next] = 1;
        res.tour.push_back(next);
        total += bestD;
        cur = next;
    }
    total += dist[cur][start];  // return leg
    res.length = total;
    res.optimal = false;
    return res;
}

}  // namespace drone
