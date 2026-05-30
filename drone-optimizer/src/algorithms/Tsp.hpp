#ifndef DRONE_TSP_HPP
#define DRONE_TSP_HPP

#include <vector>
#include <cstdint>

namespace drone {

using Matrix = std::vector<std::vector<double>>;

struct TspResult {
    std::vector<int> tour;     // node indices, starts and ends implicitly at start
    double           length = 0.0;
    bool             optimal = false;
};

// Exact TSP via Held-Karp dynamic programming.
// Time  O(n^2 * 2^n), Space O(n * 2^n). Tractable to ~n <= 18-20.
// `dist` must be a complete square matrix; tour returns to `start`.
TspResult heldKarp(const Matrix& dist, int start = 0);

// Nearest-neighbour heuristic. Time O(n^2). No optimality guarantee, but fast
// enough for real-time rerouting when the checkpoint set changes mid-flight.
TspResult nearestNeighbour(const Matrix& dist, int start = 0);

}  // namespace drone

#endif  // DRONE_TSP_HPP
