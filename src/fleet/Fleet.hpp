#ifndef DRONE_FLEET_HPP
#define DRONE_FLEET_HPP

#include "graph/Graph.hpp"
#include "algorithms/ShortestPath.hpp"
#include <string>
#include <vector>

namespace drone {

enum class DroneState { Patrolling, EnRoute, Returning, Charging };

inline std::string droneStateToString(DroneState s) {
    switch (s) {
        case DroneState::Patrolling: return "PATROLLING";
        case DroneState::EnRoute:    return "EN_ROUTE";
        case DroneState::Returning:  return "RETURNING";
        case DroneState::Charging:   return "CHARGING";
    }
    return "UNKNOWN";
}

struct DroneAgent {
    std::string id;
    std::string position;          // current node id
    std::string home_base;
    double      battery_pct = 100.0;
    double      speed_kmh = 60.0;
    double      flight_hours_left = 6.0;
    DroneState  state = DroneState::Charging;
    std::string assigned_target;
};

class DroneFleet {
public:
    void add(const DroneAgent& d) { drones_.push_back(d); }
    std::vector<DroneAgent>& drones() { return drones_; }
    const std::vector<DroneAgent>& drones() const { return drones_; }

    // Which available drone reaches `target` fastest, by shortest flight-path
    // cost from its current position? Returns index into drones() or -1.
    // Travel time = path cost (km) / speed (km/h).
    int nearestAvailable(const Projection& flight, const std::string& target,
                         double& bestTimeHours, PathResult& bestPath) const {
        int best = -1;
        bestTimeHours = 1e18;
        for (int i = 0; i < static_cast<int>(drones_.size()); ++i) {
            const auto& d = drones_[i];
            if (d.state == DroneState::Charging && d.battery_pct < 20.0) continue;
            PathResult r = dijkstra(flight, d.position, target);
            if (!r.found) continue;
            double t = r.cost / (d.speed_kmh > 0 ? d.speed_kmh : 1.0);
            if (t < bestTimeHours) {
                bestTimeHours = t;
                best = i;
                bestPath = r;
            }
        }
        return best;
    }

private:
    std::vector<DroneAgent> drones_;
};

}  // namespace drone

#endif  // DRONE_FLEET_HPP
