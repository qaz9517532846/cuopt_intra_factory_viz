#include "cuopt_intra_factory_viz/waypoint_graph.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace cuopt_intra_factory_viz {

const std::vector<Waypoint>& waypoints() {
  static const std::vector<Waypoint> kWaypoints = {
    {0, -4.0, 0.0, WaypointRole::kDepot},
    {1, -2.0, -2.0, WaypointRole::kTransit},
    {2, -2.0, 0.0, WaypointRole::kTransit},
    {3, -2.0, 2.0, WaypointRole::kTransit},
    {4, 0.0, -2.0, WaypointRole::kStation},
    {5, 0.0, 0.0, WaypointRole::kStation},
    {6, 0.0, 2.0, WaypointRole::kStation},
    {7, 1.0, -2.0, WaypointRole::kTransit},
    {8, 1.0, 0.0, WaypointRole::kTransit},
    {9, 1.0, 2.0, WaypointRole::kTransit},
    {10, 3.0, 0.0, WaypointRole::kDepot},
  };
  return kWaypoints;
}

const std::vector<Edge>& edges() {
  static const std::vector<Edge> kEdges = {
    {0, 2}, {1, 2}, {2, 3}, {1, 4}, {2, 5},
    {3, 6}, {4, 7}, {5, 8}, {6, 9}, {7, 8}, {8, 9}, {8, 10},
  };
  return kEdges;
}

const Waypoint& waypoint_by_id(int id) {
  for (const auto& wp : waypoints()) {
    if (wp.id == id) return wp;
  }
  throw std::out_of_range("waypoint_by_id: no waypoint with id " + std::to_string(id));
}

bool is_edge(int from, int to, double& weight) {
  for (const auto& e : edges()) {
    if ((e.from == from && e.to == to) || (e.from == to && e.to == from)) {
      const Waypoint& a = waypoint_by_id(e.from);
      const Waypoint& b = waypoint_by_id(e.to);
      weight = std::hypot(b.x - a.x, b.y - a.y);
      return true;
    }
  }
  return false;
}

double heading_between(int from_id, int to_id) {
  const Waypoint& a = waypoint_by_id(from_id);
  const Waypoint& b = waypoint_by_id(to_id);
  return std::atan2(b.y - a.y, b.x - a.x);
}

}  // namespace cuopt_intra_factory_viz
