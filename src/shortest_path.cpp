#include "cuopt_intra_factory_viz/shortest_path.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "cuopt_intra_factory_viz/waypoint_graph.hpp"

namespace cuopt_intra_factory_viz {

namespace {

struct Neighbor {
  int id;
  double weight;
};

std::unordered_map<int, std::vector<Neighbor>> build_adjacency() {
  std::unordered_map<int, std::vector<Neighbor>> adjacency;
  for (const auto& e : edges()) {
    double weight;
    is_edge(e.from, e.to, weight);  // always true here -- also resolves the distance
    adjacency[e.from].push_back({e.to, weight});
    adjacency[e.to].push_back({e.from, weight});
  }
  return adjacency;
}

}  // namespace

std::vector<int> shortest_path(int from_id, int to_id) {
  waypoint_by_id(from_id);  // throws std::out_of_range if either id is unknown
  waypoint_by_id(to_id);

  if (from_id == to_id) return {from_id};

  const auto adjacency = build_adjacency();

  std::unordered_map<int, double> dist;
  std::unordered_map<int, int> prev;
  for (const auto& wp : waypoints()) dist[wp.id] = std::numeric_limits<double>::infinity();
  dist[from_id] = 0.0;

  using QueueEntry = std::pair<double, int>;  // (distance so far, waypoint id)
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> frontier;
  frontier.push({0.0, from_id});
  std::unordered_map<int, bool> visited;

  while (!frontier.empty()) {
    const int u = frontier.top().second;
    const double d = frontier.top().first;
    frontier.pop();
    if (visited[u]) continue;
    visited[u] = true;
    if (u == to_id) break;

    const auto it = adjacency.find(u);
    if (it == adjacency.end()) continue;
    for (const auto& neighbor : it->second) {
      const double alt = d + neighbor.weight;
      if (alt < dist[neighbor.id]) {
        dist[neighbor.id] = alt;
        prev[neighbor.id] = u;
        frontier.push({alt, neighbor.id});
      }
    }
  }

  if (dist[to_id] == std::numeric_limits<double>::infinity()) {
    throw std::invalid_argument("shortest_path: no path from " + std::to_string(from_id) +
                                 " to " + std::to_string(to_id));
  }

  std::vector<int> path;
  for (int at = to_id;; at = prev.at(at)) {
    path.push_back(at);
    if (at == from_id) break;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<int> build_tour(const std::vector<int>& stops) {
  if (stops.size() < 2) {
    throw std::invalid_argument("build_tour requires at least 2 stops");
  }
  std::vector<int> path;
  for (std::size_t i = 0; i + 1 < stops.size(); ++i) {
    const auto leg = shortest_path(stops[i], stops[i + 1]);
    if (path.empty()) {
      path.insert(path.end(), leg.begin(), leg.end());
    } else {
      path.insert(path.end(), leg.begin() + 1, leg.end());
    }
  }
  return path;
}

}  // namespace cuopt_intra_factory_viz
