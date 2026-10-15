#pragma once

#include <vector>

namespace cuopt_intra_factory_viz {

// Dijkstra shortest path between two waypoints, weighted by edge length
// (see waypoint_graph.hpp's is_edge()). Returns the full sequence of
// waypoint ids from `from_id` to `to_id`, inclusive of both endpoints.
// Throws std::invalid_argument if either id is unknown or no path exists
// (the latter can't actually happen on this graph -- it's connected -- but
// the check is kept in case the graph is ever edited to no longer be).
std::vector<int> shortest_path(int from_id, int to_id);

// Concatenates shortest_path() legs for consecutive stops in `stops` (e.g.
// {0, 4, 6, 0} for a depot -> station 4 -> station 6 -> depot tour) into one
// continuous walk, dropping the duplicate waypoint at each leg boundary.
// Note this optimizes each leg independently, not the tour as a whole: a
// station reachable only through a single corridor (like 4, 5, 6 here) can
// still force the same corridor to be walked twice in a row when the next
// leg doubles back through it. Finding the globally shortest *tour* over
// several stops is a harder problem -- that's what wiring in an actual
// cuOpt solve (see README's Known limitations) is for.
std::vector<int> build_tour(const std::vector<int>& stops);

}  // namespace cuopt_intra_factory_viz
