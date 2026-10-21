#pragma once

#include <vector>

namespace cuopt_intra_factory_viz {

enum class WaypointRole { kDepot, kStation, kTransit };

struct Waypoint {
  int id;
  double x;
  double y;
  WaypointRole role;
};

struct Edge {
  int from;
  int to;
};

// The 11-node intra-factory graph: node 0 is AMR 1's depot (start/return
// point), node 10 is AMR 2's depot, 4/5/6 are the pickup/delivery stations,
// the rest are unlabeled transit junctions.
// Only connectivity is hardcoded here -- edge length is always derived from
// the two waypoints' coordinates (see is_edge()), never duplicated as a
// literal, so it can't drift out of sync if a waypoint's position changes.
const std::vector<Waypoint>& waypoints();
const std::vector<Edge>& edges();

const Waypoint& waypoint_by_id(int id);

// True if `from` and `to` are directly connected (either direction), with
// `weight` set to the Euclidean distance between them in meters.
bool is_edge(int from, int to, double& weight);

// Heading (radians, atan2 convention) of the straight line from `from_id` to
// `to_id`. Does not require the pair to be a graph edge.
double heading_between(int from_id, int to_id);

}  // namespace cuopt_intra_factory_viz
