#pragma once

#include <vector>

#include "cuopt_intra_factory_viz/diff_drive_path.hpp"

namespace cuopt_intra_factory_viz {

// A solved, fully-expanded route for one AMR: the closed-loop sequence of
// graph waypoint ids to drive (first == that vehicle's start node for this
// particular solve, last == its return node -- see ReplanRequest; always
// its home depot, even once rolling re-solves are in play), plus the dwell
// events (pickups/deliveries) positioned along it.
struct VehiclePlan {
  int vehicle_id;
  std::vector<int> waypoint_path;
  std::vector<DwellEvent> dwell_events;
};

struct SolveResult {
  double total_objective;  // result.get_total_objective() -- see README's "cuOpt 的 cost 怎麼算"
  std::vector<VehiclePlan> plans;
};

// Lifecycle of one of this demo's 6 recurring pickup-delivery "slots".
// kPending means neither end has happened yet -- solved as a normal
// pickup-then-delivery visit to its two real stations. kOnboard means the
// pickup already happened for real and `holder_vehicle_id` is physically
// carrying it -- solved as a zero-distance synthetic "pickup" at that
// vehicle's own current position (service time 0, so it costs nothing and
// inserts no visible dwell) followed by the real delivery, and pinned to
// that vehicle so cuOpt can't hand already-loaded cargo to a different AMR.
enum class PairState { kPending, kOnboard };

struct PairStatus {
  PairState state = PairState::kPending;
  int holder_vehicle_id = -1;  // valid, and required, when state == kOnboard
};

// One cuOpt solve request. `vehicle_start_node[v]` is where vehicle v
// starts this solve from -- for the startup solve that's its own depot;
// for a later rolling re-solve it's wherever that AMR physically is right
// now (so the new route is a seamless continuation, not a teleport).
// `vehicle_return_node[v]` is always that vehicle's home depot, so every
// re-solve still routes it back home at the end of the loop instead of
// permanently re-anchoring it to wherever a re-solve happened to catch it.
// `congested_nodes_by_vehicle[v]` lists graph nodes currently locked by some
// *other* AMR (from TrafficController), used to weight vehicle v's own
// cost/transit-time matrix away from them -- so cuOpt's own route choice
// starts avoiding live congestion instead of only reacting to it at
// execution time. Empty for the startup solve.
struct ReplanRequest {
  std::vector<int> vehicle_start_node;
  std::vector<int> vehicle_return_node;
  std::vector<PairStatus> pairs;  // size == kNumPairs, same order every call
  std::vector<std::vector<int>> congested_nodes_by_vehicle;  // size == vehicle_start_node.size()
};

inline constexpr int kNumPairs = 6;

// The startup request: every kDepot-role waypoint (see waypoint_graph.cpp)
// becomes one vehicle's start/return node, in ascending waypoint-id order --
// one AMR per depot -- all pairs pending, no congestion yet.
ReplanRequest initial_replan_request();

// Solves the Intra-factory Transport scenario -- 6 pickup-delivery pairs
// among stations A(4)/B(5)/C(6), time windows and 10s service times -- via
// a real cuOpt solve over this package's factory floor graph (see
// waypoint_graph.cpp), and expands the result back to waypoint-level paths
// via cuopt::distance_engine::waypoint_matrix_t. This is the only entry
// point in the package that touches CUDA/RAFT/RMM/libcuopt; the
// implementation lives in cuopt_solver.cu so every other file stays plain
// C++.
//
// Throws std::runtime_error if the solve does not succeed.
SolveResult solve_intra_factory_transport(const ReplanRequest& request);

}  // namespace cuopt_intra_factory_viz
