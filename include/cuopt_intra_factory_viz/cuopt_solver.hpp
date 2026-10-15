#pragma once

#include <vector>

#include "cuopt_intra_factory_viz/diff_drive_path.hpp"

namespace cuopt_intra_factory_viz {

// A solved, fully-expanded route for one AMR: the closed-loop sequence of
// graph waypoint ids to drive (first == last == depot), plus the dwell
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

// Solves the Intra-factory Transport scenario -- 6 pickup-delivery pairs
// among stations A(4)/B(5)/C(6), 2 AMRs with capacity 2, time windows and
// 10s service times -- via a real cuOpt solve over this package's 10-node
// factory floor graph (see waypoint_graph.cpp), and expands the result back
// to waypoint-level paths via cuopt::distance_engine::waypoint_matrix_t.
// This is the only entry point in the package that touches CUDA/RAFT/RMM/
// libcuopt; the implementation lives in cuopt_solver.cu so every other file
// stays plain C++.
//
// Throws std::runtime_error if the solve does not succeed.
SolveResult solve_intra_factory_transport();

}  // namespace cuopt_intra_factory_viz
