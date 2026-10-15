#include "cuopt_intra_factory_viz/cuopt_solver.hpp"

#include <raft/core/handle.hpp>
#include <rmm/device_uvector.hpp>
#include <cuopt/routing/data_model_view.hpp>
#include <cuopt/routing/solver_settings.hpp>
#include <cuopt/routing/solve.hpp>
#include <cuopt/routing/distance_engine/waypoint_matrix.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cuopt_intra_factory_viz/copy_helpers.hpp"
#include "cuopt_intra_factory_viz/waypoint_graph.hpp"

namespace cuopt_intra_factory_viz {

namespace {

constexpr double kServiceTimeSec = 10.0;

// compute_cost_matrix() returns distances in meters (the CSR graph's edge
// weights are meters -- see waypoint_graph.cpp). Without a separate
// add_transit_time_matrix() call, cuOpt would reuse that meters-valued cost
// matrix as the travel time for order_earliest/order_latest feasibility
// checking too, making the "seconds" in this file's time windows meaningless
// (implicitly 1 meter == 1 second). Converting by this assumed speed --
// matching vehicle_tf_broadcaster_node's default v_max parameter -- gives a
// real transit_time_matrix in actual seconds instead.
constexpr float kAssumedSpeedMps = 0.5f;

// Divides every cost-matrix entry by kAssumedSpeedMps to turn meters into
// seconds; used as a thrust::transform functor (a struct rather than a
// device lambda so no --extended-lambda nvcc flag is needed).
struct MetersToSeconds {
  float speed_mps;
  __host__ __device__ float operator()(float meters) const { return meters / speed_mps; }
};

// Builds the CSR graph (offsets/indices/weights) straight from
// waypoint_graph.cpp's waypoints()/edges() -- the single source of truth
// this package's RViz2 visualization also reads -- rather than duplicating
// the factory floor layout as a second, driftable copy here.
void build_csr(std::vector<int>& offsets, std::vector<int>& indices, std::vector<float>& weights) {
  const auto& wps = waypoints();
  const int n = static_cast<int>(wps.size());
  std::vector<std::vector<std::pair<int, double>>> adjacency(n);
  for (const auto& e : edges()) {
    double w;
    is_edge(e.from, e.to, w);
    adjacency[e.from].push_back({e.to, w});
    adjacency[e.to].push_back({e.from, w});
  }
  offsets.assign(n + 1, 0);
  indices.clear();
  weights.clear();
  for (int i = 0; i < n; ++i) {
    offsets[i + 1] = offsets[i] + static_cast<int>(adjacency[i].size());
    for (const auto& [dst, w] : adjacency[i]) {
      indices.push_back(dst);
      weights.push_back(static_cast<float>(w));
    }
  }
}

// Compressed target-location index 0 is the depot; 1..N are the pickup/
// delivery stations, in ascending waypoint-id order (so {0, 4, 5, 6} here).
std::vector<int> compute_target_locations() {
  int depot_id = -1;
  std::vector<int> station_ids;
  for (const auto& wp : waypoints()) {
    if (wp.role == WaypointRole::kDepot) {
      depot_id = wp.id;
    } else if (wp.role == WaypointRole::kStation) {
      station_ids.push_back(wp.id);
    }
  }
  if (depot_id < 0) throw std::runtime_error("solve_intra_factory_transport: no depot waypoint");
  std::vector<int> targets = {depot_id};
  targets.insert(targets.end(), station_ids.begin(), station_ids.end());
  return targets;
}

}  // namespace

SolveResult solve_intra_factory_transport() {
  using namespace cuopt::routing;
  using cuopt::distance_engine::waypoint_matrix_t;

  raft::handle_t handle;
  auto stream = handle.get_stream();

  std::vector<int> h_offsets, h_indices;
  std::vector<float> h_weights;
  build_csr(h_offsets, h_indices, h_weights);
  const std::vector<int> target_locations = compute_target_locations();
  const int n_targets = static_cast<int>(target_locations.size());

  waypoint_matrix_t<int, float> graph(handle, h_offsets.data(),
                                       static_cast<int>(h_offsets.size()) - 1, h_indices.data(),
                                       h_weights.data());

  rmm::device_uvector<float> d_cost_matrix(static_cast<size_t>(n_targets) * n_targets, stream);
  graph.compute_cost_matrix(d_cost_matrix.data(), target_locations.data(), n_targets);
  handle.sync_stream();

  // Real seconds for time-window feasibility, not a meters-as-seconds stand-in.
  rmm::device_uvector<float> d_transit_time_matrix(static_cast<size_t>(n_targets) * n_targets, stream);
  thrust::transform(rmm::exec_policy(stream), d_cost_matrix.begin(), d_cost_matrix.end(),
                     d_transit_time_matrix.begin(), MetersToSeconds{kAssumedSpeedMps});
  handle.sync_stream();

  // The scenario: 6 pickup-delivery pairs among stations A(compressed 1) /
  // B(compressed 2) / C(compressed 3), 2 AMRs capacity 2. Mirrors the
  // official Intra-factory Transport Python example and cuopt_routing_node's
  // intra_factory_transport_demo.
  const int fleet_size = 2;
  const int num_orders = 12;

  std::vector<int> h_order_locations = {
    1, 2,  // pair 1: A -> B
    2, 3,  // pair 2: B -> C
    3, 1,  // pair 3: C -> A
    1, 3,  // pair 4: A -> C
    2, 1,  // pair 5: B -> A
    3, 2,  // pair 6: C -> B
  };
  std::vector<int> h_pickup_orders = {0, 2, 4, 6, 8, 10};
  std::vector<int> h_delivery_orders = {1, 3, 5, 7, 9, 11};
  std::vector<int> h_demands = {1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1};
  std::vector<int> h_order_earliest = {0, 0, 0, 0, 0, 0, 20, 20, 20, 20, 20, 20};
  std::vector<int> h_order_latest = {60, 120, 60, 120, 60, 120, 90, 150, 90, 150, 90, 150};
  std::vector<int> h_service_times(num_orders, static_cast<int>(kServiceTimeSec));

  std::vector<int> h_capacities = {2, 2};
  std::vector<int> h_vehicle_start = {0, 0};
  std::vector<int> h_vehicle_end = {0, 0};
  std::vector<int> h_vehicle_earliest = {0, 0};
  std::vector<int> h_vehicle_latest = {300, 300};

  auto v_order_locations = cuopt::device_copy(h_order_locations, stream);
  auto v_pickup_orders = cuopt::device_copy(h_pickup_orders, stream);
  auto v_delivery_orders = cuopt::device_copy(h_delivery_orders, stream);
  auto v_demands = cuopt::device_copy(h_demands, stream);
  auto v_order_earliest = cuopt::device_copy(h_order_earliest, stream);
  auto v_order_latest = cuopt::device_copy(h_order_latest, stream);
  auto v_service_times = cuopt::device_copy(h_service_times, stream);
  auto v_capacities = cuopt::device_copy(h_capacities, stream);
  auto v_vehicle_start = cuopt::device_copy(h_vehicle_start, stream);
  auto v_vehicle_end = cuopt::device_copy(h_vehicle_end, stream);
  auto v_vehicle_earliest = cuopt::device_copy(h_vehicle_earliest, stream);
  auto v_vehicle_latest = cuopt::device_copy(h_vehicle_latest, stream);

  data_model_view_t<int, float> data_model(&handle, n_targets, fleet_size, num_orders);
  data_model.add_cost_matrix(d_cost_matrix.data());
  data_model.add_transit_time_matrix(d_transit_time_matrix.data());
  data_model.set_order_locations(v_order_locations.data());
  data_model.set_pickup_delivery_pairs(v_pickup_orders.data(), v_delivery_orders.data());
  data_model.add_capacity_dimension("parts", v_demands.data(), v_capacities.data());
  data_model.set_order_time_windows(v_order_earliest.data(), v_order_latest.data());
  data_model.set_order_service_times(v_service_times.data());
  data_model.set_vehicle_locations(v_vehicle_start.data(), v_vehicle_end.data());
  data_model.set_vehicle_time_windows(v_vehicle_earliest.data(), v_vehicle_latest.data());

  solver_settings_t<int, float> settings;
  settings.set_time_limit(5.0f);

  assignment_t<int> result = solve(data_model, settings);
  handle.sync_stream();

  if (result.get_status() != solution_status_t::SUCCESS) {
    throw std::runtime_error("solve_intra_factory_transport: solve failed: " +
                              result.get_status_string() + " (" + result.get_error_status().what() +
                              ")");
  }

  host_assignment_t<int> host_route(result);

  // cuOpt gives a pickup and its matching delivery two different order
  // indices (e.g. pair 1 here is pickup order 2 + delivery order 3) -- fine
  // for the solver, but confusing for cargo tracking downstream, where
  // "loaded order 2" and "unloaded order 3" should read as the same
  // shipment. Map both halves of each pair to one shared pair id instead.
  std::vector<int> pair_id_of_order(num_orders, -1);
  for (size_t k = 0; k < h_pickup_orders.size(); ++k) {
    pair_id_of_order[h_pickup_orders[k]] = static_cast<int>(k);
    pair_id_of_order[h_delivery_orders[k]] = static_cast<int>(k);
  }

  // Split the flat (locations/node_types/route/truck_id) arrays by vehicle.
  struct Stop {
    int compressed_location;
    int node_type;  // node_type_t: DEPOT=0, PICKUP=1, DELIVERY=2, BREAK=3
    int pair_id;     // shared between a pickup and its matching delivery; -1 at depot/break
  };
  std::vector<int> truck_ids;
  std::vector<std::vector<Stop>> per_vehicle_stops;
  int current_truck = -1;
  for (size_t i = 0; i < host_route.locations.size(); ++i) {
    if (host_route.truck_id[i] != current_truck) {
      current_truck = host_route.truck_id[i];
      truck_ids.push_back(current_truck);
      per_vehicle_stops.emplace_back();
    }
    const int order_id = host_route.route[i];
    const int pair_id = (order_id >= 0 && order_id < num_orders) ? pair_id_of_order[order_id] : -1;
    per_vehicle_stops.back().push_back(Stop{host_route.locations[i], host_route.node_types[i], pair_id});
  }

  std::vector<VehiclePlan> plans;
  for (size_t v = 0; v < per_vehicle_stops.size(); ++v) {
    const auto& stops = per_vehicle_stops[v];
    std::vector<int> compressed_route;
    compressed_route.reserve(stops.size());
    for (const auto& s : stops) compressed_route.push_back(s.compressed_location);

    auto v_route = cuopt::device_copy(compressed_route, stream);
    auto waypoint_result = graph.compute_waypoint_sequence(
      target_locations.data(), n_targets, v_route.data(), static_cast<int>(compressed_route.size()));
    handle.sync_stream();

    const int* offsets_ptr = static_cast<const int*>(waypoint_result.first->data());
    const size_t n_offsets = waypoint_result.first->size() / sizeof(int);
    const int* path_ptr = static_cast<const int*>(waypoint_result.second->data());
    const size_t n_path_elems = waypoint_result.second->size() / sizeof(int);
    auto h_offsets = cuopt::host_copy(offsets_ptr, n_offsets, stream);
    auto h_path = cuopt::host_copy(path_ptr, n_path_elems, stream);

    // Reassemble one continuous waypoint-level walk (see
    // cuopt_routing_node's README for why this dedup is needed: each hop's
    // sub-array in h_path repeats the boundary node the previous hop ended
    // on), while recording which index in the final walk corresponds to the
    // *arrival* at each original stop -- that's where a dwell belongs.
    VehiclePlan plan;
    plan.vehicle_id = truck_ids[v];
    std::vector<std::size_t> stop_to_path_index(stops.size(), 0);
    for (std::size_t hop = 0; hop + 1 < h_offsets.size(); ++hop) {
      const int seg_start = h_offsets[hop] + (hop == 0 ? 0 : 1);
      const int seg_end = h_offsets[hop + 1];
      for (int k = seg_start; k < seg_end; ++k) plan.waypoint_path.push_back(h_path[k]);
      stop_to_path_index[hop + 1] = plan.waypoint_path.size() - 1;
    }

    for (std::size_t s = 0; s < stops.size(); ++s) {
      const auto node_type = stops[s].node_type;
      if (node_type == static_cast<int>(node_type_t::PICKUP) ||
          node_type == static_cast<int>(node_type_t::DELIVERY)) {
        plan.dwell_events.push_back(DwellEvent{stop_to_path_index[s], kServiceTimeSec,
                                                stops[s].pair_id,
                                                node_type == static_cast<int>(node_type_t::PICKUP)});
      }
    }
    plans.push_back(std::move(plan));
  }

  return SolveResult{result.get_total_objective(), std::move(plans)};
}

}  // namespace cuopt_intra_factory_viz
