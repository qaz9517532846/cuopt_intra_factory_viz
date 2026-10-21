#include "cuopt_intra_factory_viz/cuopt_solver.hpp"

#include <raft/core/handle.hpp>
#include <rmm/device_uvector.hpp>
#include <cuopt/routing/data_model_view.hpp>
#include <cuopt/routing/solver_settings.hpp>
#include <cuopt/routing/solve.hpp>
#include <cuopt/routing/distance_engine/waypoint_matrix.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
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

// Multiplier applied to an edge's weight when either endpoint is in a
// vehicle's congested-node set (see ReplanRequest), so that vehicle's own
// cost/transit-time matrix -- and the shortest paths derived from it --
// steers away from nodes another AMR currently holds, instead of only
// finding out about the conflict at execution time (TrafficController).
constexpr double kCongestionPenalty = 8.0;

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
// the factory floor layout as a second, driftable copy here. Any edge
// touching a node in `congested` has its weight inflated, biasing this
// vehicle's own route choice away from it.
void build_csr(const std::vector<int>& congested, std::vector<int>& offsets,
                std::vector<int>& indices, std::vector<float>& weights) {
  const auto& wps = waypoints();
  const int n = static_cast<int>(wps.size());
  std::vector<std::vector<std::pair<int, double>>> adjacency(n);
  for (const auto& e : edges()) {
    double w;
    is_edge(e.from, e.to, w);
    const bool blocked = std::find(congested.begin(), congested.end(), e.from) != congested.end() ||
                         std::find(congested.begin(), congested.end(), e.to) != congested.end();
    if (blocked) w *= kCongestionPenalty;
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

// Compressed target-location index for every distinct waypoint id this
// solve touches: each vehicle's start/return node (in vehicle order), then
// every station -- deduplicated, since a vehicle can be sitting exactly at
// a station when a rolling re-solve fires.
struct TargetLocations {
  std::vector<int> all;                   // compressed index -> waypoint id
  std::unordered_map<int, int> index_of;  // waypoint id -> compressed index

  int operator[](int waypoint_id) const { return index_of.at(waypoint_id); }
};

TargetLocations compute_target_locations(const std::vector<int>& vehicle_anchor_nodes) {
  TargetLocations result;
  auto add = [&](int id) {
    if (result.index_of.count(id)) return;
    result.index_of[id] = static_cast<int>(result.all.size());
    result.all.push_back(id);
  };
  for (int id : vehicle_anchor_nodes) add(id);
  for (const auto& wp : waypoints()) {
    if (wp.role == WaypointRole::kStation) add(wp.id);
  }
  return result;
}

struct PairStations {
  int pickup;
  int delivery;
};

// The 6 recurring pickup-delivery "slots" this demo's fleet keeps
// servicing among stations A(4)/B(5)/C(6). Mirrors the official
// Intra-factory Transport Python example and cuopt_routing_node's
// intra_factory_transport_demo.
constexpr int kStationA = 4;
constexpr int kStationB = 5;
constexpr int kStationC = 6;
constexpr PairStations kPairStations[kNumPairs] = {
  {kStationA, kStationB},  // pair 0: A -> B
  {kStationB, kStationC},  // pair 1: B -> C
  {kStationC, kStationA},  // pair 2: C -> A
  {kStationA, kStationC},  // pair 3: A -> C
  {kStationB, kStationA},  // pair 4: B -> A
  {kStationC, kStationB},  // pair 5: C -> B
};

}  // namespace

ReplanRequest initial_replan_request() {
  ReplanRequest request;
  for (const auto& wp : waypoints()) {
    if (wp.role == WaypointRole::kDepot) request.vehicle_start_node.push_back(wp.id);
  }
  if (request.vehicle_start_node.empty()) {
    throw std::runtime_error("initial_replan_request: no depot waypoint");
  }
  request.vehicle_return_node = request.vehicle_start_node;  // every AMR's home depot is also its start
  request.pairs.assign(kNumPairs, PairStatus{});
  request.congested_nodes_by_vehicle.assign(request.vehicle_start_node.size(), {});
  return request;
}

SolveResult solve_intra_factory_transport(const ReplanRequest& request) {
  using namespace cuopt::routing;
  using cuopt::distance_engine::waypoint_matrix_t;

  const int fleet_size = static_cast<int>(request.vehicle_start_node.size());
  if (static_cast<int>(request.vehicle_return_node.size()) != fleet_size) {
    throw std::invalid_argument(
      "solve_intra_factory_transport: vehicle_return_node must have one entry per vehicle");
  }
  if (static_cast<int>(request.congested_nodes_by_vehicle.size()) != fleet_size) {
    throw std::invalid_argument(
      "solve_intra_factory_transport: congested_nodes_by_vehicle must have one entry per vehicle");
  }
  if (static_cast<int>(request.pairs.size()) != kNumPairs) {
    throw std::invalid_argument("solve_intra_factory_transport: expected kNumPairs pair statuses");
  }

  raft::handle_t handle;
  auto stream = handle.get_stream();

  // Anchors include both ends of every vehicle's trip -- its current
  // position *and* its home depot -- since a rolling re-solve's start and
  // return nodes can now differ (see ReplanRequest).
  std::vector<int> anchor_nodes = request.vehicle_start_node;
  anchor_nodes.insert(anchor_nodes.end(), request.vehicle_return_node.begin(),
                      request.vehicle_return_node.end());
  const TargetLocations targets = compute_target_locations(anchor_nodes);
  const int n_targets = static_cast<int>(targets.all.size());
  const int num_orders = 2 * kNumPairs;

  data_model_view_t<int, float> data_model(&handle, n_targets, fleet_size, num_orders);

  // One cost/transit-time matrix per vehicle (vehicle_type == vehicle_id),
  // each built from that vehicle's own congestion-weighted view of the
  // graph. The waypoint_matrix_t objects are reused below for each
  // vehicle's own route expansion, and the CSR host arrays + device
  // matrices must all outlive solve() (none of these APIs copy them) --
  // kept in per-vehicle slots of these vectors rather than loop-local
  // temporaries for exactly that reason.
  std::vector<std::vector<int>> all_offsets(fleet_size), all_indices(fleet_size);
  std::vector<std::vector<float>> all_weights(fleet_size);
  std::vector<std::unique_ptr<waypoint_matrix_t<int, float>>> graphs(fleet_size);
  std::vector<rmm::device_uvector<float>> cost_matrices;
  std::vector<rmm::device_uvector<float>> time_matrices;
  cost_matrices.reserve(fleet_size);
  time_matrices.reserve(fleet_size);
  std::vector<uint8_t> h_vehicle_types(fleet_size);

  for (int v = 0; v < fleet_size; ++v) {
    build_csr(request.congested_nodes_by_vehicle[v], all_offsets[v], all_indices[v], all_weights[v]);
    graphs[v] = std::make_unique<waypoint_matrix_t<int, float>>(
      handle, all_offsets[v].data(), static_cast<int>(all_offsets[v].size()) - 1, all_indices[v].data(),
      all_weights[v].data());

    cost_matrices.emplace_back(static_cast<size_t>(n_targets) * n_targets, stream);
    graphs[v]->compute_cost_matrix(cost_matrices[v].data(), targets.all.data(), n_targets);
    handle.sync_stream();

    time_matrices.emplace_back(static_cast<size_t>(n_targets) * n_targets, stream);
    thrust::transform(rmm::exec_policy(stream), cost_matrices[v].begin(), cost_matrices[v].end(),
                       time_matrices[v].begin(), MetersToSeconds{kAssumedSpeedMps});
    handle.sync_stream();

    h_vehicle_types[v] = static_cast<uint8_t>(v);
    data_model.add_cost_matrix(cost_matrices[v].data(), h_vehicle_types[v]);
    data_model.add_transit_time_matrix(time_matrices[v].data(), h_vehicle_types[v]);
  }

  std::vector<int> h_order_locations(num_orders);
  std::vector<int> h_pickup_orders(kNumPairs);
  std::vector<int> h_delivery_orders(kNumPairs);
  std::vector<int> h_demands(num_orders);
  std::vector<int> h_order_earliest(num_orders);
  std::vector<int> h_order_latest(num_orders);
  std::vector<int> h_service_times(num_orders);

  // Pin buffers for add_vehicle_order_match -- same "must outlive solve()"
  // constraint as the matrices above, hence the pre-sized outer vector
  // instead of a loop-local temporary.
  std::vector<rmm::device_uvector<int>> pin_buffers;
  pin_buffers.reserve(2 * kNumPairs);  // up to 2 per onboard pair (pickup + delivery)

  for (int p = 0; p < kNumPairs; ++p) {
    const int pickup_order = 2 * p;
    const int delivery_order = 2 * p + 1;
    h_pickup_orders[p] = pickup_order;
    h_delivery_orders[p] = delivery_order;

    const PairStatus& status = request.pairs[p];
    const bool onboard = status.state == PairState::kOnboard;
    if (onboard && (status.holder_vehicle_id < 0 || status.holder_vehicle_id >= fleet_size)) {
      throw std::invalid_argument(
        "solve_intra_factory_transport: onboard pair missing a valid holder_vehicle_id");
    }

    const int pickup_waypoint =
      onboard ? request.vehicle_start_node[status.holder_vehicle_id] : kPairStations[p].pickup;
    h_order_locations[pickup_order] = targets[pickup_waypoint];
    h_order_locations[delivery_order] = targets[kPairStations[p].delivery];

    h_demands[pickup_order] = 1;
    h_demands[delivery_order] = -1;

    // Staggered windows, relative to this solve's own t=0 (not absolute sim
    // time) -- same spirit for every re-solve as the original one-shot
    // scenario. An onboard pair's synthetic pickup is unconstrained since
    // the vehicle is trivially already there.
    h_order_earliest[pickup_order] = onboard ? 0 : (p < kNumPairs / 2 ? 0 : 20);
    h_order_latest[pickup_order] = onboard ? 300 : (p < kNumPairs / 2 ? 60 : 90);
    h_order_earliest[delivery_order] = onboard ? 0 : (p < kNumPairs / 2 ? 0 : 20);
    h_order_latest[delivery_order] = onboard ? 300 : (p < kNumPairs / 2 ? 120 : 150);

    // An onboard pair was for-real picked up earlier; this synthetic
    // zero-distance "pickup" is bookkeeping only, not a second dwell.
    h_service_times[pickup_order] = onboard ? 0 : static_cast<int>(kServiceTimeSec);
    h_service_times[delivery_order] = static_cast<int>(kServiceTimeSec);

    if (onboard) {
      // Pin *the order* to its holder -- add_order_vehicle_match restricts
      // which vehicles may serve this one order, as opposed to
      // add_vehicle_order_match, which would restrict the holder to
      // *exclusively* these two orders and nothing else it needs to serve.
      // (Confirmed by testing: using the vehicle-order-match form made
      // every re-solve with an onboard pair infeasible once that vehicle
      // also had to pick up any other pending pair.)
      pin_buffers.push_back(cuopt::device_copy(std::vector<int>{status.holder_vehicle_id}, stream));
      data_model.add_order_vehicle_match(pickup_order, pin_buffers.back().data(), 1);
      pin_buffers.push_back(cuopt::device_copy(std::vector<int>{status.holder_vehicle_id}, stream));
      data_model.add_order_vehicle_match(delivery_order, pin_buffers.back().data(), 1);
    }
  }

  std::vector<int> h_capacities(fleet_size, 2);
  std::vector<int> h_vehicle_start(fleet_size);
  std::vector<int> h_vehicle_end(fleet_size);
  for (int v = 0; v < fleet_size; ++v) {
    h_vehicle_start[v] = targets[request.vehicle_start_node[v]];
    h_vehicle_end[v] = targets[request.vehicle_return_node[v]];
  }
  std::vector<int> h_vehicle_earliest(fleet_size, 0);
  std::vector<int> h_vehicle_latest(fleet_size, 300);

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
  auto v_vehicle_types = cuopt::device_copy(h_vehicle_types, stream);

  data_model.set_order_locations(v_order_locations.data());
  data_model.set_pickup_delivery_pairs(v_pickup_orders.data(), v_delivery_orders.data());
  data_model.add_capacity_dimension("parts", v_demands.data(), v_capacities.data());
  data_model.set_order_time_windows(v_order_earliest.data(), v_order_latest.data());
  data_model.set_order_service_times(v_service_times.data());
  data_model.set_vehicle_locations(v_vehicle_start.data(), v_vehicle_end.data());
  data_model.set_vehicle_time_windows(v_vehicle_earliest.data(), v_vehicle_latest.data());
  data_model.set_vehicle_types(v_vehicle_types.data());
  // Without this, cuOpt is free to leave a vehicle completely unused
  // whenever the rest of the fleet can cost-effectively cover current
  // demand alone -- confirmed in testing: once an AMR went idle at its
  // depot, cuOpt would stop producing any plan for it at all once the
  // other AMR alone could serve the (regenerating) pending pairs, so it
  // sat idle forever even after its teammate also came home (nothing was
  // ever in pending_plan_by_vehicle_ for it to apply). Forcing the full
  // fleet to be used keeps every re-solve covering every vehicle.
  data_model.set_min_vehicles(fleet_size);

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
  // indices -- fine for the solver, but confusing for cargo tracking
  // downstream, where "loaded order 2" and "unloaded order 3" should read
  // as the same shipment. Map both halves of each pair to one shared pair
  // id instead.
  std::vector<int> pair_id_of_order(num_orders, -1);
  for (int p = 0; p < kNumPairs; ++p) {
    pair_id_of_order[h_pickup_orders[p]] = p;
    pair_id_of_order[h_delivery_orders[p]] = p;
  }

  // Split the flat (locations/node_types/route/truck_id) arrays by vehicle.
  struct Stop {
    int compressed_location;
    int node_type;         // node_type_t: DEPOT=0, PICKUP=1, DELIVERY=2, BREAK=3
    int pair_id;            // shared between a pickup and its matching delivery; -1 at depot/break
    double service_seconds; // the real per-order service time, not a constant (onboard pickups are 0s)
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
    const bool is_order = order_id >= 0 && order_id < num_orders;
    const int pair_id = is_order ? pair_id_of_order[order_id] : -1;
    const double service_seconds = is_order ? static_cast<double>(h_service_times[order_id]) : 0.0;
    per_vehicle_stops.back().push_back(
      Stop{host_route.locations[i], host_route.node_types[i], pair_id, service_seconds});
  }

  std::vector<VehiclePlan> plans;
  for (size_t v = 0; v < per_vehicle_stops.size(); ++v) {
    const auto& stops = per_vehicle_stops[v];
    std::vector<int> compressed_route;
    compressed_route.reserve(stops.size());
    for (const auto& s : stops) compressed_route.push_back(s.compressed_location);

    auto v_route = cuopt::device_copy(compressed_route, stream);
    // Expand using THIS vehicle's own (congestion-weighted) graph, so the
    // actual path chosen consistently reflects the same avoidance that
    // informed its cost matrix.
    auto waypoint_result = graphs[truck_ids[v]]->compute_waypoint_sequence(
      targets.all.data(), n_targets, v_route.data(), static_cast<int>(compressed_route.size()));
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
        plan.dwell_events.push_back(DwellEvent{stop_to_path_index[s], stops[s].service_seconds,
                                                stops[s].pair_id,
                                                node_type == static_cast<int>(node_type_t::PICKUP)});
      }
    }
    plans.push_back(std::move(plan));
  }

  return SolveResult{result.get_total_objective(), std::move(plans)};
}

}  // namespace cuopt_intra_factory_viz
