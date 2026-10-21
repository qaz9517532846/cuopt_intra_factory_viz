#pragma once

#include <unordered_map>
#include <vector>

namespace cuopt_intra_factory_viz {

// Mutual-exclusion lock on waypoint-graph nodes, so at most one AMR ever
// occupies (or is mid-transit into) a given node at a time. A vehicle must
// successfully reserve a node before moving into it, and releases the node
// it is leaving only once it has arrived at the next one -- see AmrTask in
// vehicle_tf_broadcaster_node.cpp for how reservations are acquired/held
// against the original (unmerged) waypoint path.
//
// This is the classic AGV "cell reservation" scheme: simple and sufficient
// for this demo's small, lightly-contended graph, but it can deadlock if
// two AMRs' routes require them to swap positions across a 2-node segment
// at the same time -- neither will ever release the node the other needs.
class TrafficController {
 public:
  bool try_reserve(int node_id, int vehicle_id) {
    auto [it, inserted] = owner_by_node_.try_emplace(node_id, vehicle_id);
    return inserted || it->second == vehicle_id;
  }

  void release(int node_id, int vehicle_id) {
    auto it = owner_by_node_.find(node_id);
    if (it != owner_by_node_.end() && it->second == vehicle_id) owner_by_node_.erase(it);
  }

  // All nodes currently reserved by some vehicle *other than* `vehicle_id`
  // -- used to build a per-vehicle congestion-weighted matrix for the next
  // cuOpt re-solve (see ReplanRequest::congested_nodes_by_vehicle).
  std::vector<int> nodes_reserved_by_others(int vehicle_id) const {
    std::vector<int> result;
    for (const auto& [node_id, owner] : owner_by_node_) {
      if (owner != vehicle_id) result.push_back(node_id);
    }
    return result;
  }

 private:
  std::unordered_map<int, int> owner_by_node_;  // node_id -> vehicle_id
};

}  // namespace cuopt_intra_factory_viz
