#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cuopt_intra_factory_viz/cuopt_solver.hpp"
#include "cuopt_intra_factory_viz/diff_drive_path.hpp"
#include "cuopt_intra_factory_viz/traffic_controller.hpp"
#include "cuopt_intra_factory_viz/waypoint_graph.hpp"

using namespace std::chrono_literals;

namespace {

struct VehicleColor {
  float r;
  float g;
  float b;
};

constexpr VehicleColor kVehicleColors[] = {
  {0.85f, 0.35f, 0.0f},
  {0.55f, 0.0f, 0.85f},
  {0.0f, 0.6f, 0.3f},
  {0.85f, 0.75f, 0.0f},
};

// Squared arrival-detection radius: how close pose() must get to a route
// checkpoint's real coordinates to count as "arrived there" (see step()).
// Tick travel at v_max is well under a centimeter per millisecond, so 5cm
// comfortably catches arrival without overshoot risk on this graph's
// shortest (1m) edges.
constexpr double kArrivalEpsilonSq = 0.05 * 0.05;

// Number of graph edges touching `node_id`. This waypoint graph is built
// from exactly two 4-way junctions (nodes 2 and 8) joined by single-lane
// "spokes" of plain degree-2 transit/station nodes, plus two degree-1 depot
// dead-ends (0 and 10) hanging off them. A degree != 2 node is therefore a
// safe place for an AMR to sit still -- other AMRs can still reach every
// other spoke through it -- whereas a degree-2 node is only safe to occupy
// as part of an uninterrupted pass through its whole spoke (see step()'s
// stretch reservation: trying to lock spoke nodes one at a time, instead of
// the whole spoke atomically, is exactly what let two AMRs deadlock
// swapping positions mid-spoke in testing).
int node_degree(int node_id) {
  int degree = 0;
  for (const auto& e : cuopt_intra_factory_viz::edges()) {
    if (e.from == node_id || e.to == node_id) ++degree;
  }
  return degree;
}

struct AmrTask {
  std::unique_ptr<cuopt_intra_factory_viz::DiffDrivePath> path;
  int vehicle_id = -1;
  std::vector<int> carried_pair_ids;

  // Traffic-control state, tracked against the solved plan's original
  // (unmerged) waypoint path -- not against DiffDrivePath's own primitives,
  // since those merge consecutive collinear edges into one translation and
  // would otherwise let the vehicle sail through a shared junction (e.g.
  // node 8, where both depots' routes meet) without ever registering
  // arrival there.
  std::vector<int> route;        // = plan.waypoint_path; always ends at this vehicle's home depot
                                  // (see cuopt_solver.cu's vehicle_return_node) -- front == back for
                                  // a normal lap, front != back for a one-way mid-route rescue plan
                                  // (see maybe_replan's deferred-start-node handling below)
  std::size_t next_idx = 1;      // route[] index to resume stretch-scanning from once window empties
  int held_node = -1;            // node this vehicle is physically parked at right now
  std::vector<int> window;       // remaining nodes to walk within the current reserved stretch (may
                                  // repeat a node on an out-and-back); empty => need to reserve the next one
  std::vector<int> reserved_set; // distinct nodes reserved for the current stretch; released together
                                  // once window empties, so a vehicle never partially occupies a spoke
  bool waiting = false;          // true while blocked reserving the next stretch (for status logging)
  int waiting_for = -1;           // first node of the stretch currently blocked on
  int waiting_ticks = 0;          // consecutive ticks spent waiting -- drives the reactive re-solve trigger
  bool trip_complete = false;    // true once this plan has walked all the way to route.back() (always
                                  // this vehicle's home depot). Every plan -- even a normal closed lap,
                                  // not just a one-way rescue trip -- stops here and waits rather than
                                  // auto-looping forever: cuOpt re-solves fast enough (a few seconds)
                                  // that pausing once per lap is cheap, and it's the only way to get a
                                  // *reliable* swap point -- a plan built from this AMR's position while
                                  // it's still moving almost never lands (its window is only briefly
                                  // empty at each junction, nowhere near long enough to outlast a ~5s
                                  // background solve), which in testing left the other AMR stuck
                                  // forever re-running its stale original loop. At trip_complete there
                                  // is nothing further in route[] to walk, so the vehicle just idles --
                                  // still holding its one reservation (the depot) -- until
                                  // apply_pending_plan_if_ready hands it a fresh plan.

  void step(double dt_sec, cuopt_intra_factory_viz::TrafficController& traffic) {
    if (window.empty()) {
      if (trip_complete) return;
      if (!try_reserve_next_stretch(traffic)) {
        ++waiting_ticks;
        return;
      }
    }
    waiting_ticks = 0;

    const auto before = path->status();
    path->step(dt_sec);

    const auto& target = cuopt_intra_factory_viz::waypoint_by_id(window.front());
    const double dx = path->pose().x - target.x;
    const double dy = path->pose().y - target.y;
    if (dx * dx + dy * dy < kArrivalEpsilonSq) {
      held_node = window.front();
      window.erase(window.begin());
      if (window.empty()) {
        for (int n : reserved_set) {
          if (n != held_node) traffic.release(n, vehicle_id);
        }
        reserved_set.clear();
      }
    }

    // Cargo changes are detected as a dwell *finishing* (not starting -- the
    // vehicle is still empty for the whole LOADING dwell, still loaded for
    // the whole UNLOADING one), since DiffDrivePath::status() only reports
    // "which dwell is active right now", not transitions.
    if (before.kind != cuopt_intra_factory_viz::PrimitiveKind::kDwell || before.pair_id < 0) return;

    const auto after = path->status();
    const bool still_same_dwell = after.kind == cuopt_intra_factory_viz::PrimitiveKind::kDwell &&
                                   after.pair_id == before.pair_id && after.to_id == before.to_id;
    if (still_same_dwell) return;

    if (before.is_pickup) {
      // A freshly re-solved route can carry a zero-duration synthetic
      // "pickup" dwell for a pair this AMR is already physically holding
      // (see cuopt_solver.cu's onboard-pair handling) -- idempotent so that
      // replaying it doesn't duplicate an id already in carried_pair_ids.
      if (std::find(carried_pair_ids.begin(), carried_pair_ids.end(), before.pair_id) ==
          carried_pair_ids.end()) {
        carried_pair_ids.push_back(before.pair_id);
      }
    } else {
      carried_pair_ids.erase(
        std::remove(carried_pair_ids.begin(), carried_pair_ids.end(), before.pair_id),
        carried_pair_ids.end());
    }
  }

 private:
  // Scans route[] from next_idx for the next "stretch": a run of plain
  // transit nodes ending at (and including) the next node where an AMR can
  // safely sit still -- a junction, a depot, or back at a node this stretch
  // already needed anyway. Reserves every distinct node in that stretch
  // atomically (all-or-nothing) before the vehicle is allowed to move into
  // any of it, so it's never caught holding only part of a spoke.
  bool try_reserve_next_stretch(cuopt_intra_factory_viz::TrafficController& traffic) {
    std::vector<int> candidate;
    std::size_t idx = next_idx;
    while (true) {
      candidate.push_back(route[idx]);
      if (node_degree(route[idx]) != 2) break;  // reached a junction/depot -- safe stretch boundary
      idx = (idx % (route.size() - 1)) + 1;
    }

    std::vector<int> distinct;
    for (int n : candidate) {
      if (std::find(distinct.begin(), distinct.end(), n) == distinct.end()) distinct.push_back(n);
    }

    std::size_t acquired = 0;
    for (; acquired < distinct.size(); ++acquired) {
      if (!traffic.try_reserve(distinct[acquired], vehicle_id)) break;
    }
    if (acquired < distinct.size()) {
      for (std::size_t k = 0; k < acquired; ++k) traffic.release(distinct[k], vehicle_id);
      waiting = true;
      waiting_for = candidate.front();
      return false;  // the next stretch isn't fully free yet -- stay put and retry next tick
    }

    waiting = false;
    // We're committed to departing held_node now that the whole next stretch
    // is secured, so free it unless the new stretch loops back through it
    // (an out-and-back like [9,6,9,8] re-needs node 8) -- in that case
    // `distinct` already re-reserved it above, and the existing cleanup in
    // step() will correctly keep holding it once this stretch's window
    // empties. Without this release, a node a vehicle departs for good (not
    // as part of a loop) stays permanently reserved to it forever -- which
    // in testing silently starved the other AMR out of ever reaching a node
    // this one had abandoned early in its very first lap.
    if (std::find(distinct.begin(), distinct.end(), held_node) == distinct.end()) {
      traffic.release(held_node, vehicle_id);
    }
    // Reached route.back() -- always this vehicle's home depot, whether
    // this plan was a normal closed lap or a one-way rescue trip -- so
    // there's nothing meaningful left to walk; mark it so step() idles
    // there instead of wrapping around route[1..] again. See trip_complete's
    // comment for why every plan stops here instead of just one-way ones.
    if (candidate.back() == route.back()) trip_complete = true;
    window = std::move(candidate);
    reserved_set = std::move(distinct);
    next_idx = (idx % (route.size() - 1)) + 1;
    return true;
  }
};

geometry_msgs::msg::TransformStamped make_transform(const rclcpp::Time& stamp,
                                                      const std::string& child_frame,
                                                      const cuopt_intra_factory_viz::Pose2D& pose) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = child_frame;
  tf_msg.transform.translation.x = pose.x;
  tf_msg.transform.translation.y = pose.y;
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.theta);
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();
  return tf_msg;
}

// A tiny box drawn in the vehicle's own tf frame -- it tracks the broadcast
// transform automatically, so it only needs to be published at origin/identity.
visualization_msgs::msg::Marker make_body_marker(const rclcpp::Time& stamp, const std::string& frame,
                                                    int id, const VehicleColor& color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = "vehicle_bodies";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.z = 0.1;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.3;
  marker.scale.z = 0.2;
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = 1.0f;
  return marker;
}

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

std::string format_cargo(const std::vector<int>& pair_ids) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < pair_ids.size(); ++i) {
    if (i) oss << ",";
    oss << pair_ids[i];
  }
  oss << "]";
  return oss.str();
}

// Diffs an AmrTask's carried_pair_ids across one step() call to tell which
// pairs it just picked up or just delivered -- used to keep the fleet-wide
// order-lifecycle state (see VehicleTfBroadcaster::order_state_) in sync
// without AmrTask itself needing to know about re-solve bookkeeping.
void diff_cargo(const std::vector<int>& before, const std::vector<int>& after,
                 std::vector<int>& picked_up, std::vector<int>& delivered) {
  for (int id : after) {
    if (std::find(before.begin(), before.end(), id) == before.end()) picked_up.push_back(id);
  }
  for (int id : before) {
    if (std::find(after.begin(), after.end(), id) == after.end()) delivered.push_back(id);
  }
}

std::string join_path(const std::vector<int>& path) {
  std::ostringstream oss;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i) oss << " -> ";
    oss << path[i];
  }
  return oss.str();
}

// One-time dump of a solved VehiclePlan right after solving, so the full
// route/dwell result can be read directly instead of pieced together from
// the per-second status log.
void log_plan(const rclcpp::Logger& logger, const cuopt_intra_factory_viz::VehiclePlan& plan) {
  RCLCPP_INFO(logger, "[plan amr%d] waypoint path (%zu stops): %s", plan.vehicle_id,
              plan.waypoint_path.size(), join_path(plan.waypoint_path).c_str());
  for (const auto& dwell : plan.dwell_events) {
    RCLCPP_INFO(logger, "[plan amr%d]   %s pair %d at waypoint %d (path index %zu), %.0fs",
                plan.vehicle_id, dwell.is_pickup ? "PICKUP " : "DELIVER", dwell.pair_id,
                plan.waypoint_path[dwell.waypoint_index], dwell.waypoint_index, dwell.seconds);
  }
}

void log_vehicle_status(const rclcpp::Logger& logger, const AmrTask& task) {
  const auto& pose = task.path->pose();
  const auto status = task.path->status();
  const std::string name = "amr" + std::to_string(task.vehicle_id);
  const std::string cargo = format_cargo(task.carried_pair_ids);

  if (task.waiting) {
    RCLCPP_INFO(logger, "[%s] pos=(%.2f, %.2f) | WAITING to enter node %d (spoke busy) | cargo=%s",
                name.c_str(), pose.x, pose.y, task.waiting_for, cargo.c_str());
    return;
  }
  if (task.trip_complete && task.window.empty()) {
    // trip_complete alone only means the final leg has been *reserved* --
    // the AMR could still be rotating/translating toward it; window.empty()
    // additionally is what confirms it has actually arrived and has
    // nothing left to walk (confirmed in testing: without this check, the
    // status log claimed "IDLE at home depot" while the AMR was still
    // mid-rotation a couple of metres away).
    RCLCPP_INFO(logger, "[%s] pos=(%.2f, %.2f) | IDLE at home depot, awaiting next route | cargo=%s",
                name.c_str(), pose.x, pose.y, cargo.c_str());
    return;
  }

  using cuopt_intra_factory_viz::PrimitiveKind;
  switch (status.kind) {
    case PrimitiveKind::kDwell: {
      const char* action = status.is_pickup ? "LOADING" : "UNLOADING";
      RCLCPP_INFO(logger,
                  "[%s] pos=(%.2f, %.2f) | %s pair %d at waypoint %d (%.1f/%.1f s) | cargo=%s",
                  name.c_str(), pose.x, pose.y, action, status.pair_id, status.to_id,
                  status.progress, status.amount, cargo.c_str());
      break;
    }
    case PrimitiveKind::kRotate:
      RCLCPP_INFO(logger,
                  "[%s] pos=(%.2f, %.2f) heading=%.1f deg | ROTATING %.2f/%.2f rad (%.2f rad/s) "
                  "-> heading to waypoint %d | cargo=%s",
                  name.c_str(), pose.x, pose.y, pose.theta * kRadToDeg, status.progress,
                  status.amount, status.speed, status.to_id, cargo.c_str());
      break;
    case PrimitiveKind::kTranslate:
      RCLCPP_INFO(logger,
                  "[%s] pos=(%.2f, %.2f) heading=%.1f deg | TRANSLATING %.2f/%.2f m (%.2f m/s) "
                  "-> heading to waypoint %d | cargo=%s",
                  name.c_str(), pose.x, pose.y, pose.theta * kRadToDeg, status.progress,
                  status.amount, status.speed, status.to_id, cargo.c_str());
      break;
  }
}

}  // namespace

class VehicleTfBroadcaster : public rclcpp::Node {
 public:
  VehicleTfBroadcaster() : Node("vehicle_tf_broadcaster") {
    limits_ = cuopt_intra_factory_viz::KinematicLimits{
      this->declare_parameter<double>("v_max", 0.5),
      this->declare_parameter<double>("a_lin", 0.3),
      this->declare_parameter<double>("w_max", 1.0),
      this->declare_parameter<double>("a_ang", 1.5),
    };

    // Blocking, one-shot solve at startup -- same spirit as cuopt_milp_node's
    // GPU warmup. Each returned VehiclePlan is already the fully-expanded
    // waypoint-level route plus positioned pickup/delivery dwell events; see
    // cuopt_solver.cu for how the real cuOpt solve + waypoint_matrix_t
    // expansion produces that. Every *later* re-solve (see maybe_replan())
    // goes through this exact same function, just with a request describing
    // wherever the fleet currently is instead of the two depots.
    RCLCPP_INFO(this->get_logger(), "solving Intra-factory Transport scenario via cuOpt...");
    const auto initial_request = cuopt_intra_factory_viz::initial_replan_request();
    home_depot_ = initial_request.vehicle_start_node;  // each AMR's permanent home, by vehicle_id
    auto solve_result = cuopt_intra_factory_viz::solve_intra_factory_transport(initial_request);
    auto& plans = solve_result.plans;
    RCLCPP_INFO(this->get_logger(), "solved: total cost = %.1f, %zu vehicle(s) used",
                solve_result.total_objective, plans.size());

    amrs_.reserve(plans.size());
    for (auto& plan : plans) {
      log_plan(this->get_logger(), plan);
      AmrTask task;
      task.vehicle_id = plan.vehicle_id;
      task.path = std::make_unique<cuopt_intra_factory_viz::DiffDrivePath>(
        plan.waypoint_path, limits_,
        cuopt_intra_factory_viz::heading_between(plan.waypoint_path[0], plan.waypoint_path[1]),
        plan.dwell_events);
      task.route = plan.waypoint_path;
      task.held_node = task.route.front();
      traffic_.try_reserve(task.held_node, task.vehicle_id);
      amrs_.push_back(std::move(task));
    }

    // Fixed priority, lowest vehicle_id first: when two AMRs want the same
    // stretch reservation in the same tick, whichever is earlier in this
    // order gets it and the other waits. Stepping in plain amrs_ order
    // would mean the same thing here (plans already come back ordered by
    // vehicle_id), but this makes the tie-break an explicit, documented
    // rule rather than an accidental side effect of vector order.
    priority_order_.resize(amrs_.size());
    for (std::size_t i = 0; i < amrs_.size(); ++i) priority_order_[i] = i;
    std::sort(priority_order_.begin(), priority_order_.end(), [this](std::size_t a, std::size_t b) {
      return amrs_[a].vehicle_id < amrs_[b].vehicle_id;
    });

    // All 6 pickup-delivery "slots" start pending, matching what
    // initial_replan_request() just assumed; kept in sync from here on by
    // on_timer()'s diff_cargo() calls as pickups/deliveries actually happen.
    order_state_.assign(cuopt_intra_factory_viz::kNumPairs, cuopt_intra_factory_viz::PairStatus{});
    pending_plan_by_vehicle_.resize(amrs_.size());

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vehicle_markers", 10);
    timer_ = this->create_wall_timer(33ms, std::bind(&VehicleTfBroadcaster::on_timer, this));
    status_timer_ = this->create_wall_timer(1s, std::bind(&VehicleTfBroadcaster::on_status_timer, this));
    // Periodic half of the hybrid re-solve trigger; the reactive half (an
    // AMR stuck too long) is checked every tick inside on_timer() instead,
    // since it needs per-tick freshness that a coarser timer can't give.
    replan_timer_ = this->create_wall_timer(18s, std::bind(&VehicleTfBroadcaster::on_replan_timer, this));
  }

  ~VehicleTfBroadcaster() override {
    if (replan_thread_.joinable()) replan_thread_.join();
  }

 private:
  // Consecutive ticks of AmrTask::waiting before the reactive half of the
  // hybrid re-solve trigger fires for that vehicle (~6s at the 33ms tick).
  static constexpr double kDtSec = 0.033;
  static constexpr double kStuckThresholdSec = 6.0;
  static constexpr int kStuckTicksThreshold = static_cast<int>(kStuckThresholdSec / kDtSec);

  void on_timer() {
    const rclcpp::Time now = this->get_clock()->now();
    visualization_msgs::msg::MarkerArray markers;

    // Captured once, before any task in this tick is touched: whether to
    // *apply* a pending plan is a fleet-wide decision for this tick, not a
    // per-vehicle one. all_idle() itself depends on every task's
    // trip_complete/window state -- recomputing it fresh inside each
    // task's own apply_pending_plan_if_ready call would make it see the
    // *other* tasks already-mutated-this-tick state, not the state the
    // fleet was actually in when the tick started. Concretely: amr0 (first
    // in priority_order_) would apply its plan and flip its own
    // trip_complete to false, so by the time amr1's own gate ran moments
    // later in the very same tick, all_idle() had already gone false --
    // amr1 would then miss the very same synchronized moment it was
    // genuinely part of, and have to wait for a whole new trigger cycle
    // (confirmed in testing: amr0 switched immediately but amr1 sat idle
    // for several more re-solve cycles before it ever got its turn).
    const bool fleet_was_idle = all_idle();
    for (std::size_t i : priority_order_) {
      AmrTask& task = amrs_[i];
      apply_pending_plan_if_ready(task, fleet_was_idle);

      const std::vector<int> cargo_before = task.carried_pair_ids;
      task.step(kDtSec, traffic_);

      std::vector<int> picked_up, delivered;
      diff_cargo(cargo_before, task.carried_pair_ids, picked_up, delivered);
      for (int pair_id : picked_up) {
        order_state_[pair_id] = {cuopt_intra_factory_viz::PairState::kOnboard, task.vehicle_id};
      }
      for (int pair_id : delivered) {
        order_state_[pair_id] = {cuopt_intra_factory_viz::PairState::kPending, -1};
      }

      if (task.waiting_ticks == kStuckTicksThreshold) {
        maybe_replan("amr" + std::to_string(task.vehicle_id) + " stuck " +
                     std::to_string(static_cast<int>(kStuckThresholdSec)) + "s+");
      }
    }

    // Only ask cuOpt to reassign once *every* AMR has actually stopped at
    // its own home depot (see AmrTask::trip_complete and all_idle()) --
    // not as soon as the first one gets there. Re-solving per-vehicle the
    // moment it went idle meant this request's snapshot of the *other*,
    // still-moving vehicle had to be deferred to its home depot instead of
    // its live position (see maybe_replan), so the computed plan wasn't
    // really final until that vehicle also stopped on its own anyway --
    // needless early/duplicate GPU work. Waiting for all_idle() first means
    // every vehicle's position is already stable and exactly matches its
    // home depot by construction, so the solve that runs is already the
    // final one: compute once, then move once, together.
    // sync_replan_requested_ debounces this to one request per synchronized
    // stop (reset below once the fleet is moving again, so the next
    // simultaneous return asks again).
    if (all_idle()) {
      // Only latch the debounce flag once a solve was *actually* launched
      // for this event -- if maybe_replan() found some other trigger's
      // solve already in flight and dropped this request, latching the
      // flag anyway would mean nobody ever asks cuOpt specifically for
      // this synchronized stop again (see maybe_replan's comment). Leaving
      // it unset here means on_timer() just retries on the very next tick,
      // which will succeed as soon as that other solve finishes.
      if (!sync_replan_requested_) {
        sync_replan_requested_ = maybe_replan("both AMRs idle at home depot");
      }
    } else {
      sync_replan_requested_ = false;
    }

    for (size_t i = 0; i < amrs_.size(); ++i) {
      const std::string frame = "amr" + std::to_string(amrs_[i].vehicle_id) + "/base_link";
      broadcaster_->sendTransform(make_transform(now, frame, amrs_[i].path->pose()));
      const VehicleColor& color = kVehicleColors[i % (sizeof(kVehicleColors) / sizeof(kVehicleColors[0]))];
      markers.markers.push_back(make_body_marker(now, frame, static_cast<int>(i), color));
    }
    marker_pub_->publish(markers);
  }

  void on_status_timer() {
    for (const auto& task : amrs_) log_vehicle_status(this->get_logger(), task);
  }

  void on_replan_timer() { maybe_replan("periodic"); }

  // True once every AMR has finished its current trip and is idling at its
  // own home depot (see AmrTask::trip_complete). Gates the fleet's restart:
  // an AMR that gets home first should wait there rather than immediately
  // setting off on a new one-way trip while a teammate is still working.
  bool all_idle() const {
    for (const auto& task : amrs_) {
      if (!(task.trip_complete && task.window.empty())) return false;
    }
    return true;
  }

  // Applies a freshly re-solved route to `task` once it's safe to: only at
  // a window-empty boundary, where AmrTask holds exactly one reservation
  // (its own held_node) and nothing mid-stretch -- see AmrTask::step()'s
  // stretch-reservation cleanup. Swapping at any other point would hand
  // the new route a traffic-control state it doesn't match.
  //
  // `fleet_was_idle` is all_idle(), captured once by the caller before any
  // task this tick was touched -- not recomputed here -- so that every
  // task in the same tick agrees on whether the fleet was synchronized,
  // regardless of processing order (see on_timer()'s comment on why
  // recomputing it live here let one AMR's own update make the *next*
  // AMR in priority order miss the very moment it was genuinely part of).
  void apply_pending_plan_if_ready(AmrTask& task, bool fleet_was_idle) {
    if (!task.window.empty()) return;
    // task is idling at its own depot (trip_complete) -- hold it there
    // until every other AMR is back home too, so the fleet always starts
    // its next round together instead of one AMR dashing off again while
    // another is still mid-task.
    if (task.trip_complete && !fleet_was_idle) return;
    std::optional<cuopt_intra_factory_viz::VehiclePlan> plan;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      auto& slot = pending_plan_by_vehicle_[task.vehicle_id];
      if (!slot) return;
      // The solve (background thread, ~5s) started from wherever this
      // vehicle was *when maybe_replan() snapshotted it*. Two reasons that
      // might not be where it is right now:
      //  (a) it was live/moving at snapshot time and has since crossed one
      //      or more stretch boundaries -- this plan describes a position
      //      it's already past and will never revisit, so it'll never
      //      match (until a newer solve overwrites this slot).
      //  (b) it was deliberately *deferred* to this vehicle's home depot
      //      (see maybe_replan) because it was still mid-route -- this
      //      plan is *supposed* to mismatch at every ordinary junction
      //      along the way; it only becomes valid once the vehicle
      //      actually finishes its current lap and arrives home.
      // Either way, popping and discarding on a mismatch here is wrong:
      // for (a) it's harmless but pointless, and for (b) it actively
      // destroys the one plan meant for this vehicle's eventual depot
      // arrival long before it gets there -- confirmed in testing, this
      // is exactly what starved an AMR of its own synchronized-restart
      // plan, forcing a wait for a whole new solve instead of using the
      // one already computed and waiting for it. So: just peek, and only
      // actually consume the slot once every check below passes.
      if (slot->waypoint_path.front() != task.held_node) return;
      // The position check above isn't enough on its own: this vehicle may
      // have *picked up* a pair after this plan's snapshot was taken but
      // before it finished solving, e.g. mid-dwell on an already-active
      // plan's own scheduled pickup. Applying this plan would then discard
      // the active plan's still-pending delivery for that pair without this
      // one ever scheduling a replacement -- the pair would be stuck
      // onboard forever (reproduced in testing: cargo=[4] sat on an AMR
      // that reached IDLE at its depot, because the plan it had switched
      // to earlier never mentioned pair 4 at all). Require every
      // currently-carried pair to have a delivery dwell somewhere in the
      // incoming plan too -- same peek-don't-pop treatment on failure.
      for (int pair_id : task.carried_pair_ids) {
        const bool has_delivery =
          std::any_of(slot->dwell_events.begin(), slot->dwell_events.end(), [pair_id](const auto& dwell) {
            return !dwell.is_pickup && dwell.pair_id == pair_id;
          });
        if (!has_delivery) return;
      }
      plan = std::move(slot);
      slot.reset();
    }
    // held_node matches plan->waypoint_path.front(), so this is a seamless
    // continuation, not a teleport. Keep the vehicle's actual current
    // heading instead of recomputing one from the new path's first
    // segment, so there's no visual snap-turn either.
    const double current_heading = task.path->pose().theta;
    task.path = std::make_unique<cuopt_intra_factory_viz::DiffDrivePath>(
      plan->waypoint_path, limits_, current_heading, plan->dwell_events);
    task.route = plan->waypoint_path;
    task.next_idx = 1;
    task.waiting = false;
    task.waiting_ticks = 0;
    task.trip_complete = false;
    RCLCPP_INFO(this->get_logger(), "[amr%d] switched to a freshly re-solved route", task.vehicle_id);
    log_plan(this->get_logger(), *plan);
  }

  // Launches a background cuOpt re-solve from the fleet's current live
  // state (positions, order lifecycle, live congestion from `traffic_`) if
  // one isn't already in flight. Never touches amrs_/traffic_ from the
  // background thread -- only reads a snapshot taken here, on the main
  // thread, and only ever writes into pending_plan_by_vehicle_ under
  // pending_mutex_. Returns whether a solve was actually launched -- false
  // means some *other* trigger's solve (e.g. the periodic timer) was
  // already in flight and this request was silently dropped, which the
  // all_idle() trigger below needs to know about: if it didn't, it would
  // mark the synchronized-stop event as "already requested" even though
  // nothing was ever launched specifically for it, and then never retry --
  // leaving the fleet to hope that whatever the *other* in-flight solve
  // happens to produce covers both vehicles cleanly (confirmed in testing:
  // it doesn't always, and when it doesn't, the vehicle whose portion
  // didn't land is stuck waiting a full extra lap before anyone asks
  // cuOpt again on its behalf).
  bool maybe_replan(const std::string& reason) {
    if (replan_in_progress_.exchange(true)) return false;

    cuopt_intra_factory_viz::ReplanRequest request;
    request.vehicle_start_node.resize(amrs_.size());
    request.vehicle_return_node = home_depot_;  // always route back home, never re-anchor elsewhere
    request.congested_nodes_by_vehicle.resize(amrs_.size());
    for (const auto& task : amrs_) {
      // Use this vehicle's actual current position only while it's
      // genuinely stable there -- idling at depot, or blocked on
      // contention (window stays empty the whole time it's stuck, so using
      // its live position here is exactly what lets the reactive
      // stuck-recovery trigger find it an escape route right now). While
      // it's actively moving through a stretch, its window is only ever
      // empty for a single ~33ms tick per junction -- nowhere near long
      // enough for a ~5s background solve to land a plan against, so a
      // snapshot taken here would almost certainly be stale by the time
      // it's checked (confirmed in testing: the still-moving AMR just kept
      // re-running its old loop forever, since every re-solve for it kept
      // getting silently discarded by apply_pending_plan_if_ready's
      // position check). Defer to its home depot instead -- that plan
      // isn't meant to apply yet; it naturally lands the next time this
      // vehicle finishes its current lap and arrives back home (see
      // trip_complete), which is exactly the "reassign only once everyone
      // is back at their own origin" behavior we want.
      request.vehicle_start_node[task.vehicle_id] =
        task.window.empty() ? task.held_node : home_depot_[task.vehicle_id];
      request.congested_nodes_by_vehicle[task.vehicle_id] =
        traffic_.nodes_reserved_by_others(task.vehicle_id);
    }
    request.pairs = order_state_;

    RCLCPP_INFO(this->get_logger(), "triggering cuOpt re-solve (%s)...", reason.c_str());

    if (replan_thread_.joinable()) replan_thread_.join();
    replan_thread_ = std::thread([this, request = std::move(request)]() {
      try {
        auto result = cuopt_intra_factory_viz::solve_intra_factory_transport(request);
        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (auto& plan : result.plans) pending_plan_by_vehicle_[plan.vehicle_id] = std::move(plan);
        RCLCPP_INFO(this->get_logger(), "re-solve finished: total cost = %.1f", result.total_objective);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "re-solve failed, keeping current routes: %s", e.what());
      }
      replan_in_progress_.store(false);
    });
    return true;
  }

  std::vector<AmrTask> amrs_;
  std::vector<std::size_t> priority_order_;  // indices into amrs_, sorted by fixed priority (vehicle_id asc)
  cuopt_intra_factory_viz::TrafficController traffic_;
  cuopt_intra_factory_viz::KinematicLimits limits_{};
  std::vector<cuopt_intra_factory_viz::PairStatus> order_state_;  // size kNumPairs; main-thread-only
  std::vector<int> home_depot_;  // by vehicle_id; set once at startup, never changes
  bool sync_replan_requested_ = false;  // debounces the all_idle() re-solve trigger in on_timer() to
                                         // one request per synchronized stop

  std::atomic<bool> replan_in_progress_{false};
  std::thread replan_thread_;
  std::mutex pending_mutex_;
  std::vector<std::optional<cuopt_intra_factory_viz::VehiclePlan>> pending_plan_by_vehicle_;  // guarded by pending_mutex_

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VehicleTfBroadcaster>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("vehicle_tf_broadcaster"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
