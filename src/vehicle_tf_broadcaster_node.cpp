#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cuopt_intra_factory_viz/cuopt_solver.hpp"
#include "cuopt_intra_factory_viz/diff_drive_path.hpp"
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

// Wraps a solved DiffDrivePath with the set of pickup-delivery pair ids
// currently on board.
// Cargo changes are detected as a dwell *finishing* (not starting -- the
// vehicle is still empty for the whole LOADING dwell, still loaded for the
// whole UNLOADING one), since DiffDrivePath::status() only reports "which
// dwell is active right now", not transitions.
struct AmrTask {
  std::unique_ptr<cuopt_intra_factory_viz::DiffDrivePath> path;
  int vehicle_id = -1;
  std::vector<int> carried_pair_ids;

  void step(double dt_sec) {
    const auto before = path->status();
    path->step(dt_sec);
    if (before.kind != cuopt_intra_factory_viz::PrimitiveKind::kDwell || before.pair_id < 0) return;

    const auto after = path->status();
    const bool still_same_dwell = after.kind == cuopt_intra_factory_viz::PrimitiveKind::kDwell &&
                                   after.pair_id == before.pair_id && after.to_id == before.to_id;
    if (still_same_dwell) return;

    if (before.is_pickup) {
      carried_pair_ids.push_back(before.pair_id);
    } else {
      carried_pair_ids.erase(
        std::remove(carried_pair_ids.begin(), carried_pair_ids.end(), before.pair_id),
        carried_pair_ids.end());
    }
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
    const cuopt_intra_factory_viz::KinematicLimits limits{
      this->declare_parameter<double>("v_max", 0.5),
      this->declare_parameter<double>("a_lin", 0.3),
      this->declare_parameter<double>("w_max", 1.0),
      this->declare_parameter<double>("a_ang", 1.5),
    };

    // Blocking, one-shot solve at startup -- same spirit as cuopt_milp_node's
    // GPU warmup. Each returned VehiclePlan is already the fully-expanded
    // waypoint-level route plus positioned pickup/delivery dwell events; see
    // cuopt_solver.cu for how the real cuOpt solve + waypoint_matrix_t
    // expansion produces that.
    RCLCPP_INFO(this->get_logger(), "solving Intra-factory Transport scenario via cuOpt...");
    auto solve_result = cuopt_intra_factory_viz::solve_intra_factory_transport();
    auto& plans = solve_result.plans;
    RCLCPP_INFO(this->get_logger(), "solved: total cost = %.1f, %zu vehicle(s) used",
                solve_result.total_objective, plans.size());

    amrs_.reserve(plans.size());
    for (auto& plan : plans) {
      log_plan(this->get_logger(), plan);
      AmrTask task;
      task.vehicle_id = plan.vehicle_id;
      task.path = std::make_unique<cuopt_intra_factory_viz::DiffDrivePath>(
        plan.waypoint_path, limits,
        cuopt_intra_factory_viz::heading_between(plan.waypoint_path[0], plan.waypoint_path[1]),
        plan.dwell_events);
      amrs_.push_back(std::move(task));
    }

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vehicle_markers", 10);
    timer_ = this->create_wall_timer(33ms, std::bind(&VehicleTfBroadcaster::on_timer, this));
    status_timer_ = this->create_wall_timer(1s, std::bind(&VehicleTfBroadcaster::on_status_timer, this));
  }

 private:
  void on_timer() {
    constexpr double kDtSec = 0.033;
    const rclcpp::Time now = this->get_clock()->now();
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < amrs_.size(); ++i) {
      amrs_[i].step(kDtSec);
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

  std::vector<AmrTask> amrs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
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
