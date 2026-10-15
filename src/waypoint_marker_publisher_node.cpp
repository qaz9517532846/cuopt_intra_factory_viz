#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "cuopt_intra_factory_viz/waypoint_graph.hpp"

using namespace std::chrono_literals;

namespace {

visualization_msgs::msg::Marker make_node_marker(const rclcpp::Time& stamp,
                                                   const cuopt_intra_factory_viz::Waypoint& wp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = "waypoint_nodes";
  marker.id = wp.id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = wp.x;
  marker.pose.position.y = wp.y;
  marker.pose.position.z = 0.15;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
  marker.color.a = 1.0f;

  using cuopt_intra_factory_viz::WaypointRole;
  switch (wp.role) {
    case WaypointRole::kDepot:
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.color.r = 0.0f;
      marker.color.g = 0.7f;
      marker.color.b = 0.0f;
      break;
    case WaypointRole::kStation:
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.color.r = 0.1f;
      marker.color.g = 0.4f;
      marker.color.b = 0.9f;
      break;
    default:
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.color.r = marker.color.g = marker.color.b = 0.6f;
      break;
  }
  return marker;
}

visualization_msgs::msg::Marker make_label_marker(const rclcpp::Time& stamp,
                                                    const cuopt_intra_factory_viz::Waypoint& wp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = "waypoint_labels";
  marker.id = wp.id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = wp.x;
  marker.pose.position.y = wp.y;
  marker.pose.position.z = 0.55;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.3;
  marker.color.r = marker.color.g = marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.text = std::to_string(wp.id);
  return marker;
}

visualization_msgs::msg::Marker make_edge_marker(const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = "waypoint_edges";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.color.r = marker.color.g = marker.color.b = 0.5f;
  marker.color.a = 0.8f;

  for (const auto& edge : cuopt_intra_factory_viz::edges()) {
    const auto& a = cuopt_intra_factory_viz::waypoint_by_id(edge.from);
    const auto& b = cuopt_intra_factory_viz::waypoint_by_id(edge.to);
    geometry_msgs::msg::Point pa;
    pa.x = a.x;
    pa.y = a.y;
    pa.z = 0.02;
    geometry_msgs::msg::Point pb;
    pb.x = b.x;
    pb.y = b.y;
    pb.z = 0.02;
    marker.points.push_back(pa);
    marker.points.push_back(pb);
  }
  return marker;
}

}  // namespace

class WaypointMarkerPublisher : public rclcpp::Node {
 public:
  WaypointMarkerPublisher() : Node("waypoint_marker_publisher") {
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&WaypointMarkerPublisher::on_timer, this));
  }

 private:
  void on_timer() {
    const rclcpp::Time now = this->get_clock()->now();
    visualization_msgs::msg::MarkerArray markers;
    for (const auto& wp : cuopt_intra_factory_viz::waypoints()) {
      markers.markers.push_back(make_node_marker(now, wp));
      markers.markers.push_back(make_label_marker(now, wp));
    }
    markers.markers.push_back(make_edge_marker(now));
    pub_->publish(markers);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
