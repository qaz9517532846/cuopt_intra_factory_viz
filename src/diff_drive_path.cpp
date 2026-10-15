#include "cuopt_intra_factory_viz/diff_drive_path.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "cuopt_intra_factory_viz/waypoint_graph.hpp"

namespace cuopt_intra_factory_viz {

namespace {

// Signed shortest angular distance from `from` to `to`, in (-pi, pi].
double angle_diff(double to, double from) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

// Multiple orders can be served back-to-back at the very same physical
// waypoint with zero travel in between (e.g. a vehicle picking up 2+ orders
// at one station before moving on), so this must return every dwell event
// attached to `index`, not just the first.
std::vector<const DwellEvent*> dwell_events_at_index(const std::vector<DwellEvent>& dwell_events,
                                                      std::size_t index) {
  std::vector<const DwellEvent*> matches;
  for (const auto& de : dwell_events) {
    if (de.waypoint_index == index) matches.push_back(&de);
  }
  return matches;
}

}  // namespace

DiffDrivePath::DiffDrivePath(std::vector<int> waypoint_ids, KinematicLimits limits,
                              double initial_heading_rad, std::vector<DwellEvent> dwell_events)
  : limits_(limits) {
  if (waypoint_ids.size() < 2) {
    throw std::invalid_argument("DiffDrivePath requires at least 2 waypoints");
  }
  const Waypoint& start = waypoint_by_id(waypoint_ids.front());
  pose_ = Pose2D{start.x, start.y, initial_heading_rad};
  build_primitives(waypoint_ids, initial_heading_rad, dwell_events);
  if (primitives_.empty()) {
    throw std::invalid_argument("DiffDrivePath: path produced no motion primitives");
  }
}

void DiffDrivePath::build_primitives(const std::vector<int>& ids, double initial_heading_rad,
                                      const std::vector<DwellEvent>& dwell_events) {
  constexpr double kAngleEpsilon = 1e-6;
  double current_heading = initial_heading_rad;
  double accumulated_distance = 0.0;
  bool have_open_translation = false;
  int translation_destination_id = ids.front();

  // A dwell event at the path's own starting index (e.g. a pickup dock the
  // vehicle begins parked at) would otherwise never be visited as an
  // "arrival" -- the main loop below only checks index i+1, never index 0.
  for (const DwellEvent* initial : dwell_events_at_index(dwell_events, 0)) {
    primitives_.push_back(Primitive{PrimitiveKind::kDwell, initial->seconds, 0.0, 0.0, ids.front(),
                                     initial->pair_id, initial->is_pickup});
  }

  auto flush_translation = [&]() {
    if (have_open_translation && accumulated_distance > 1e-9) {
      primitives_.push_back(Primitive{PrimitiveKind::kTranslate, accumulated_distance, 1.0,
                                       current_heading, translation_destination_id});
    }
    accumulated_distance = 0.0;
    have_open_translation = false;
  };

  for (std::size_t i = 0; i + 1 < ids.size(); ++i) {
    double weight;
    if (!is_edge(ids[i], ids[i + 1], weight)) {
      throw std::invalid_argument("DiffDrivePath: waypoints " + std::to_string(ids[i]) + " and " +
                                   std::to_string(ids[i + 1]) + " are not connected");
    }
    const double segment_heading = heading_between(ids[i], ids[i + 1]);
    const double delta = angle_diff(segment_heading, current_heading);
    if (std::abs(delta) > kAngleEpsilon) {
      flush_translation();
      primitives_.push_back(
        Primitive{PrimitiveKind::kRotate, std::abs(delta), delta >= 0.0 ? 1.0 : -1.0, 0.0, ids[i + 1]});
      current_heading = segment_heading;
    }
    accumulated_distance += weight;
    translation_destination_id = ids[i + 1];
    have_open_translation = true;

    // A dwell event (pickup/delivery dock) always forces a stop here, even
    // if the path would otherwise have merged straight through it. Several
    // can land on the same index (back-to-back orders at one station).
    for (const DwellEvent* dwell : dwell_events_at_index(dwell_events, i + 1)) {
      flush_translation();
      primitives_.push_back(Primitive{PrimitiveKind::kDwell, dwell->seconds, 0.0, 0.0, ids[i + 1],
                                       dwell->pair_id, dwell->is_pickup});
    }
  }
  flush_translation();

  // The path loops forever; close the loop with a rotation back to the
  // starting heading if the last segment didn't already leave the robot
  // facing that way (e.g. the depot can only be entered/left along one
  // line, so a round trip typically ends facing the opposite direction).
  const double closing_delta = angle_diff(initial_heading_rad, current_heading);
  if (std::abs(closing_delta) > kAngleEpsilon) {
    primitives_.push_back(Primitive{PrimitiveKind::kRotate, std::abs(closing_delta),
                                     closing_delta >= 0.0 ? 1.0 : -1.0, 0.0, ids.front()});
  }
}

void DiffDrivePath::step(double dt_sec) {
  Primitive& p = primitives_[current_index_];

  if (p.kind == PrimitiveKind::kDwell) {
    progress_ = std::min(p.amount, progress_ + dt_sec);
    if (progress_ >= p.amount - 1e-9) {
      progress_ = 0.0;
      current_index_ = (current_index_ + 1) % primitives_.size();
    }
    return;
  }

  const bool is_rotation = (p.kind == PrimitiveKind::kRotate);
  const double max_speed = is_rotation ? limits_.w_max : limits_.v_max;
  const double max_accel = is_rotation ? limits_.a_ang : limits_.a_lin;

  const double remaining = p.amount - progress_;
  const double decel_distance = (speed_ * speed_) / (2.0 * max_accel);

  if (remaining <= decel_distance) {
    speed_ = std::max(0.0, speed_ - max_accel * dt_sec);
  } else {
    speed_ = std::min(max_speed, speed_ + max_accel * dt_sec);
  }

  double ds = speed_ * dt_sec;
  if (ds >= remaining) {
    ds = remaining;
    speed_ = 0.0;
  }
  progress_ += ds;

  if (is_rotation) {
    pose_.theta = angle_diff(pose_.theta + p.direction * ds, 0.0);
  } else {
    pose_.x += ds * std::cos(p.heading);
    pose_.y += ds * std::sin(p.heading);
  }

  if (progress_ >= p.amount - 1e-9) {
    progress_ = 0.0;
    speed_ = 0.0;
    current_index_ = (current_index_ + 1) % primitives_.size();
  }
}

}  // namespace cuopt_intra_factory_viz
