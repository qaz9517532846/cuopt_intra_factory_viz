#pragma once

#include <cstddef>
#include <vector>

namespace cuopt_intra_factory_viz {

struct KinematicLimits {
  double v_max;  // m/s
  double a_lin;  // m/s^2
  double w_max;  // rad/s
  double a_ang;  // rad/s^2
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

enum class PrimitiveKind { kRotate, kTranslate, kDwell };

// A forced stop (e.g. a pickup or delivery dock) attached to a specific
// position in the waypoint_ids sequence given to DiffDrivePath -- keyed by
// index rather than waypoint id, because a solved route can visit the same
// physical station more than once (for different orders) when vehicle
// capacity allows carrying more than one order at a time. Forces a stop
// there even if the path would otherwise have sailed straight through on a
// merged translation.
struct DwellEvent {
  std::size_t waypoint_index;  // index into the waypoint_ids vector this dwell attaches to
  double seconds;
  int pair_id = -1;     // shared id between a pickup and its matching delivery; -1 if not order-related
  bool is_pickup = false;  // meaningful only when pair_id >= 0
};

// Snapshot of what the vehicle is doing right now, for status reporting.
struct MotionStatus {
  PrimitiveKind kind;
  double progress;  // radians / meters / seconds covered so far, depending on kind
  double amount;    // total radians / meters / seconds for the current primitive
  double speed;     // current rad/s or m/s; always 0 while dwelling
  int to_id;        // waypoint this primitive ends at (translate/dwell) or heads toward (rotate)
  int pair_id;      // -1 unless kind == kDwell; shared between a pickup and its matching delivery
  bool is_pickup;   // meaningful only when pair_id >= 0
};

// Executes a path (a sequence of connected waypoint ids) as the only two
// motions a differential-drive base can do: in-place rotation and straight
// translation, plus optional timed dwells (stopped, not rotating or moving)
// at specific positions along the path. Consecutive collinear edges are
// merged into one translation unless a dwell event forces a stop between
// them; a direction change inserts a rotation to the new heading first. The
// path is assumed to be a closed loop (first id == last id) and is replayed
// forever, with a closing rotation appended automatically if the heading at
// the end of the given ids does not already match the starting heading.
class DiffDrivePath {
 public:
  DiffDrivePath(std::vector<int> waypoint_ids, KinematicLimits limits, double initial_heading_rad,
                std::vector<DwellEvent> dwell_events = {});

  // Advances the simulated motion by dt_sec using a bang-bang (trapezoidal /
  // triangular velocity profile) controller bounded by `limits`; a dwell
  // primitive just counts down dt_sec against its duration instead.
  void step(double dt_sec);

  const Pose2D& pose() const { return pose_; }

  MotionStatus status() const {
    const Primitive& p = primitives_[current_index_];
    return MotionStatus{p.kind, progress_, p.amount, speed_, p.to_id, p.pair_id, p.is_pickup};
  }

 private:
  struct Primitive {
    PrimitiveKind kind;
    double amount;     // radians (rotate) / meters (translate) / seconds (dwell), >= 0
    double direction;  // +1/-1, rotation sign; unused otherwise
    double heading;    // heading held while translating; unused otherwise
    int to_id;          // waypoint this primitive ends at or heads toward
    int pair_id = -1;     // dwell only
    bool is_pickup = false;  // dwell only
  };

  void build_primitives(const std::vector<int>& waypoint_ids, double initial_heading_rad,
                         const std::vector<DwellEvent>& dwell_events);

  KinematicLimits limits_;
  std::vector<Primitive> primitives_;
  std::size_t current_index_ = 0;
  double progress_ = 0.0;  // meters, radians, or seconds covered within the current primitive
  double speed_ = 0.0;     // current linear or angular speed magnitude; unused while dwelling
  Pose2D pose_;
};

}  // namespace cuopt_intra_factory_viz
