// Copyright 2026 PFA Navigation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TUNNEL_ALIGN__TUNNEL_GEOMETRY_HPP_
#define TUNNEL_ALIGN__TUNNEL_GEOMETRY_HPP_

#include <vector>

namespace tunnel_align
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Vec2D
{
  double vx = 0.0;
  double vy = 0.0;
};

/// Contract for VisionToGimbal.tunnel_align_angle_deg:
/// - RELATIVE_GIMBAL (Option A, recommended): target chassis yaw relative to current gimbal yaw
///   (0 deg = align chassis flush with gimbal; wrap(theta_tunnel - theta_gimbal) = align to tunnel).
/// - IMU_WORLD (Option B): target chassis absolute yaw in C-board power-on IMU frame
///   (host subtracts calibrated map-minus-IMU offset Delta_psi).
enum class AlignAngleMode
{
  RELATIVE_GIMBAL = 0,
  IMU_WORLD = 1,
};

struct TunnelEstimatorParams
{
  double roi_min_x = -0.6;
  double roi_max_x = 2.6;
  double min_half_width = 0.28;
  double max_half_width = 1.00;
  double min_corridor_width = 0.40;
  double max_corridor_width = 1.30;
  double slice_length = 0.20;
  double wall_inlier_band = 0.30;
  int min_slices_per_side = 3;
  double max_heading_correction_deg = 40.0;
  double min_elongation_ratio = 3.0;
  // Layer 1 anti-false-positive invariants (reject L-shaped room corners & capped dead-ends)
  double min_wall_length = 0.40;
  double min_bilateral_overlap = 0.35;
  double max_side_rms = 0.08;
  double max_side_angle_diff_deg = 16.0;
  double interior_margin_m = 0.09;
  int max_interior_block_points = 5;
  // Layer 2 doorway-gap + depth detector parameters (far / off-center occluded doorway approach)
  double doorway_min_wall_dist = 0.30;
  double doorway_max_wall_dist = 4.80;
  double doorway_wall_band_m = 0.12;
  double doorway_min_flank_length = 0.30;
  int doorway_min_flank_points = 4;
  double doorway_depth_min_behind = 0.22;
  double doorway_depth_max_behind = 2.80;
  double doorway_min_depth_span = 0.30;
  int doorway_min_depth_points = 4;
  double front_obstacle_min_x = 0.15;
  double front_obstacle_check_dist = 0.45;
  double front_obstacle_half_width = 0.24;
  int front_obstacle_min_points = 3;
  // Physical spin-clearance box around robot center: RESTORE (resuming spin) is forbidden
  // while >= spin_clear_min_points wall points lie inside [-half_length, +half_length] x [-half_width, +half_width].
  double spin_clear_half_length = 0.45;
  double spin_clear_half_width = 0.42;
  int spin_clear_min_points = 3;
};

struct TunnelEstimate
{
  bool valid = false;
  bool front_blocked = false;
  bool spin_zone_blocked = false;
  int spin_zone_points = 0;
  double axis_heading_rad = 0.0;
  /// Positive when anchor (robot) is to the LEFT of corridor centerline; negative when RIGHT.
  double lateral_offset_m = 0.0;
  double corridor_width_m = 0.0;
  double left_wall_dist_m = 0.0;
  double right_wall_dist_m = 0.0;
  double bilateral_overlap_m = 0.0;
  int left_inliers = 0;
  int right_inliers = 0;
};

struct DoorwayEstimate
{
  bool valid = false;
  Point2D center_odom;
  double axis_heading_rad = 0.0;
  double opening_width_m = 0.0;
  double wall_dist_m = 0.0;
  double depth_span_m = 0.0;
  int depth_support_points = 0;
};

/// Wrap angle to (-pi, pi].
double normalizeAngleRad(double angle_rad);

/// Wrap angle to (-180, 180].
double normalizeAngleDeg(double angle_deg);

/// Pick the symmetric equivalent of `tunnel_axis_rad` closest to `ref_heading_rad`.
/// symmetry_fold: 4 for square chassis (90 deg), 2 for rectangular (180 deg), 1 for exact.
double selectNearestSymmetricHeading(
  double tunnel_axis_rad, double ref_heading_rad, int symmetry_fold);

/// Compute VisionToGimbal.tunnel_align_angle_deg under Option A or Option B.
/// sign_multiplier: +1.0 for CCW-positive (standard ROS), -1.0 if firmware expects CW-positive.
double computeTunnelAlignAngleDeg(
  AlignAngleMode mode, double tunnel_axis_map_rad, double gimbal_map_rad, double chassis_map_rad,
  double map_minus_imu_rad, int symmetry_fold, double sign_multiplier = 1.0);

/// Extract parallel corridor heading and lateral centerline offset from 2D wall points.
/// Performs a coarse angle search around `path_heading_rad` (within `±max_heading_correction_deg`,
/// or full 360 deg if `full_angle_search` is true) followed by joint parallel-wall PCA refinement.
/// Falls back to `valid = false` and `axis_heading_rad = path_heading_rad` if walls are insufficient.
TunnelEstimate estimateTunnelFromWalls(
  const std::vector<Point2D> & obstacle_pts, const Point2D & anchor, double path_heading_rad,
  const TunnelEstimatorParams & params = TunnelEstimatorParams{}, bool full_angle_search = false);

/// Evaluate corridor walls and front-obstacle safety strictly along a single locked travel heading
/// (NO oblique ±40 deg sweep). Use during active TRAVERSE so perpendicular room walls at a tunnel
/// exit can never hijack heading or trigger oblique front_blocked false positives.
TunnelEstimate evaluateTunnelAtLockedHeading(
  const std::vector<Point2D> & obstacle_pts, const Point2D & anchor, double locked_heading_rad,
  const TunnelEstimatorParams & params = TunnelEstimatorParams{});

/// Evaluate front-obstacle safety box strictly on fresh/live points along `heading_rad`,
/// isolated from longer rolling wall memory so odometry drift can never create ghost front blocks.
bool isFrontPathBlocked(
  const std::vector<Point2D> & live_pts, const Point2D & anchor, double heading_rad,
  const TunnelEstimatorParams & params = TunnelEstimatorParams{});

/// Detect a doorway opening (wall gap flanked by colinear wall segments + verified tunnel depth
/// behind the gap) for approaching narrow tunnels from far away or off-center positions where one
/// inner tunnel wall is occluded by the doorway wall.
DoorwayEstimate detectDoorwayOpening(
  const std::vector<Point2D> & obstacle_pts, const Point2D & anchor,
  const TunnelEstimatorParams & params = TunnelEstimatorParams{});

/// Compute map/odom-frame velocity along `axis_heading_rad` plus bounded centering lateral PD.
Vec2D computeCenteredTraverseVelocity(
  double axis_heading_rad, double lateral_offset_m, double forward_speed, double lateral_kp,
  double max_lateral_speed);

/// Circular exponential moving average filter for angular offsets (handles ±pi smoothly).
class CircularOffsetFilter
{
public:
  explicit CircularOffsetFilter(double alpha = 0.2);

  void reset();
  double update(double measured_offset_rad);
  bool initialized() const { return initialized_; }
  double value() const;

private:
  double alpha_ = 0.2;
  bool initialized_ = false;
  double ema_cos_ = 1.0;
  double ema_sin_ = 0.0;
};

}  // namespace tunnel_align

#endif  // TUNNEL_ALIGN__TUNNEL_GEOMETRY_HPP_
