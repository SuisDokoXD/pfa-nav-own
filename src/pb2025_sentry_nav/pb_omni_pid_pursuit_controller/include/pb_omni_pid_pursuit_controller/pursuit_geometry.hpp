// Copyright 2025 Lihan Chen
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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__PURSUIT_GEOMETRY_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__PURSUIT_GEOMETRY_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace pb_omni_pid_pursuit_controller
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct LookaheadParams
{
  double lookahead_dist = 0.5;
  double max_chord_deviation = 0.06;
  bool use_interpolation = true;
  bool use_costmap_los_clamp = true;
  uint8_t los_cost_threshold = 253;
  uint8_t inscribed_cost = 253;
  double los_step_size = 0.05;
};

struct LookaheadResult
{
  Point2D point;
  size_t segment_index = 0;
  double arc_length = 0.0;
  double max_chord_deviation = 0.0;
  bool clamped_by_chord = false;
  bool clamped_by_los = false;
};

struct CrossTrackResult
{
  Point2D projection;
  Point2D perpendicular_error;
  Point2D tangent;
  double distance = 0.0;
  double arc_length = 0.0;
};

struct CurvatureResult
{
  double curvature = 0.0;
  Point2D backward_point;
  Point2D forward_point;
};

using CostQuery = std::function<std::optional<uint8_t>(const Point2D &)>;

/// Compute cumulative arc lengths along a 2D polyline.
std::vector<double> computeCumulativeArcLengths(const std::vector<Point2D> & path);

/// Sample a point on a 2D polyline at a target cumulative arc length.
Point2D samplePointAtArcLength(
  const std::vector<Point2D> & path, const std::vector<double> & cumulative_arc_lengths,
  double target_arc_length);

/// Perpendicular distance from point p to finite segment [a, b].
double pointToSegmentDistance(const Point2D & p, const Point2D & a, const Point2D & b);

/// Maximum perpendicular deviation of intermediate path points path[0..end_idx]
/// from the chord connecting path[0] to candidate.
double computeMaxChordDeviation(
  const std::vector<Point2D> & path, size_t end_idx, const Point2D & candidate);

/// Check whether straight segment origin(0,0)->target avoids high-cost/inscribed cells
/// in local costmap when advancing the lookahead carrot.
bool isSegmentSafeForLookahead(
  const Point2D & target, const CostQuery & cost_query, uint8_t los_cost_threshold,
  uint8_t inscribed_cost, double step_size);

/// Check whether straight segment a->b enters an inscribed/lethal obstacle cell.
/// If robot_is_at_a is true and the start cell is already >= inscribed_cost,
/// allows a short 1-2 step escape toward strictly lower cost.
bool isSegmentInCollision(
  const Point2D & a, const Point2D & b, const CostQuery & cost_query, uint8_t inscribed_cost,
  double step_size, bool robot_is_at_a = false);

/// Select lookahead carrot point in robot frame (origin = robot), walking sequentially
/// along the path and clamping by lookahead distance, max chord deviation, and optional LOS cost.
LookaheadResult selectClampedLookaheadPoint(
  const std::vector<Point2D> & path, const LookaheadParams & params,
  const CostQuery & cost_query = nullptr);

/// Compute perpendicular cross-track correction vector from robot origin (0,0)
/// to the closest point on the forward prefix of the path.
CrossTrackResult computeCrossTrackVector(
  const std::vector<Point2D> & path, double max_search_arc_length = 1.0);

/// Compute 3-point circumcircle curvature at a given true arc length along the path.
CurvatureResult computeArcLengthCurvature(
  const std::vector<Point2D> & path, const std::vector<double> & cumulative_arc_lengths,
  double center_arc_length, double backward_dist, double forward_dist);

/// Smooth and rate-limit nominal lookahead distance to reject odometry velocity spikes.
double smoothLookaheadDistance(
  double raw_lookahead_dist, double prev_lookahead_dist, double alpha, double max_rate,
  double dt, double min_dist, double max_dist);

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__PURSUIT_GEOMETRY_HPP_
