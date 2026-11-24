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

#include "pb_omni_pid_pursuit_controller/pursuit_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pb_omni_pid_pursuit_controller
{

namespace
{

constexpr double kEps = 1e-9;

double dist2D(const Point2D & a, const Point2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

Point2D lerp2D(const Point2D & a, const Point2D & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

double circumcircleRadius(const Point2D & p1, const Point2D & p2, const Point2D & p3)
{
  const double denom = 2.0 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
  if (std::abs(denom) < kEps) {
    return 1e9;
  }
  const double p1_sq = p1.x * p1.x + p1.y * p1.y;
  const double p2_sq = p2.x * p2.x + p2.y * p2.y;
  const double p3_sq = p3.x * p3.x + p3.y * p3.y;

  const double cx = (p1_sq * (p2.y - p3.y) + p2_sq * (p3.y - p1.y) + p3_sq * (p1.y - p2.y)) / denom;
  const double cy = (p1_sq * (p3.x - p2.x) + p2_sq * (p1.x - p3.x) + p3_sq * (p2.x - p1.x)) / denom;
  const double radius = std::hypot(p2.x - cx, p2.y - cy);
  if (!std::isfinite(radius) || radius < kEps) {
    return 1e9;
  }
  return radius;
}

}  // namespace

std::vector<double> computeCumulativeArcLengths(const std::vector<Point2D> & path)
{
  std::vector<double> cumulative;
  if (path.empty()) {
    return cumulative;
  }
  cumulative.reserve(path.size());
  cumulative.push_back(0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    cumulative.push_back(cumulative.back() + dist2D(path[i - 1], path[i]));
  }
  return cumulative;
}

Point2D samplePointAtArcLength(
  const std::vector<Point2D> & path, const std::vector<double> & cumulative_arc_lengths,
  double target_arc_length)
{
  if (path.empty() || cumulative_arc_lengths.empty()) {
    return {};
  }
  if (target_arc_length <= 0.0 || path.size() == 1) {
    return path.front();
  }
  if (target_arc_length >= cumulative_arc_lengths.back()) {
    return path.back();
  }
  auto it = std::lower_bound(
    cumulative_arc_lengths.begin(), cumulative_arc_lengths.end(), target_arc_length);
  const size_t idx = static_cast<size_t>(std::distance(cumulative_arc_lengths.begin(), it));
  if (idx == 0) {
    return path.front();
  }
  const double seg_len = cumulative_arc_lengths[idx] - cumulative_arc_lengths[idx - 1];
  if (seg_len < kEps) {
    return path[idx];
  }
  const double ratio = (target_arc_length - cumulative_arc_lengths[idx - 1]) / seg_len;
  return lerp2D(path[idx - 1], path[idx], ratio);
}

double pointToSegmentDistance(const Point2D & p, const Point2D & a, const Point2D & b)
{
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double len_sq = abx * abx + aby * aby;
  if (len_sq < kEps) {
    return dist2D(p, a);
  }
  const double t = std::clamp(((p.x - a.x) * abx + (p.y - a.y) * aby) / len_sq, 0.0, 1.0);
  const Point2D proj{a.x + t * abx, a.y + t * aby};
  return dist2D(p, proj);
}

double computeMaxChordDeviation(
  const std::vector<Point2D> & path, size_t end_idx, const Point2D & candidate)
{
  if (path.empty() || end_idx == 0) {
    return 0.0;
  }
  const size_t clamped_end = std::min(end_idx, path.size() - 1);
  const Point2D & start = path.front();
  double max_dev = 0.0;
  for (size_t j = 1; j <= clamped_end; ++j) {
    max_dev = std::max(max_dev, pointToSegmentDistance(path[j], start, candidate));
  }
  return max_dev;
}

bool isSegmentSafeForLookahead(
  const Point2D & target, const CostQuery & cost_query, uint8_t los_cost_threshold,
  uint8_t inscribed_cost, double step_size)
{
  if (!cost_query) {
    return true;
  }
  const double length = std::hypot(target.x, target.y);
  const double step = std::max(step_size, 0.01);
  if (length < kEps) {
    return true;
  }

  const auto origin_cost_opt = cost_query(Point2D{0.0, 0.0});
  const uint8_t origin_cost = origin_cost_opt.value_or(0);

  const int steps = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Point2D sample{target.x * t, target.y * t};
    const auto cost_opt = cost_query(sample);
    if (!cost_opt.has_value()) {
      continue;
    }
    const uint8_t c = *cost_opt;
    // Allow escaping a brief 1-2 step border overlap if strictly moving to lower cost.
    const bool escaping_origin =
      (origin_cost >= inscribed_cost) && (t * length <= 2.0 * step) && (c < origin_cost);
    if (c >= inscribed_cost && !escaping_origin) {
      return false;
    }
    if (origin_cost < los_cost_threshold) {
      if (c >= los_cost_threshold) {
        return false;
      }
    } else if (c > origin_cost) {
      // Already in an inflated cell: do not allow cutting into even higher cost.
      return false;
    }
  }
  return true;
}

bool isSegmentInCollision(
  const Point2D & a, const Point2D & b, const CostQuery & cost_query, uint8_t inscribed_cost,
  double step_size, bool robot_is_at_a)
{
  if (!cost_query) {
    return false;
  }
  const double length = dist2D(a, b);
  const double step = std::max(step_size, 0.01);
  const uint8_t start_cost = robot_is_at_a ? cost_query(a).value_or(0) : 0;

  if (length < kEps) {
    const auto c = cost_query(b);
    return c.has_value() && *c >= inscribed_cost && !robot_is_at_a;
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(length / step)));
  const int start_i = robot_is_at_a ? 1 : 0;
  for (int i = start_i; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Point2D sample = lerp2D(a, b, t);
    const auto cost_opt = cost_query(sample);
    if (!cost_opt.has_value()) {
      continue;
    }
    const uint8_t c = *cost_opt;
    if (c >= inscribed_cost) {
      const bool escaping =
        robot_is_at_a && (start_cost >= inscribed_cost) && (t * length <= 2.0 * step) &&
        (c < start_cost);
      if (!escaping) {
        return true;
      }
    }
  }
  return false;
}

LookaheadResult selectClampedLookaheadPoint(
  const std::vector<Point2D> & path, const LookaheadParams & params, const CostQuery & cost_query)
{
  LookaheadResult result;
  if (path.empty()) {
    return result;
  }
  if (path.size() == 1) {
    result.point = path.front();
    return result;
  }

  const auto cumulative = computeCumulativeArcLengths(path);
  const double target_dist = std::max(params.lookahead_dist, 0.0);
  const bool check_chord = params.max_chord_deviation > 0.0;
  const bool check_los = params.use_costmap_los_clamp && static_cast<bool>(cost_query);

  // If the start of the path is already beyond lookahead_dist, aim straight at path[0].
  if (std::hypot(path[0].x, path[0].y) >= target_dist) {
    result.point = path[0];
    result.segment_index = 0;
    result.arc_length = 0.0;
    return result;
  }

  auto evaluateCandidate = [&](size_t seg_idx, const Point2D & cand, bool & chord_ok, bool & los_ok,
                               double & dev) {
    dev = check_chord ? computeMaxChordDeviation(path, seg_idx, cand) : 0.0;
    chord_ok = !check_chord || (dev <= params.max_chord_deviation);
    los_ok = !check_los || isSegmentSafeForLookahead(
                             cand, cost_query, params.los_cost_threshold, params.inscribed_cost,
                             params.los_step_size);
    return chord_ok && los_ok;
  };

  size_t best_idx = 0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Point2D & p1 = path[i];
    const Point2D & p2 = path[i + 1];
    const double seg_len = cumulative[i + 1] - cumulative[i];
    const double d2 = std::hypot(p2.x, p2.y);

    bool chord_ok = true;
    bool los_ok = true;
    double dev = 0.0;
    const bool p2_valid = evaluateCandidate(i, p2, chord_ok, los_ok, dev);

    if (p2_valid && d2 < target_dist) {
      best_idx = i + 1;
      continue;
    }

    // Either p2 reached/exceeded lookahead_dist, or p2 violated chord/LOS constraints.
    if (!params.use_interpolation) {
      if (p2_valid) {
        result.point = p2;
        result.segment_index = i + 1;
        result.arc_length = cumulative[i + 1];
        result.max_chord_deviation = dev;
        return result;
      }
      result.point = path[best_idx];
      result.segment_index = best_idx;
      result.arc_length = cumulative[best_idx];
      result.max_chord_deviation =
        check_chord ? computeMaxChordDeviation(path, best_idx, path[best_idx]) : 0.0;
      result.clamped_by_chord = !chord_ok;
      result.clamped_by_los = !los_ok;
      return result;
    }

    // Binary search largest t in [0, 1] satisfying distance <= target_dist, chord, and LOS.
    double lo = 0.0;
    double hi = 1.0;
    bool hit_chord = false;
    bool hit_los = false;
    for (int iter = 0; iter < 14; ++iter) {
      const double mid = 0.5 * (lo + hi);
      const Point2D cand = lerp2D(p1, p2, mid);
      const double d_cand = std::hypot(cand.x, cand.y);
      bool mid_chord_ok = true;
      bool mid_los_ok = true;
      double mid_dev = 0.0;
      const bool mid_ok = evaluateCandidate(i, cand, mid_chord_ok, mid_los_ok, mid_dev);
      if (mid_ok && d_cand <= target_dist) {
        lo = mid;
      } else {
        if (!mid_chord_ok) {
          hit_chord = true;
        }
        if (!mid_los_ok) {
          hit_los = true;
        }
        hi = mid;
      }
    }

    const Point2D chosen = lerp2D(p1, p2, lo);
    result.point = chosen;
    result.segment_index = i;
    result.arc_length = cumulative[i] + lo * seg_len;
    result.max_chord_deviation = check_chord ? computeMaxChordDeviation(path, i, chosen) : 0.0;
    result.clamped_by_chord = hit_chord && !chord_ok;
    result.clamped_by_los = hit_los && !los_ok;
    return result;
  }

  // Reached end of path without violating constraints.
  result.point = path.back();
  result.segment_index = path.size() - 1;
  result.arc_length = cumulative.back();
  result.max_chord_deviation =
    check_chord ? computeMaxChordDeviation(path, path.size() - 1, path.back()) : 0.0;
  return result;
}

CrossTrackResult computeCrossTrackVector(
  const std::vector<Point2D> & path, double max_search_arc_length)
{
  CrossTrackResult res;
  if (path.empty()) {
    return res;
  }
  if (path.size() == 1) {
    res.projection = path.front();
    res.perpendicular_error = path.front();
    res.distance = std::hypot(path.front().x, path.front().y);
    return res;
  }

  const auto cumulative = computeCumulativeArcLengths(path);
  double best_dist_sq = std::numeric_limits<double>::max();
  Point2D best_proj = path.front();
  Point2D best_tangent{1.0, 0.0};
  double best_arc = 0.0;

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (i > 0 && cumulative[i] > max_search_arc_length) {
      break;
    }
    const Point2D & a = path[i];
    const Point2D & b = path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    if (len_sq < kEps) {
      continue;
    }
    const double t = std::clamp(((-a.x) * dx + (-a.y) * dy) / len_sq, 0.0, 1.0);
    const Point2D proj{a.x + t * dx, a.y + t * dy};
    const double d_sq = proj.x * proj.x + proj.y * proj.y;
    if (d_sq < best_dist_sq) {
      best_dist_sq = d_sq;
      best_proj = proj;
      const double seg_len = std::sqrt(len_sq);
      best_tangent = {dx / seg_len, dy / seg_len};
      best_arc = cumulative[i] + t * seg_len;
    }
  }

  res.projection = best_proj;
  res.tangent = best_tangent;
  res.arc_length = best_arc;

  // Keep only the component perpendicular to the path tangent so CTE never opposes forward motion.
  const double along = best_proj.x * best_tangent.x + best_proj.y * best_tangent.y;
  res.perpendicular_error = {
    best_proj.x - along * best_tangent.x,
    best_proj.y - along * best_tangent.y,
  };
  res.distance = std::hypot(res.perpendicular_error.x, res.perpendicular_error.y);
  return res;
}

CurvatureResult computeArcLengthCurvature(
  const std::vector<Point2D> & path, const std::vector<double> & cumulative_arc_lengths,
  double center_arc_length, double backward_dist, double forward_dist)
{
  CurvatureResult res;
  if (path.size() < 3 || cumulative_arc_lengths.size() != path.size()) {
    if (!path.empty()) {
      res.backward_point = path.front();
      res.forward_point = path.back();
    }
    return res;
  }

  const Point2D center = samplePointAtArcLength(path, cumulative_arc_lengths, center_arc_length);
  const Point2D back =
    samplePointAtArcLength(path, cumulative_arc_lengths, center_arc_length - backward_dist);
  const Point2D fwd =
    samplePointAtArcLength(path, cumulative_arc_lengths, center_arc_length + forward_dist);

  res.backward_point = back;
  res.forward_point = fwd;
  const double radius = circumcircleRadius(back, center, fwd);
  res.curvature = (radius > kEps) ? (1.0 / radius) : 0.0;
  return res;
}

double smoothLookaheadDistance(
  double raw_lookahead_dist, double prev_lookahead_dist, double alpha, double max_rate,
  double dt, double min_dist, double max_dist)
{
  const double low = std::min(min_dist, max_dist);
  const double high = std::max(min_dist, max_dist);
  const double raw = std::clamp(raw_lookahead_dist, low, high);
  const double prev = std::clamp(prev_lookahead_dist, low, high);

  double filtered = raw;
  if (alpha > 0.0 && alpha < 1.0) {
    filtered = prev + alpha * (raw - prev);
  }
  if (max_rate > 0.0 && dt > 0.0) {
    const double max_step = max_rate * dt;
    filtered = prev + std::clamp(filtered - prev, -max_step, max_step);
  }
  return std::clamp(filtered, low, high);
}

}  // namespace pb_omni_pid_pursuit_controller
