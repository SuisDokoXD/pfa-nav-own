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

#include "tunnel_align/tunnel_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace tunnel_align
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double computeMedian(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<long>(mid), values.end());
  double med = values[mid];
  if (values.size() % 2 == 0) {
    auto max_it = std::max_element(values.begin(), values.begin() + static_cast<long>(mid));
    med = 0.5 * (med + *max_it);
  }
  return med;
}

}  // namespace

double normalizeAngleRad(double angle_rad)
{
  double a = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (a <= 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

double normalizeAngleDeg(double angle_deg)
{
  double a = std::fmod(angle_deg + 180.0, 360.0);
  if (a <= 0.0) {
    a += 360.0;
  }
  return a - 180.0;
}

double selectNearestSymmetricHeading(
  double tunnel_axis_rad, double ref_heading_rad, int symmetry_fold)
{
  const int fold = std::max(1, symmetry_fold);
  const double step = 2.0 * kPi / static_cast<double>(fold);

  double best_heading = normalizeAngleRad(tunnel_axis_rad);
  double best_err = std::numeric_limits<double>::infinity();

  for (int k = 0; k < fold; ++k) {
    const double cand = normalizeAngleRad(tunnel_axis_rad + static_cast<double>(k) * step);
    const double err = std::abs(normalizeAngleRad(cand - ref_heading_rad));
    if (err < best_err) {
      best_err = err;
      best_heading = cand;
    }
  }
  return best_heading;
}

double computeTunnelAlignAngleDeg(
  AlignAngleMode mode, double tunnel_axis_map_rad, double gimbal_map_rad, double chassis_map_rad,
  double map_minus_imu_rad, int symmetry_fold, double sign_multiplier)
{
  const double target_chassis_map =
    selectNearestSymmetricHeading(tunnel_axis_map_rad, chassis_map_rad, symmetry_fold);

  double cmd_rad = 0.0;
  if (mode == AlignAngleMode::RELATIVE_GIMBAL) {
    cmd_rad = normalizeAngleRad(target_chassis_map - gimbal_map_rad);
  } else {
    cmd_rad = normalizeAngleRad(target_chassis_map - map_minus_imu_rad);
  }

  return normalizeAngleDeg(sign_multiplier * cmd_rad * 180.0 / kPi);
}

namespace
{

TunnelEstimate evaluateSingleHeading(
  const std::vector<Point2D> & obstacle_pts, const Point2D & anchor, double seed_heading_rad,
  const TunnelEstimatorParams & params, double & out_quality, bool require_forward_walls)
{
  out_quality = -1.0;
  TunnelEstimate est;
  est.axis_heading_rad = normalizeAngleRad(seed_heading_rad);

  const double ch = std::cos(seed_heading_rad);
  const double sh = std::sin(seed_heading_rad);
  const double slice_len = std::max(params.slice_length, 0.05);

  struct SliceEdge
  {
    bool has_left = false;
    Point2D left_pt;
    bool has_right = false;
    Point2D right_pt;
  };

  std::map<int, SliceEdge> slices;
  int front_hits = 0;
  int spin_hits = 0;

  for (const auto & p : obstacle_pts) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    const double dx = p.x - anchor.x;
    const double dy = p.y - anchor.y;
    const double lx = dx * ch + dy * sh;
    const double ly = -dx * sh + dy * ch;

    if (
      lx >= params.front_obstacle_min_x && lx <= params.front_obstacle_check_dist &&
      std::abs(ly) <= params.front_obstacle_half_width)
    {
      front_hits++;
    }

    if (
      std::abs(lx) <= params.spin_clear_half_length &&
      std::abs(ly) <= params.spin_clear_half_width)
    {
      spin_hits++;
    }

    if (lx < params.roi_min_x || lx > params.roi_max_x) {
      continue;
    }
    const double abs_y = std::abs(ly);
    if (abs_y < params.min_half_width || abs_y > params.max_half_width) {
      continue;
    }

    const int bin = static_cast<int>(std::floor(lx / slice_len));
    auto & edge = slices[bin];
    if (ly > 0.0) {
      if (!edge.has_left || ly < edge.left_pt.y) {
        edge.has_left = true;
        edge.left_pt = {lx, ly};
      }
    } else {
      if (!edge.has_right || ly > edge.right_pt.y) {
        edge.has_right = true;
        edge.right_pt = {lx, ly};
      }
    }
  }

  est.front_blocked = (front_hits >= params.front_obstacle_min_points);
  est.spin_zone_points = spin_hits;
  est.spin_zone_blocked = (spin_hits >= params.spin_clear_min_points);

  std::vector<Point2D> raw_left;
  std::vector<Point2D> raw_right;
  std::vector<double> left_ys;
  std::vector<double> right_ys;
  for (const auto & kv : slices) {
    if (kv.second.has_left) {
      raw_left.push_back(kv.second.left_pt);
      left_ys.push_back(kv.second.left_pt.y);
    }
    if (kv.second.has_right) {
      raw_right.push_back(kv.second.right_pt);
      right_ys.push_back(kv.second.right_pt.y);
    }
  }

  if (
    static_cast<int>(raw_left.size()) < params.min_slices_per_side ||
    static_cast<int>(raw_right.size()) < params.min_slices_per_side)
  {
    return est;
  }

  const double med_left_y = computeMedian(left_ys);
  const double med_right_y = computeMedian(right_ys);

  std::vector<Point2D> left_inliers;
  std::vector<Point2D> right_inliers;
  for (const auto & p : raw_left) {
    if (std::abs(p.y - med_left_y) <= params.wall_inlier_band) {
      left_inliers.push_back(p);
    }
  }
  for (const auto & p : raw_right) {
    if (std::abs(p.y - med_right_y) <= params.wall_inlier_band) {
      right_inliers.push_back(p);
    }
  }

  if (
    static_cast<int>(left_inliers.size()) < params.min_slices_per_side ||
    static_cast<int>(right_inliers.size()) < params.min_slices_per_side)
  {
    return est;
  }

  // Require meaningful wall support ahead of the robot so 360-deg search picks the direction
  // leading INTO the tunnel rather than away from it into an open room.
  int fwd_left = 0;
  int fwd_right = 0;
  for (const auto & p : left_inliers) {
    if (p.x > 0.0) {
      fwd_left++;
    }
  }
  for (const auto & p : right_inliers) {
    if (p.x > 0.0) {
      fwd_right++;
    }
  }
  const int min_fwd = std::max(1, params.min_slices_per_side - 1);
  if (require_forward_walls && (fwd_left < min_fwd || fwd_right < min_fwd)) {
    return est;
  }

  Point2D mean_l{0.0, 0.0};
  for (const auto & p : left_inliers) {
    mean_l.x += p.x;
    mean_l.y += p.y;
  }
  mean_l.x /= static_cast<double>(left_inliers.size());
  mean_l.y /= static_cast<double>(left_inliers.size());

  Point2D mean_r{0.0, 0.0};
  for (const auto & p : right_inliers) {
    mean_r.x += p.x;
    mean_r.y += p.y;
  }
  mean_r.x /= static_cast<double>(right_inliers.size());
  mean_r.y /= static_cast<double>(right_inliers.size());

  double cxx = 0.0;
  double cxy = 0.0;
  double cyy = 0.0;
  for (const auto & p : left_inliers) {
    const double dx = p.x - mean_l.x;
    const double dy = p.y - mean_l.y;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }
  for (const auto & p : right_inliers) {
    const double dx = p.x - mean_r.x;
    const double dy = p.y - mean_r.y;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }

  const double trace = cxx + cyy;
  const double diff = cxx - cyy;
  const double disc = std::hypot(diff, 2.0 * cxy);
  const double lambda_max = 0.5 * (trace + disc);
  const double lambda_min = std::max(0.0, 0.5 * (trace - disc));
  if (lambda_max < 1e-4) {
    return est;
  }
  if (lambda_min > 1e-6 && (lambda_max / lambda_min) < params.min_elongation_ratio) {
    return est;
  }

  double delta_rad = 0.5 * std::atan2(2.0 * cxy, diff);
  if (std::cos(delta_rad) < 0.0) {
    delta_rad = normalizeAngleRad(delta_rad + kPi);
  }
  // Single-seed refinement should stay within ±20 deg of the tested coarse heading.
  if (std::abs(delta_rad) > 20.0 * kPi / 180.0) {
    return est;
  }

  const double nx = -std::sin(delta_rad);
  const double ny = std::cos(delta_rad);
  const double left_dist = nx * mean_l.x + ny * mean_l.y;
  const double right_dist = -(nx * mean_r.x + ny * mean_r.y);
  const double corridor_width = left_dist + right_dist;
  if (
    left_dist <= 0.05 || right_dist <= 0.05 ||
    corridor_width < params.min_corridor_width ||
    corridor_width > params.max_corridor_width)
  {
    return est;
  }

  // Layer 1 anti-false-positive invariants: verify EACH side independently in the refined axis frame
  // so an L-shaped room corner (one long wall + 3 tilted perpendicular wall fragments) can never pass.
  const double cd = std::cos(delta_rad);
  const double sd = std::sin(delta_rad);
  struct SideStats
  {
    double min_u = 0.0;
    double max_u = 0.0;
    double span_u = 0.0;
    double rms_w = 0.0;
    double angle_diff_deg = 90.0;
    bool ok = false;
  };
  auto analyzeSide = [&](const std::vector<Point2D> & side_pts) -> SideStats {
    SideStats s;
    if (side_pts.size() < 2) {
      return s;
    }
    s.min_u = std::numeric_limits<double>::infinity();
    s.max_u = -std::numeric_limits<double>::infinity();
    double sum_u = 0.0;
    double sum_w = 0.0;
    for (const auto & p : side_pts) {
      const double u = p.x * cd + p.y * sd;
      const double w = -p.x * sd + p.y * cd;
      s.min_u = std::min(s.min_u, u);
      s.max_u = std::max(s.max_u, u);
      sum_u += u;
      sum_w += w;
    }
    const double inv_n = 1.0 / static_cast<double>(side_pts.size());
    const double mu = sum_u * inv_n;
    const double mw = sum_w * inv_n;
    double suu = 0.0;
    double suw = 0.0;
    double sww = 0.0;
    for (const auto & p : side_pts) {
      const double du = (p.x * cd + p.y * sd) - mu;
      const double dw = (-p.x * sd + p.y * cd) - mw;
      suu += du * du;
      suw += du * dw;
      sww += dw * dw;
    }
    s.span_u = s.max_u - s.min_u;
    s.rms_w = std::sqrt(sww * inv_n);
    if (suu < 1e-4) {
      return s;
    }
    // 0.5 * atan2(2*suw, suu - sww) lies in (-90 deg, +90 deg]; perpendicular walls yield ±90 deg!
    s.angle_diff_deg = std::abs(0.5 * std::atan2(2.0 * suw, suu - sww)) * 180.0 / kPi;
    s.ok = true;
    return s;
  };

  const SideStats left_stats = analyzeSide(left_inliers);
  const SideStats right_stats = analyzeSide(right_inliers);
  const double req_wall_len =
    require_forward_walls ? params.min_wall_length : std::min(params.min_wall_length, 0.30);
  const double req_overlap =
    require_forward_walls ? params.min_bilateral_overlap : std::min(params.min_bilateral_overlap, 0.25);
  const double overlap_min_u = std::max(left_stats.min_u, right_stats.min_u);
  const double overlap_max_u = std::min(left_stats.max_u, right_stats.max_u);
  const double overlap_u = overlap_max_u - overlap_min_u;

  if (
    !left_stats.ok || !right_stats.ok ||
    left_stats.span_u < req_wall_len || right_stats.span_u < req_wall_len ||
    overlap_u < req_overlap ||
    left_stats.rms_w > params.max_side_rms || right_stats.rms_w > params.max_side_rms ||
    left_stats.angle_diff_deg > params.max_side_angle_diff_deg ||
    right_stats.angle_diff_deg > params.max_side_angle_diff_deg)
  {
    return est;
  }

  const double refined_heading = normalizeAngleRad(seed_heading_rad + delta_rad);

  // During corridor discovery, verify the overlapping corridor interior is passable (not capped by
  // a solid perpendicular room wall across >=45% of the corridor width).
  if (require_forward_walls && params.max_interior_block_points > 0) {
    const double cr = std::cos(refined_heading);
    const double sr = std::sin(refined_heading);
    const double w_lo = -right_dist + params.interior_margin_m;
    const double w_hi = left_dist - params.interior_margin_m;
    if (w_hi > w_lo) {
      int interior_hits = 0;
      double min_int_w = std::numeric_limits<double>::infinity();
      double max_int_w = -std::numeric_limits<double>::infinity();
      for (const auto & p : obstacle_pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
          continue;
        }
        const double dx = p.x - anchor.x;
        const double dy = p.y - anchor.y;
        const double u = dx * cr + dy * sr;
        const double w = -dx * sr + dy * cr;
        if (u >= overlap_min_u && u <= overlap_max_u && w >= w_lo && w <= w_hi) {
          interior_hits++;
          min_int_w = std::min(min_int_w, w);
          max_int_w = std::max(max_int_w, w);
        }
      }
      if (
        interior_hits >= params.max_interior_block_points &&
        (max_int_w - min_int_w) >= 0.45 * corridor_width)
      {
        return est;
      }
    }
  }

  est.valid = true;
  est.axis_heading_rad = refined_heading;
  est.left_wall_dist_m = left_dist;
  est.right_wall_dist_m = right_dist;
  est.corridor_width_m = corridor_width;
  est.bilateral_overlap_m = overlap_u;
  est.lateral_offset_m = 0.5 * (right_dist - left_dist);
  est.left_inliers = static_cast<int>(left_inliers.size());
  est.right_inliers = static_cast<int>(right_inliers.size());

  // Quality favors strong bilateral overlap, forward coverage, and tight linear fit.
  const double n_fwd = static_cast<double>(fwd_left + fwd_right);
  const double n_total = static_cast<double>(est.left_inliers + est.right_inliers);
  const double rms_perp = std::sqrt(lambda_min / std::max(n_total, 1.0));
  const double balance = static_cast<double>(std::min(fwd_left, fwd_right));
  out_quality = (n_fwd + 0.35 * n_total + 0.5 * balance + 4.0 * overlap_u) / (rms_perp + 0.03);
  return est;
}

}  // namespace

TunnelEstimate estimateTunnelFromWalls(
  const std::vector<Point2D> & obstacle_pts, const Point2D & anchor, double path_heading_rad,
  const TunnelEstimatorParams & params, bool full_angle_search)
{
  // Coarse-to-fine search handles oblique approaches and full 360-deg tunnel discovery
  // (e.g. tunnel located directly behind a world-stabilized gimbal at 180 deg).
  const double search_half_deg =
    full_angle_search ? 180.0 : std::max(10.0, params.max_heading_correction_deg);
  const double step_deg = 10.0;

  double best_quality = -1.0;
  TunnelEstimate best;
  best.axis_heading_rad = normalizeAngleRad(path_heading_rad);

  auto tryAcceptCandidate = [&](const TunnelEstimate & cand) {
    if (!cand.valid) {
      return;
    }
    // Always re-verify strictly at the refined heading BEFORE accepting into best!
    // Prevents a tilted coarse seed (e.g. 70 deg) from passing an L-corner that collapses at 90 deg.
    double q_ref = -1.0;
    const TunnelEstimate refined =
      evaluateSingleHeading(obstacle_pts, anchor, cand.axis_heading_rad, params, q_ref, true);
    if (!refined.valid) {
      return;
    }
    if (!full_angle_search) {
      const double corr_deg =
        std::abs(normalizeAngleRad(refined.axis_heading_rad - path_heading_rad)) * 180.0 / kPi;
      if (corr_deg > params.max_heading_correction_deg) {
        return;
      }
    }
    const double ang_diff = std::abs(normalizeAngleRad(refined.axis_heading_rad - path_heading_rad));
    const double adj_q = q_ref * (1.0 - 0.05 * (ang_diff / kPi));
    if (adj_q > best_quality) {
      best_quality = adj_q;
      best = refined;
    }
  };

  // Check front safety along path_heading_rad as fallback
  double q0 = -1.0;
  const TunnelEstimate direct =
    evaluateSingleHeading(obstacle_pts, anchor, path_heading_rad, params, q0, true);
  best.front_blocked = direct.front_blocked;
  best.spin_zone_blocked = direct.spin_zone_blocked;
  best.spin_zone_points = direct.spin_zone_points;
  tryAcceptCandidate(direct);

  const int n_steps = static_cast<int>(std::ceil(search_half_deg / step_deg));
  for (int k = -n_steps; k <= n_steps; ++k) {
    if (k == 0) {
      continue;
    }
    const double offset_deg = static_cast<double>(k) * step_deg;
    if (std::abs(offset_deg) > search_half_deg + 1e-3) {
      continue;
    }
    const double cand_rad = normalizeAngleRad(path_heading_rad + offset_deg * kPi / 180.0);
    double q = -1.0;
    const TunnelEstimate cand =
      evaluateSingleHeading(obstacle_pts, anchor, cand_rad, params, q, true);
    tryAcceptCandidate(cand);
  }

  return best;
}

TunnelEstimate evaluateTunnelAtLockedHeading(
  const std::vector<Point2D> & obstacle_pts, const Point2D & anchor, double locked_heading_rad,
  const TunnelEstimatorParams & params)
{
  double q = -1.0;
  // During locked-heading TRAVERSE, keep bilateral wall tracking alive using side/rear tunnel walls
  // (require_forward_walls = false) so Point-LIO's forward blind sphere never triggers premature exit.
  return evaluateSingleHeading(obstacle_pts, anchor, locked_heading_rad, params, q, false);
}

bool isFrontPathBlocked(
  const std::vector<Point2D> & live_pts, const Point2D & anchor, double heading_rad,
  const TunnelEstimatorParams & params)
{
  const double ch = std::cos(heading_rad);
  const double sh = std::sin(heading_rad);
  int hits = 0;
  for (const auto & p : live_pts) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    const double dx = p.x - anchor.x;
    const double dy = p.y - anchor.y;
    const double lx = dx * ch + dy * sh;
    const double ly = -dx * sh + dy * ch;
    if (
      lx >= params.front_obstacle_min_x && lx <= params.front_obstacle_check_dist &&
      std::abs(ly) <= params.front_obstacle_half_width)
    {
      hits++;
      if (hits >= params.front_obstacle_min_points) {
        return true;
      }
    }
  }
  return false;
}

DoorwayEstimate detectDoorwayOpening(
  const std::vector<Point2D> & obstacle_pts, const Point2D & anchor,
  const TunnelEstimatorParams & params)
{
  DoorwayEstimate best;
  double best_score = -1.0;

  if (obstacle_pts.size() < 12) {
    return best;
  }

  const double u_min = std::max(0.15, params.doorway_min_wall_dist);
  const double u_max = std::max(u_min + 0.5, params.doorway_max_wall_dist);
  const double hist_res = 0.08;
  const int n_bins = static_cast<int>(std::ceil((u_max - u_min) / hist_res)) + 1;
  const double max_v_extent = u_max + 0.5;

  std::vector<Point2D> proj;
  proj.reserve(obstacle_pts.size());
  std::vector<int> hist(static_cast<size_t>(n_bins), 0);

  for (int deg = -180; deg < 180; deg += 5) {
    const double theta = static_cast<double>(deg) * kPi / 180.0;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    proj.clear();
    std::fill(hist.begin(), hist.end(), 0);

    for (const auto & p : obstacle_pts) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        continue;
      }
      const double dx = p.x - anchor.x;
      const double dy = p.y - anchor.y;
      const double u = dx * ct + dy * st;
      const double v = -dx * st + dy * ct;
      proj.push_back({u, v});
      if (u >= u_min && u <= u_max && std::abs(v) <= max_v_extent) {
        const int b = static_cast<int>(std::floor((u - u_min) / hist_res));
        if (b >= 0 && b < n_bins) {
          hist[static_cast<size_t>(b)]++;
        }
      }
    }

    // Find up to 3 strongest separated wall-plane peaks along normal direction `theta`
    std::vector<std::pair<int, int>> peaks;
    for (int b = 0; b < n_bins; ++b) {
      int sum3 = hist[static_cast<size_t>(b)];
      if (b > 0) {
        sum3 += hist[static_cast<size_t>(b - 1)];
      }
      if (b + 1 < n_bins) {
        sum3 += hist[static_cast<size_t>(b + 1)];
      }
      if (sum3 >= 2 * params.doorway_min_flank_points) {
        peaks.push_back({sum3, b});
      }
    }
    std::sort(
      peaks.begin(), peaks.end(),
      [](const auto & a, const auto & b) { return a.first > b.first; });

    std::vector<int> chosen_bins;
    for (const auto & pk : peaks) {
      bool far_enough = true;
      for (int prev : chosen_bins) {
        if (std::abs(pk.second - prev) < 3) {
          far_enough = false;
          break;
        }
      }
      if (far_enough) {
        chosen_bins.push_back(pk.second);
        if (chosen_bins.size() >= 3) {
          break;
        }
      }
    }

    for (int bin : chosen_bins) {
      const double u_peak = u_min + (static_cast<double>(bin) + 0.5) * hist_res;
      std::vector<double> band_us;
      std::vector<Point2D> wall_band;
      for (const auto & uv : proj) {
        if (std::abs(uv.x - u_peak) <= params.doorway_wall_band_m && std::abs(uv.y) <= max_v_extent) {
          band_us.push_back(uv.x);
          wall_band.push_back(uv);
        }
      }
      if (static_cast<int>(wall_band.size()) < 2 * params.doorway_min_flank_points) {
        continue;
      }

      const double u_wall = computeMedian(band_us);
      wall_band.erase(
        std::remove_if(
          wall_band.begin(), wall_band.end(),
          [&](const Point2D & uv) { return std::abs(uv.x - u_wall) > params.doorway_wall_band_m; }),
        wall_band.end());
      if (static_cast<int>(wall_band.size()) < 2 * params.doorway_min_flank_points) {
        continue;
      }

      std::sort(
        wall_band.begin(), wall_band.end(),
        [](const Point2D & a, const Point2D & b) { return a.y < b.y; });

      // Scan gaps between consecutive wall-band points sorted by tangential coordinate `v`
      for (size_t i = 0; i + 1 < wall_band.size(); ++i) {
        const double v_lo = wall_band[i].y;
        const double v_hi = wall_band[i + 1].y;
        const double gap = v_hi - v_lo;
        if (gap < params.min_corridor_width || gap > params.max_corridor_width) {
          continue;
        }

        // Verify contiguous flanking wall segment on the -v side
        size_t neg_start = i;
        while (neg_start > 0 && (wall_band[neg_start].y - wall_band[neg_start - 1].y) <= 0.28) {
          --neg_start;
        }
        const int neg_cnt = static_cast<int>(i - neg_start + 1);
        const double neg_span = wall_band[i].y - wall_band[neg_start].y;

        // Verify contiguous flanking wall segment on the +v side
        size_t pos_end = i + 1;
        while (pos_end + 1 < wall_band.size() &&
               (wall_band[pos_end + 1].y - wall_band[pos_end].y) <= 0.28)
        {
          ++pos_end;
        }
        const int pos_cnt = static_cast<int>(pos_end - (i + 1) + 1);
        const double pos_span = wall_band[pos_end].y - wall_band[i + 1].y;

        if (
          neg_cnt < params.doorway_min_flank_points || pos_cnt < params.doorway_min_flank_points ||
          neg_span < params.doorway_min_flank_length || pos_span < params.doorway_min_flank_length)
        {
          continue;
        }

        // Linear fit u = a + b*v on the flanking segments to verify straightness and refine wall normal
        double mean_u = 0.0;
        double mean_v = 0.0;
        const int flank_n = neg_cnt + pos_cnt;
        for (size_t k = neg_start; k <= i; ++k) {
          mean_u += wall_band[k].x;
          mean_v += wall_band[k].y;
        }
        for (size_t k = i + 1; k <= pos_end; ++k) {
          mean_u += wall_band[k].x;
          mean_v += wall_band[k].y;
        }
        mean_u /= static_cast<double>(flank_n);
        mean_v /= static_cast<double>(flank_n);

        double svv = 0.0;
        double suv = 0.0;
        for (size_t k = neg_start; k <= i; ++k) {
          const double dv = wall_band[k].y - mean_v;
          const double du = wall_band[k].x - mean_u;
          svv += dv * dv;
          suv += du * dv;
        }
        for (size_t k = i + 1; k <= pos_end; ++k) {
          const double dv = wall_band[k].y - mean_v;
          const double du = wall_band[k].x - mean_u;
          svv += dv * dv;
          suv += du * dv;
        }
        if (svv < 1e-4) {
          continue;
        }
        const double slope_b = suv / svv;
        const double delta_theta = -std::atan(slope_b);
        if (std::abs(delta_theta) > 12.0 * kPi / 180.0) {
          continue;
        }

        double sse = 0.0;
        for (size_t k = neg_start; k <= i; ++k) {
          const double err = (wall_band[k].x - mean_u) - slope_b * (wall_band[k].y - mean_v);
          sse += err * err;
        }
        for (size_t k = i + 1; k <= pos_end; ++k) {
          const double err = (wall_band[k].x - mean_u) - slope_b * (wall_band[k].y - mean_v);
          sse += err * err;
        }
        const double wall_rms = std::sqrt(sse / static_cast<double>(flank_n));
        if (wall_rms > 0.06) {
          continue;
        }

        // Refined wall normal pointing INTO the doorway
        const double theta_ref = normalizeAngleRad(theta + delta_theta);
        const double cr = std::cos(theta_ref);
        const double sr = std::sin(theta_ref);

        // Compute refined gap endpoints in the refined frame
        const double v_mid_rough = 0.5 * (v_lo + v_hi);
        const double u_mid_rough = mean_u + slope_b * (v_mid_rough - mean_v);
        const Point2D center_odom{
          anchor.x + u_mid_rough * ct - v_mid_rough * st,
          anchor.y + u_mid_rough * st + v_mid_rough * ct};
        const double u_ref = (center_odom.x - anchor.x) * cr + (center_odom.y - anchor.y) * sr;
        const double v_ref = -(center_odom.x - anchor.x) * sr + (center_odom.y - anchor.y) * cr;
        if (u_ref < u_min || u_ref > u_max) {
          continue;
        }
        const double opening_w = gap * std::cos(delta_theta);
        if (opening_w < params.min_corridor_width || opening_w > params.max_corridor_width) {
          continue;
        }

        // Verify: (a) opening plane itself is unblocked; (b) real tunnel depth exists behind the gap!
        int opening_block_hits = 0;
        int depth_pts = 0;
        double min_depth_u = std::numeric_limits<double>::infinity();
        double max_depth_u = -std::numeric_limits<double>::infinity();
        const double half_open = 0.5 * opening_w;
        const double clear_half = std::max(0.05, half_open - 0.08);
        const double depth_v_band = half_open + 0.16;

        for (const auto & p : obstacle_pts) {
          if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            continue;
          }
          const double dx = p.x - anchor.x;
          const double dy = p.y - anchor.y;
          const double u = dx * cr + dy * sr;
          const double v = -dx * sr + dy * cr;
          const double dv = std::abs(v - v_ref);

          if (std::abs(u - u_ref) <= 0.16 && dv <= clear_half) {
            opening_block_hits++;
          }
          if (
            u >= u_ref + params.doorway_depth_min_behind &&
            u <= u_ref + params.doorway_depth_max_behind &&
            dv <= depth_v_band)
          {
            depth_pts++;
            min_depth_u = std::min(min_depth_u, u);
            max_depth_u = std::max(max_depth_u, u);
          }
        }

        if (opening_block_hits > 1 || depth_pts < params.doorway_min_depth_points) {
          continue;
        }
        const double depth_span = max_depth_u - min_depth_u;
        if (depth_span < params.doorway_min_depth_span) {
          continue;
        }

        const double dist_to_center = std::hypot(center_odom.x - anchor.x, center_odom.y - anchor.y);
        const double flank_bonus = std::min(neg_span, 1.5) + std::min(pos_span, 1.5);
        const double score =
          (10.0 * depth_span + 0.4 * static_cast<double>(std::min(depth_pts, 40)) + flank_bonus) /
          ((dist_to_center + 0.5) * (wall_rms + 0.03));

        if (score > best_score) {
          best_score = score;
          best.valid = true;
          best.center_odom = center_odom;
          best.axis_heading_rad = theta_ref;
          best.opening_width_m = opening_w;
          best.wall_dist_m = u_ref;
          best.depth_span_m = depth_span;
          best.depth_support_points = depth_pts;
        }
      }
    }
  }

  return best;
}

Vec2D computeCenteredTraverseVelocity(
  double axis_heading_rad, double lateral_offset_m, double forward_speed, double lateral_kp,
  double max_lateral_speed)
{
  const double v_lat = std::clamp(
    -lateral_kp * lateral_offset_m, -std::abs(max_lateral_speed), std::abs(max_lateral_speed));
  const double c = std::cos(axis_heading_rad);
  const double s = std::sin(axis_heading_rad);
  return {forward_speed * c - v_lat * s, forward_speed * s + v_lat * c};
}

CircularOffsetFilter::CircularOffsetFilter(double alpha)
: alpha_(std::clamp(alpha, 0.01, 1.0))
{
}

void CircularOffsetFilter::reset()
{
  initialized_ = false;
  ema_cos_ = 1.0;
  ema_sin_ = 0.0;
}

double CircularOffsetFilter::update(double measured_offset_rad)
{
  const double c = std::cos(measured_offset_rad);
  const double s = std::sin(measured_offset_rad);
  if (!initialized_) {
    ema_cos_ = c;
    ema_sin_ = s;
    initialized_ = true;
  } else {
    ema_cos_ = (1.0 - alpha_) * ema_cos_ + alpha_ * c;
    ema_sin_ = (1.0 - alpha_) * ema_sin_ + alpha_ * s;
    const double norm = std::hypot(ema_cos_, ema_sin_);
    if (norm > 1e-6) {
      ema_cos_ /= norm;
      ema_sin_ /= norm;
    }
  }
  return value();
}

double CircularOffsetFilter::value() const
{
  return std::atan2(ema_sin_, ema_cos_);
}

}  // namespace tunnel_align
