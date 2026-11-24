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

#include "voxel_clearance/clearance_grid.hpp"

#include <algorithm>
#include <cmath>

namespace voxel_clearance
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct VerticalCluster
{
  double min_z = 0.0;
  double max_z = 0.0;
  uint32_t total_points = 0;
};

/// Fit plane z = a*(x-x0) + b*(y-y0) + c via 3x3 normal equations.
/// Returns true and sets slope_rad = atan(hypot(a, b)) when well-conditioned.
bool fitLocalPlaneSlope(
  const std::vector<Point3D> & samples, double x0, double y0, double & slope_rad)
{
  if (samples.size() < 3) {
    return false;
  }

  double s_xx = 0.0, s_xy = 0.0, s_x = 0.0;
  double s_yy = 0.0, s_y = 0.0;
  double s_xz = 0.0, s_yz = 0.0, s_z = 0.0;
  const double n = static_cast<double>(samples.size());

  for (const auto & p : samples) {
    const double dx = p.x - x0;
    const double dy = p.y - y0;
    s_xx += dx * dx;
    s_xy += dx * dy;
    s_x += dx;
    s_yy += dy * dy;
    s_y += dy;
    s_xz += dx * p.z;
    s_yz += dy * p.z;
    s_z += p.z;
  }

  // Symmetric 3x3 system:
  // [s_xx s_xy s_x] [a]   [s_xz]
  // [s_xy s_yy s_y] [b] = [s_yz]
  // [s_x  s_y  n  ] [c]   [s_z ]
  const double m00 = s_xx, m01 = s_xy, m02 = s_x;
  const double m10 = s_xy, m11 = s_yy, m12 = s_y;
  const double m20 = s_x, m21 = s_y, m22 = n;

  const double c00 = m11 * m22 - m12 * m21;
  const double c01 = -(m10 * m22 - m12 * m20);
  const double c02 = m10 * m21 - m11 * m20;
  const double det = m00 * c00 + m01 * c01 + m02 * c02;
  // Require well-conditioned 2D spread (reject near-collinear sparse points that amplify Z noise).
  if (std::abs(det) < 1e-4) {
    return false;
  }

  const double c10 = -(m01 * m22 - m02 * m21);
  const double c11 = m00 * m22 - m02 * m20;
  const double c12 = -(m00 * m21 - m01 * m20);

  const double a = (c00 * s_xz + c10 * s_yz + (m01 * m12 - m02 * m11) * s_z) / det;
  const double b = (c01 * s_xz + c11 * s_yz + (-(m00 * m12 - m02 * m10)) * s_z) / det;
  // Note: adjugate row 0 is (c00, c10, c20) where c20 = m01*m12 - m02*m11.
  //       adjugate row 1 is (c01, c11, c21) where c21 = -(m00*m12 - m02*m10).
  (void)c12;

  if (!std::isfinite(a) || !std::isfinite(b)) {
    return false;
  }
  slope_rad = std::atan(std::hypot(a, b));
  return true;
}

}  // namespace

ClearanceGrid::ClearanceGrid(const ClearanceParams & params)
{
  setParams(params);
}

void ClearanceGrid::reset()
{
  for (auto & col : columns_) {
    col = Column{};
  }
  classified_cells_.clear();
}

void ClearanceGrid::setParams(const ClearanceParams & params)
{
  const bool realloc_needed =
    columns_.empty() ||
    std::abs(params.xy_resolution - params_.xy_resolution) > 1e-9 ||
    std::abs(params.grid_radius - params_.grid_radius) > 1e-9;

  params_ = params;
  params_.xy_resolution = std::max(params_.xy_resolution, 0.02);
  params_.z_resolution = std::max(params_.z_resolution, 0.005);
  params_.grid_radius = std::max(params_.grid_radius, params_.xy_resolution * 4.0);

  if (realloc_needed) {
    half_width_ = static_cast<int>(std::ceil(params_.grid_radius / params_.xy_resolution));
    grid_width_ = 2 * half_width_ + 1;
    columns_.assign(static_cast<size_t>(grid_width_ * grid_width_), Column{});
    classified_cells_.clear();
  }
}

void ClearanceGrid::updateRobotPose(double robot_x, double robot_y, double base_z)
{
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  base_z_ = base_z;
  center_ix_ = worldToGrid(robot_x_);
  center_iy_ = worldToGrid(robot_y_);
  pose_initialized_ = true;
}

int ClearanceGrid::worldToGrid(double v) const
{
  return static_cast<int>(std::floor(v / params_.xy_resolution));
}

int ClearanceGrid::wrapIndex(int idx) const
{
  const int m = idx % grid_width_;
  return (m < 0) ? (m + grid_width_) : m;
}

ClearanceGrid::Column * ClearanceGrid::getOrCreateColumn(int world_ix, int world_iy)
{
  if (std::abs(world_ix - center_ix_) > half_width_ ||
      std::abs(world_iy - center_iy_) > half_width_)
  {
    return nullptr;
  }
  const int sx = wrapIndex(world_ix);
  const int sy = wrapIndex(world_iy);
  Column & col = columns_[static_cast<size_t>(sy * grid_width_ + sx)];
  if (!col.initialized || col.world_ix != world_ix || col.world_iy != world_iy) {
    col = Column{};
    col.world_ix = world_ix;
    col.world_iy = world_iy;
    col.initialized = true;
  }
  return &col;
}

const ClearanceGrid::Column * ClearanceGrid::findColumn(int world_ix, int world_iy) const
{
  if (std::abs(world_ix - center_ix_) > half_width_ ||
      std::abs(world_iy - center_iy_) > half_width_)
  {
    return nullptr;
  }
  const int sx = wrapIndex(world_ix);
  const int sy = wrapIndex(world_iy);
  const Column & col = columns_[static_cast<size_t>(sy * grid_width_ + sx)];
  if (!col.initialized || col.world_ix != world_ix || col.world_iy != world_iy) {
    return nullptr;
  }
  return &col;
}

ClearanceGrid::Column * ClearanceGrid::findColumn(int world_ix, int world_iy)
{
  return const_cast<Column *>(
    static_cast<const ClearanceGrid *>(this)->findColumn(world_ix, world_iy));
}

void ClearanceGrid::insertPoints(const std::vector<Point3D> & points, double stamp_sec)
{
  if (frozen_ || !pose_initialized_) {
    return;
  }

  const double self_max_z =
    base_z_ + std::max(params_.robot_height_high, params_.robot_height_low) +
    params_.self_filter_max_z_margin;
  const double self_min_z = base_z_ + params_.self_filter_min_z;

  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }

    const double dx = p.x - robot_x_;
    const double dy = p.y - robot_y_;
    const double dis = std::hypot(dx, dy);
    if (dis > params_.grid_radius) {
      continue;
    }

    // Robot self-filter box
    if (std::abs(dx) <= params_.self_filter_half_x &&
        std::abs(dy) <= params_.self_filter_half_y &&
        p.z >= self_min_z && p.z <= self_max_z)
    {
      continue;
    }

    // Slope-aware vertical crop relative to chassis base_z
    const double rel_z = p.z - base_z_;
    if (rel_z < params_.min_rel_z - params_.dis_ratio_z * dis ||
        rel_z > params_.max_rel_z + params_.dis_ratio_z * dis)
    {
      continue;
    }

    const int wix = worldToGrid(p.x);
    const int wiy = worldToGrid(p.y);
    Column * col = getOrCreateColumn(wix, wiy);
    if (!col) {
      continue;
    }

    const int iz = static_cast<int>(std::floor(p.z / params_.z_resolution));
    auto it = std::lower_bound(
      col->bins.begin(), col->bins.end(), iz,
      [](const ZBin & bin, int target_iz) { return bin.iz < target_iz; });

    if (it != col->bins.end() && it->iz == iz) {
      it->count += 1;
      it->min_z = std::min(it->min_z, p.z);
      it->max_z = std::max(it->max_z, p.z);
      it->last_stamp = std::max(it->last_stamp, stamp_sec);
    } else {
      ZBin bin;
      bin.iz = iz;
      bin.count = 1;
      bin.min_z = p.z;
      bin.max_z = p.z;
      bin.last_stamp = stamp_sec;
      col->bins.insert(it, bin);
    }
  }
}

void ClearanceGrid::classify(double now_sec)
{
  classified_cells_.clear();
  if (!pose_initialized_) {
    return;
  }

  const int min_ix = center_ix_ - half_width_;
  const int max_ix = center_ix_ + half_width_;
  const int min_iy = center_iy_ - half_width_;
  const int max_iy = center_iy_ + half_width_;

  // Pass 1: decay stale bins, vertical clustering, and single-column wall thickness test (Test A)
  std::vector<Column *> active_cols;
  active_cols.reserve(2048);

  for (int iy = min_iy; iy <= max_iy; ++iy) {
    for (int ix = min_ix; ix <= max_ix; ++ix) {
      Column * col = findColumn(ix, iy);
      if (!col) {
        continue;
      }

      col->active = false;
      col->has_ground = false;
      col->has_overhead = false;
      col->slope_rad = 0.0;
      col->classification = CellClass::UNKNOWN;

      // Decay expired bins (skip decay while frozen so static map is preserved during lift)
      if (!frozen_ && params_.decay_time > 0.0) {
        col->bins.erase(
          std::remove_if(
            col->bins.begin(), col->bins.end(),
            [&](const ZBin & bin) { return (now_sec - bin.last_stamp) > params_.decay_time; }),
          col->bins.end());
      }

      if (col->bins.empty()) {
        // Keep latched LOW_CLEARANCE even if currently in LiDAR blind spot
        if (now_sec <= col->low_clearance_latch_until) {
          col->active = true;
          col->classification = CellClass::LOW_CLEARANCE;
          active_cols.push_back(col);
        }
        continue;
      }

      // Group sorted Z-bins into contiguous vertical clusters
      std::vector<VerticalCluster> clusters;
      clusters.reserve(4);
      for (const auto & bin : col->bins) {
        if (clusters.empty() || (bin.min_z - clusters.back().max_z) > params_.cluster_z_gap) {
          clusters.push_back({bin.min_z, bin.max_z, bin.count});
        } else {
          clusters.back().max_z = std::max(clusters.back().max_z, bin.max_z);
          clusters.back().total_points += bin.count;
        }
      }

      const auto & ground_cluster = clusters.front();
      if (static_cast<int>(ground_cluster.total_points) < params_.min_ground_points) {
        continue;
      }

      col->active = true;
      active_cols.push_back(col);

      const double thickness = ground_cluster.max_z - ground_cluster.min_z;
      if (thickness > params_.max_ground_thickness) {
        // Continuous vertical wall/face starting from ground
        col->surface_z = ground_cluster.min_z;
        col->classification = CellClass::HARD_OBSTACLE;
        continue;
      }

      // Valid ground surface candidate: midpoint is unbiased on tilted ramps
      col->has_ground = true;
      col->surface_z = 0.5 * (ground_cluster.min_z + ground_cluster.max_z);

      // Check for suspended overhead clusters above the ground cluster
      for (size_t k = 1; k < clusters.size(); ++k) {
        if (static_cast<int>(clusters[k].total_points) >= params_.min_overhead_points) {
          col->has_overhead = true;
          col->overhead_min_z = clusters[k].min_z;
          break;
        }
      }
    }
  }

  // Pass 2 & 3: 8-neighbor slope continuity + step/curb detection + overhead clearance (Tests B & C)
  const double max_slope_rad = params_.max_traversable_slope_deg * kPi / 180.0;
  const double res = params_.xy_resolution;

  std::vector<Point3D> plane_samples;
  plane_samples.reserve(9);

  for (Column * col : active_cols) {
    if (col->classification == CellClass::HARD_OBSTACLE) {
      continue;
    }
    if (!col->has_ground) {
      // Latched blind-spot LOW_CLEARANCE column
      continue;
    }

    const double cx = (static_cast<double>(col->world_ix) + 0.5) * res;
    const double cy = (static_cast<double>(col->world_iy) + 0.5) * res;

    plane_samples.clear();
    plane_samples.push_back({cx, cy, col->surface_z});

    int sharp_step_neighbors = 0;
    bool extreme_step_detected = false;
    double max_smooth_neighbor_slope = 0.0;

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const Column * nb = findColumn(col->world_ix + dx, col->world_iy + dy);
        if (!nb || !nb->has_ground) {
          continue;
        }
        const double signed_dz = col->surface_z - nb->surface_z;
        const double dz = std::abs(signed_dz);
        const double dd = res * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
        const double allowed_step =
          std::max(params_.max_step_height, dd * std::tan(max_slope_rad) + 0.04);

        // One-sided sharp step check scaled by neighbor distance so steep ramps (up to max_slope)
        // never trigger false steps across diagonal neighbors, while real curbs/walls still trigger.
        if (signed_dz > allowed_step) {
          sharp_step_neighbors++;
          if (signed_dz > 1.5 * allowed_step) {
            extreme_step_detected = true;
          }
        } else if (dz <= allowed_step) {
          const double nbx = (static_cast<double>(nb->world_ix) + 0.5) * res;
          const double nby = (static_cast<double>(nb->world_iy) + 0.5) * res;
          plane_samples.push_back({nbx, nby, nb->surface_z});
          max_smooth_neighbor_slope = std::max(max_smooth_neighbor_slope, std::atan2(dz, dd));
        }
      }
    }

    if (extreme_step_detected || sharp_step_neighbors >= 2) {
      col->classification = CellClass::HARD_OBSTACLE;
      continue;
    }

    double local_slope_rad = 0.0;
    const bool plane_ok =
      static_cast<int>(plane_samples.size()) >= params_.min_plane_fit_neighbors &&
      fitLocalPlaneSlope(plane_samples, cx, cy, local_slope_rad);
    if (!plane_ok) {
      // Sparse neighborhood: use smooth neighbor slope only for tilt compensation,
      // do not raise false steep-slope obstacle from 2-3 sparse noisy points.
      local_slope_rad = std::min(max_smooth_neighbor_slope, max_slope_rad);
    }
    col->slope_rad = local_slope_rad;

    if (plane_ok && local_slope_rad > max_slope_rad) {
      col->classification = CellClass::HARD_OBSTACLE;
      continue;
    }

    // Test C: Overhead clearance with 1 / cos(alpha) ramp tilt compensation
    const double cos_alpha = std::max(0.5, std::cos(local_slope_rad));
    const double h_req_low = params_.robot_height_low / cos_alpha + params_.clearance_margin_low;
    const double h_req_high = params_.robot_height_high / cos_alpha + params_.clearance_margin_high;

    if (col->has_overhead) {
      const double clearance = col->overhead_min_z - col->surface_z;
      if (clearance < h_req_low) {
        col->classification = CellClass::HARD_OBSTACLE;
      } else if (clearance < h_req_high) {
        col->classification = CellClass::LOW_CLEARANCE;
        col->low_clearance_latch_until = now_sec + params_.low_clearance_latch_time;
        col->latched_clearance_m = clearance;
      } else {
        col->classification = CellClass::FREE;
      }
    } else if (now_sec <= col->low_clearance_latch_until) {
      col->classification = CellClass::LOW_CLEARANCE;
    } else {
      col->classification = CellClass::FREE;
    }
  }

  // Pass 4: Despeckle isolated 1-cell HARD_OBSTACLE road noise (prevent slam_toolbox
  // occupancy_threshold=0.015 from stamping transient single-cell speckles into /map).
  if (params_.despeckle_isolated_obstacles) {
    std::vector<Column *> to_clear;
    for (Column * col : active_cols) {
      if (col->classification != CellClass::HARD_OBSTACLE) {
        continue;
      }
      int hard_neighbors = 0;
      int free_neighbors = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const Column * nb = findColumn(col->world_ix + dx, col->world_iy + dy);
          if (!nb || !nb->active) {
            continue;
          }
          if (nb->classification == CellClass::HARD_OBSTACLE) {
            hard_neighbors++;
          } else if (nb->classification == CellClass::FREE) {
            free_neighbors++;
          }
        }
      }
      if (hard_neighbors == 0 && free_neighbors >= 2) {
        to_clear.push_back(col);
      }
    }
    for (Column * col : to_clear) {
      col->classification = CellClass::FREE;
    }
  }

  // Pass 5: Temporal confirmation for HARD_OBSTACLE to prevent 1-frame glitches from
  // getting permanently stamped into /map by slam_toolbox.
  const int min_confirm = std::max(1, params_.min_obstacle_confirm_frames);
  for (Column * col : active_cols) {
    if (col->classification == CellClass::HARD_OBSTACLE) {
      col->consecutive_hard_frames += 1;
      if (col->consecutive_hard_frames < min_confirm) {
        col->classification = col->has_ground ? CellClass::FREE : CellClass::UNKNOWN;
      }
    } else {
      col->consecutive_hard_frames = 0;
    }
  }

  // Export classified cells
  classified_cells_.reserve(active_cols.size());
  for (const Column * col : active_cols) {
    if (col->classification == CellClass::UNKNOWN) {
      continue;
    }
    ClassifiedCell out;
    out.world_ix = col->world_ix;
    out.world_iy = col->world_iy;
    out.x = (static_cast<double>(col->world_ix) + 0.5) * res;
    out.y = (static_cast<double>(col->world_iy) + 0.5) * res;
    out.surface_z = col->surface_z;
    out.slope_rad = col->slope_rad;
    if (col->has_overhead) {
      out.clearance_m = col->overhead_min_z - col->surface_z;
    } else if (col->classification == CellClass::LOW_CLEARANCE) {
      out.clearance_m = col->latched_clearance_m;
    }
    out.classification = col->classification;
    classified_cells_.push_back(out);
  }
}

CellClass ClearanceGrid::queryCell(double x, double y) const
{
  const Column * col = findColumn(worldToGrid(x), worldToGrid(y));
  if (!col || !col->active) {
    return CellClass::UNKNOWN;
  }
  return col->classification;
}

}  // namespace voxel_clearance
