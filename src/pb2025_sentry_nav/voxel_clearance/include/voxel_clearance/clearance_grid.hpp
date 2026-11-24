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

#ifndef VOXEL_CLEARANCE__CLEARANCE_GRID_HPP_
#define VOXEL_CLEARANCE__CLEARANCE_GRID_HPP_

#include <cstdint>
#include <vector>

namespace voxel_clearance
{

enum class CellClass : uint8_t
{
  UNKNOWN = 0,
  FREE = 1,
  LOW_CLEARANCE = 2,
  HARD_OBSTACLE = 3,
};

struct Point3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ClearanceParams
{
  // Rolling 3D column grid geometry
  double xy_resolution = 0.10;
  double z_resolution = 0.02;
  double grid_radius = 10.0;
  double decay_time = 2.0;
  double low_clearance_latch_time = 6.0;

  // Slope-aware Z crop relative to robot base_z
  double min_rel_z = -2.5;
  double max_rel_z = 2.5;
  double dis_ratio_z = 0.5;

  // Robot self-filter box around (robot_x, robot_y, base_z)
  double self_filter_half_x = 0.30;
  double self_filter_half_y = 0.30;
  double self_filter_min_z = -0.10;
  double self_filter_max_z_margin = 0.15;

  // Step A: vertical clustering & single-column wall thickness test
  double cluster_z_gap = 0.08;
  double max_ground_thickness = 0.12;
  int min_ground_points = 1;
  int min_overhead_points = 1;

  // Step B: neighbor slope continuity & step/curb detection
  double max_traversable_slope_deg = 30.0;
  double max_step_height = 0.12;
  bool despeckle_isolated_obstacles = true;
  int min_plane_fit_neighbors = 5;
  int min_obstacle_confirm_frames = 1;

  // Step C: overhead vertical clearance classification (supports dual-height lifter)
  double robot_height_high = 0.55;
  double robot_height_low = 0.55;
  double clearance_margin_high = 0.05;
  double clearance_margin_low = 0.005;
};

struct ClassifiedCell
{
  int world_ix = 0;
  int world_iy = 0;
  double x = 0.0;
  double y = 0.0;
  double surface_z = 0.0;
  double slope_rad = 0.0;
  double clearance_m = -1.0;  // >= 0 when overhead cluster detected
  CellClass classification = CellClass::UNKNOWN;
};

class ClearanceGrid
{
public:
  explicit ClearanceGrid(const ClearanceParams & params = ClearanceParams{});

  void reset();
  void setParams(const ClearanceParams & params);
  const ClearanceParams & params() const { return params_; }

  /// Freeze/unfreeze new point cloud insertion (used during stationary lifter stroke).
  void setFrozen(bool frozen) { frozen_ = frozen; }
  bool isFrozen() const { return frozen_; }

  /// Update robot 3D ground-reference pose in odom frame.
  void updateRobotPose(double robot_x, double robot_y, double base_z);

  /// Insert a 3D point cloud in odom frame at timestamp `stamp_sec`.
  void insertPoints(const std::vector<Point3D> & points, double stamp_sec);

  /// Decay stale voxels and classify all active cells in the rolling window.
  void classify(double now_sec);

  /// Access classified cells from the most recent `classify()` call.
  const std::vector<ClassifiedCell> & classifiedCells() const { return classified_cells_; }

  /// Look up the classification at world (x, y) from the latest `classify()` call.
  CellClass queryCell(double x, double y) const;

  int centerIx() const { return center_ix_; }
  int centerIy() const { return center_iy_; }
  int gridWidth() const { return grid_width_; }
  int halfWidth() const { return half_width_; }

private:
  struct ZBin
  {
    int iz = 0;
    uint32_t count = 0;
    double min_z = 0.0;
    double max_z = 0.0;
    double last_stamp = 0.0;
  };

  struct Column
  {
    int world_ix = 0;
    int world_iy = 0;
    bool initialized = false;
    std::vector<ZBin> bins;
    double low_clearance_latch_until = -1.0;
    double latched_clearance_m = -1.0;
    int consecutive_hard_frames = 0;

    // Transient per-classification fields
    bool active = false;
    bool has_ground = false;
    bool has_overhead = false;
    double surface_z = 0.0;
    double overhead_min_z = 0.0;
    double slope_rad = 0.0;
    CellClass classification = CellClass::UNKNOWN;
  };

  int worldToGrid(double v) const;
  int wrapIndex(int idx) const;
  Column * getOrCreateColumn(int world_ix, int world_iy);
  const Column * findColumn(int world_ix, int world_iy) const;
  Column * findColumn(int world_ix, int world_iy);

  ClearanceParams params_;
  int half_width_ = 0;
  int grid_width_ = 0;
  std::vector<Column> columns_;
  std::vector<ClassifiedCell> classified_cells_;

  bool pose_initialized_ = false;
  bool frozen_ = false;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double base_z_ = 0.0;
  int center_ix_ = 0;
  int center_iy_ = 0;
};

}  // namespace voxel_clearance

#endif  // VOXEL_CLEARANCE__CLEARANCE_GRID_HPP_
