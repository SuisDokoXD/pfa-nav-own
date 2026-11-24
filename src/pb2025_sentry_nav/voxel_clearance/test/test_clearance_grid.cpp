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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "voxel_clearance/clearance_grid.hpp"

using namespace voxel_clearance;  // NOLINT

namespace
{

constexpr double kPi = 3.14159265358979323846;

void appendFlatSurface(
  std::vector<Point3D> & pts, double x0, double x1, double y0, double y1, double z,
  double step = 0.04)
{
  for (double x = x0; x <= x1 + 1e-6; x += step) {
    for (double y = y0; y <= y1 + 1e-6; y += step) {
      pts.push_back({x, y, z});
    }
  }
}

void appendRampX(
  std::vector<Point3D> & pts, double x0, double x1, double y0, double y1, double z0,
  double angle_deg, double step = 0.04)
{
  const double slope = std::tan(angle_deg * kPi / 180.0);
  for (double x = x0; x <= x1 + 1e-6; x += step) {
    const double z = z0 + (x - x0) * slope;
    for (double y = y0; y <= y1 + 1e-6; y += step) {
      pts.push_back({x, y, z});
    }
  }
}

void appendVerticalWallY(
  std::vector<Point3D> & pts, double x_wall, double y0, double y1, double z0, double z1,
  double step_y = 0.04, double step_z = 0.04)
{
  for (double y = y0; y <= y1 + 1e-6; y += step_y) {
    for (double z = z0; z <= z1 + 1e-6; z += step_z) {
      pts.push_back({x_wall, y, z});
    }
  }
}

}  // namespace

TEST(ClearanceGridTest, FlatGroundVerticalWallAndHighCeiling)
{
  ClearanceParams params;
  params.robot_height_high = 0.55;
  params.robot_height_low = 0.55;
  ClearanceGrid grid(params);
  grid.updateRobotPose(0.0, 0.0, 0.0);

  std::vector<Point3D> pts;
  appendFlatSurface(pts, 0.5, 3.0, -1.0, 1.0, 0.0);
  // High ceiling at z = 1.8 m (well above robot_height_high)
  appendFlatSurface(pts, 0.8, 2.0, -0.8, 0.8, 1.8);
  // Vertical wall at x = 2.55 m
  appendVerticalWallY(pts, 2.55, -0.8, 0.8, 0.0, 1.2);

  grid.insertPoints(pts, 1.0);
  grid.classify(1.0);

  EXPECT_EQ(grid.queryCell(1.25, 0.05), CellClass::FREE);
  EXPECT_EQ(grid.queryCell(2.05, 0.05), CellClass::FREE);
  EXPECT_EQ(grid.queryCell(2.55, 0.05), CellClass::HARD_OBSTACLE);
}

TEST(ClearanceGridTest, Ramp14And23DegOccludedCrestProducesZeroFalseObstacles)
{
  // Reproduces the exact failure observed at the top crest of the 14 deg ramp (and steeper 23 deg):
  // LiDAR sees the ramp up to the crest at x = 3.5 m, while the flat platform beyond x > 3.5 m
  // is occluded by grazing angle (no points beyond x = 3.5 m).
  for (double angle_deg : {14.0, 23.0}) {
    ClearanceParams params;
    params.max_traversable_slope_deg = 28.0;
    params.max_step_height = 0.10;
    ClearanceGrid grid(params);
    grid.updateRobotPose(0.0, 0.0, 0.0);

    std::vector<Point3D> pts;
    appendFlatSurface(pts, 0.4, 1.0, -0.8, 0.8, 0.0);
    appendRampX(pts, 1.0, 3.5, -0.8, 0.8, 0.0, angle_deg);
    // Only the very crest lip (5 cm strip) of the upper flat platform is visible
    const double z_top = 2.5 * std::tan(angle_deg * kPi / 180.0);
    appendFlatSurface(pts, 3.5, 3.55, -0.8, 0.8, z_top);

    grid.insertPoints(pts, 1.0);
    grid.classify(1.0);

    for (double x = 0.65; x <= 3.45; x += 0.10) {
      EXPECT_EQ(grid.queryCell(x, 0.05), CellClass::FREE)
        << "False obstacle at x=" << x << " on " << angle_deg << " deg ramp/crest";
    }
  }
}

TEST(ClearanceGridTest, VerticalCurbAndSteep35DegSlopeAreHardObstacles)
{
  ClearanceParams params;
  params.max_traversable_slope_deg = 28.0;
  params.max_step_height = 0.10;
  ClearanceGrid grid(params);
  grid.updateRobotPose(0.0, 0.0, 0.0);

  std::vector<Point3D> pts;
  // 15 cm vertical curb starting at x = 1.5 m (no vertical face points, only horizontal top!)
  appendFlatSurface(pts, 0.5, 1.48, -0.6, 0.6, 0.0);
  appendFlatSurface(pts, 1.52, 2.2, -0.6, 0.6, 0.15);

  // Separate 35 deg unclimbable slope on the negative x side
  appendRampX(pts, -3.0, -1.0, -0.6, 0.6, 0.0, 35.0);

  grid.insertPoints(pts, 1.0);
  grid.classify(1.0);

  // Floor right before the curb stays FREE (no inward wall thickening)
  EXPECT_EQ(grid.queryCell(1.35, 0.05), CellClass::FREE);
  // Raised curb edge is HARD_OBSTACLE
  EXPECT_EQ(grid.queryCell(1.55, 0.05), CellClass::HARD_OBSTACLE);
  // Sustained 35 deg slope is HARD_OBSTACLE
  EXPECT_EQ(grid.queryCell(-2.05, 0.05), CellClass::HARD_OBSTACLE);
}

TEST(ClearanceGridTest, Ramp11DegWithHorizontalOverheadBarAnd10mmMargin)
{
  // 11 deg ramp with a flat horizontal bar (~1/3 robot length = 0.2 m) above x in [2.0, 2.2].
  // Low height = 0.533 m, High height = 0.725 m (delta = 0.19179 m).
  // At 11 deg, required vertical clearance in low state = 0.533 / cos(11 deg) + 0.005 = 0.5480 m.
  ClearanceParams params;
  params.robot_height_high = 0.725;
  params.robot_height_low = 0.533;
  params.clearance_margin_high = 0.05;
  params.clearance_margin_low = 0.005;
  params.low_clearance_latch_time = 5.0;

  ClearanceGrid grid(params);
  grid.updateRobotPose(0.0, 0.0, 0.0);

  const double slope = std::tan(11.0 * kPi / 180.0);
  const double z_ground_upper_edge = (2.2 - 1.0) * slope;
  const double z_bar_passable = z_ground_upper_edge + (0.533 / std::cos(11.0 * kPi / 180.0)) + 0.010;

  std::vector<Point3D> pts;
  appendRampX(pts, 1.0, 3.5, -0.8, 0.8, 0.0, 11.0);
  // Horizontal bar at constant world altitude z_bar_passable
  appendFlatSurface(pts, 2.0, 2.2, -0.6, 0.6, z_bar_passable);

  grid.insertPoints(pts, 1.0);
  grid.classify(1.0);

  // Ramp before and after the bar is FREE
  EXPECT_EQ(grid.queryCell(1.55, 0.05), CellClass::FREE);
  EXPECT_EQ(grid.queryCell(2.85, 0.05), CellClass::FREE);
  // Both lower and tightest upper edge under the horizontal bar are LOW_CLEARANCE
  EXPECT_EQ(grid.queryCell(2.05, 0.05), CellClass::LOW_CLEARANCE);
  EXPECT_EQ(grid.queryCell(2.15, 0.05), CellClass::LOW_CLEARANCE);

  // Blind-spot latch: even after points decay (t = 4.0 > decay_time=2.0), LOW_CLEARANCE stays latched (< 1+5=6.0)
  grid.classify(4.0);
  EXPECT_EQ(grid.queryCell(2.15, 0.05), CellClass::LOW_CLEARANCE);

  // Lowering the bar by 20 mm (below low-state tilted height) -> must become HARD_OBSTACLE
  grid.reset();
  grid.updateRobotPose(0.0, 0.0, 0.0);
  std::vector<Point3D> too_low_pts;
  appendRampX(too_low_pts, 1.0, 3.5, -0.8, 0.8, 0.0, 11.0);
  appendFlatSurface(too_low_pts, 2.0, 2.2, -0.6, 0.6, z_bar_passable - 0.020);
  grid.insertPoints(too_low_pts, 10.0);
  grid.classify(10.0);
  EXPECT_EQ(grid.queryCell(2.15, 0.05), CellClass::HARD_OBSTACLE);
}

TEST(ClearanceGridTest, HighElevationAndNegativeDownhillWorkWithoutClipping)
{
  ClearanceParams params;
  ClearanceGrid grid(params);

  // Summit at z = 3.20 m (matching switchback_ramps P5)
  grid.updateRobotPose(2.0, 11.0, 3.20);
  std::vector<Point3D> high_pts;
  appendFlatSurface(high_pts, 1.0, 3.5, 10.2, 12.2, 3.20);
  appendVerticalWallY(high_pts, 3.55, 10.4, 12.0, 3.20, 4.40);
  grid.insertPoints(high_pts, 1.0);
  grid.classify(1.0);
  EXPECT_EQ(grid.queryCell(2.65, 11.05), CellClass::FREE);
  EXPECT_EQ(grid.queryCell(3.55, 11.05), CellClass::HARD_OBSTACLE);

  // Negative elevation downhill at z = -1.50 m
  grid.reset();
  grid.updateRobotPose(0.0, 0.0, -1.50);
  std::vector<Point3D> low_pts;
  appendFlatSurface(low_pts, 0.5, 2.5, -0.6, 0.6, -1.50);
  appendVerticalWallY(low_pts, 2.05, -0.6, 0.6, -1.50, -0.30);
  grid.insertPoints(low_pts, 2.0);
  grid.classify(2.0);
  EXPECT_EQ(grid.queryCell(1.25, 0.05), CellClass::FREE);
  EXPECT_EQ(grid.queryCell(2.05, 0.05), CellClass::HARD_OBSTACLE);
}

TEST(ClearanceGridTest, FreezePreventsTransientPointsDuringStationaryLift)
{
  ClearanceParams params;
  ClearanceGrid grid(params);
  grid.updateRobotPose(0.0, 0.0, 0.0);

  std::vector<Point3D> initial_pts;
  appendFlatSurface(initial_pts, 0.5, 2.0, -0.6, 0.6, 0.0);
  grid.insertPoints(initial_pts, 1.0);
  grid.classify(1.0);
  EXPECT_EQ(grid.queryCell(1.25, 0.05), CellClass::FREE);

  // Freeze during stationary lift and inject smeared bogus wall points -> must be ignored
  grid.setFrozen(true);
  std::vector<Point3D> smeared_pts;
  appendVerticalWallY(smeared_pts, 1.25, -0.5, 0.5, 0.0, 1.0);
  grid.insertPoints(smeared_pts, 2.0);
  grid.classify(2.0);
  EXPECT_EQ(grid.queryCell(1.25, 0.05), CellClass::FREE);
}

TEST(ClearanceGridTest, IsolatedSingleNoiseCellOnRoadIsDespeckled)
{
  ClearanceParams params;
  params.xy_resolution = 0.15;
  params.despeckle_isolated_obstacles = true;
  params.min_obstacle_confirm_frames = 1;
  ClearanceGrid grid(params);
  grid.updateRobotPose(0.0, 0.0, 0.0);

  std::vector<Point3D> pts;
  appendFlatSurface(pts, 0.5, 2.5, -0.8, 0.8, 0.0, 0.05);
  // Single isolated stray spike inside one 0.15 m road cell (surrounded by FREE floor)
  pts.push_back({1.55, 0.05, 0.35});
  pts.push_back({1.55, 0.05, 0.45});
  grid.insertPoints(pts, 1.0);
  grid.classify(1.0);

  EXPECT_EQ(grid.queryCell(1.55, 0.05), CellClass::FREE);
}

TEST(ClearanceGridTest, TemporalConfirmationSuppressesOneFrameGlitch)
{
  ClearanceParams params;
  params.min_obstacle_confirm_frames = 2;
  ClearanceGrid grid(params);
  grid.updateRobotPose(0.0, 0.0, 0.0);

  std::vector<Point3D> pts;
  appendFlatSurface(pts, 0.5, 3.0, -0.8, 0.8, 0.0);
  appendVerticalWallY(pts, 2.25, -0.6, 0.6, 0.0, 1.0);
  grid.insertPoints(pts, 1.0);

  // Frame 1: not yet confirmed -> suppressed so slam_toolbox ignores 1-frame glitches
  grid.classify(1.0);
  EXPECT_NE(grid.queryCell(2.25, 0.05), CellClass::HARD_OBSTACLE);

  // Frame 2: real persistent wall confirmed -> HARD_OBSTACLE
  grid.classify(1.1);
  EXPECT_EQ(grid.queryCell(2.25, 0.05), CellClass::HARD_OBSTACLE);
}
