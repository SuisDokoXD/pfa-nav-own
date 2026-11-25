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

#include "tunnel_align/tunnel_geometry.hpp"

namespace tunnel_align
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
double deg2rad(double d) { return d * kPi / 180.0; }
double rad2deg(double r) { return r * 180.0 / kPi; }
}  // namespace

TEST(TunnelGeometryTest, SquareChassisSymmetryNeverTurnsMoreThan45Deg)
{
  const double tunnel_north = deg2rad(90.0);
  // Chassis at +30 deg: nearest 90-deg equivalent of 90 deg is 0 deg (-30 deg turn).
  const double chosen = selectNearestSymmetricHeading(tunnel_north, deg2rad(30.0), 4);
  EXPECT_NEAR(rad2deg(chosen), 0.0, 1e-6);

  // Sweep all initial chassis headings and verify error <= 45 deg.
  for (int deg = -180; deg <= 180; deg += 5) {
    const double h = selectNearestSymmetricHeading(deg2rad(17.0), deg2rad(deg), 4);
    const double err_deg = std::abs(rad2deg(normalizeAngleRad(h - deg2rad(deg))));
    EXPECT_LE(err_deg, 45.001);
  }
}

TEST(TunnelGeometryTest, OptionARelativeGimbalMatchesWorkedExample)
{
  // Tunnel = +90 deg, gimbal = +30 deg, chassis currently aligned with gimbal (+30 deg).
  // Exact heading (fold=1): chassis target = 90 deg -> rel gimbal = +60 deg.
  const double cmd_exact = computeTunnelAlignAngleDeg(
    AlignAngleMode::RELATIVE_GIMBAL, deg2rad(90.0), deg2rad(30.0), deg2rad(30.0), 0.0, 1);
  EXPECT_NEAR(cmd_exact, 60.0, 1e-6);

  // Square chassis (fold=4): nearest symmetric target to +30 deg is 0 deg -> rel gimbal = -30 deg.
  const double cmd_sq = computeTunnelAlignAngleDeg(
    AlignAngleMode::RELATIVE_GIMBAL, deg2rad(90.0), deg2rad(30.0), deg2rad(30.0), 0.0, 4);
  EXPECT_NEAR(cmd_sq, -30.0, 1e-6);

  // Gimbal rotates independently to +50 deg while chassis stays near 0 deg:
  // target stays 0 deg -> rel gimbal updates to -50 deg so chassis stays locked to tunnel!
  const double cmd_tracking = computeTunnelAlignAngleDeg(
    AlignAngleMode::RELATIVE_GIMBAL, deg2rad(90.0), deg2rad(50.0), deg2rad(0.0), 0.0, 4);
  EXPECT_NEAR(cmd_tracking, -50.0, 1e-6);
}

TEST(TunnelGeometryTest, OptionBImuWorldSubtractsMapImuOffset)
{
  // Tunnel = +90 deg, gimbal map = +30 deg, gimbal IMU = -15 deg -> offset = +45 deg.
  // Exact heading target in IMU frame = 90 - 45 = +45 deg.
  const double cmd = computeTunnelAlignAngleDeg(
    AlignAngleMode::IMU_WORLD, deg2rad(90.0), deg2rad(30.0), deg2rad(80.0), deg2rad(45.0), 1);
  EXPECT_NEAR(cmd, 45.0, 1e-6);

  // CW-positive firmware sign_multiplier = -1.0 flips sign cleanly.
  const double cmd_cw = computeTunnelAlignAngleDeg(
    AlignAngleMode::IMU_WORLD, deg2rad(90.0), deg2rad(30.0), deg2rad(80.0), deg2rad(45.0), 1, -1.0);
  EXPECT_NEAR(cmd_cw, -45.0, 1e-6);
}

TEST(TunnelGeometryTest, EstimateTunnelFromAngledParallelWallsAndOffset)
{
  // True corridor runs at +25 deg, width = 1.00 m (half-width = 0.50 m).
  // Robot is at (0, 0), shifted +0.12 m to the LEFT of corridor centerline
  // (left wall at 0.38 m, right wall at 0.62 m).
  const double true_heading = deg2rad(25.0);
  const double c = std::cos(true_heading);
  const double s = std::sin(true_heading);
  const double nx = -s;
  const double ny = c;

  std::vector<Point2D> pts;
  for (double t = -0.5; t <= 2.4; t += 0.10) {
    // Left wall (at +0.38 m) + outer wall layer at +0.54 m (tests inner-face selection)
    pts.push_back({t * c + 0.38 * nx, t * s + 0.38 * ny});
    pts.push_back({t * c + 0.54 * nx, t * s + 0.54 * ny});
    // Right wall (at -0.62 m)
    pts.push_back({t * c - 0.62 * nx, t * s - 0.62 * ny});
  }

  // Provide an initial path heading with 10 deg error (+15 deg instead of +25 deg).
  const TunnelEstimate est = estimateTunnelFromWalls(pts, {0.0, 0.0}, deg2rad(15.0));
  ASSERT_TRUE(est.valid);
  EXPECT_FALSE(est.front_blocked);
  EXPECT_NEAR(rad2deg(est.axis_heading_rad), 25.0, 0.5);
  EXPECT_NEAR(est.corridor_width_m, 1.00, 0.03);
  EXPECT_NEAR(est.left_wall_dist_m, 0.38, 0.02);
  EXPECT_NEAR(est.right_wall_dist_m, 0.62, 0.02);
  EXPECT_NEAR(est.lateral_offset_m, 0.12, 0.02);
}

TEST(TunnelGeometryTest, FrontObstacleDetectionTriggersSafetyStop)
{
  std::vector<Point2D> pts;
  for (double t = -0.4; t <= 2.0; t += 0.10) {
    pts.push_back({t, 0.50});
    pts.push_back({t, -0.50});
  }
  // Place an obstacle straight ahead inside the corridor at x = 0.30 m
  pts.push_back({0.30, -0.05});
  pts.push_back({0.30, 0.00});
  pts.push_back({0.30, 0.05});

  const TunnelEstimate est = estimateTunnelFromWalls(pts, {0.0, 0.0}, 0.0);
  EXPECT_TRUE(est.valid);
  EXPECT_TRUE(est.front_blocked);
}

TEST(TunnelGeometryTest, CenteredTraverseVelocityPushesBackToCenterline)
{
  // Tunnel along +X (0 rad), robot shifted +0.10 m LEFT -> must command negative vy (RIGHT).
  const Vec2D v = computeCenteredTraverseVelocity(0.0, 0.10, 0.45, 1.5, 0.25);
  EXPECT_NEAR(v.vx, 0.45, 1e-6);
  EXPECT_NEAR(v.vy, -0.15, 1e-6);

  // Large offset clamps lateral velocity cleanly.
  const Vec2D v_clamped = computeCenteredTraverseVelocity(deg2rad(90.0), -0.50, 0.40, 2.0, 0.20);
  // Tunnel along +Y (90 deg), robot shifted RIGHT (-0.50) -> v_lat = +0.20 (LEFT = -X in map).
  EXPECT_NEAR(v_clamped.vx, -0.20, 1e-6);
  EXPECT_NEAR(v_clamped.vy, 0.40, 1e-6);
}

TEST(TunnelGeometryTest, CircularOffsetFilterHandlesPiWrapSmoothly)
{
  CircularOffsetFilter filter(0.3);
  filter.update(deg2rad(178.0));
  const double v = filter.update(deg2rad(-178.0));
  // Average across ±180 deg must stay near ±180 deg, NEVER jump near 0 deg!
  EXPECT_GT(std::abs(rad2deg(v)), 175.0);
}

TEST(TunnelGeometryTest, FullAngleSearchFindsObliqueCorridor60DegOffSeed)
{
  // Corridor along +90 deg (+Y), robot at (-0.12, 0.0) shifted +0.12 m left of center,
  // but seed heading (e.g. gimbal without active plan) is +30 deg (60 deg off!).
  std::vector<Point2D> pts;
  for (double y = -0.5; y <= 2.2; y += 0.12) {
    pts.push_back({-0.50, y});
    pts.push_back({+0.50, y});
  }
  const TunnelEstimate est =
    estimateTunnelFromWalls(pts, {-0.12, 0.0}, deg2rad(30.0), TunnelEstimatorParams{}, true);
  ASSERT_TRUE(est.valid);
  EXPECT_NEAR(rad2deg(est.axis_heading_rad), 90.0, 1.0);
  EXPECT_NEAR(est.corridor_width_m, 1.00, 0.04);
  EXPECT_NEAR(est.lateral_offset_m, 0.12, 0.03);
}

TEST(TunnelGeometryTest, Full360SearchFindsTunnelDirectlyBehindGimbalAt180Deg)
{
  // Room B -> Room A return scenario: robot sits at x=+2.20, y=+0.05 with gimbal facing East (0 deg),
  // while the 0.533m dumbbell tunnel lies directly BEHIND it to the West (x in [-1.7, +1.7], 180 deg)!
  std::vector<Point2D> pts;
  for (double x = -1.70; x <= 1.72; x += 0.10) {
    pts.push_back({x, +0.2665});
    pts.push_back({x, -0.2665});
  }
  // Room B west wall around doorway (x = +1.70, |y| >= 0.2665)
  for (int i = 3; i <= 15; ++i) {
    pts.push_back({1.70, +i * 0.12});
    pts.push_back({1.70, -i * 0.12});
  }
  TunnelEstimatorParams p;
  p.min_half_width = 0.14;
  p.roi_max_x = 3.2;
  p.slice_length = 0.16;
  p.wall_inlier_band = 0.22;

  const TunnelEstimate est = estimateTunnelFromWalls(pts, {2.20, 0.05}, 0.0, p, true);
  ASSERT_TRUE(est.valid);
  // Must choose 180 deg (West, INTO the tunnel), NOT 0 deg (East, deeper into Room B)!
  EXPECT_GT(std::abs(rad2deg(est.axis_heading_rad)), 178.0);
  EXPECT_NEAR(est.corridor_width_m, 0.533, 0.03);
  // Facing West (180 deg), +Y is to the robot's RIGHT -> y=+0.05 means robot is shifted RIGHT (-0.05 m)!
  EXPECT_NEAR(est.lateral_offset_m, -0.05, 0.02);
}

TEST(TunnelGeometryTest, LockedHeadingAtTunnelExitIgnoresPerpendicularRoomWall)
{
  // Robot is exiting dumbbell tunnel at x=1.58, y=-0.01 heading East (0 deg).
  // Tunnel walls are behind it; perpendicular Room B west wall stands at x=1.70, |y|>=0.27.
  std::vector<Point2D> pts;
  for (int i = 3; i <= 15; ++i) {
    pts.push_back({1.70, +i * 0.10});
    pts.push_back({1.70, -i * 0.10});
  }
  TunnelEstimatorParams p;
  p.min_half_width = 0.04;
  p.front_obstacle_half_width = 0.12;

  const TunnelEstimate est = evaluateTunnelAtLockedHeading(pts, {1.58, -0.01}, 0.0, p);
  EXPECT_FALSE(est.valid);
  EXPECT_FALSE(est.front_blocked);
  // Doorway wall at x=1.70 (lx=+0.12, |ly| in [0.30, 0.40]) is still inside the spin clearance envelope!
  EXPECT_TRUE(est.spin_zone_blocked);
}

TEST(TunnelGeometryTest, LockedHeadingStaysValidAndBlocksSpinNearTunnelExit)
{
  // Regression for Room B -> Room A return jam at x=-1.25 (0.35m before West exit at x=-1.60):
  // Point-LIO's 0.50m blind sphere hides live forward wall points (<0.58m ahead), leaving memorized
  // side/rear tunnel walls at x in [-1.60, -0.60]. Heading is locked to 180 deg (West).
  std::vector<Point2D> pts;
  for (double x = -1.60; x <= -0.60; x += 0.04) {
    pts.push_back({x, +0.2665});
    pts.push_back({x, -0.2665});
  }
  TunnelEstimatorParams p;
  p.roi_min_x = -0.6;
  p.roi_max_x = 3.2;
  p.min_half_width = 0.04;
  p.slice_length = 0.16;
  p.wall_inlier_band = 0.22;
  p.spin_clear_half_length = 0.45;
  p.spin_clear_half_width = 0.42;

  // 1) Still 0.35 m inside tunnel (x = -1.25): MUST stay valid AND block spin!
  const TunnelEstimate inside = evaluateTunnelAtLockedHeading(pts, {-1.25, 0.0}, kPi, p);
  EXPECT_TRUE(inside.valid);
  EXPECT_TRUE(inside.spin_zone_blocked);
  EXPECT_NEAR(inside.corridor_width_m, 0.533, 0.03);

  // 2) Right at West doorway exit (x = -1.60): rear tunnel walls still fill [-0.6, 0.0] -> valid & spin blocked!
  const TunnelEstimate at_exit = evaluateTunnelAtLockedHeading(pts, {-1.60, 0.0}, kPi, p);
  EXPECT_TRUE(at_exit.valid);
  EXPECT_TRUE(at_exit.spin_zone_blocked);

  // 3) 0.55 m past West exit (x = -2.15): completely clear of tunnel -> invalid & spin safe!
  const TunnelEstimate clear = evaluateTunnelAtLockedHeading(pts, {-2.15, 0.0}, kPi, p);
  EXPECT_FALSE(clear.valid);
  EXPECT_FALSE(clear.spin_zone_blocked);
}

TEST(TunnelGeometryTest, LiveFrontPathBlockedIgnoresClearCorridorAndDetectsRealObstacle)
{
  TunnelEstimatorParams p;
  p.front_obstacle_min_x = 0.15;
  p.front_obstacle_check_dist = 0.42;
  p.front_obstacle_half_width = 0.12;
  p.front_obstacle_min_points = 3;

  // Clean 0.533 m corridor side walls at |y| = 0.2665 m: must NOT block front path!
  std::vector<Point2D> clear_pts;
  for (double x = 0.0; x <= 1.0; x += 0.05) {
    clear_pts.push_back({x, +0.2665});
    clear_pts.push_back({x, -0.2665});
  }
  EXPECT_FALSE(isFrontPathBlocked(clear_pts, {0.0, 0.0}, 0.0, p));

  // Real obstacle placed dead ahead at x = 0.30 m, |y| <= 0.05 m: MUST block!
  auto blocked_pts = clear_pts;
  blocked_pts.push_back({0.30, -0.04});
  blocked_pts.push_back({0.30, 0.00});
  blocked_pts.push_back({0.30, +0.04});
  EXPECT_TRUE(isFrontPathBlocked(blocked_pts, {0.0, 0.0}, 0.0, p));
}

TEST(TunnelGeometryTest, Full360SearchRejectsWideRoomWallAndFindsNarrowTunnel)
{
  // Regression: standing 0.50m in front of a long room wall (x=+0.50) with distant wall/clutter
  // 1.30m behind (x=-1.30) forms a fake 1.80m wide corridor along ±90 deg with MORE slices than
  // the real 0.533m tunnel along 0 deg! max_corridor_width must reject the 1.80m room gap and
  // select the real 0.533m tunnel along 0 deg.
  std::vector<Point2D> pts;
  // Long room wall on +X (x = +0.50, y in [-2.8, -0.27] U [0.27, 2.8])
  for (double y = 0.30; y <= 2.80; y += 0.10) {
    pts.push_back({0.50, +y});
    pts.push_back({0.50, -y});
  }
  // Distant wall on -X (x = -1.30, y in [-2.8, 2.8]) -> 1.80m total width along ±90 deg
  for (double y = -2.80; y <= 2.80; y += 0.10) {
    pts.push_back({-1.30, y});
  }
  // Real 0.533m tunnel extending +X from x = +0.50 to x = +2.20
  for (double x = 0.55; x <= 2.20; x += 0.10) {
    pts.push_back({x, +0.2665});
    pts.push_back({x, -0.2665});
  }

  TunnelEstimatorParams p;
  p.roi_max_x = 3.2;
  p.min_half_width = 0.04;
  p.max_half_width = 1.00;
  p.min_corridor_width = 0.40;
  p.max_corridor_width = 1.25;
  p.slice_length = 0.16;
  p.wall_inlier_band = 0.22;

  const TunnelEstimate est = estimateTunnelFromWalls(pts, {0.0, 0.0}, deg2rad(-90.0), p, true);
  ASSERT_TRUE(est.valid);
  EXPECT_NEAR(rad2deg(est.axis_heading_rad), 0.0, 2.0);
  EXPECT_NEAR(est.corridor_width_m, 0.533, 0.04);
}

TEST(TunnelGeometryTest, Full360SearchRejectsLShapedRoomCorner)
{
  // Exact regression for the Room B (6.70, 0.48) false positive:
  // b_west_north runs along x = 6.20, y in [0.27, 3.00] (0.50m west of robot);
  // b_north runs perpendicular along y = 3.00, x in [6.20, 10.40];
  // tunnel north wall is completely occluded by b_west_north, only tunnel south wall visible!
  std::vector<Point2D> pts;
  for (double y = 0.27; y <= 3.00; y += 0.05) {
    pts.push_back({6.20, y});
  }
  for (double y = -3.00; y <= -0.27; y += 0.05) {
    pts.push_back({6.20, y});
  }
  for (double x = 6.20; x <= 10.40; x += 0.05) {
    pts.push_back({x, 3.00});
  }
  // Only south inner tunnel wall visible through opening
  for (double x = 4.60; x <= 6.16; x += 0.05) {
    pts.push_back({x, -0.2665});
  }

  TunnelEstimatorParams p;
  p.roi_min_x = -0.6;
  p.roi_max_x = 3.2;
  p.min_half_width = 0.04;
  p.max_half_width = 1.00;
  p.min_corridor_width = 0.40;
  p.max_corridor_width = 1.25;
  p.slice_length = 0.16;
  p.wall_inlier_band = 0.22;

  const TunnelEstimate est = estimateTunnelFromWalls(pts, {6.70, 0.48}, 0.0, p, true);
  EXPECT_FALSE(est.valid);
}

TEST(TunnelGeometryTest, DiscoveryRejectsCappedDeadEndCorridor)
{
  // Parallel side walls at y = ±0.2665, but solid back wall seals the corridor at x = 1.20!
  std::vector<Point2D> pts;
  for (double x = 0.20; x <= 1.20; x += 0.05) {
    pts.push_back({x, +0.2665});
    pts.push_back({x, -0.2665});
  }
  for (double y = -0.26; y <= 0.26; y += 0.04) {
    pts.push_back({1.20, y});
  }

  TunnelEstimatorParams p;
  p.min_half_width = 0.04;
  p.max_half_width = 1.00;
  p.min_corridor_width = 0.40;
  p.max_corridor_width = 1.25;
  p.slice_length = 0.16;

  const TunnelEstimate est = estimateTunnelFromWalls(pts, {0.0, 0.0}, 0.0, p, true);
  EXPECT_FALSE(est.valid);
}

TEST(TunnelGeometryTest, DetectDoorwayOpeningFindsOccludedTunnelFromOffCenterAndFar)
{
  // Room B doorway at x = 6.20, opening y in [-0.2665, +0.2665], perpendicular wall b_north at y = 3.00.
  // Only south inner tunnel wall (x in [4.60, 6.16], y = -0.2665) is visible through the doorway.
  std::vector<Point2D> pts;
  for (double y = 0.2665; y <= 3.00; y += 0.05) {
    pts.push_back({6.20, y});
  }
  for (double y = -3.00; y <= -0.2665; y += 0.05) {
    pts.push_back({6.20, y});
  }
  for (double x = 6.20; x <= 10.40; x += 0.05) {
    pts.push_back({x, 3.00});
  }
  for (double x = 4.60; x <= 6.16; x += 0.05) {
    pts.push_back({x, -0.2665});
  }

  TunnelEstimatorParams p;
  p.min_corridor_width = 0.40;
  p.max_corridor_width = 1.25;

  // Case 1: exact 13:11 off-center position (6.70, 0.48)
  const DoorwayEstimate dw1 = detectDoorwayOpening(pts, {6.70, 0.48}, p);
  ASSERT_TRUE(dw1.valid);
  EXPECT_NEAR(dw1.center_odom.x, 6.20, 0.06);
  EXPECT_NEAR(dw1.center_odom.y, 0.00, 0.06);
  EXPECT_GT(std::abs(rad2deg(dw1.axis_heading_rad)), 176.0);
  EXPECT_NEAR(dw1.opening_width_m, 0.533, 0.08);

  // Case 2: far AND off-center (8.50, 1.00) -> 2.3 m from wall, 1.0 m north of centerline
  const DoorwayEstimate dw2 = detectDoorwayOpening(pts, {8.50, 1.00}, p);
  ASSERT_TRUE(dw2.valid);
  EXPECT_NEAR(dw2.center_odom.x, 6.20, 0.08);
  EXPECT_NEAR(dw2.center_odom.y, 0.00, 0.08);
  EXPECT_GT(std::abs(rad2deg(dw2.axis_heading_rad)), 176.0);
}

TEST(TunnelGeometryTest, DetectDoorwayOpeningRejectsSolidWallCornerAndShadowGap)
{
  // L-corner + a 0.60 m shadow hole in a solid wall with ZERO tunnel depth behind it!
  std::vector<Point2D> pts;
  for (double y = -2.50; y <= -0.30; y += 0.05) {
    pts.push_back({6.20, y});
  }
  for (double y = +0.30; y <= 2.50; y += 0.05) {
    pts.push_back({6.20, y});
  }
  for (double x = 6.20; x <= 9.50; x += 0.05) {
    pts.push_back({x, 2.50});
  }
  // Notice: NO points behind x = 6.20 -> must NOT be mistaken for a real tunnel doorway!
  TunnelEstimatorParams p;
  p.min_corridor_width = 0.40;
  p.max_corridor_width = 1.25;

  const DoorwayEstimate dw = detectDoorwayOpening(pts, {7.20, 0.50}, p);
  EXPECT_FALSE(dw.valid);
}

}  // namespace tunnel_align
