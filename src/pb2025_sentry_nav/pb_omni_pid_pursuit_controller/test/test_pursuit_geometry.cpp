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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pb_omni_pid_pursuit_controller/pursuit_geometry.hpp"

using namespace pb_omni_pid_pursuit_controller;  // NOLINT

namespace
{

std::vector<Point2D> makeStraightPath(
  double x0, double y0, double x1, double y1, double step = 0.05)
{
  std::vector<Point2D> pts;
  const double len = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(std::round(len / step)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    pts.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
  }
  return pts;
}

std::vector<Point2D> makeRightAnglePath(double leg1_len, double leg2_len, double step = 0.05)
{
  auto pts = makeStraightPath(0.0, 0.0, leg1_len, 0.0, step);
  auto leg2 = makeStraightPath(leg1_len, 0.0, leg1_len, leg2_len, step);
  pts.insert(pts.end(), leg2.begin() + 1, leg2.end());
  return pts;
}

}  // namespace

TEST(PursuitGeometryTest, StraightPathKeepsFullLookahead)
{
  const auto path = makeStraightPath(0.0, 0.0, 2.0, 0.0, 0.05);
  LookaheadParams params;
  params.lookahead_dist = 0.8;
  params.max_chord_deviation = 0.06;
  params.use_interpolation = true;

  const auto res = selectClampedLookaheadPoint(path, params);
  EXPECT_NEAR(res.point.x, 0.8, 1e-3);
  EXPECT_NEAR(res.point.y, 0.0, 1e-3);
  EXPECT_NEAR(res.arc_length, 0.8, 1e-3);
  EXPECT_FALSE(res.clamped_by_chord);
  EXPECT_FALSE(res.clamped_by_los);
}

TEST(PursuitGeometryTest, StraightPathWithLateralOffsetDoesNotCollapse)
{
  // Robot is at (0,0), straight path runs parallel at y = +0.15 m.
  const auto path = makeStraightPath(0.0, 0.15, 2.0, 0.15, 0.05);
  LookaheadParams params;
  params.lookahead_dist = 0.8;
  params.max_chord_deviation = 0.06;
  params.use_interpolation = true;

  const auto res = selectClampedLookaheadPoint(path, params);
  EXPECT_GT(res.point.x, 0.75);
  EXPECT_NEAR(res.point.y, 0.15, 1e-3);
  EXPECT_FALSE(res.clamped_by_chord);
}

TEST(PursuitGeometryTest, RightAngleCornerClampsChordDeviationInterpolated)
{
  // 90-degree corner at (0.4, 0.0), lookahead = 1.0 m would badly cut the corner without clamping.
  const auto path = makeRightAnglePath(0.4, 1.2, 0.05);
  LookaheadParams params;
  params.lookahead_dist = 1.0;
  params.max_chord_deviation = 0.06;
  params.use_interpolation = true;

  const auto res = selectClampedLookaheadPoint(path, params);
  EXPECT_TRUE(res.clamped_by_chord);
  EXPECT_LE(res.max_chord_deviation, params.max_chord_deviation + 1e-3);
  EXPECT_NEAR(res.point.x, 0.4, 1e-3);
  // Corner apex is at (0.4, 0), chord from (0,0) to (0.4, y) has apex deviation <= 0.06 -> y <= ~0.061.
  EXPECT_LT(res.point.y, 0.07);
}

TEST(PursuitGeometryTest, RightAngleCornerClampsChordDeviationDiscreteMode)
{
  // Matches nav2_params.yaml where use_interpolation is false.
  const auto path = makeRightAnglePath(0.4, 1.2, 0.05);
  LookaheadParams params;
  params.lookahead_dist = 1.0;
  params.max_chord_deviation = 0.06;
  params.use_interpolation = false;

  const auto res = selectClampedLookaheadPoint(path, params);
  EXPECT_TRUE(res.clamped_by_chord);
  EXPECT_LE(res.max_chord_deviation, params.max_chord_deviation + 1e-6);
  EXPECT_NEAR(res.point.x, 0.4, 1e-6);
  EXPECT_LE(res.point.y, 0.06);
}

TEST(PursuitGeometryTest, CornerReleasesLookaheadOnceRobotReachesApex)
{
  // Robot is 0.04 m from the 90-degree apex: corner deviation <= 0.04 < 0.06, so carrot advances down leg 2.
  const auto path = makeRightAnglePath(0.04, 1.2, 0.05);
  LookaheadParams params;
  params.lookahead_dist = 0.8;
  params.max_chord_deviation = 0.06;
  params.use_interpolation = true;

  const auto res = selectClampedLookaheadPoint(path, params);
  EXPECT_FALSE(res.clamped_by_chord);
  EXPECT_NEAR(res.point.x, 0.04, 1e-3);
  EXPECT_GT(res.point.y, 0.75);
}

TEST(PursuitGeometryTest, CostmapLosClampsCarrotBeforeInnerCornerWall)
{
  // Gentle bend with a sharp inner-corner obstacle box at x in [0.25, 0.55], y in [0.05, 0.50].
  const auto path = makeRightAnglePath(0.6, 1.0, 0.05);
  auto cost_query = [](const Point2D & p) -> std::optional<uint8_t> {
    if (p.x >= 0.25 && p.x <= 0.55 && p.y >= 0.05 && p.y <= 0.50) {
      return uint8_t{253};
    }
    return uint8_t{0};
  };

  LookaheadParams params;
  params.lookahead_dist = 1.0;
  params.max_chord_deviation = 0.0;  // Disable geometric clamp to isolate LOS clamping.
  params.use_interpolation = true;
  params.use_costmap_los_clamp = true;
  params.los_cost_threshold = 180;
  params.inscribed_cost = 253;
  params.los_step_size = 0.02;

  const auto res = selectClampedLookaheadPoint(path, params, cost_query);
  EXPECT_TRUE(res.clamped_by_los);
  EXPECT_FALSE(isSegmentInCollision({0.0, 0.0}, res.point, cost_query, 253, 0.02, true));
}

TEST(PursuitGeometryTest, NarrowPassableCorridorWithHighInflationCostIsNotBlocked)
{
  // Robot in open space (cost=20) entering a narrow navigable passage whose centerline has cost=210 (< 253).
  const auto path = makeStraightPath(0.0, 0.0, 1.5, 0.0, 0.05);
  auto cost_query = [](const Point2D & p) -> std::optional<uint8_t> {
    return (p.x >= 0.30) ? uint8_t{210} : uint8_t{20};
  };

  LookaheadParams params;  // Uses default los_cost_threshold = 253
  params.lookahead_dist = 0.8;
  params.use_interpolation = true;
  params.use_costmap_los_clamp = true;

  const auto res = selectClampedLookaheadPoint(path, params, cost_query);
  EXPECT_FALSE(res.clamped_by_los);
  EXPECT_NEAR(res.point.x, 0.8, 1e-3);
}

TEST(PursuitGeometryTest, ActualMotionSegmentDetectsInscribedObstacle253)
{
  auto cost_query = [](const Point2D & p) -> std::optional<uint8_t> {
    if (p.x >= 0.15 && p.x <= 0.30 && p.y >= 0.15 && p.y <= 0.30) {
      return uint8_t{253};  // INSCRIBED_INFLATED_OBSTACLE (< LETHAL 254)
    }
    return uint8_t{0};
  };

  // Diagonal shortcut from (0,0) to (0.4, 0.4) passes directly through the 253 inner corner.
  EXPECT_TRUE(isSegmentInCollision({0.0, 0.0}, {0.4, 0.4}, cost_query, 253, 0.02, true));
  // Staying along x-axis (0,0) -> (0.4, 0.0) is clear.
  EXPECT_FALSE(isSegmentInCollision({0.0, 0.0}, {0.4, 0.0}, cost_query, 253, 0.02, true));
}

TEST(PursuitGeometryTest, RobotOn253BorderCanEscapeTowardLowerCost)
{
  auto cost_query = [](const Point2D & p) -> std::optional<uint8_t> {
    if (p.x < 0.02) {
      return uint8_t{253};
    }
    return uint8_t{50};
  };
  EXPECT_FALSE(isSegmentInCollision({0.0, 0.0}, {0.3, 0.0}, cost_query, 253, 0.05, true));
}

TEST(PursuitGeometryTest, CrossTrackErrorPullsPerpendicularToPath)
{
  // Robot at (0,0), straight path at y = +0.15 m going along +x.
  const auto path = makeStraightPath(-0.2, 0.15, 1.5, 0.15, 0.05);
  const auto cte = computeCrossTrackVector(path, 1.0);
  EXPECT_NEAR(cte.perpendicular_error.x, 0.0, 1e-6);
  EXPECT_NEAR(cte.perpendicular_error.y, 0.15, 1e-6);
  EXPECT_NEAR(cte.distance, 0.15, 1e-6);
}

TEST(PursuitGeometryTest, TrueArcLengthCurvatureDetectsRightAngleApex)
{
  const auto path = makeRightAnglePath(0.6, 1.0, 0.05);
  const auto cumulative = computeCumulativeArcLengths(path);

  // Straight section far from corner -> near-zero curvature.
  const auto straight_curv = computeArcLengthCurvature(path, cumulative, 0.15, 0.15, 0.15);
  EXPECT_LT(straight_curv.curvature, 1e-3);

  // At the corner apex (arc length = 0.6 m) -> high curvature (well above curvature_max = 0.7).
  const auto corner_curv = computeArcLengthCurvature(path, cumulative, 0.60, 0.30, 0.70);
  EXPECT_GT(corner_curv.curvature, 2.0);
}

TEST(PursuitGeometryTest, LookaheadFilterRejectsSingleFrameOdometrySpike)
{
  double prev = 0.5;
  // Single-frame odom finite-difference spike jumps raw lookahead from 0.5 m to 1.0 m.
  const double after_spike = smoothLookaheadDistance(1.0, prev, 0.2, 0.5, 0.05, 0.5, 1.0);
  EXPECT_LE(after_spike - prev, 0.025 + 1e-6);

  // Next frame returns to 0.5 m -> smoothly returns without oscillation.
  const double after_recover = smoothLookaheadDistance(0.5, after_spike, 0.2, 0.5, 0.05, 0.5, 1.0);
  EXPECT_LT(after_recover, after_spike);
  EXPECT_GE(after_recover, 0.5);
}

TEST(PursuitGeometryTest, LookaheadFilterTracksSustainedAcceleration)
{
  double dist = 0.5;
  for (int i = 0; i < 60; ++i) {
    dist = smoothLookaheadDistance(0.9, dist, 0.2, 0.5, 0.05, 0.5, 1.0);
  }
  EXPECT_NEAR(dist, 0.9, 1e-2);
}
