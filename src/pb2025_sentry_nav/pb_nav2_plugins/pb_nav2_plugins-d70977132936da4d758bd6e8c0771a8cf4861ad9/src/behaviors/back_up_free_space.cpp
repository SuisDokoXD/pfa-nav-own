// Copyright 2024 Polaris Xia
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

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace pb_nav2_behaviors
{

namespace
{

constexpr uint8_t kInscribedCost = 253;
constexpr uint8_t kLethalCost = 254;

float normalizeAngle(float a)
{
  while (a > static_cast<float>(M_PI)) {
    a -= 2.0f * static_cast<float>(M_PI);
  }
  while (a < -static_cast<float>(M_PI)) {
    a += 2.0f * static_cast<float>(M_PI);
  }
  return a;
}

}  // namespace

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
    return nav2_behaviors::Status::FAILED;
  }

  // get costmap
  auto costmap = result.get()->map;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  // get current pose
  geometry_msgs::msg::Pose2D pose;
  pose.x = initial_pose_.pose.position.x;
  pose.y = initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  // Find the best direction to back up (escape inscribed/lethal space toward open corridor)
  const DirectionResult dir =
    findBestDirectionDetailed(costmap, pose, -M_PI, M_PI, max_radius_, M_PI / 32.0);
  if (!dir.found) {
    RCLCPP_WARN(
      logger_,
      "No safe free-space backup direction found around (%.2f, %.2f); aborting backup.",
      pose.x, pose.y);
    return nav2_behaviors::Status::FAILED;
  }

  best_world_angle_ = static_cast<double>(dir.angle);
  backup_speed_ = std::fabs(command->speed);
  escape_distance_ = static_cast<double>(dir.escape_distance);
  const double local_angle = best_world_angle_ - pose.theta;

  // Calculate move command in robot_base_frame
  twist_x_ = std::cos(local_angle) * backup_speed_;
  twist_y_ = std::sin(local_angle) * backup_speed_;

  // Clamp travel distance so narrow-corridor escapes stop inside free space instead of hitting
  // the opposite wall.
  const double requested_dist = std::fabs(command->target.x);
  const double safe_dist = std::max(0.20, static_cast<double>(dir.safe_distance) - 0.15);
  command_x_ = std::min(requested_dist, safe_dist);
  command_time_allowance_ = command->time_allowance;

  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }
  RCLCPP_WARN(
    logger_, "backing up %.2f meters (safe %.2f m) towards free space at world angle %.2f rad",
    command_x_, dir.safe_distance, best_world_angle_);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  const double local_angle = best_world_angle_ - pose.theta;
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = std::cos(local_angle) * backup_speed_;
  cmd_vel->linear.y = std::sin(local_angle) * backup_speed_;

  if (!isOmniPathCollisionFree(distance, pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting BackUpFreeSpace");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

bool BackUpFreeSpace::isOmniPathCollisionFree(
  double distance_traveled, const geometry_msgs::msg::Pose2D & pose)
{
  const double remaining_dist = std::fabs(command_x_) - distance_traveled;
  if (remaining_dist <= 0.0 || backup_speed_ <= 1e-4) {
    return true;
  }

  const double remaining_escape = std::max(0.0, escape_distance_ - distance_traveled) + 0.05;
  const double dt = 1.0 / std::max(this->cycle_frequency_, 1.0);
  const int max_steps = std::max(1, static_cast<int>(std::ceil(simulate_ahead_time_ / dt)));
  const double dir_x = std::cos(best_world_angle_);
  const double dir_y = std::sin(best_world_angle_);

  bool fetch_data = true;
  for (int step = 1; step <= max_steps; ++step) {
    const double sim_dist = backup_speed_ * static_cast<double>(step) * dt;
    if (sim_dist > remaining_dist) {
      break;
    }
    if (sim_dist <= remaining_escape) {
      continue;
    }
    geometry_msgs::msg::Pose2D sim_pose = pose;
    sim_pose.x = pose.x + sim_dist * dir_x;
    sim_pose.y = pose.y + sim_dist * dir_y;
    if (!this->collision_checker_->isCollisionFree(sim_pose, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

BackUpFreeSpace::DirectionResult BackUpFreeSpace::findBestDirectionDetailed(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  DirectionResult res;

  const float resolution = std::max(costmap.metadata.resolution, 0.01f);
  const float origin_x = costmap.metadata.origin.position.x;
  const float origin_y = costmap.metadata.origin.position.y;
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);
  if (size_x <= 0 || size_y <= 0 || costmap.data.size() < static_cast<size_t>(size_x * size_y)) {
    return res;
  }

  auto queryCost = [&](float x, float y, uint8_t & out_cost) -> bool {
    const int i = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int j = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (i < 0 || i >= size_x || j < 0 || j >= size_y) {
      return false;
    }
    out_cost = costmap.data[static_cast<size_t>(i + j * size_x)];
    return true;
  };

  uint8_t start_cost = kInscribedCost;
  queryCost(pose.x, pose.y, start_cost);

  const float span = std::max(end_angle - start_angle, angle_increment);
  const int num_rays = std::max(8, static_cast<int>(std::round(span / angle_increment)));
  const float step = span / static_cast<float>(num_rays);
  const float escape_limit = std::min(radius, 0.8f);

  struct RayEval
  {
    float angle{0.0f};
    bool escaped{false};
    float escape_dist{0.0f};
    float clear_dist{0.0f};
    float mean_cost{255.0f};
    float score{-1e6f};
  };

  std::vector<RayEval> rays(static_cast<size_t>(num_rays));

  for (int k = 0; k < num_rays; ++k) {
    RayEval & ray = rays[static_cast<size_t>(k)];
    ray.angle = start_angle + static_cast<float>(k) * step;
    const float ca = std::cos(ray.angle);
    const float sa = std::sin(ray.angle);

    bool escaped = (start_cost < kInscribedCost);
    float escape_dist = 0.0f;
    float clear_dist = 0.0f;
    float cost_sum = 0.0f;
    int clear_steps = 0;

    for (float r = resolution; r <= radius + 1e-4f; r += resolution) {
      uint8_t c = 255;
      if (!queryCost(pose.x + r * ca, pose.y + r * sa, c)) {
        break;
      }

      if (!escaped) {
        // Never tunnel through a real lethal wall to reach free space on the other side.
        if (c == kLethalCost && (start_cost < kLethalCost || r > resolution * 1.5f)) {
          break;
        }
        if (c < kInscribedCost) {
          escaped = true;
          escape_dist = r;
          clear_dist = resolution;
          cost_sum += static_cast<float>(c);
          clear_steps += 1;
        } else if (r > escape_limit) {
          break;
        }
      } else {
        if (c >= kInscribedCost) {
          break;
        }
        clear_dist = r - escape_dist;
        cost_sum += static_cast<float>(c);
        clear_steps += 1;
      }
    }

    ray.escaped = escaped && (clear_steps > 0);
    ray.escape_dist = escape_dist;
    ray.clear_dist = clear_dist;
    ray.mean_cost = (clear_steps > 0) ? (cost_sum / static_cast<float>(clear_steps)) : 255.0f;
    if (ray.escaped) {
      ray.score = ray.clear_dist - 1.5f * ray.escape_dist - 0.002f * ray.mean_cost;
    }
  }

  // Tier 1: Circularly contiguous wide-open sector (handles ±pi wrap-around properly).
  const float wide_clear_threshold = std::min(radius * 0.8f, 1.0f);
  std::vector<bool> wide_safe(static_cast<size_t>(num_rays), false);
  int safe_count = 0;
  for (int k = 0; k < num_rays; ++k) {
    const auto & ray = rays[static_cast<size_t>(k)];
    if (ray.escaped && ray.escape_dist <= 0.45f && ray.clear_dist >= wide_clear_threshold) {
      wide_safe[static_cast<size_t>(k)] = true;
      safe_count++;
    }
  }

  if (safe_count == num_rays) {
    // All directions wide open; pick the ray with lowest mean cost.
    int best_k = 0;
    for (int k = 1; k < num_rays; ++k) {
      if (rays[static_cast<size_t>(k)].mean_cost < rays[static_cast<size_t>(best_k)].mean_cost) {
        best_k = k;
      }
    }
    res.found = true;
    res.angle = normalizeAngle(rays[static_cast<size_t>(best_k)].angle);
    res.escape_distance = rays[static_cast<size_t>(best_k)].escape_dist;
    res.safe_distance =
      rays[static_cast<size_t>(best_k)].escape_dist + rays[static_cast<size_t>(best_k)].clear_dist;
    res.arc_start = start_angle;
    res.arc_end = end_angle;
  } else if (safe_count > 0) {
    // Find an unsafe starting index so circular runs never get split at ±pi.
    int first_unsafe = 0;
    while (first_unsafe < num_rays && wide_safe[static_cast<size_t>(first_unsafe)]) {
      first_unsafe++;
    }

    int best_start = 0;
    int best_len = 0;
    int cur_start = 0;
    int cur_len = 0;
    for (int offset = 0; offset < num_rays; ++offset) {
      const int idx = (first_unsafe + offset) % num_rays;
      if (wide_safe[static_cast<size_t>(idx)]) {
        if (cur_len == 0) {
          cur_start = idx;
        }
        cur_len++;
        if (cur_len > best_len) {
          best_len = cur_len;
          best_start = cur_start;
        }
      } else {
        cur_len = 0;
      }
    }

    const float arc_start = rays[static_cast<size_t>(best_start)].angle;
    const float arc_span = static_cast<float>(best_len - 1) * step;
    const int mid_idx = (best_start + best_len / 2) % num_rays;
    res.found = true;
    res.angle = normalizeAngle(arc_start + 0.5f * arc_span);
    res.escape_distance = rays[static_cast<size_t>(mid_idx)].escape_dist;
    res.safe_distance =
      rays[static_cast<size_t>(mid_idx)].escape_dist +
      rays[static_cast<size_t>(mid_idx)].clear_dist;
    res.arc_start = arc_start;
    res.arc_end = arc_start + arc_span;
  } else {
    // Tier 2: Tight corridor / corner escape — pick the smoothest, longest, lowest-cost escape ray.
    int best_k = -1;
    float best_smooth_score = -std::numeric_limits<float>::infinity();
    for (int k = 0; k < num_rays; ++k) {
      const auto & c_ray = rays[static_cast<size_t>(k)];
      if (!c_ray.escaped || c_ray.clear_dist < resolution) {
        continue;
      }
      const auto & prev_ray = rays[static_cast<size_t>((k - 1 + num_rays) % num_rays)];
      const auto & next_ray = rays[static_cast<size_t>((k + 1) % num_rays)];
      const float prev_s = prev_ray.escaped ? prev_ray.score : -0.5f;
      const float next_s = next_ray.escaped ? next_ray.score : -0.5f;
      const float smooth_score = 0.5f * c_ray.score + 0.25f * prev_s + 0.25f * next_s;
      if (smooth_score > best_smooth_score) {
        best_smooth_score = smooth_score;
        best_k = k;
      }
    }

    if (best_k >= 0) {
      const auto & best_ray = rays[static_cast<size_t>(best_k)];
      res.found = true;
      res.angle = normalizeAngle(best_ray.angle);
      res.escape_distance = best_ray.escape_dist;
      res.safe_distance = best_ray.escape_dist + best_ray.clear_dist;
      res.arc_start = best_ray.angle - step;
      res.arc_end = best_ray.angle + step;
    }
  }

  if (visualize_ && res.found) {
    visualize(pose, radius, res.arc_start, res.arc_end);
  }

  return res;
}

float BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  const DirectionResult dir =
    findBestDirectionDetailed(costmap, pose, start_angle, end_angle, radius, angle_increment);
  return dir.found ? dir.angle : 0.0f;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
