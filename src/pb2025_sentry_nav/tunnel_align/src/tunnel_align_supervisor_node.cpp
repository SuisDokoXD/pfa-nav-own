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

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tunnel_align/tunnel_geometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace tunnel_align
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

enum class State
{
  IDLE = 0,
  STOP_AND_ALIGN_GIMBAL = 1,
  APPROACH_DOORWAY = 2,
  ALIGN_TUNNEL = 3,
  HOLD_ALIGNED = 4,
  TRAVERSE = 5,
  RESTORE = 6,
};

const char * stateName(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::STOP_AND_ALIGN_GIMBAL: return "STOP_AND_ALIGN_GIMBAL";
    case State::APPROACH_DOORWAY: return "APPROACH_DOORWAY";
    case State::ALIGN_TUNNEL: return "ALIGN_TUNNEL";
    case State::HOLD_ALIGNED: return "HOLD_ALIGNED";
    case State::TRAVERSE: return "TRAVERSE";
    case State::RESTORE: return "RESTORE";
  }
  return "UNKNOWN";
}

}  // namespace

class TunnelAlignSupervisorNode : public rclcpp::Node
{
public:
  TunnelAlignSupervisorNode()
  : Node("tunnel_align_supervisor")
  {
    // Topics
    const auto trigger_topic = declare_parameter<std::string>("trigger_topic", "tunnel_align_trigger");
    const auto align_only_topic =
      declare_parameter<std::string>("align_only_trigger_topic", "tunnel_align_only");
    const auto wall_cloud_topic =
      declare_parameter<std::string>("wall_cloud_topic", "registered_scan");
    const auto terrain_map_topic = declare_parameter<std::string>("terrain_map_topic", "terrain_map");
    const auto plan_topic = declare_parameter<std::string>("plan_topic", "plan");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "odometry");
    const auto lidar_odom_topic = declare_parameter<std::string>("lidar_odom_topic", "lidar_odometry");
    const auto lifter_state_topic = declare_parameter<std::string>("lifter_state_topic", "lifter_state");
    const auto gimbal_state_topic = declare_parameter<std::string>("gimbal_state_topic", "gimbal_state");
    const auto cmd_spin_topic = declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
    const auto override_active_topic =
      declare_parameter<std::string>("override_active_topic", "cmd_vel_override_active");
    const auto override_cmd_vel_topic =
      declare_parameter<std::string>("override_cmd_vel_topic", "cmd_vel_override");
    const auto tunnel_align_topic = declare_parameter<std::string>("tunnel_align_topic", "tunnel_align");
    const auto status_topic = declare_parameter<std::string>("status_topic", "tunnel_align_status");
    const auto marker_topic = declare_parameter<std::string>("marker_topic", "tunnel_align_markers");

    // Frames
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal_yaw");
    chassis_frame_ = declare_parameter<std::string>("chassis_frame", "base_footprint");

    // Contract parameters
    const auto mode_str = declare_parameter<std::string>("align_angle_mode", "relative_gimbal");
    align_mode_ = (mode_str == "imu_world") ? AlignAngleMode::IMU_WORLD : AlignAngleMode::RELATIVE_GIMBAL;
    align_angle_sign_ = declare_parameter<double>("align_angle_sign", 1.0);
    chassis_symmetry_fold_ = declare_parameter<int>("chassis_symmetry_fold", 4);

    // Timing & motion parameters
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    resume_spin_speed_ = declare_parameter<double>("resume_spin_speed", 3.14);
    align_gimbal_first_ = declare_parameter<bool>("align_gimbal_first", true);
    stop_linear_vel_threshold_ = declare_parameter<double>("stop_linear_vel_threshold", 0.15);
    min_stage_settle_sec_ = declare_parameter<double>("min_stage_settle_sec", 0.12);
    align_timeout_sec_ = declare_parameter<double>("align_timeout_sec", 4.0);
    restore_settle_sec_ = declare_parameter<double>("restore_settle_sec", 0.08);

    traverse_speed_ = declare_parameter<double>("traverse_speed", 0.75);
    traverse_accel_time_sec_ = declare_parameter<double>("traverse_accel_time_sec", 0.35);
    lateral_kp_ = declare_parameter<double>("lateral_kp", 2.0);
    max_lateral_speed_ = declare_parameter<double>("max_lateral_speed", 0.38);
    min_entry_lateral_speed_ = declare_parameter<double>("min_entry_lateral_speed", 0.16);
    entry_centering_timeout_sec_ = declare_parameter<double>("entry_centering_timeout_sec", 1.5);
    lateral_ema_alpha_ = declare_parameter<double>("lateral_ema_alpha", 0.55);
    lateral_deadband_m_ = declare_parameter<double>("lateral_deadband_m", 0.005);
    lateral_entry_tol_m_ = declare_parameter<double>("lateral_entry_tol_m", 0.018);
    lateral_cruise_slow_m_ = declare_parameter<double>("lateral_cruise_slow_m", 0.032);
    max_traverse_distance_ = declare_parameter<double>("max_traverse_distance", 6.0);
    exit_on_corridor_open_ = declare_parameter<bool>("exit_on_corridor_open", true);
    exit_clear_distance_ = declare_parameter<double>("exit_clear_distance", 0.35);
    require_wall_fit_to_traverse_ = declare_parameter<bool>("require_wall_fit_to_traverse", true);
    wall_obstacle_min_intensity_ = declare_parameter<double>("wall_obstacle_min_intensity", 0.1);
    wall_cloud_use_z_band_ = declare_parameter<bool>("wall_cloud_use_z_band", true);
    wall_z_min_rel_ = declare_parameter<double>("wall_z_min_rel", 0.08);
    wall_z_max_rel_ = declare_parameter<double>("wall_z_max_rel", 0.34);
    lidar_z_offset_ = declare_parameter<double>("lidar_z_offset", 0.256);
    wall_cloud_decay_sec_ = declare_parameter<double>("wall_cloud_decay_sec", 3.0);
    front_obstacle_decay_sec_ = declare_parameter<double>("front_obstacle_decay_sec", 0.5);
    wall_voxel_size_ = declare_parameter<double>("wall_voxel_size", 0.02);
    wall_self_filter_radius_ = declare_parameter<double>("wall_self_filter_radius", 0.22);
    wall_max_range_ = declare_parameter<double>("wall_max_range", 5.0);
    plan_timeout_sec_ = declare_parameter<double>("plan_timeout_sec", 3.0);

    // Doorway-gap approach parameters (used when off-center/far so one inner wall is occluded)
    enable_doorway_approach_ = declare_parameter<bool>("enable_doorway_approach", true);
    doorway_stage_dist_m_ = declare_parameter<double>("doorway_stage_dist_m", 0.65);
    approach_speed_ = declare_parameter<double>("approach_speed", 0.65);
    approach_timeout_sec_ = declare_parameter<double>("approach_timeout_sec", 8.0);
    approach_settle_wait_sec_ = declare_parameter<double>("approach_settle_wait_sec", 0.60);
    discovery_roi_max_x_ = declare_parameter<double>("discovery_roi_max_x", 4.5);

    // Estimator parameters
    est_params_.roi_min_x = declare_parameter<double>("roi_min_x", -0.6);
    est_params_.roi_max_x = declare_parameter<double>("roi_max_x", 3.2);
    est_params_.min_half_width = declare_parameter<double>("min_half_width", 0.28);
    est_params_.max_half_width = declare_parameter<double>("max_half_width", 1.00);
    est_params_.min_corridor_width = declare_parameter<double>("min_corridor_width", 0.40);
    est_params_.max_corridor_width = declare_parameter<double>("max_corridor_width", 1.30);
    est_params_.slice_length = declare_parameter<double>("slice_length", 0.20);
    est_params_.wall_inlier_band = declare_parameter<double>("wall_inlier_band", 0.30);
    est_params_.min_slices_per_side = declare_parameter<int>("min_slices_per_side", 3);
    est_params_.max_heading_correction_deg =
      declare_parameter<double>("max_heading_correction_deg", 40.0);
    est_params_.min_elongation_ratio = declare_parameter<double>("min_elongation_ratio", 3.0);
    est_params_.min_wall_length = declare_parameter<double>("min_wall_length", 0.40);
    est_params_.min_bilateral_overlap = declare_parameter<double>("min_bilateral_overlap", 0.35);
    est_params_.max_side_rms = declare_parameter<double>("max_side_rms", 0.08);
    est_params_.max_side_angle_diff_deg =
      declare_parameter<double>("max_side_angle_diff_deg", 16.0);
    est_params_.interior_margin_m = declare_parameter<double>("interior_margin_m", 0.09);
    est_params_.max_interior_block_points =
      declare_parameter<int>("max_interior_block_points", 5);
    est_params_.doorway_min_wall_dist = declare_parameter<double>("doorway_min_wall_dist", 0.30);
    est_params_.doorway_max_wall_dist = declare_parameter<double>("doorway_max_wall_dist", 4.80);
    est_params_.doorway_wall_band_m = declare_parameter<double>("doorway_wall_band_m", 0.12);
    est_params_.doorway_min_flank_length =
      declare_parameter<double>("doorway_min_flank_length", 0.30);
    est_params_.doorway_min_flank_points =
      declare_parameter<int>("doorway_min_flank_points", 4);
    est_params_.doorway_depth_min_behind =
      declare_parameter<double>("doorway_depth_min_behind", 0.22);
    est_params_.doorway_depth_max_behind =
      declare_parameter<double>("doorway_depth_max_behind", 2.80);
    est_params_.doorway_min_depth_span =
      declare_parameter<double>("doorway_min_depth_span", 0.30);
    est_params_.doorway_min_depth_points =
      declare_parameter<int>("doorway_min_depth_points", 4);
    est_params_.front_obstacle_min_x = declare_parameter<double>("front_obstacle_min_x", 0.15);
    est_params_.front_obstacle_check_dist =
      declare_parameter<double>("front_obstacle_check_dist", 0.42);
    est_params_.front_obstacle_half_width =
      declare_parameter<double>("front_obstacle_half_width", 0.24);
    est_params_.front_obstacle_min_points =
      declare_parameter<int>("front_obstacle_min_points", 3);
    est_params_.spin_clear_half_length =
      declare_parameter<double>("spin_clear_half_length", 0.45);
    est_params_.spin_clear_half_width =
      declare_parameter<double>("spin_clear_half_width", 0.42);
    est_params_.spin_clear_min_points =
      declare_parameter<int>("spin_clear_min_points", 3);

    est_params_discovery_ = est_params_;
    est_params_discovery_.roi_max_x = std::max(est_params_.roi_max_x, discovery_roi_max_x_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);

    trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
      trigger_topic, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        handleTrigger(msg->data, false);
      });
    align_only_sub_ = create_subscription<std_msgs::msg::Bool>(
      align_only_topic, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        handleTrigger(msg->data, true);
      });
    wall_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      wall_cloud_topic, sensor_qos,
      std::bind(&TunnelAlignSupervisorNode::wallCloudCallback, this, std::placeholders::_1));
    terrain_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      terrain_map_topic, sensor_qos,
      std::bind(&TunnelAlignSupervisorNode::terrainMapCallback, this, std::placeholders::_1));
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic, sensor_qos, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        latest_plan_ = *msg;
        latest_plan_stamp_ = now();
        has_plan_ = !msg->poses.empty();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, sensor_qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        latest_odom_ = *msg;
        has_odom_ = true;
      });
    lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lidar_odom_topic, sensor_qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        base_z_odom_ = msg->pose.pose.position.z - lidar_z_offset_;
        has_lidar_odom_ = true;
      });
    lifter_state_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      lifter_state_topic, 10, [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        if (msg->data.size() >= 4) {
          lifter_pos_ = msg->data[0];
          tunnel_yaw_aligned_ = (msg->data[3] != 0);
          has_lifter_state_ = true;
        }
      });
    gimbal_state_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      gimbal_state_topic, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        gimbal_imu_yaw_rad_ = tf2::getYaw(msg->orientation);
        has_gimbal_imu_ = true;
      });

    cmd_spin_pub_ = create_publisher<example_interfaces::msg::Float32>(cmd_spin_topic, 10);
    override_active_pub_ = create_publisher<std_msgs::msg::Bool>(override_active_topic, 10);
    override_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(override_cmd_vel_topic, 10);
    tunnel_align_pub_ = create_publisher<geometry_msgs::msg::Vector3>(tunnel_align_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 5);

    const auto period_ms =
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(2.0, control_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&TunnelAlignSupervisorNode::controlStep, this));

    RCLCPP_INFO(
      get_logger(),
      "tunnel_align_supervisor started: mode=%s sign=%.0f symmetry=%d trigger=%s",
      (align_mode_ == AlignAngleMode::IMU_WORLD) ? "imu_world" : "relative_gimbal",
      align_angle_sign_, chassis_symmetry_fold_, trigger_topic.c_str());
  }

private:
  void handleTrigger(bool enable, bool align_only)
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (enable) {
      align_only_mode_ = align_only;
      if (state_ == State::IDLE || state_ == State::RESTORE) {
        enterStateLocked(State::STOP_AND_ALIGN_GIMBAL);
        RCLCPP_INFO(
          get_logger(), "Tunnel align triggered (align_only=%s): stopping spin & aligning chassis to gimbal.",
          align_only ? "true" : "false");
      } else if (!align_only && state_ == State::HOLD_ALIGNED) {
        // Allow transitioning directly from HOLD_ALIGNED into TRAVERSE when full trigger arrives.
        entry_centered_ = false;
        entry_centered_ticks_ = 0;
        has_filtered_lat_ = false;
        filtered_lat_err_ = 0.0;
        cruise_start_time_ = now();
        enterStateLocked(State::TRAVERSE);
        RCLCPP_INFO(get_logger(), "Full trigger received in HOLD_ALIGNED -> entering TRAVERSE.");
      }
    } else {
      if (state_ != State::IDLE && state_ != State::RESTORE) {
        enterStateLocked(State::RESTORE);
        RCLCPP_INFO(get_logger(), "Tunnel align release signal received -> restoring normal navigation.");
      }
    }
  }

  void wallCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!wall_cloud_use_z_band_) {
      return;
    }
    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto & f : msg->fields) {
      if (f.name == "x") {
        x_off = static_cast<int>(f.offset);
      } else if (f.name == "y") {
        y_off = static_cast<int>(f.offset);
      } else if (f.name == "z") {
        z_off = static_cast<int>(f.offset);
      }
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) {
      return;
    }

    const size_t n_pts = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const size_t step = static_cast<size_t>(msg->point_step);
    if (step == 0 || msg->data.size() < n_pts * step) {
      return;
    }

    std::lock_guard<std::mutex> lk(data_mtx_);
    const double rx = has_odom_ ? latest_odom_.pose.pose.position.x : 0.0;
    const double ry = has_odom_ ? latest_odom_.pose.pose.position.y : 0.0;
    const double base_z = has_lidar_odom_ ? base_z_odom_ : 0.0;
    const double z_lo = base_z + wall_z_min_rel_;
    const double z_hi = base_z + wall_z_max_rel_;

    std::vector<Point2D> frame_pts;
    frame_pts.reserve(n_pts / 4 + 16);
    for (size_t idx = 0; idx < n_pts; ++idx) {
      const uint8_t * ptr = msg->data.data() + idx * step;
      float x = 0.0f, y = 0.0f, z = 0.0f;
      std::memcpy(&x, ptr + x_off, sizeof(float));
      std::memcpy(&y, ptr + y_off, sizeof(float));
      std::memcpy(&z, ptr + z_off, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (static_cast<double>(z) < z_lo || static_cast<double>(z) > z_hi) {
        continue;
      }
      const double dist = std::hypot(static_cast<double>(x) - rx, static_cast<double>(y) - ry);
      if (dist < wall_self_filter_radius_ || dist > wall_max_range_) {
        continue;
      }
      frame_pts.push_back({static_cast<double>(x), static_cast<double>(y)});
    }

    const rclcpp::Time t_now = now();
    wall_cloud_buffer_.push_back({t_now, std::move(frame_pts)});
    while (
      !wall_cloud_buffer_.empty() &&
      (t_now - wall_cloud_buffer_.front().stamp).seconds() > wall_cloud_decay_sec_)
    {
      wall_cloud_buffer_.pop_front();
    }

    // Deduplicate rolling 3D Z-band slice into:
    //  1) wall_points_3d_odom_ (full rolling window for wall fitting & spin-clearance envelope)
    //  2) live_wall_points_3d_odom_ (short <=0.5s live window for front_blocked safety check,
    //     immune to multi-second Point-LIO odometry drift creating ghost side-wall hits ahead!)
    const double inv_res = 1.0 / std::max(wall_voxel_size_, 0.01);
    std::unordered_set<int64_t> seen;
    std::unordered_set<int64_t> seen_live;
    std::vector<Point2D> merged;
    std::vector<Point2D> merged_live;
    merged.reserve(1024);
    merged_live.reserve(256);
    for (const auto & frame : wall_cloud_buffer_) {
      const bool is_live = (t_now - frame.stamp).seconds() <= front_obstacle_decay_sec_;
      for (const auto & p : frame.points) {
        const int64_t ix = static_cast<int64_t>(std::floor(p.x * inv_res));
        const int64_t iy = static_cast<int64_t>(std::floor(p.y * inv_res));
        const int64_t key = (ix << 32) ^ (iy & 0xFFFFFFFFLL);
        if (seen.insert(key).second) {
          merged.push_back(p);
        }
        if (is_live && seen_live.insert(key).second) {
          merged_live.push_back(p);
        }
      }
    }
    wall_points_3d_odom_ = std::move(merged);
    live_wall_points_3d_odom_ = std::move(merged_live);
    received_wall_cloud_3d_ = true;
  }

  void terrainMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    int x_off = -1, y_off = -1, i_off = -1;
    for (const auto & f : msg->fields) {
      if (f.name == "x") {
        x_off = static_cast<int>(f.offset);
      } else if (f.name == "y") {
        y_off = static_cast<int>(f.offset);
      } else if (f.name == "intensity") {
        i_off = static_cast<int>(f.offset);
      }
    }
    if (x_off < 0 || y_off < 0) {
      return;
    }

    std::vector<Point2D> walls;
    const size_t n_pts = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    walls.reserve(n_pts / 4 + 16);
    const size_t step = static_cast<size_t>(msg->point_step);
    if (step == 0 || msg->data.size() < n_pts * step) {
      return;
    }

    for (size_t idx = 0; idx < n_pts; ++idx) {
      const uint8_t * ptr = msg->data.data() + idx * step;
      float x = 0.0f, y = 0.0f, inten = 1.0f;
      std::memcpy(&x, ptr + x_off, sizeof(float));
      std::memcpy(&y, ptr + y_off, sizeof(float));
      if (i_off >= 0) {
        std::memcpy(&inten, ptr + i_off, sizeof(float));
      }
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      if (static_cast<double>(inten) >= wall_obstacle_min_intensity_) {
        walls.push_back({static_cast<double>(x), static_cast<double>(y)});
      }
    }

    std::lock_guard<std::mutex> lk(data_mtx_);
    wall_points_odom_ = std::move(walls);
  }

  bool lookupRobotPoseOdom(
    Point2D & pos_odom, double & gimbal_odom_rad, double & chassis_odom_rad)
  {
    bool got_gimbal = false;
    try {
      const auto tf_g = tf_buffer_->lookupTransform(
        odom_frame_, gimbal_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      pos_odom.x = tf_g.transform.translation.x;
      pos_odom.y = tf_g.transform.translation.y;
      gimbal_odom_rad = tf2::getYaw(tf_g.transform.rotation);
      got_gimbal = true;
    } catch (const tf2::TransformException &) {
      if (has_odom_) {
        pos_odom.x = latest_odom_.pose.pose.position.x;
        pos_odom.y = latest_odom_.pose.pose.position.y;
        gimbal_odom_rad = tf2::getYaw(latest_odom_.pose.pose.orientation);
        got_gimbal = true;
      }
    }
    if (!got_gimbal) {
      return false;
    }

    chassis_odom_rad = gimbal_odom_rad;
    try {
      const auto tf_c = tf_buffer_->lookupTransform(
        odom_frame_, chassis_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      chassis_odom_rad = tf2::getYaw(tf_c.transform.rotation);
    } catch (const tf2::TransformException &) {
      // Fallback to gimbal heading if independent chassis frame is unavailable.
    }
    return true;
  }

  bool computePlanHeadingOdom(const Point2D & robot_odom, double & heading_odom_rad)
  {
    if (!has_plan_ || latest_plan_.poses.size() < 2) {
      return false;
    }
    if ((now() - latest_plan_stamp_).seconds() > plan_timeout_sec_) {
      return false;
    }

    geometry_msgs::msg::TransformStamped tf_plan_to_odom;
    bool need_tf = (!latest_plan_.header.frame_id.empty() && latest_plan_.header.frame_id != odom_frame_);
    if (need_tf) {
      try {
        tf_plan_to_odom = tf_buffer_->lookupTransform(
          odom_frame_, latest_plan_.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException &) {
        need_tf = false;
      }
    }

    std::vector<Point2D> pts;
    pts.reserve(latest_plan_.poses.size());
    for (const auto & ps : latest_plan_.poses) {
      geometry_msgs::msg::PoseStamped out = ps;
      if (need_tf) {
        tf2::doTransform(ps, out, tf_plan_to_odom);
      }
      pts.push_back({out.pose.position.x, out.pose.position.y});
    }

    // Find closest point on plan to robot
    size_t closest_idx = 0;
    double best_d2 = 1e18;
    for (size_t i = 0; i < pts.size(); ++i) {
      const double dx = pts[i].x - robot_odom.x;
      const double dy = pts[i].y - robot_odom.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        closest_idx = i;
      }
    }

    // Find a forward point ~1.0 m ahead along the path
    size_t fwd_idx = closest_idx;
    double accum = 0.0;
    while (fwd_idx + 1 < pts.size() && accum < 1.0) {
      accum += std::hypot(pts[fwd_idx + 1].x - pts[fwd_idx].x, pts[fwd_idx + 1].y - pts[fwd_idx].y);
      ++fwd_idx;
    }
    if (fwd_idx == closest_idx && closest_idx > 0) {
      closest_idx--;
    }
    const double dx = pts[fwd_idx].x - pts[closest_idx].x;
    const double dy = pts[fwd_idx].y - pts[closest_idx].y;
    if (std::hypot(dx, dy) < 0.15) {
      return false;
    }
    heading_odom_rad = std::atan2(dy, dx);
    return true;
  }

  TunnelEstimate detectCorridorOdom(
    const Point2D & robot_odom, double seed_heading_rad, bool full_angle_search,
    bool use_discovery_roi = false) const
  {
    // Once 3D Z-band registered_scan has been received, NEVER fall back to 2D terrain_map
    // when the 3D slice temporarily empties at a tunnel exit (terrain_map projects low roofs
    // onto the centerline and would falsely trigger front_blocked!).
    const auto & pts =
      (wall_cloud_use_z_band_ && received_wall_cloud_3d_) ? wall_points_3d_odom_ : wall_points_odom_;
    const auto & params = use_discovery_roi ? est_params_discovery_ : est_params_;
    return estimateTunnelFromWalls(pts, robot_odom, seed_heading_rad, params, full_angle_search);
  }

  DoorwayEstimate detectDoorwayOdom(const Point2D & robot_odom) const
  {
    const auto & pts =
      (wall_cloud_use_z_band_ && received_wall_cloud_3d_) ? wall_points_3d_odom_ : wall_points_odom_;
    return detectDoorwayOpening(pts, robot_odom, est_params_discovery_);
  }

  TunnelEstimate evaluateLockedCorridorOdom(
    const Point2D & robot_odom, double locked_heading_rad) const
  {
    const auto & pts =
      (wall_cloud_use_z_band_ && received_wall_cloud_3d_) ? wall_points_3d_odom_ : wall_points_odom_;
    return evaluateTunnelAtLockedHeading(pts, robot_odom, locked_heading_rad, est_params_);
  }

  bool isLiveFrontBlockedOdom(const Point2D & robot_odom, double locked_heading_rad) const
  {
    const auto & pts =
      (wall_cloud_use_z_band_ && received_wall_cloud_3d_) ? live_wall_points_3d_odom_ : wall_points_odom_;
    return isFrontPathBlocked(pts, robot_odom, locked_heading_rad, est_params_);
  }

  void enterStateLocked(State next)
  {
    state_ = next;
    state_enter_time_ = now();
  }

  void publishSpin(double wz)
  {
    example_interfaces::msg::Float32 msg;
    msg.data = static_cast<float>(wz);
    cmd_spin_pub_->publish(msg);
  }

  void publishOverride(bool active, double vx_odom, double vy_odom)
  {
    std_msgs::msg::Bool flag;
    flag.data = active;
    override_active_pub_->publish(flag);

    geometry_msgs::msg::Twist tw;
    tw.linear.x = vx_odom;
    tw.linear.y = vy_odom;
    tw.angular.z = 0.0;
    override_cmd_vel_pub_->publish(tw);
  }

  void publishTunnelAlign(bool active, double angle_deg)
  {
    geometry_msgs::msg::Vector3 msg;
    msg.x = angle_deg;
    msg.y = active ? 1.0 : 0.0;
    msg.z = 0.0;
    tunnel_align_pub_->publish(msg);
  }

  void publishStatus(const std::string & detail)
  {
    std_msgs::msg::String msg;
    msg.data = detail;
    status_pub_->publish(msg);
  }

  void publishMarkers(const Point2D & robot_odom, const TunnelEstimate & est, double axis_rad)
  {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    // Centerline arrow
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = odom_frame_;
    arrow.header.stamp = stamp;
    arrow.ns = "tunnel_align";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.06;
    arrow.scale.y = 0.14;
    arrow.scale.z = 0.14;
    arrow.color.r = est.valid ? 0.1f : 0.9f;
    arrow.color.g = est.valid ? 0.95f : 0.7f;
    arrow.color.b = 0.2f;
    arrow.color.a = 0.95f;

    const double nx = -std::sin(axis_rad);
    const double ny = std::cos(axis_rad);
    // Centerline origin shifted by -lateral_offset_m along corridor normal
    const double cx = robot_odom.x - (est.valid ? est.lateral_offset_m * nx : 0.0);
    const double cy = robot_odom.y - (est.valid ? est.lateral_offset_m * ny : 0.0);

    geometry_msgs::msg::Point p0, p1;
    p0.x = cx;
    p0.y = cy;
    p0.z = 0.35;
    p1.x = cx + 1.8 * std::cos(axis_rad);
    p1.y = cy + 1.8 * std::sin(axis_rad);
    p1.z = 0.35;
    arrow.points.push_back(p0);
    arrow.points.push_back(p1);
    arr.markers.push_back(arrow);

    // Left / right fitted walls if valid
    for (int side = 0; side < 2; ++side) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = odom_frame_;
      line.header.stamp = stamp;
      line.ns = "tunnel_align";
      line.id = side + 1;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = est.valid ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
      line.scale.x = 0.04;
      line.color.r = 1.0f;
      line.color.g = 0.85f;
      line.color.b = 0.1f;
      line.color.a = 0.9f;
      if (est.valid) {
        const double signed_d = (side == 0) ? est.left_wall_dist_m : -est.right_wall_dist_m;
        const double wx = robot_odom.x + signed_d * nx;
        const double wy = robot_odom.y + signed_d * ny;
        geometry_msgs::msg::Point a, b;
        a.x = wx - 0.5 * std::cos(axis_rad);
        a.y = wy - 0.5 * std::sin(axis_rad);
        a.z = 0.30;
        b.x = wx + 2.2 * std::cos(axis_rad);
        b.y = wy + 2.2 * std::sin(axis_rad);
        b.z = 0.30;
        line.points.push_back(a);
        line.points.push_back(b);
      }
      arr.markers.push_back(line);
    }

    marker_pub_->publish(arr);
  }

  void controlStep()
  {
    std::lock_guard<std::mutex> lk(data_mtx_);

    Point2D robot_odom{0.0, 0.0};
    double gimbal_odom_rad = 0.0;
    double chassis_odom_rad = 0.0;
    const bool has_pose = lookupRobotPoseOdom(robot_odom, gimbal_odom_rad, chassis_odom_rad);

    // Continuously calibrate map/odom-minus-IMU yaw offset for Option B
    if (has_pose && has_gimbal_imu_) {
      offset_filter_.update(normalizeAngleRad(gimbal_odom_rad - gimbal_imu_yaw_rad_));
    }

    const double linear_speed = has_odom_
      ? std::hypot(latest_odom_.twist.twist.linear.x, latest_odom_.twist.twist.linear.y)
      : 0.0;

    if (state_ == State::IDLE) {
      publishStatus("IDLE");
      return;
    }

    if (!has_pose) {
      publishOverride(true, 0.0, 0.0);
      publishSpin(0.0);
      publishStatus(std::string(stateName(state_)) + " (waiting for TF/odom)");
      return;
    }

    const double elapsed = (now() - state_enter_time_).seconds();

    if (state_ == State::STOP_AND_ALIGN_GIMBAL) {
      publishSpin(0.0);
      publishOverride(true, 0.0, 0.0);

      // Step 1: Stop spin, and optionally align chassis flush with current gimbal orientation.
      if (align_gimbal_first_) {
        double cmd_deg = 0.0;
        if (align_mode_ == AlignAngleMode::RELATIVE_GIMBAL) {
          cmd_deg = 0.0;
        } else {
          const double target_imu = has_gimbal_imu_
            ? gimbal_imu_yaw_rad_
            : normalizeAngleRad(gimbal_odom_rad - offset_filter_.value());
          cmd_deg = normalizeAngleDeg(align_angle_sign_ * target_imu * 180.0 / kPi);
        }
        publishTunnelAlign(true, cmd_deg);
      } else {
        publishTunnelAlign(false, 0.0);
      }

      const bool stopped = (linear_speed <= stop_linear_vel_threshold_);
      const bool aligned = !align_gimbal_first_ || (has_lifter_state_ && tunnel_yaw_aligned_);
      if ((elapsed >= min_stage_settle_sec_ && stopped && aligned) || elapsed >= align_timeout_sec_) {
        if (elapsed >= align_timeout_sec_) {
          RCLCPP_WARN(
            get_logger(),
            "STOP_AND_ALIGN_GIMBAL timed out (speed=%.2f aligned=%d); proceeding to ALIGN_TUNNEL.",
            linear_speed, aligned ? 1 : 0);
        }
        // Seed tunnel axis from fresh Nav2 plan or full 360-deg wall discovery around robot
        double seed_heading = gimbal_odom_rad;
        const bool has_plan_heading = computePlanHeadingOdom(robot_odom, seed_heading);
        TunnelEstimate est = detectCorridorOdom(robot_odom, seed_heading, !has_plan_heading, true);
        // If plan heading failed to see a corridor (e.g. short/curved plan), try full 360-deg search.
        if (!est.valid && has_plan_heading) {
          est = detectCorridorOdom(robot_odom, seed_heading, true, true);
        }

        // Layer 2 fallback: when far or off-center, doorway walls occlude one inner tunnel wall.
        // Detect a wall opening with verified tunnel depth behind it and glide to staging point.
        if (!est.valid && enable_doorway_approach_ && !align_only_mode_) {
          const DoorwayEstimate dw = detectDoorwayOdom(robot_odom);
          if (dw.valid) {
            approach_doorway_ = dw;
            approach_stage_odom_ = {
              dw.center_odom.x - doorway_stage_dist_m_ * std::cos(dw.axis_heading_rad),
              dw.center_odom.y - doorway_stage_dist_m_ * std::sin(dw.axis_heading_rad)};
            approach_arrived_ = false;
            locked_tunnel_heading_odom_ = dw.axis_heading_rad;
            locked_chassis_target_odom_ = selectNearestSymmetricHeading(
              dw.axis_heading_rad, chassis_odom_rad, chassis_symmetry_fold_);

            TunnelEstimate marker_est;
            marker_est.valid = true;
            marker_est.axis_heading_rad = dw.axis_heading_rad;
            marker_est.left_wall_dist_m = 0.5 * dw.opening_width_m;
            marker_est.right_wall_dist_m = 0.5 * dw.opening_width_m;
            publishMarkers(dw.center_odom, marker_est, dw.axis_heading_rad);

            RCLCPP_INFO(
              get_logger(),
              "Bilateral walls occluded -> APPROACH_DOORWAY: center=(%.2f,%.2f) axis=%.1fdeg width=%.2fm depth=%.2fm(%dpts) stage=(%.2f,%.2f)",
              dw.center_odom.x, dw.center_odom.y, dw.axis_heading_rad * 180.0 / kPi,
              dw.opening_width_m, dw.depth_span_m, dw.depth_support_points,
              approach_stage_odom_.x, approach_stage_odom_.y);
            enterStateLocked(State::APPROACH_DOORWAY);
            publishStatus("APPROACH_DOORWAY");
            return;
          }
        }

        if (!est.valid && require_wall_fit_to_traverse_ && !align_only_mode_) {
          RCLCPP_WARN(
            get_logger(),
            "No valid tunnel or doorway opening detected within %.1fm around (%.2f,%.2f) -> aborting to RESTORE.",
            wall_max_range_, robot_odom.x, robot_odom.y);
          enterStateLocked(State::RESTORE);
          publishStatus("NO_TUNNEL_FOUND -> RESTORE");
          return;
        }
        locked_tunnel_heading_odom_ = est.axis_heading_rad;
        initial_tunnel_heading_odom_ = est.axis_heading_rad;
        initial_corridor_width_m_ = est.valid ? est.corridor_width_m : 0.0;
        // Latch nearest symmetric target heading ONCE so ±45 deg boundary never chatters
        locked_chassis_target_odom_ = selectNearestSymmetricHeading(
          locked_tunnel_heading_odom_, chassis_odom_rad, chassis_symmetry_fold_);
        publishMarkers(robot_odom, est, locked_tunnel_heading_odom_);

        RCLCPP_INFO(
          get_logger(),
          "Chassis ready -> ALIGN_TUNNEL: method=%s axis=%.1fdeg target_chassis=%.1fdeg width=%.2fm overlap=%.2fm lat=%.2fm",
          est.valid ? "bilateral" : "fallback", locked_tunnel_heading_odom_ * 180.0 / kPi,
          locked_chassis_target_odom_ * 180.0 / kPi, est.corridor_width_m,
          est.bilateral_overlap_m, est.lateral_offset_m);
        enterStateLocked(State::ALIGN_TUNNEL);
      }
      publishStatus("STOP_AND_ALIGN_GIMBAL");
      return;
    }

    if (state_ == State::APPROACH_DOORWAY) {
      publishSpin(0.0);

      // Pre-align chassis toward the doorway normal while gliding so the front-tilted LiDAR
      // looks straight down the tunnel as soon as the robot enters the doorway cone.
      const double cmd_deg = computeTunnelAlignAngleDeg(
        align_mode_, locked_chassis_target_odom_, gimbal_odom_rad, locked_chassis_target_odom_,
        offset_filter_.value(), 1, align_angle_sign_);
      publishTunnelAlign(true, cmd_deg);

      // Continuously check whether both inner tunnel walls are now visible along the doorway axis.
      const TunnelEstimate est =
        detectCorridorOdom(robot_odom, approach_doorway_.axis_heading_rad, false, true);
      const double dist_to_doorway = std::hypot(
        approach_doorway_.center_odom.x - robot_odom.x,
        approach_doorway_.center_odom.y - robot_odom.y);
      const double axis_err_deg =
        std::abs(normalizeAngleRad(est.axis_heading_rad - approach_doorway_.axis_heading_rad)) *
        180.0 / kPi;

      if (
        est.valid && axis_err_deg <= 20.0 &&
        std::abs(est.lateral_offset_m) <= 0.30 && dist_to_doorway <= 2.6)
      {
        locked_tunnel_heading_odom_ = est.axis_heading_rad;
        initial_tunnel_heading_odom_ = est.axis_heading_rad;
        initial_corridor_width_m_ = est.corridor_width_m;
        locked_chassis_target_odom_ = selectNearestSymmetricHeading(
          locked_tunnel_heading_odom_, chassis_odom_rad, chassis_symmetry_fold_);
        publishMarkers(robot_odom, est, locked_tunnel_heading_odom_);
        publishOverride(true, 0.0, 0.0);
        RCLCPP_INFO(
          get_logger(),
          "APPROACH_DOORWAY locked bilateral tunnel walls (axis=%.1fdeg width=%.2fm overlap=%.2fm lat=%.2fm) -> ALIGN_TUNNEL.",
          est.axis_heading_rad * 180.0 / kPi, est.corridor_width_m,
          est.bilateral_overlap_m, est.lateral_offset_m);
        enterStateLocked(State::ALIGN_TUNNEL);
        publishStatus("APPROACH_LOCKED -> ALIGN_TUNNEL");
        return;
      }

      if (elapsed >= approach_timeout_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "APPROACH_DOORWAY timed out (%.1fs) without bilateral tunnel lock -> RESTORE.", elapsed);
        publishOverride(true, 0.0, 0.0);
        enterStateLocked(State::RESTORE);
        publishStatus("APPROACH_TIMEOUT -> RESTORE");
        return;
      }

      const double dx = approach_stage_odom_.x - robot_odom.x;
      const double dy = approach_stage_odom_.y - robot_odom.y;
      const double dist_to_stage = std::hypot(dx, dy);

      if (dist_to_stage > 0.12) {
        approach_arrived_ = false;
        const double move_heading = std::atan2(dy, dx);
        if (isLiveFrontBlockedOdom(robot_odom, move_heading)) {
          RCLCPP_WARN(
            get_logger(),
            "APPROACH_DOORWAY path blocked along %.1fdeg -> aborting to RESTORE.",
            move_heading * 180.0 / kPi);
          publishOverride(true, 0.0, 0.0);
          enterStateLocked(State::RESTORE);
          publishStatus("APPROACH_BLOCKED -> RESTORE");
          return;
        }
        const double speed = std::clamp(
          1.5 * dist_to_stage, std::max(0.18, min_entry_lateral_speed_), approach_speed_);
        publishOverride(true, speed * std::cos(move_heading), speed * std::sin(move_heading));
        publishStatus(
          "APPROACH_DOORWAY dist=" + std::to_string(dist_to_stage).substr(0, 4) + "m");
      } else {
        publishOverride(true, 0.0, 0.0);
        const bool yaw_ready = !has_lifter_state_ || tunnel_yaw_aligned_;
        if (!approach_arrived_) {
          if (yaw_ready) {
            approach_arrived_ = true;
            approach_arrive_time_ = now();
          }
        } else if ((now() - approach_arrive_time_).seconds() >= approach_settle_wait_sec_) {
          RCLCPP_WARN(
            get_logger(),
            "APPROACH_DOORWAY reached stage (%.2f,%.2f) but no bilateral tunnel walls appeared -> RESTORE.",
            robot_odom.x, robot_odom.y);
          enterStateLocked(State::RESTORE);
          publishStatus("APPROACH_NO_WALLS -> RESTORE");
          return;
        }
        publishStatus("APPROACH_DOORWAY settling");
      }
      return;
    }

    if (state_ == State::ALIGN_TUNNEL || state_ == State::HOLD_ALIGNED) {
      publishSpin(0.0);
      publishOverride(true, 0.0, 0.0);

      // Compute fresh tunnel_align_angle_deg against live gimbal heading
      const double cmd_deg = computeTunnelAlignAngleDeg(
        align_mode_, locked_chassis_target_odom_, gimbal_odom_rad, locked_chassis_target_odom_,
        offset_filter_.value(), 1, align_angle_sign_);
      publishTunnelAlign(true, cmd_deg);

      const TunnelEstimate est = detectCorridorOdom(robot_odom, locked_tunnel_heading_odom_, false);
      publishMarkers(robot_odom, est, locked_tunnel_heading_odom_);

      if (state_ == State::ALIGN_TUNNEL) {
        const bool aligned = has_lifter_state_ && tunnel_yaw_aligned_;
        if ((elapsed >= min_stage_settle_sec_ && aligned) || elapsed >= align_timeout_sec_) {
          if (elapsed >= align_timeout_sec_) {
            RCLCPP_WARN(get_logger(), "ALIGN_TUNNEL timed out (cmd=%.1fdeg).", cmd_deg);
          }
          if (align_only_mode_) {
            RCLCPP_INFO(
              get_logger(), "Tunnel alignment locked (cmd=%.1fdeg) -> HOLD_ALIGNED.", cmd_deg);
            enterStateLocked(State::HOLD_ALIGNED);
          } else {
            traverse_start_odom_ = robot_odom;
            was_in_corridor_ = est.valid || est.spin_zone_blocked;
            corridor_open_dist_ = 0.0;
            last_traverse_odom_ = robot_odom;
            entry_centered_ = false;
            entry_centered_ticks_ = 0;
            has_filtered_lat_ = false;
            filtered_lat_err_ = 0.0;
            cruise_start_time_ = now();
            RCLCPP_INFO(
              get_logger(),
              "Tunnel alignment locked (cmd=%.1fdeg) -> TRAVERSE along %.1fdeg (speed=%.2fm/s, start=(%.2f,%.2f)).",
              cmd_deg, locked_tunnel_heading_odom_ * 180.0 / kPi, traverse_speed_,
              robot_odom.x, robot_odom.y);
            enterStateLocked(State::TRAVERSE);
          }
        }
      }
      publishStatus(
        std::string(stateName(state_)) + " cmd_deg=" + std::to_string(static_cast<int>(std::round(cmd_deg))));
      return;
    }

    if (state_ == State::TRAVERSE) {
      publishSpin(0.0);

      // Strictly evaluate along locked travel heading (no ±40 deg oblique sweep that could
      // latch onto perpendicular exit-doorway room walls and trigger oblique front_blocked).
      const TunnelEstimate est = evaluateLockedCorridorOdom(robot_odom, locked_tunnel_heading_odom_);
      const bool width_consistent =
        (initial_corridor_width_m_ <= 0.0) ||
        (std::abs(est.corridor_width_m - initial_corridor_width_m_) <= 0.12);
      const bool corridor_walls_present = (est.valid && width_consistent);
      if (corridor_walls_present || est.spin_zone_blocked) {
        was_in_corridor_ = true;
        corridor_open_dist_ = 0.0;
        // Only refine heading when BOTH side walls have strong bilateral support (never let
        // perpendicular exit-doorway walls pull the heading off-axis near the tunnel mouth!).
        if (corridor_walls_present && est.left_inliers >= 6 && est.right_inliers >= 6) {
          const double heading_diff =
            normalizeAngleRad(est.axis_heading_rad - locked_tunnel_heading_odom_);
          const double candidate =
            normalizeAngleRad(locked_tunnel_heading_odom_ + 0.10 * heading_diff);
          const double max_drift_rad = 2.5 * kPi / 180.0;
          const double drift = normalizeAngleRad(candidate - initial_tunnel_heading_odom_);
          locked_tunnel_heading_odom_ = normalizeAngleRad(
            initial_tunnel_heading_odom_ + std::clamp(drift, -max_drift_rad, max_drift_rad));
          locked_chassis_target_odom_ = selectNearestSymmetricHeading(
            locked_tunnel_heading_odom_, locked_chassis_target_odom_, chassis_symmetry_fold_);
        }
      } else if (was_in_corridor_) {
        corridor_open_dist_ += std::hypot(
          robot_odom.x - last_traverse_odom_.x, robot_odom.y - last_traverse_odom_.y);
      }
      last_traverse_odom_ = robot_odom;

      const double cmd_deg = computeTunnelAlignAngleDeg(
        align_mode_, locked_chassis_target_odom_, gimbal_odom_rad, locked_chassis_target_odom_,
        offset_filter_.value(), 1, align_angle_sign_);
      publishTunnelAlign(true, cmd_deg);
      publishMarkers(robot_odom, est, locked_tunnel_heading_odom_);

      const double traveled = std::hypot(
        robot_odom.x - traverse_start_odom_.x, robot_odom.y - traverse_start_odom_.y);

      const double raw_lat_err = corridor_walls_present ? est.lateral_offset_m : 0.0;
      if (corridor_walls_present) {
        if (!has_filtered_lat_) {
          filtered_lat_err_ = raw_lat_err;
          has_filtered_lat_ = true;
        } else {
          filtered_lat_err_ =
            (1.0 - lateral_ema_alpha_) * filtered_lat_err_ + lateral_ema_alpha_ * raw_lat_err;
        }
      } else {
        filtered_lat_err_ *= 0.7;
      }

      const double lat_err = (std::abs(filtered_lat_err_) > lateral_deadband_m_) ? filtered_lat_err_ : 0.0;
      const double abs_lat = std::abs(filtered_lat_err_);

      // Two-stage centering gate:
      // 1) Before entry_centered_ latches: damped pure-lateral centering (fwd=0) until settled
      //    inside <= lateral_entry_tol_m_ for 2 consecutive ticks (prevents lateral momentum
      //    overshoot into the 50mm doorframe when accelerating forward).
      // 2) Once latched: smooth cruise at full traverse_speed_ without stuttering on minor jitter;
      //    only gently throttle down if error ever grows past lateral_cruise_slow_m_.
      if (!entry_centered_) {
        const bool within_tol = (!corridor_walls_present || abs_lat <= lateral_entry_tol_m_);
        const bool safe_timeout =
          (elapsed >= entry_centering_timeout_sec_ && abs_lat <= 0.038);
        if (within_tol || safe_timeout) {
          entry_centered_ticks_++;
          if (!corridor_walls_present || safe_timeout || entry_centered_ticks_ >= 2) {
            entry_centered_ = true;
            cruise_start_time_ = now();
          }
        } else {
          entry_centered_ticks_ = 0;
        }
      }

      double fwd_speed = 0.0;
      if (entry_centered_) {
        const double ramp_t = std::max(traverse_accel_time_sec_, 0.05);
        const double ramp = std::clamp((now() - cruise_start_time_).seconds() / ramp_t, 0.25, 1.0);
        fwd_speed = traverse_speed_ * ramp;
        if (corridor_walls_present && abs_lat > lateral_cruise_slow_m_) {
          const double excess = abs_lat - lateral_cruise_slow_m_;
          const double slow_scale = std::clamp(1.0 - excess / 0.035, 0.30, 1.0);
          fwd_speed *= slow_scale;
        }
      }

      // Only trigger front_blocked from fresh/live scans (<=0.5s) when actually pushing forward,
      // so multi-second Point-LIO odometry drift can never turn old side walls into ghost obstacles.
      const bool live_front_blocked = isLiveFrontBlockedOdom(robot_odom, locked_tunnel_heading_odom_);
      if (live_front_blocked && fwd_speed > 0.05) {
        // Hold aligned at zero velocity (do NOT resume spinning inside a tight tunnel!).
        RCLCPP_WARN(
          get_logger(),
          "Live front obstacle detected inside corridor during TRAVERSE (odom=(%.2f,%.2f)) -> HOLD_ALIGNED (spin kept off).",
          robot_odom.x, robot_odom.y);
        publishOverride(true, 0.0, 0.0);
        enterStateLocked(State::HOLD_ALIGNED);
        publishStatus("TRAVERSE_BLOCKED -> HOLD_ALIGNED");
        return;
      }

      // Never allow corridor-exit RESTORE (which resumes spinning!) while wall points remain
      // inside the robot's physical spin-clearance envelope.
      const bool exited_corridor =
        exit_on_corridor_open_ && was_in_corridor_ && !est.spin_zone_blocked &&
        (corridor_open_dist_ >= exit_clear_distance_);
      if (traveled >= max_traverse_distance_ || exited_corridor) {
        RCLCPP_INFO(
          get_logger(),
          "TRAVERSE complete (traveled=%.2fm exited_corridor=%s end=(%.2f,%.2f) open_dist=%.2fm spin_pts=%d) -> RESTORE.",
          traveled, exited_corridor ? "true" : "false", robot_odom.x, robot_odom.y,
          corridor_open_dist_, est.spin_zone_points);
        publishOverride(true, 0.0, 0.0);
        enterStateLocked(State::RESTORE);
        publishStatus("TRAVERSE_DONE -> RESTORE");
        return;
      }

      Vec2D vel_odom{0.0, 0.0};
      if (!entry_centered_) {
        // Stationary pure-lateral centering: apply minimum breakaway speed when outside tolerance
        // so wheel static friction never stalls the chassis at the doorway; hold 0 while settling.
        if (corridor_walls_present && abs_lat > lateral_entry_tol_m_) {
          const double mag = std::clamp(
            lateral_kp_ * abs_lat, std::abs(min_entry_lateral_speed_),
            std::abs(max_lateral_speed_));
          const double v_lat = (filtered_lat_err_ > 0.0) ? -mag : mag;
          const double c = std::cos(locked_tunnel_heading_odom_);
          const double s = std::sin(locked_tunnel_heading_odom_);
          vel_odom = {-v_lat * s, v_lat * c};
        }
      } else {
        vel_odom = computeCenteredTraverseVelocity(
          locked_tunnel_heading_odom_, lat_err, fwd_speed, lateral_kp_, max_lateral_speed_);
      }
      publishOverride(true, vel_odom.vx, vel_odom.vy);
      publishStatus(
        "TRAVERSE dist=" + std::to_string(traveled).substr(0, 4) +
        "m lat_err=" + std::to_string(filtered_lat_err_).substr(0, 5) +
        "m fwd=" + std::to_string(fwd_speed).substr(0, 4));
      return;
    }

    if (state_ == State::RESTORE) {
      publishOverride(true, 0.0, 0.0);
      if (elapsed >= restore_settle_sec_) {
        publishTunnelAlign(false, 0.0);
        publishOverride(false, 0.0, 0.0);
        publishSpin(resume_spin_speed_);
        enterStateLocked(State::IDLE);
        RCLCPP_INFO(
          get_logger(), "Restored normal navigation (spin=%.2f rad/s).", resume_spin_speed_);
      }
      publishStatus("RESTORE");
    }
  }

  struct TimedWallFrame
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::vector<Point2D> points;
  };

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr align_only_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr wall_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr lifter_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gimbal_state_sub_;

  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr cmd_spin_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_active_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr override_cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr tunnel_align_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_frame_;
  std::string gimbal_frame_;
  std::string chassis_frame_;
  AlignAngleMode align_mode_{AlignAngleMode::RELATIVE_GIMBAL};
  double align_angle_sign_{1.0};
  int chassis_symmetry_fold_{4};
  double control_rate_hz_{20.0};
  double resume_spin_speed_{3.14};
  bool align_gimbal_first_{true};
  double stop_linear_vel_threshold_{0.15};
  double min_stage_settle_sec_{0.12};
  double align_timeout_sec_{4.0};
  double restore_settle_sec_{0.08};
  double traverse_speed_{0.85};
  double traverse_accel_time_sec_{0.30};
  double lateral_kp_{2.0};
  double max_lateral_speed_{0.38};
  double min_entry_lateral_speed_{0.16};
  double entry_centering_timeout_sec_{1.5};
  double lateral_ema_alpha_{0.55};
  double lateral_deadband_m_{0.005};
  double lateral_entry_tol_m_{0.022};
  double lateral_cruise_slow_m_{0.038};
  double max_traverse_distance_{5.0};
  bool exit_on_corridor_open_{true};
  double exit_clear_distance_{0.95};
  bool require_wall_fit_to_traverse_{true};
  double wall_obstacle_min_intensity_{0.1};
  bool wall_cloud_use_z_band_{true};
  double wall_z_min_rel_{0.08};
  double wall_z_max_rel_{0.38};
  double lidar_z_offset_{0.256};
  double wall_cloud_decay_sec_{3.0};
  double front_obstacle_decay_sec_{0.5};
  double wall_voxel_size_{0.02};
  double wall_self_filter_radius_{0.22};
  double wall_max_range_{5.0};
  double plan_timeout_sec_{3.0};
  bool enable_doorway_approach_{true};
  double doorway_stage_dist_m_{0.65};
  double approach_speed_{0.65};
  double approach_timeout_sec_{8.0};
  double approach_settle_wait_sec_{0.60};
  double discovery_roi_max_x_{4.5};
  TunnelEstimatorParams est_params_;
  TunnelEstimatorParams est_params_discovery_;

  std::mutex data_mtx_;
  State state_{State::IDLE};
  bool align_only_mode_{false};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  std::deque<TimedWallFrame> wall_cloud_buffer_;
  std::vector<Point2D> wall_points_3d_odom_;
  std::vector<Point2D> live_wall_points_3d_odom_;
  bool received_wall_cloud_3d_{false};
  std::vector<Point2D> wall_points_odom_;
  nav_msgs::msg::Path latest_plan_;
  rclcpp::Time latest_plan_stamp_{0, 0, RCL_ROS_TIME};
  bool has_plan_{false};
  DoorwayEstimate approach_doorway_;
  Point2D approach_stage_odom_{0.0, 0.0};
  bool approach_arrived_{false};
  rclcpp::Time approach_arrive_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_{false};
  double base_z_odom_{0.0};
  bool has_lidar_odom_{false};
  int lifter_pos_{0};
  bool tunnel_yaw_aligned_{false};
  bool has_lifter_state_{false};
  double gimbal_imu_yaw_rad_{0.0};
  bool has_gimbal_imu_{false};
  CircularOffsetFilter offset_filter_{0.2};

  double locked_tunnel_heading_odom_{0.0};
  double initial_tunnel_heading_odom_{0.0};
  double initial_corridor_width_m_{0.0};
  double locked_chassis_target_odom_{0.0};
  Point2D traverse_start_odom_{0.0, 0.0};
  Point2D last_traverse_odom_{0.0, 0.0};
  bool was_in_corridor_{false};
  double corridor_open_dist_{0.0};
  bool entry_centered_{false};
  int entry_centered_ticks_{0};
  bool has_filtered_lat_{false};
  double filtered_lat_err_{0.0};
  rclcpp::Time cruise_start_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace tunnel_align

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tunnel_align::TunnelAlignSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
