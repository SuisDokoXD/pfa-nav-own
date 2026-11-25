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
#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tunnel_align/tunnel_geometry.hpp"

namespace tunnel_align
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr uint8_t kLifterNoChange = 0xFFU;

}  // namespace

/// Simulation drop-in emulator for the C-board + gimbal_bridge tunnel/lifter protocol.
/// Consumes `/tunnel_align` and `/lifter_cmd`, closes chassis yaw alignment via `cmd_align_wz`
/// into `fake_vel_transform`, and publishes `/lifter_state` + `/gimbal_state` identically to
/// `gimbal_bridge_node`.
class SimCBoardEmulatorNode : public rclcpp::Node
{
public:
  SimCBoardEmulatorNode()
  : Node("sim_cboard_emulator")
  {
    const auto tunnel_align_topic = declare_parameter<std::string>("tunnel_align_topic", "tunnel_align");
    const auto lifter_cmd_topic = declare_parameter<std::string>("lifter_cmd_topic", "lifter_cmd");
    const auto joint_states_topic = declare_parameter<std::string>("joint_states_topic", "joint_states");
    const auto align_wz_active_topic =
      declare_parameter<std::string>("align_wz_active_topic", "cmd_align_wz_active");
    const auto align_wz_topic = declare_parameter<std::string>("align_wz_topic", "cmd_align_wz");
    const auto lifter_state_topic = declare_parameter<std::string>("lifter_state_topic", "lifter_state");
    const auto gimbal_state_topic = declare_parameter<std::string>("gimbal_state_topic", "gimbal_state");

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal_yaw");
    chassis_frame_ = declare_parameter<std::string>("chassis_frame", "base_footprint");
    gimbal_yaw_joint_name_ = declare_parameter<std::string>("gimbal_yaw_joint_name", "gimbal_yaw_joint");

    const auto mode_str = declare_parameter<std::string>("align_angle_mode", "relative_gimbal");
    align_mode_ = (mode_str == "imu_world") ? AlignAngleMode::IMU_WORLD : AlignAngleMode::RELATIVE_GIMBAL;
    align_angle_sign_ = declare_parameter<double>("align_angle_sign", 1.0);
    imu_yaw_offset_rad_ = declare_parameter<double>("imu_yaw_offset_deg", 0.0) * kPi / 180.0;

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    yaw_kp_ = declare_parameter<double>("yaw_kp", 4.5);
    yaw_kd_ = declare_parameter<double>("yaw_kd", 0.35);
    max_wz_ = declare_parameter<double>("max_wz", 3.0);
    aligned_angle_tol_rad_ = declare_parameter<double>("aligned_angle_tol_deg", 2.5) * kPi / 180.0;
    aligned_vel_tol_rad_s_ = declare_parameter<double>("aligned_vel_tol_rad_s", 0.15);
    aligned_hold_sec_ = declare_parameter<double>("aligned_hold_sec", 0.18);
    lifter_stroke_duration_sec_ = declare_parameter<double>("lifter_stroke_duration_sec", 1.2);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    tunnel_align_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      tunnel_align_topic, 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        const bool new_active = (msg->y != 0.0);
        const double new_deg = msg->x;
        // Clear latched aligned flag whenever target changes significantly or reactivates
        if (new_active != align_active_ || std::abs(normalizeAngleDeg(new_deg - target_angle_deg_)) > 2.0) {
          aligned_hold_accum_ = 0.0;
          tunnel_yaw_aligned_ = false;
        }
        align_active_ = new_active;
        target_angle_deg_ = new_deg;
      });

    lifter_cmd_sub_ = create_subscription<std_msgs::msg::UInt8>(
      lifter_cmd_topic, 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        const uint8_t target = msg->data;
        if (target == kLifterNoChange || target > 2U) {
          return;
        }
        if (static_cast<int>(target) != lifter_target_pos_) {
          lifter_target_pos_ = static_cast<int>(target);
          lifter_moving_ = true;
          lifter_progress_ = 0.0;
          lifter_start_time_ = now();
        }
      });

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS().keep_last(5),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == gimbal_yaw_joint_name_) {
            if (i < msg->position.size()) {
              joint_yaw_pos_ = msg->position[i];
              has_joint_state_ = true;
            }
            if (i < msg->velocity.size()) {
              joint_yaw_vel_ = msg->velocity[i];
            }
            break;
          }
        }
      });

    align_wz_active_pub_ = create_publisher<std_msgs::msg::Bool>(align_wz_active_topic, 10);
    align_wz_pub_ = create_publisher<example_interfaces::msg::Float32>(align_wz_topic, 10);
    lifter_state_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(lifter_state_topic, 10);
    gimbal_state_pub_ = create_publisher<sensor_msgs::msg::Imu>(gimbal_state_topic, 10);

    const auto period_ms =
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(5.0, control_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&SimCBoardEmulatorNode::step, this));

    RCLCPP_INFO(
      get_logger(),
      "sim_cboard_emulator started: mode=%s sign=%.0f imu_offset=%.1fdeg",
      (align_mode_ == AlignAngleMode::IMU_WORLD) ? "imu_world" : "relative_gimbal",
      align_angle_sign_, imu_yaw_offset_rad_ * 180.0 / kPi);
  }

private:
  void step()
  {
    std::lock_guard<std::mutex> lk(mtx_);

    const auto t_now = now();
    double dt = 1.0 / std::max(5.0, control_rate_hz_);
    if (last_step_time_.nanoseconds() > 0) {
      const double measured_dt = (t_now - last_step_time_).seconds();
      if (measured_dt > 1e-4 && measured_dt < 0.5) {
        dt = measured_dt;
      }
    }
    last_step_time_ = t_now;

    // Update simulated lifter motion
    if (lifter_moving_) {
      const double elapsed = (t_now - lifter_start_time_).seconds();
      const double frac = std::clamp(elapsed / std::max(lifter_stroke_duration_sec_, 0.1), 0.0, 1.0);
      lifter_progress_ = frac * 100.0;
      if (frac >= 1.0) {
        lifter_current_pos_ = lifter_target_pos_;
        lifter_moving_ = false;
      }
    }

    // Read gimbal and chassis orientations in odom frame
    double gimbal_odom_rad = 0.0;
    double chassis_odom_rad = 0.0;
    bool has_tf = false;
    try {
      const auto tf_g = tf_buffer_->lookupTransform(
        odom_frame_, gimbal_frame_, tf2::TimePointZero, tf2::durationFromSec(0.03));
      const auto tf_c = tf_buffer_->lookupTransform(
        odom_frame_, chassis_frame_, tf2::TimePointZero, tf2::durationFromSec(0.03));
      gimbal_odom_rad = tf2::getYaw(tf_g.transform.rotation);
      chassis_odom_rad = tf2::getYaw(tf_c.transform.rotation);
      has_tf = true;
    } catch (const tf2::TransformException &) {
      has_tf = false;
    }

    // Publish simulated gimbal IMU (identical to gimbal_bridge /gimbal_state)
    if (has_tf) {
      const double gimbal_imu_yaw = normalizeAngleRad(gimbal_odom_rad - imu_yaw_offset_rad_);
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = t_now;
      imu_msg.header.frame_id = "gimbal";
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, gimbal_imu_yaw);
      imu_msg.orientation = tf2::toMsg(q);
      gimbal_state_pub_->publish(imu_msg);

      if (has_prev_chassis_yaw_) {
        const double raw_wz = normalizeAngleRad(chassis_odom_rad - prev_chassis_yaw_) / dt;
        est_chassis_wz_ = 0.7 * est_chassis_wz_ + 0.3 * raw_wz;
      }
      prev_chassis_yaw_ = chassis_odom_rad;
      has_prev_chassis_yaw_ = true;
    }

    // Chassis yaw alignment PID when align_active_ is true
    std_msgs::msg::Bool active_msg;
    active_msg.data = align_active_;
    align_wz_active_pub_->publish(active_msg);

    if (align_active_ && has_tf) {
      const double signed_target_rad =
        normalizeAngleRad(align_angle_sign_ * target_angle_deg_ * kPi / 180.0);
      double err_rad = 0.0;
      if (align_mode_ == AlignAngleMode::RELATIVE_GIMBAL) {
        const double chassis_rel_gimbal = normalizeAngleRad(chassis_odom_rad - gimbal_odom_rad);
        err_rad = normalizeAngleRad(signed_target_rad - chassis_rel_gimbal);
      } else {
        const double chassis_imu_yaw = normalizeAngleRad(chassis_odom_rad - imu_yaw_offset_rad_);
        err_rad = normalizeAngleRad(signed_target_rad - chassis_imu_yaw);
      }

      const double wz = std::clamp(yaw_kp_ * err_rad - yaw_kd_ * est_chassis_wz_, -max_wz_, max_wz_);
      example_interfaces::msg::Float32 wz_msg;
      wz_msg.data = static_cast<float>(wz);
      align_wz_pub_->publish(wz_msg);

      if (
        std::abs(err_rad) <= aligned_angle_tol_rad_ &&
        std::abs(est_chassis_wz_) <= aligned_vel_tol_rad_s_)
      {
        aligned_hold_accum_ += dt;
        if (aligned_hold_accum_ >= aligned_hold_sec_) {
          tunnel_yaw_aligned_ = true;
        }
      } else {
        aligned_hold_accum_ = 0.0;
        if (std::abs(err_rad) > aligned_angle_tol_rad_ * 1.5) {
          tunnel_yaw_aligned_ = false;
        }
      }
    } else if (!align_active_) {
      aligned_hold_accum_ = 0.0;
      tunnel_yaw_aligned_ = false;
      example_interfaces::msg::Float32 wz_msg;
      wz_msg.data = 0.0f;
      align_wz_pub_->publish(wz_msg);
    }

    // Publish /lifter_state matching gimbal_bridge layout:
    // [lifter_current_pos, is_transformable, transform_state_centi, tunnel_yaw_aligned]
    std_msgs::msg::Int32MultiArray state_msg;
    state_msg.data.resize(4);
    state_msg.data[0] = lifter_current_pos_;
    state_msg.data[1] = lifter_moving_ ? 0 : 1;
    state_msg.data[2] = static_cast<int32_t>(std::round(lifter_progress_ * 100.0));
    state_msg.data[3] = tunnel_yaw_aligned_ ? 1 : 0;
    lifter_state_pub_->publish(state_msg);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr tunnel_align_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr lifter_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr align_wz_active_pub_;
  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr align_wz_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr lifter_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gimbal_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_frame_;
  std::string gimbal_frame_;
  std::string chassis_frame_;
  std::string gimbal_yaw_joint_name_;
  AlignAngleMode align_mode_{AlignAngleMode::RELATIVE_GIMBAL};
  double align_angle_sign_{1.0};
  double imu_yaw_offset_rad_{0.0};
  double control_rate_hz_{50.0};
  double yaw_kp_{4.5};
  double yaw_kd_{0.35};
  double max_wz_{3.0};
  double aligned_angle_tol_rad_{0.0436};
  double aligned_vel_tol_rad_s_{0.15};
  double aligned_hold_sec_{0.18};
  double lifter_stroke_duration_sec_{1.2};

  std::mutex mtx_;
  bool align_active_{false};
  double target_angle_deg_{0.0};
  double aligned_hold_accum_{0.0};
  bool tunnel_yaw_aligned_{false};

  bool has_joint_state_{false};
  double joint_yaw_pos_{0.0};
  double joint_yaw_vel_{0.0};
  bool has_prev_chassis_yaw_{false};
  double prev_chassis_yaw_{0.0};
  double est_chassis_wz_{0.0};
  rclcpp::Time last_step_time_{0, 0, RCL_ROS_TIME};

  int lifter_current_pos_{0};
  int lifter_target_pos_{0};
  bool lifter_moving_{false};
  double lifter_progress_{100.0};
  rclcpp::Time lifter_start_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace tunnel_align

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tunnel_align::SimCBoardEmulatorNode>());
  rclcpp::shutdown();
  return 0;
}
