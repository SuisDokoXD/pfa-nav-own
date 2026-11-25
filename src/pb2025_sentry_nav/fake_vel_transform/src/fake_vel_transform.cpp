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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("override_active_topic", "cmd_vel_override_active");
  this->declare_parameter<std::string>("override_cmd_vel_topic", "cmd_vel_override");
  this->declare_parameter<std::string>("align_wz_active_topic", "cmd_align_wz_active");
  this->declare_parameter<std::string>("align_wz_topic", "cmd_align_wz");
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<double>("override_timeout_sec", 0.5);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("override_active_topic", override_active_topic_);
  this->get_parameter("override_cmd_vel_topic", override_cmd_vel_topic_);
  this->get_parameter("align_wz_active_topic", align_wz_active_topic_);
  this->get_parameter("align_wz_topic", align_wz_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);
  this->get_parameter("override_timeout_sec", override_timeout_sec_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  override_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    override_active_topic_, 10,
    std::bind(&FakeVelTransform::overrideActiveCallback, this, std::placeholders::_1));
  override_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    override_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::overrideCmdVelCallback, this, std::placeholders::_1));
  align_wz_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    align_wz_active_topic_, 10,
    std::bind(&FakeVelTransform::alignWzActiveCallback, this, std::placeholders::_1));
  align_wz_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    align_wz_topic_, 10,
    std::bind(&FakeVelTransform::alignWzCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  spin_speed_ = msg->data;
  if (!override_active_) {
    publishActiveCommandLocked();
  }
}

void FakeVelTransform::overrideActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const bool prev = override_active_;
  override_active_ = msg->data;
  last_override_time_ = std::chrono::steady_clock::now();
  if (override_active_ && !prev) {
    override_cmd_vel_ = geometry_msgs::msg::Twist();
    publishActiveCommandLocked();
  } else if (!override_active_ && prev) {
    override_cmd_vel_ = geometry_msgs::msg::Twist();
    latest_cmd_vel_.reset();
    publishActiveCommandLocked();
  }
}

void FakeVelTransform::overrideCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  override_cmd_vel_ = *msg;
  last_override_time_ = std::chrono::steady_clock::now();
  if (override_active_) {
    publishActiveCommandLocked();
  }
}

void FakeVelTransform::alignWzActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const bool prev = align_wz_active_;
  align_wz_active_ = msg->data;
  last_align_wz_time_ = std::chrono::steady_clock::now();
  if (!align_wz_active_) {
    align_wz_ = 0.0f;
  }
  if (align_wz_active_ != prev) {
    publishActiveCommandLocked();
  }
}

void FakeVelTransform::alignWzCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  align_wz_ = msg->data;
  last_align_wz_time_ = std::chrono::steady_clock::now();
  if (align_wz_active_) {
    publishActiveCommandLocked();
  }
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  has_odom_ = true;

  // Keep gimbal yaw fresh when controller is idle OR when override/align mode is active
  if (
    override_active_ || align_wz_active_ ||
    (rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT)
  {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  if (override_active_) {
    return;
  }
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // If received velocity cannot be synchronized, publish it directly
    auto aft_tf_vel =
      transformVelocity(*msg, static_cast<float>(current_robot_base_angle_), !align_wz_active_);
    if (align_wz_active_) {
      aft_tf_vel.angular.z = align_wz_;
    }
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = rclcpp::Clock().now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  has_odom_ = true;

  if (override_active_) {
    return;
  }
  if (!latest_cmd_vel_) {
    return;
  }

  float yaw_diff = static_cast<float>(current_robot_base_angle_);
  geometry_msgs::msg::Twist aft_tf_vel =
    transformVelocity(*latest_cmd_vel_, yaw_diff, !align_wz_active_);
  if (align_wz_active_) {
    aft_tf_vel.angular.z = align_wz_;
  }

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::publishActiveCommandLocked()
{
  geometry_msgs::msg::Twist base_twist;
  bool use_spin = false;
  if (override_active_) {
    base_twist = override_cmd_vel_;
    use_spin = false;
  } else {
    const bool controller_active =
      latest_cmd_vel_ &&
      (rclcpp::Clock().now() - last_controller_activate_time_).seconds() <= CONTROLLER_TIMEOUT;
    if (controller_active) {
      base_twist = *latest_cmd_vel_;
    }
    use_spin = !align_wz_active_;
  }

  auto aft_tf_vel =
    transformVelocity(base_twist, static_cast<float>(current_robot_base_angle_), use_spin);
  if (align_wz_active_) {
    aft_tf_vel.angular.z = align_wz_;
  }
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::publishTransform()
{
  double angle_snapshot = 0.0;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (!has_odom_) {
      return;
    }
    angle_snapshot = current_robot_base_angle_;

    const auto now_steady = std::chrono::steady_clock::now();
    if (override_active_) {
      const double elapsed =
        std::chrono::duration<double>(now_steady - last_override_time_).count();
      if (override_timeout_sec_ > 0.0 && elapsed > override_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "cmd_vel_override timed out (%.2fs); releasing override safely.", elapsed);
        override_active_ = false;
        override_cmd_vel_ = geometry_msgs::msg::Twist();
        publishActiveCommandLocked();
      } else {
        publishActiveCommandLocked();
      }
    } else if (align_wz_active_) {
      const double elapsed =
        std::chrono::duration<double>(now_steady - last_align_wz_time_).count();
      if (override_timeout_sec_ > 0.0 && elapsed > override_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "cmd_align_wz timed out (%.2fs); releasing align wz safely.", elapsed);
        align_wz_active_ = false;
        align_wz_ = 0.0f;
        publishActiveCommandLocked();
      } else if (
        (rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT)
      {
        publishActiveCommandLocked();
      }
    }
  }

  const auto stamp = this->get_clock()->now();
  if (stamp.nanoseconds() == 0) {
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -angle_snapshot);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist, float yaw_diff, bool use_spin_speed) const
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = use_spin_speed ? (twist.angular.z + spin_speed_) : twist.angular.z;
  aft_tf_vel.linear.x = twist.linear.x * cos(yaw_diff) + twist.linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist.linear.x * sin(yaw_diff) + twist.linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
