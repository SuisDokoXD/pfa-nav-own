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

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "voxel_clearance/clearance_grid.hpp"

namespace voxel_clearance
{

class VoxelClearanceNode : public rclcpp::Node
{
public:
  VoxelClearanceNode()
  : Node("voxel_clearance")
  {
    declareAndLoadParameters();
    grid_.setParams(params_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    pub_terrain_local_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "terrain_map", rclcpp::QoS(5).reliable());
    pub_terrain_ext_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "terrain_map_ext", rclcpp::QoS(5).reliable());
    pub_low_clearance_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "low_clearance_map", rclcpp::QoS(2).reliable());
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "clearance_markers", rclcpp::QoS(2).reliable());

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 5,
      std::bind(&VoxelClearanceNode::odomCallback, this, std::placeholders::_1));
    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, 5,
      std::bind(&VoxelClearanceNode::scanCallback, this, std::placeholders::_1));
    sub_lifter_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      lifter_state_topic_, 10,
      std::bind(&VoxelClearanceNode::lifterStateCallback, this, std::placeholders::_1));

    for (const auto & topic : extra_cloud_topics_) {
      if (topic.empty()) {
        continue;
      }
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          this->extraCloudCallback(msg);
        });
      extra_subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "Subscribed extra LiDAR cloud topic: %s", topic.c_str());
    }

    dyn_param_handle_ = this->add_on_set_parameters_callback(
      std::bind(&VoxelClearanceNode::onParametersSet, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&VoxelClearanceNode::publishLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "voxel_clearance started: xy_res=%.2fm z_res=%.2fm max_slope=%.1fdeg "
      "max_step=%.2fm project_output_z=%s",
      params_.xy_resolution, params_.z_resolution, params_.max_traversable_slope_deg,
      params_.max_step_height, project_output_z_ ? "true" : "false");
  }

private:
  void declareAndLoadParameters()
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "lidar_odometry");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "registered_scan");
    lifter_state_topic_ = declare_parameter<std::string>("lifter_state_topic", "/lifter_state");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    extra_cloud_topics_ =
      declare_parameter<std::vector<std::string>>("extra_cloud_topics", std::vector<std::string>{});

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    local_radius_ = declare_parameter<double>("local_radius", 5.0);
    ext_radius_ = declare_parameter<double>("ext_radius", 10.0);
    project_output_z_ = declare_parameter<bool>("project_output_z", true);
    output_projected_z_ = declare_parameter<double>("output_projected_z", 0.25);
    obstacle_intensity_ = declare_parameter<double>("obstacle_intensity", 1.0);
    free_intensity_ = declare_parameter<double>("free_intensity", 0.0);
    treat_low_clearance_as_obstacle_ =
      declare_parameter<bool>("treat_low_clearance_as_obstacle", true);
    freeze_during_lift_ = declare_parameter<bool>("freeze_during_lift", true);
    lidar_z_offset_high_ = declare_parameter<double>("lidar_z_offset_high", 0.256);
    lidar_z_offset_low_ = declare_parameter<double>("lidar_z_offset_low", 0.06421);
    current_lidar_z_offset_ = lidar_z_offset_high_;
    publish_markers_ = declare_parameter<bool>("publish_markers", true);

    params_.xy_resolution = declare_parameter<double>("xy_resolution", params_.xy_resolution);
    params_.z_resolution = declare_parameter<double>("z_resolution", params_.z_resolution);
    params_.grid_radius = declare_parameter<double>("grid_radius", ext_radius_);
    params_.decay_time = declare_parameter<double>("decay_time", params_.decay_time);
    params_.low_clearance_latch_time =
      declare_parameter<double>("low_clearance_latch_time", params_.low_clearance_latch_time);
    params_.min_rel_z = declare_parameter<double>("min_rel_z", params_.min_rel_z);
    params_.max_rel_z = declare_parameter<double>("max_rel_z", params_.max_rel_z);
    params_.dis_ratio_z = declare_parameter<double>("dis_ratio_z", params_.dis_ratio_z);
    params_.self_filter_half_x =
      declare_parameter<double>("self_filter_half_x", params_.self_filter_half_x);
    params_.self_filter_half_y =
      declare_parameter<double>("self_filter_half_y", params_.self_filter_half_y);
    params_.self_filter_min_z =
      declare_parameter<double>("self_filter_min_z", params_.self_filter_min_z);
    params_.self_filter_max_z_margin =
      declare_parameter<double>("self_filter_max_z_margin", params_.self_filter_max_z_margin);
    params_.cluster_z_gap = declare_parameter<double>("cluster_z_gap", params_.cluster_z_gap);
    params_.max_ground_thickness =
      declare_parameter<double>("max_ground_thickness", params_.max_ground_thickness);
    params_.min_ground_points =
      declare_parameter<int>("min_ground_points", params_.min_ground_points);
    params_.min_overhead_points =
      declare_parameter<int>("min_overhead_points", params_.min_overhead_points);
    params_.max_traversable_slope_deg =
      declare_parameter<double>("max_traversable_slope_deg", params_.max_traversable_slope_deg);
    params_.max_step_height =
      declare_parameter<double>("max_step_height", params_.max_step_height);
    params_.despeckle_isolated_obstacles = declare_parameter<bool>(
      "despeckle_isolated_obstacles", params_.despeckle_isolated_obstacles);
    params_.min_plane_fit_neighbors =
      declare_parameter<int>("min_plane_fit_neighbors", params_.min_plane_fit_neighbors);
    params_.min_obstacle_confirm_frames =
      declare_parameter<int>("min_obstacle_confirm_frames", params_.min_obstacle_confirm_frames);
    params_.robot_height_high =
      declare_parameter<double>("robot_height_high", params_.robot_height_high);
    params_.robot_height_low =
      declare_parameter<double>("robot_height_low", params_.robot_height_low);
    params_.clearance_margin_high =
      declare_parameter<double>("clearance_margin_high", params_.clearance_margin_high);
    params_.clearance_margin_low =
      declare_parameter<double>("clearance_margin_low", params_.clearance_margin_low);
  }

  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto & p : parameters) {
      const auto & name = p.get_name();
      if (name == "max_traversable_slope_deg") {
        params_.max_traversable_slope_deg = p.as_double();
      } else if (name == "max_ground_thickness") {
        params_.max_ground_thickness = p.as_double();
      } else if (name == "max_step_height") {
        params_.max_step_height = p.as_double();
      } else if (name == "cluster_z_gap") {
        params_.cluster_z_gap = p.as_double();
      } else if (name == "decay_time") {
        params_.decay_time = p.as_double();
      } else if (name == "robot_height_high") {
        params_.robot_height_high = p.as_double();
      } else if (name == "robot_height_low") {
        params_.robot_height_low = p.as_double();
      } else if (name == "clearance_margin_high") {
        params_.clearance_margin_high = p.as_double();
      } else if (name == "clearance_margin_low") {
        params_.clearance_margin_low = p.as_double();
      } else if (name == "treat_low_clearance_as_obstacle") {
        treat_low_clearance_as_obstacle_ = p.as_bool();
      } else if (name == "project_output_z") {
        project_output_z_ = p.as_bool();
      } else if (name == "output_projected_z") {
        output_projected_z_ = p.as_double();
      }
    }
    grid_.setParams(params_);

    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    const double base_z = msg->pose.pose.position.z - current_lidar_z_offset_;
    grid_.updateRobotPose(robot_x_, robot_y_, base_z);
    has_odom_ = true;
  }

  void lifterStateCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 2) {
      return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    const int pos = msg->data[0];           // 0=TOP, 1=BOTTOM, 2=MIDDLE
    const bool transformable = (msg->data[1] != 0);
    lifter_transformable_ = transformable;
    lifter_pos_ = pos;

    if (pos == 1) {
      current_lidar_z_offset_ = lidar_z_offset_low_;
    } else if (pos == 0) {
      current_lidar_z_offset_ = lidar_z_offset_high_;
    }

    if (freeze_during_lift_) {
      grid_.setFrozen(!transformable);
    }
  }

  void insertPointCloudMsg(const sensor_msgs::msg::PointCloud2 & cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_msg, pcl_cloud);

    std::vector<Point3D> pts;
    pts.reserve(pcl_cloud.points.size());
    for (const auto & p : pcl_cloud.points) {
      pts.push_back({p.x, p.y, p.z});
    }

    const double stamp_sec = rclcpp::Time(cloud_msg.header.stamp).seconds();
    grid_.insertPoints(pts, stamp_sec);
    last_cloud_stamp_ = cloud_msg.header.stamp;
    last_cloud_time_sec_ = stamp_sec;
    has_cloud_ = true;
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_odom_) {
      return;
    }
    if (msg->header.frame_id == odom_frame_ || msg->header.frame_id.empty()) {
      insertPointCloudMsg(*msg);
      return;
    }
    try {
      auto tf = tf_buffer_->lookupTransform(
        odom_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(*msg, transformed, tf);
      transformed.header.frame_id = odom_frame_;
      insertPointCloudMsg(transformed);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to transform scan to %s: %s",
        odom_frame_.c_str(), ex.what());
    }
  }

  void extraCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_odom_) {
      return;
    }
    if (msg->header.frame_id == odom_frame_) {
      insertPointCloudMsg(*msg);
      return;
    }
    try {
      auto tf = tf_buffer_->lookupTransform(
        odom_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(*msg, transformed, tf);
      transformed.header.frame_id = odom_frame_;
      insertPointCloudMsg(transformed);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to transform extra cloud (%s) to %s: %s",
        msg->header.frame_id.c_str(), odom_frame_.c_str(), ex.what());
    }
  }

  void publishLoop()
  {
    std::vector<ClassifiedCell> cells;
    rclcpp::Time stamp;
    double now_sec = 0.0;
    double robot_x = 0.0;
    double robot_y = 0.0;
    int center_ix = 0;
    int center_iy = 0;
    int grid_width = 0;
    int half_width = 0;
    double xy_res = 0.1;
    bool treat_low_as_obs = true;

    int min_confirm = 1;
    int cycle_count = 0;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_odom_ || !has_cloud_) {
        return;
      }
      now_sec = last_cloud_time_sec_;
      grid_.classify(now_sec);
      ++classify_cycles_;
      cycle_count = classify_cycles_;
      min_confirm = std::max(1, grid_.params().min_obstacle_confirm_frames);
      cells = grid_.classifiedCells();
      stamp = last_cloud_stamp_;
      robot_x = robot_x_;
      robot_y = robot_y_;
      center_ix = grid_.centerIx();
      center_iy = grid_.centerIy();
      grid_width = grid_.gridWidth();
      half_width = grid_.halfWidth();
      xy_res = grid_.params().xy_resolution;
      // Escalate LOW_CLEARANCE to obstacle if explicitly configured OR if lifter jammed in HIGH.
      treat_low_as_obs =
        treat_low_clearance_as_obstacle_ || (!lifter_transformable_ && lifter_pos_ == 0);
    }

    // Wait for temporal-confirmation warmup so slam_toolbox's initial stationary scan
    // already contains real confirmed walls instead of an empty all-inf scan (0x0 /map).
    if (cycle_count < min_confirm) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> local_cloud;
    pcl::PointCloud<pcl::PointXYZI> ext_cloud;
    local_cloud.points.reserve(cells.size());
    ext_cloud.points.reserve(cells.size());

    const double local_r_sq = local_radius_ * local_radius_;
    const double ext_r_sq = ext_radius_ * ext_radius_;

    for (const auto & c : cells) {
      const double dx = c.x - robot_x;
      const double dy = c.y - robot_y;
      const double r_sq = dx * dx + dy * dy;
      if (r_sq > ext_r_sq) {
        continue;
      }

      const bool is_obstacle =
        (c.classification == CellClass::HARD_OBSTACLE) ||
        (treat_low_as_obs && c.classification == CellClass::LOW_CLEARANCE);

      pcl::PointXYZI pt;
      pt.x = static_cast<float>(c.x);
      pt.y = static_cast<float>(c.y);
      pt.z = static_cast<float>(project_output_z_ ? output_projected_z_ : c.surface_z);
      pt.intensity = static_cast<float>(is_obstacle ? obstacle_intensity_ : free_intensity_);

      ext_cloud.points.push_back(pt);
      if (r_sq <= local_r_sq) {
        local_cloud.points.push_back(pt);
      }
    }

    sensor_msgs::msg::PointCloud2 local_msg;
    pcl::toROSMsg(local_cloud, local_msg);
    local_msg.header.stamp = stamp;
    local_msg.header.frame_id = odom_frame_;
    pub_terrain_local_->publish(local_msg);

    sensor_msgs::msg::PointCloud2 ext_msg;
    pcl::toROSMsg(ext_cloud, ext_msg);
    ext_msg.header.stamp = stamp;
    ext_msg.header.frame_id = odom_frame_;
    pub_terrain_ext_->publish(ext_msg);

    // Publish 2D low_clearance_map OccupancyGrid for downstream lifter supervisor
    nav_msgs::msg::OccupancyGrid lc_map;
    lc_map.header.stamp = stamp;
    lc_map.header.frame_id = odom_frame_;
    lc_map.info.resolution = static_cast<float>(xy_res);
    lc_map.info.width = static_cast<uint32_t>(grid_width);
    lc_map.info.height = static_cast<uint32_t>(grid_width);
    const int min_ix = center_ix - half_width;
    const int min_iy = center_iy - half_width;
    lc_map.info.origin.position.x = static_cast<double>(min_ix) * xy_res;
    lc_map.info.origin.position.y = static_cast<double>(min_iy) * xy_res;
    lc_map.info.origin.position.z = 0.0;
    lc_map.info.origin.orientation.w = 1.0;
    lc_map.data.assign(static_cast<size_t>(grid_width * grid_width), -1);

    for (const auto & c : cells) {
      const int gx = c.world_ix - min_ix;
      const int gy = c.world_iy - min_iy;
      if (gx < 0 || gx >= grid_width || gy < 0 || gy >= grid_width) {
        continue;
      }
      const size_t idx = static_cast<size_t>(gy * grid_width + gx);
      lc_map.data[idx] = (c.classification == CellClass::LOW_CLEARANCE) ? 100 : 0;
    }
    pub_low_clearance_map_->publish(lc_map);

    // Publish true-3D color-coded RViz markers when subscribed
    if (publish_markers_ && pub_markers_->get_subscription_count() > 0) {
      publishMarkers(cells, stamp, xy_res);
    }
  }

  void publishMarkers(
    const std::vector<ClassifiedCell> & cells, const rclcpp::Time & stamp, double xy_res)
  {
    auto makeCubeList = [&](int id, const char * ns, float r, float g, float b, float z_scale) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = odom_frame_;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = xy_res * 0.9;
      m.scale.y = xy_res * 0.9;
      m.scale.z = z_scale;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.75F;
      return m;
    };

    auto free_m = makeCubeList(0, "free", 0.2F, 0.85F, 0.3F, 0.03F);
    auto low_m = makeCubeList(1, "low_clearance", 0.95F, 0.85F, 0.1F, 0.08F);
    auto hard_m = makeCubeList(2, "hard_obstacle", 0.95F, 0.2F, 0.2F, 0.25F);

    for (const auto & c : cells) {
      geometry_msgs::msg::Point p;
      p.x = c.x;
      p.y = c.y;
      if (c.classification == CellClass::FREE) {
        p.z = c.surface_z;
        free_m.points.push_back(p);
      } else if (c.classification == CellClass::LOW_CLEARANCE) {
        p.z = c.surface_z + 0.04;
        low_m.points.push_back(p);
      } else if (c.classification == CellClass::HARD_OBSTACLE) {
        p.z = c.surface_z + 0.125;
        hard_m.points.push_back(p);
      }
    }

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(std::move(free_m));
    arr.markers.push_back(std::move(low_m));
    arr.markers.push_back(std::move(hard_m));
    pub_markers_->publish(arr);
  }

  std::mutex mtx_;
  ClearanceParams params_;
  ClearanceGrid grid_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string lifter_state_topic_;
  std::string odom_frame_;
  std::vector<std::string> extra_cloud_topics_;

  double publish_rate_hz_ = 10.0;
  double local_radius_ = 5.0;
  double ext_radius_ = 10.0;
  bool project_output_z_ = true;
  double output_projected_z_ = 0.25;
  double obstacle_intensity_ = 1.0;
  double free_intensity_ = 0.0;
  bool treat_low_clearance_as_obstacle_ = true;
  bool freeze_during_lift_ = true;
  bool publish_markers_ = true;

  double lidar_z_offset_high_ = 0.256;
  double lidar_z_offset_low_ = 0.06421;
  double current_lidar_z_offset_ = 0.256;
  int lifter_pos_ = 0;
  bool lifter_transformable_ = true;

  bool has_odom_ = false;
  bool has_cloud_ = false;
  int classify_cycles_ = 0;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  double last_cloud_time_sec_ = 0.0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_lifter_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> extra_subs_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_local_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_ext_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_low_clearance_map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_param_handle_;
};

}  // namespace voxel_clearance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voxel_clearance::VoxelClearanceNode>());
  rclcpp::shutdown();
  return 0;
}
