/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <waypoint_planner/velocity_set/velocity_set_info.h>

#include <Eigen/Geometry>
#include <pcl_ros/transforms.h>

#include <glog/logging.h>

void joinPoints(const pcl::PointCloud<pcl::PointXYZ>& points1, pcl::PointCloud<pcl::PointXYZ>* points2)
{
  for (const auto& p : points1)
  {
    points2->push_back(p);
  }
}

VelocitySetInfo::VelocitySetInfo()
  : stop_range_(1.3),
    deceleration_range_(0),
    points_threshold_(10),
    detection_height_top_(0.2),
    detection_height_bottom_(-1.7),
    stop_distance_obstacle_(10),
    stop_distance_stopline_(5),
    deceleration_obstacle_(0.8),
    deceleration_stopline_(0.6),
    velocity_change_limit_(2.77),
    temporal_waypoints_size_(100),
    wpidx_detectionResultByOtherNodes_(-1),
    base_to_front_(0.0),
    base_to_rear_(0.0),
    base_to_left_(0.0),
    base_to_right_(0.0),
    tangent_value_(0.0),
    publish_obstacle_pcl_(false),
    set_pose_(false),
    tf_lidar_to_baselink_(),
    pcl_for_obstacle_pub_(),
    pcl2_for_obstacle_pub_(),
    first_pnts_mtx_(),
    second_pnts_mtx_()
{
  ros::NodeHandle private_nh_("~");
  ros::NodeHandle nh;

  double vel_change_limit_kph = 9.972;
  private_nh_.param<double>("remove_points_upto", remove_points_upto_, 2.3);
  private_nh_.param<double>("stop_distance_obstacle", stop_distance_obstacle_, 10.0);
  private_nh_.param<double>("stop_distance_stopline", stop_distance_stopline_, 5.0);
  private_nh_.param<double>("detection_range", stop_range_, 1.3);
  private_nh_.param<int>("points_threshold", points_threshold_, 10);
  private_nh_.param<double>("detection_height_top", detection_height_top_, 0.2);
  private_nh_.param<double>("detection_height_bottom", detection_height_bottom_, -1.7);
  private_nh_.param<double>("deceleration_obstacle", deceleration_obstacle_, 0.8);
  private_nh_.param<double>("deceleration_stopline", deceleration_stopline_, 0.6);
  private_nh_.param<double>("velocity_change_limit", vel_change_limit_kph, 9.972);
  private_nh_.param<double>("deceleration_range", deceleration_range_, 0);
  private_nh_.param<double>("temporal_waypoints_size", temporal_waypoints_size_, 100.0);

  // X. Parameter added later.
  CHECK(private_nh_.getParam("base_to_front", base_to_front_)) << "Parameter base_to_front cannot be read.";
  CHECK(private_nh_.getParam("base_to_rear", base_to_rear_)) << "Parameter base_to_rear cannot be read.";
  CHECK(private_nh_.getParam("base_to_left", base_to_left_)) << "Parameter base_to_left cannot be read.";
  CHECK(private_nh_.getParam("base_to_right", base_to_right_)) << "Parameter base_to_right cannot be read.";
  double visible_angle_from_x_axis;
  CHECK(private_nh_.getParam("visible_angle_from_x_axis", visible_angle_from_x_axis)) << "Parameter visible_angle_from_x_axis cannot be read.";
  tangent_value_ = std::tan(std::abs(visible_angle_from_x_axis));
  CHECK(private_nh_.getParam("publish_obstacle_pcl", publish_obstacle_pcl_)) << "Publish obstacle pcl cannot be read.";

  CHECK(private_nh_.getParam("wp_stop_x_thresh", wp_stop_x_thresh_)) << "Parameter wp_stop_x_thresh_ cannot be read";
  CHECK(private_nh_.getParam("wp_stop_y_thresh", wp_stop_y_thresh_)) << "Parameter wp_stop_y_thresh_ cannot be read";
  CHECK(private_nh_.getParam("wp_decel_x_thresh", wp_decel_x_thresh_)) << "Parameter wp_decel_x_thresh_ cannot be read";
  CHECK(private_nh_.getParam("wp_decel_y_thresh", wp_decel_y_thresh_)) << "Parameter wp_decel_y_thresh_ cannot be read";

  CHECK(private_nh_.getParam("second_lidar_pnts_max_range", second_lidar_pnts_max_range_)) 
    << "Parameter second_lidar_pnts_max_range cannot be read";
  CHECK(private_nh_.getParam("second_lidar_pnts_remove_up_to", second_lidar_pnts_remove_up_to_)) 
    << "Parameter second_lidar_pnts_remove_up_to cannot be read";

  // X. Load global paramers.
  {
    double tf_x, tf_y, tf_z;
    double tf_roll, tf_pitch, tf_yaw;
    CHECK(nh.getParam("tf_x", tf_x)) << "Parameter tf_x cannot be read.";
    CHECK(nh.getParam("tf_y", tf_y)) << "Parameter tf_y cannot be read.";
    CHECK(nh.getParam("tf_z", tf_z)) << "Parameter tf_z cannot be read.";
    CHECK(nh.getParam("tf_roll", tf_roll)) << "Parameter tf_roll cannot be read.";
    CHECK(nh.getParam("tf_pitch", tf_pitch)) << "Parameter tf_pitch cannot be read.";
    CHECK(nh.getParam("tf_yaw", tf_yaw)) << "Parameter tf_yaw cannot be read.";

    Eigen::Quaternionf rot_baselink_to_lidar = 
      Eigen::AngleAxisf(tf_yaw, Eigen::Vector3f::UnitZ()) * 
      Eigen::AngleAxisf(tf_pitch, Eigen::Vector3f::UnitY()) * 
      Eigen::AngleAxisf(tf_roll, Eigen::Vector3f::UnitX());

    tf_lidar_to_baselink_.setOrigin(tf::Vector3(tf_x, tf_y, tf_z));
    tf_lidar_to_baselink_.setRotation(
        tf::Quaternion(rot_baselink_to_lidar.x(), rot_baselink_to_lidar.y(), 
                      rot_baselink_to_lidar.z(), rot_baselink_to_lidar.w()));
  }

  {
    double tf_x, tf_y, tf_z;
    double tf_roll, tf_pitch, tf_yaw;
    CHECK(nh.getParam("tf_x2", tf_x)) << "Parameter tf_x2 cannot be read.";
    CHECK(nh.getParam("tf_y2", tf_y)) << "Parameter tf_y2 cannot be read.";
    CHECK(nh.getParam("tf_z2", tf_z)) << "Parameter tf_z2 cannot be read.";
    CHECK(nh.getParam("tf_roll2", tf_roll)) << "Parameter tf_roll2 cannot be read.";
    CHECK(nh.getParam("tf_pitch2", tf_pitch)) << "Parameter tf_pitch2 cannot be read.";
    CHECK(nh.getParam("tf_yaw2", tf_yaw)) << "Parameter tf_yaw2 cannot be read.";

    Eigen::Quaternionf rot_baselink_to_lidar = 
      Eigen::AngleAxisf(tf_yaw, Eigen::Vector3f::UnitZ()) * 
      Eigen::AngleAxisf(tf_pitch, Eigen::Vector3f::UnitY()) * 
      Eigen::AngleAxisf(tf_roll, Eigen::Vector3f::UnitX());

    tf_2nd_lidar_to_baselink_.setOrigin(tf::Vector3(tf_x, tf_y, tf_z));
    tf_2nd_lidar_to_baselink_.setRotation(
        tf::Quaternion(rot_baselink_to_lidar.x(), rot_baselink_to_lidar.y(), 
                      rot_baselink_to_lidar.z(), rot_baselink_to_lidar.w()));
  }

  velocity_change_limit_ = vel_change_limit_kph / 3.6;  // kph -> mps

  if (publish_obstacle_pcl_) {
    pcl_for_obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pcl_for_obstacle", 1);
    pcl2_for_obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pcl2_for_obstacle", 1);
  }

  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh,private_nh_);
  health_checker_ptr_->ENABLE();
}

VelocitySetInfo::~VelocitySetInfo()
{
}

void VelocitySetInfo::clearPoints()
{
  std::lock_guard<std::mutex> lock1(first_pnts_mtx_);
  std::lock_guard<std::mutex> lock2(second_pnts_mtx_);

  points_in_baselink_.clear();
  points_in_localizer_.clear();
  first_points_in_baselink_.clear();
  second_points_in_baselink_.clear();
}

void VelocitySetInfo::configCallback(const autoware_config_msgs::ConfigVelocitySetConstPtr &config)
{
  stop_distance_obstacle_ = config->stop_distance_obstacle;
  stop_distance_stopline_ = config->stop_distance_stopline;
  stop_range_ = config->detection_range;
  points_threshold_ = config->threshold_points;
  detection_height_top_ = config->detection_height_top;
  detection_height_bottom_ = config->detection_height_bottom;
  deceleration_obstacle_ = config->deceleration_obstacle;
  deceleration_stopline_ = config->deceleration_stopline;
  velocity_change_limit_ = config->velocity_change_limit / 3.6; // kmph -> mps
  deceleration_range_ = config->deceleration_range;
  temporal_waypoints_size_ = config->temporal_waypoints_size;
}

void VelocitySetInfo::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  health_checker_ptr_->CHECK_RATE("topic_rate_points_no_ground_slow", 8, 5, 1, "topic points_no_ground subscribe rate slow.");
  pcl::PointCloud<pcl::PointXYZ> sub_points, filtered_sub_points;
  pcl::fromROSMsg(*msg, sub_points);

  pcl::PointCloud<pcl::PointXYZ> sub_points_in_baselink, filtered_sub_points_in_baselink;
  sensor_msgs::PointCloud2 pcl_msg_in_baselink;
  pcl_ros::transformPointCloud("base_link", tf_lidar_to_baselink_, *msg, pcl_msg_in_baselink);
  pcl::fromROSMsg(pcl_msg_in_baselink, sub_points_in_baselink);

  for (size_t idx = 0; idx < sub_points.size(); idx++) {

    const pcl::PointXYZ &loc_p = sub_points[idx];
    const pcl::PointXYZ &base_p = sub_points_in_baselink[idx];
    bool valid = true;

    if (loc_p.x == 0 && loc_p.y == 0) {
      // X. Remove invalid.
      valid = false;
    } else if (loc_p.z > detection_height_top_ || loc_p.z < detection_height_bottom_) {
      // X. Height condition.
      valid = false;
    } else if (-base_to_rear_ < base_p.x && base_p.x < base_to_front_ && 
        -base_to_right_ < base_p.y && base_p.y < base_to_left_) {
      // X. Remove inside robot.
      valid = false;
    } else if (base_p.x < base_to_front_) {
      // X. Remove points from back.
      valid = false;
    } else if (tangent_value_ < std::abs(base_p.y / base_p.x)) {
      // X. Apply 60deg line.
      valid = false;
    } else if (loc_p.x * loc_p.x + loc_p.y * loc_p.y < remove_points_upto_ * remove_points_upto_) {
      valid = false;
    }

    if (valid) {
      filtered_sub_points.push_back(loc_p);
      filtered_sub_points_in_baselink.push_back(base_p);
    } else {
      sub_points[idx].x = std::numeric_limits<float>::quiet_NaN();
      sub_points[idx].y = std::numeric_limits<float>::quiet_NaN();
      sub_points[idx].z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  {
    std::lock_guard<std::mutex> lock(first_pnts_mtx_);
    points_in_localizer_ = filtered_sub_points;
    first_points_in_baselink_ = filtered_sub_points_in_baselink;
  }

  {
    std::lock_guard<std::mutex> lock1(first_pnts_mtx_);
    std::lock_guard<std::mutex> lock2(second_pnts_mtx_);
    points_in_baselink_.clear();
    points_in_baselink_ = first_points_in_baselink_;
    points_in_baselink_ += second_points_in_baselink_;
  }

  // X. Publish for debug.
  if (publish_obstacle_pcl_) {
    sensor_msgs::PointCloud2 pcl_msg_obstacle;
    pcl_msg_obstacle.header = msg->header;
    pcl::toROSMsg(sub_points, pcl_msg_obstacle);
    pcl_for_obstacle_pub_.publish(pcl_msg_obstacle);
  }
}

void VelocitySetInfo::secondPointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {

  pcl::PointCloud<pcl::PointXYZ> sub_points;
  pcl::fromROSMsg(*msg, sub_points);

  pcl::PointCloud<pcl::PointXYZ> sub_points_in_baselink, filtered_sub_points_in_baselink;
  sensor_msgs::PointCloud2 pcl_msg_in_baselink;
  pcl_ros::transformPointCloud("base_link", tf_2nd_lidar_to_baselink_, *msg, pcl_msg_in_baselink);
  pcl::fromROSMsg(pcl_msg_in_baselink, sub_points_in_baselink);

  for (size_t idx = 0; idx < sub_points.size(); idx++) {

    const pcl::PointXYZ &loc_p = sub_points[idx];
    const pcl::PointXYZ &base_p = sub_points_in_baselink[idx];
    bool valid = true;

    if (loc_p.x == 0 && loc_p.y == 0) {
      // X. Remove invalid.
      valid = false;
    } else if (second_lidar_pnts_max_range_ * second_lidar_pnts_max_range_ 
               < base_p.x * base_p.x + base_p.y * base_p.y) {
      // X. Remove far points.
      valid = false;
    } else if (-base_to_rear_ < base_p.x && base_p.x < base_to_front_ && 
        -base_to_right_ < base_p.y && base_p.y < base_to_left_) {
      // X. Remove inside robot.
      valid = false;
    } else if (base_p.x < base_to_front_) {
      // X. Remove points from back.
      valid = false;
    } else if (tangent_value_ < std::abs(base_p.y / base_p.x)) {
      // X. Apply 60deg line.
      valid = false;
    } else if (loc_p.x * loc_p.x + loc_p.y * loc_p.y < second_lidar_pnts_remove_up_to_ * second_lidar_pnts_remove_up_to_) {
      valid = false;
    }

    if (valid) {
      filtered_sub_points_in_baselink.push_back(base_p);
    } else {
      sub_points[idx].x = std::numeric_limits<float>::quiet_NaN();
      sub_points[idx].y = std::numeric_limits<float>::quiet_NaN();
      sub_points[idx].z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  {
    std::lock_guard<std::mutex> lock(second_pnts_mtx_);
    second_points_in_baselink_ = filtered_sub_points_in_baselink;
  }

  {
    std::lock_guard<std::mutex> lock1(first_pnts_mtx_);
    std::lock_guard<std::mutex> lock2(second_pnts_mtx_);
    points_in_baselink_.clear();
    points_in_baselink_ = first_points_in_baselink_;
    points_in_baselink_ += second_points_in_baselink_;
  }

  // X. Publish for debug.
  if (publish_obstacle_pcl_) {
    sensor_msgs::PointCloud2 pcl_msg_obstacle;
    pcl_msg_obstacle.header = msg->header;
    pcl::toROSMsg(sub_points, pcl_msg_obstacle);
    pcl2_for_obstacle_pub_.publish(pcl_msg_obstacle);
  }
}

void VelocitySetInfo::detectionCallback(const std_msgs::Int32 &msg)
{
    wpidx_detectionResultByOtherNodes_ = msg.data;
}

void VelocitySetInfo::controlPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  control_pose_ = *msg;

  if (!set_pose_)
    set_pose_ = true;
}

void VelocitySetInfo::localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  health_checker_ptr_->NODE_ACTIVATE();
  health_checker_ptr_->CHECK_RATE("topic_rate_localizer_pose_slow", 8, 5, 1, "topic localizer_pose subscribe rate slow.");
  localizer_pose_ = *msg;
}
