////////////////////////////////////////////////////////////////////////////////
//
// Filename:      LidarProcessor.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// This file defines a class LidarProcessor to subscribe point cloud and
// estimate the target center with time stamp.
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2020 Yu-Han, Hsueh <zero000.ece07g@nctu.edu.tw>
//
// This file is part of {model_fitting}.
//
// {model_fitting} can not be copied and/or distributed without the express
// permission of {Yu-Han, Hsueh}
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO(Yu-Han): Consider the specification parameters should be passed
//               by function or use extern declaration directly.
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <kalman_filter.hpp>
#include <lidar_processing.hpp>
#include <model.hpp>

namespace model_fitting {
// Specification
const char* m_type;  // triangle or square
double ce_length;  // length from center to endpoints
double depth;  // depth of center compensation

// class LidarProcessor subscribe point cloud, then publish processed data
class LidarProcessor {
 public:
  explicit LidarProcessor(ros::NodeHandle* nh);

  ros::NodeHandle* p_nh;

  // Publisher
  ros::Publisher filtered_points_pub;
  ros::Publisher plane_points_pub;
  ros::Publisher target_points_pub;
  ros::Publisher edge_points_pub;

  ros::Publisher centroid_pub;
  ros::Publisher cube_pub;
  ros::Publisher model_centroid_pub;
  ros::Publisher model_pub;
  ros::Publisher normal_pub;

  // Subscriber
  ros::Subscriber lidar_sub;

  // callback function: subscribe original PointCloud and process
  void cb_lidar(const sensor_msgs::PointCloud2& msg);

  ~LidarProcessor();

 private:
  std::vector<float> center;
  std::vector<float> side_length;
  std::vector<float> lower_upper_bound;

  std::string pkg_path;
  std::ofstream outfile_l;

  std::string topic_name_lidar;
  std::string topic_frame_lidar;

  std::vector<int> cloud_size;

  double edge_points_resolution;

  KalmanFilter kf;
};

void kf_init(KalmanFilter& kf, const Vector3& x_y_phi, const double time);
}  // namespace model_fitting
