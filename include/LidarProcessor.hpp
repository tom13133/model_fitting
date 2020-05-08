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
// Copyright (C) 2019 Yu-Han, Hsueh <zero000.eed03@nctu.edu.tw>
//
// This file is part of {model_fitting}.
//
// {model_fitting} can not be copied and/or distributed without the express
// permission of {Yu-Han, Hsueh}
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO(Yu-Han): output 4 vertices of square board
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <model.hpp>
#include <lidar_processing.hpp>

namespace model_fitting {
// Specification
double horizontal_resolution;  // 0.16(deg)
double vertical_resolution;  // 1.33(deg)
const char* m_type;  // triangle or square
double ce_length;  // length from center to endpoints
double cr_length;
double depth;  // depth of corner reflector

// class LidarProcessor subscribe point cloud, then publish processed data
class LidarProcessor {
 public:
  explicit LidarProcessor(ros::NodeHandle* nh);

  ros::NodeHandle* p_nh;

  // Publisher
  ros::Publisher pass_filtered_points_pub;
  ros::Publisher plane_points_pub;
  ros::Publisher target_points_pub;
  ros::Publisher edge_points_pub;

  ros::Publisher centroid_pub;
  ros::Publisher model_centroid_pub;
  ros::Publisher model_pub;

  ros::Publisher normal_pub;

  ros::Publisher lidar_data_pub;

  // Subscriber
  ros::Subscriber lidar_sub;

  // callback function: subscribe original PointCloud and process
  void cb_lidar(const sensor_msgs::PointCloud2& msg);

  ~LidarProcessor();

 private:
  std::vector<float> center;
  std::vector<float> side_length;
  std::vector<float> lower_upper_bound;

  std::fstream topic_config;
  std::ofstream outfile_l;

  std::string topic_name_lidar;
  std::string topic_frame_lidar;

  std::vector<int> cloud_size;
};
}  // namespace model_fitting
