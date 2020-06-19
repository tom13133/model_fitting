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

#include <tf/transform_broadcaster.h>
#include <esr_msgs/Track.h>

namespace model_fitting {

struct measurement{
  int id_;
  double time_;
  Point point_;
  double intensity_;
  double range_rate_;
  Vector3 v_;
  bool valid_;
};

typedef std::vector<measurement> measurements;


// Specification
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
  ros::Publisher filtered_points_pub;
  ros::Publisher plane_points_pub;
  ros::Publisher target_points_pub;
  ros::Publisher edge_points_pub;

  ros::Publisher centroid_pub;
  ros::Publisher cube_pub;
  ros::Publisher model_centroid_pub;
  ros::Publisher model_pub;
  ros::Publisher normal_pub;

  ros::Publisher rcs_pub;
  // Subscriber
  ros::Subscriber lidar_sub;
  ros::Subscriber radar_sub;

  // callback function: subscribe original PointCloud and process
  void cb_lidar(const sensor_msgs::PointCloud2& msg);
  void cb_radar(const esr_msgs::Track &msg);
  ~LidarProcessor();
  void saveCorrespondences();

 private:
  std::vector<float> center;
  std::vector<float> side_length;
  std::vector<float> lower_upper_bound;

  std::string pkg_path;
  std::ofstream outfile_l;

  std::string topic_name_lidar;
  std::string topic_name_radar;
  std::string topic_frame_lidar;
  std::string topic_frame_radar;

  std::vector<int> cloud_size;

  double edge_points_resolution;

  KalmanFilter kf;

  Pose T_lr;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  std::vector<measurements> radar_data;
  std::vector<measurements> lidar_data;
  std::vector<measurement> searching_centers;
  std::vector<float> searching_region;
  std::vector<esr_msgs::Track> radar_buffer;
  std::string sid;
};

void kf_init(KalmanFilter& kf, const Vector3& x_y_phi, const double time);
}  // namespace model_fitting
