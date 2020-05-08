////////////////////////////////////////////////////////////////////////////////
//
// Filename:      lidar_processing.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// Process the original PointCloud using pcl_ros library
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
// TODO(Yu-Han):
//
////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <types.hpp>
#include <line.hpp>

namespace model_fitting {
// Given a centroid, discard points outside certain size cube region
void pass_filter(const Point& center,
                 const std::vector<float>& cube_side_length,
                 const PointCloud::ConstPtr& input,
                 const PointCloud::Ptr& output);

// Given intesity low/upper bound, discard points outside the bound
void intensity_filter(float low_bound,
                      float upper_bound,
                      const PointCloud::ConstPtr& input,
                      const PointCloud::Ptr& output);

// Given input cloud, output plane cloud
void plane_filter(const PointCloud::ConstPtr& input,
                  const PointCloud::Ptr& output);

// Compute distance but neglecting z axis information
double xy_distance(const Point& p1);

// Sort target cloud into lines by the information of rings
std::vector<LineData> line_classifier(const PointCloud::ConstPtr& input);

// Find the normal vector or triangle board
Eigen::Vector4f Find_Normal(const PointCloud::ConstPtr& input);

// Transform lines into a pointcloud
PointCloud::Ptr transform_to_pointcloud(const std::vector<LineData>& lines);
}  // namespace model_fitting
