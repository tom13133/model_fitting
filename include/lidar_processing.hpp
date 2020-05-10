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
// Copyright (C) 2020 Yu-Han, Hsueh <zero000.ece07g@nctu.edu.tw>
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

#include <model.hpp>
#include <types.hpp>

namespace model_fitting {
// Given a centroid, discard points outside certain size cube region
void box_filter(const PointCloud::ConstPtr& input,
                const PointCloud::Ptr& output,
                const Point& center,
                const std::vector<float>& cube_side_length);

// Given intesity low/upper bound, discard points outside the bound
void intensity_filter(float low_bound,
                      float upper_bound,
                      const PointCloud::ConstPtr& input,
                      const PointCloud::Ptr& output);

// Given input cloud, output plane cloud
void plane_filter(const PointCloud::ConstPtr& input,
                  const PointCloud::Ptr& output,
                  const double distance_threshold,
                  const bool non_plane);

// Find the normal vector or triangle board
Eigen::Vector4f Find_Normal(const PointCloud::ConstPtr& input);

// Extract edge points of a plane
void edge_extract(const PointCloud::ConstPtr& input,
                  const PointCloud::Ptr& output,
                  const double resolution);
}  // namespace model_fitting
