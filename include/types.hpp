////////////////////////////////////////////////////////////////////////////////
//
// Filename:      types.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// Define class name make it easier to use
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2018 Yuhan,XUE <zero000.eed03@nctu.edu.tw>
//
// This file is part of {model_fitting}.
//
// {model_fitting} can not be copied and/or distributed without the express
// permission of {Yu-Han, Hsueh}
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO(Yuhan):
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sophus/se3.hpp>

#include <point_types.h>

namespace model_fitting {

// typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

typedef Eigen::AngleAxisd AngleAxis;
typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Rotation2Dd Rotation2D;
typedef Eigen::Translation3d Translation;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Quaternion Orientation;
typedef Vector3 Point;
typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;
typedef SE3 Pose;


// Point-to-Point distance
double Distance(const Point& p1, const Point& p2);

inline double DegToRad(const double deg) {
  return deg * M_PI / 180.;
}

inline double RadToDeg(const double rad) {
  return rad * 180. / M_PI;
}

}  // namespace model_fitting
