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
// Copyright (C) 2020 Yu-Han, Hsueh <zero000.ece07g@nctu.edu.tw>
//
// This file is part of {model_fitting}.
//
// {model_fitting} can not be copied and/or distributed without the express
// permission of {Yu-Han, Hsueh}
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sophus/se3.hpp>

namespace model_fitting {

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

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
inline double Distance(const Point& p1, const Point& p2) {
  return (p1 - p2).norm();
}

inline double DegToRad(const double deg) {
  return deg * M_PI / 180.;
}

inline double RadToDeg(const double rad) {
  return rad * 180. / M_PI;
}

}  // namespace model_fitting
