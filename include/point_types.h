////////////////////////////////////////////////////////////////////////////////
//
// Filename:      point_types.h
// Authors:       YuHan-XUE
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// Define a custom PointT type to extend the ability to store ring info. of
// Veledyne Lidar
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2018 Yuhan,XUE <zero000.eed03@nctu.edu.tw>
//
// This file is part of {model_fitting}.
//
// {model_fitting} can not be copied and/or distributed without the express
// permission of {Yuhan,XUE}
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#define PCL_NO_PRECOMPILE

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/point_types.h>

namespace velodyne_pointcloud {
/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;
};  // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> PointCloud;
#endif  // __VELODYNE_POINTCLOUD_POINT_TYPES_H
