////////////////////////////////////////////////////////////////////////////////
//
// Filename:      LidarProcessor_node.cpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////
//
// This node create class LidarProcessor to estimate target center from
// point cloud
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
//
////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <LidarProcessor.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "LidarProcessor_node");
  ros::NodeHandle nh("~");
  model_fitting::LidarProcessor p(&nh);

  // We can change r to adjust the dense of output data
  ros::Rate r(1);
  while (ros::ok()) {
    ros::spinOnce();
  }
}
