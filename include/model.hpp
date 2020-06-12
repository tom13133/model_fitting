////////////////////////////////////////////////////////////////////////////////
//
// Filename:      model.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// Define one model for fitting target board, then return the target centroid
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

#include <string>
#include <vector>

#include <visualization_msgs/Marker.h>

#include <types.hpp>

namespace model_fitting {
// Given a point with a vector, return the rotation matrix
// that rotate an angle t by this axis
Eigen::Matrix4f rot_mat(const Point& point,
                        const Vector3& vector,
                        const double t);

// Given a centroid and an angle phi,
// compute the position of given point after rotation
Vector2 rot_2D(const Vector2& centroid,
               const double phi,
               const Vector2& point);

// Given a model described as one Vector3 x_y_phi,
// return the endpoints of the polygon model
std::vector<Vector2> x_y_phi_2_triangle(const Vector3& x_y_phi);
std::vector<Vector2> x_y_phi_2_square(const Vector3& x_y_phi);

// Determine if one point is located inside the polygon model
bool IsPointInTriangle(const Vector2& point,
                       const std::vector<Vector2>& tri_points);
bool IsPointInSquare(const Vector2& point,
                       const std::vector<Vector2>& tri_points);

// Compute the distance between one point and the nearst model edge
double point_to_triangle_distance(const double* const x_y_phi,
                                  const Vector2& point);
double point_to_square_distance(const double* const x_y_phi,
                                  const Vector2& point);

// Project 3D-pointcloud into 2D, and return the triangle model
// described as x_y_phi
Vector3 model_fitting_2D(const std::vector<Vector2>& edge_points,
                         const Vector2& centroid,
                         double phi);


// Visualization
visualization_msgs::Marker mark_centroid(const Point& c,
                                         const Vector3& color,
                                         const std::string& lidar_frame);

visualization_msgs::Marker mark_cube(const Point& c,
                                     const std::vector<float>& side_length,
                                     const Vector3& color,
                                     const std::string& lidar_frame);

// Print polygon model on rviz given all endpoints
visualization_msgs::Marker print_Model(const std::vector<Point>& pts,
                                       const Vector3& color,
                                       const std::string& lidar_frame);

// Print computed normal of triangle model on rviz
visualization_msgs::Marker print_Normal(const Point& pt,
                                        const Eigen::Vector4f& plane_params,
                                        const std::string& lidar_frame,
                                        const double& scale);
}  // namespace model_fitting
