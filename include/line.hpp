////////////////////////////////////////////////////////////////////////////////
//
// Filename:      line.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// Define a class(structure) to store point cloud
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
// TODO(Yu-Han): Consider function purify should be
//               in-class function or return the output
//
////////////////////////////////////////////////////////////////////////////////


#pragma once

#include <vector>
#include <utility>
#include <types.hpp>

namespace model_fitting {

class LineData {
 public:
  LineData() = default;
  explicit LineData(std::vector<std::pair<Point, int>> line);
  inline bool isEmpty() {
    if (line_.empty())
      return true;
    return false;
  }

  inline void add_point(Point p, int intensity) {
    line_.push_back(std::make_pair(p, intensity));
  }

  inline void erase_point(int idx) {
    line_.erase(line_.begin() + idx);
  }

  inline void reserve_point(int num) {
    line_.reserve(num);
  }

  inline void set_id(int id) {
    id_ = id;
  }

  inline void set_stamp(uint64_t stamp) {
    stamp_ = stamp;
  }

  inline std::vector<std::pair<Point, int>> get_line() const {
    return line_;
  }

  inline int get_id() const {
    return id_;
  }

  inline uint64_t get_stamp() const {
    return stamp_;
  }

  inline int get_size() const {
    return line_.size();
  }

  Point get_avg();  // compute the middle point of a line

  // apply a transform to all points
  void applyTransform(const Eigen::Matrix4f& rot_matrix);

  // reorder the points by x coordinate
  void reorder(int option);

 private:
  // Point -> point position; int -> point intensity
  std::vector<std::pair<Point, int>> line_;
  int id_;  // ring id
  uint64_t stamp_;
};

// Function to discard outliers
void purify(std::vector<LineData>& lines);

// Function to extract edge points
std::vector<LineData> edge_extract(const std::vector<LineData>& input);

// Function to compute the average of all lines
Point compute3Dcentroid(const std::vector<LineData>& lines);

// Function to apply a transform to all lines
std::vector<LineData> transformLines(const Eigen::Matrix4f& rot_matrix,
                                     const std::vector<LineData>& lines);
}  // namespace model_fitting
