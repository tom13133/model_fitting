#include <iostream>
#include <random>
#include <ransac.hpp>

namespace ransac_cpp {
Ransac::Ransac(const std::vector<Point>& data) {
  data_.assign(data.begin(), data.end());
  max_iterations_ = 100;
  probability_ = 0.99;
  threshold_ = 0.1;
  hypo_success = true;
  point_params_ = Vector3{0, 0, 0};
}

Ransac::~Ransac() {}
void Ransac::setMaxIterations(const int max_iterations) {
  max_iterations_ = max_iterations;
}

void Ransac::setProbability(const double probability) {
  probability_ = probability;
}

void Ransac::setDistanceThreshold(const double threshold) {
  threshold_ = threshold;
}

std::vector<int> Ransac::getInliers() {
  if (!hypo_success)
    inliers_.clear();
  return inliers_;
}

Vector3 Ransac::getModel() {
  return point_params_;
}

std::vector<int> Ransac::alsoInliers(const Point& point_params) {
  std::vector<int> res;
  for (int i = 0; i < data_.size(); i++) {
    if ((data_[i]-point_params).norm() < threshold_)
      res.push_back(i);
  }
  return res;
}

bool Ransac::computeModel() {
  int max_count = 0;
  std::uniform_int_distribution<unsigned> u(0, data_.size()-1);
  std::default_random_engine e;
  Point hypo_inliers;
  std::vector<int> temp_inliers, inliers;
  Point point_params;
  for (int i = 0; i < max_iterations_; i++) {
    int choice = u(e);
    hypo_inliers = data_[choice];

    point_params = hypo_inliers;
    temp_inliers = alsoInliers(point_params);
    if (temp_inliers.size() > max_count) {
      max_count = temp_inliers.size();
      inliers_ = temp_inliers;
      point_params_ = point_params;
    }
    if (inliers.size()/data_.size() > probability_) {
      return true;
    }
  }
  return false;
}
}  // namespace ransac_cpp
