#include <iostream>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <lidar_processing.hpp>

namespace model_fitting {
void box_filter(const PointCloud::ConstPtr& input,
                const PointCloud::Ptr& output,
                const Point& center,
                const std::vector<float>& cube_side_length) {
  pcl::CropBox<pcl::PointXYZI> box_filter;
  double x1 = center[0] - cube_side_length[0] / 2.;
  double x2 = center[0] + cube_side_length[0] / 2.;
  double y1 = center[1] - cube_side_length[1] / 2.;
  double y2 = center[1] + cube_side_length[1] / 2.;
  double z1 = center[2] - cube_side_length[2] / 2.;
  double z2 = center[2] + cube_side_length[2] / 2.;
  box_filter.setMin(Eigen::Vector4f(x1, y1, z1, 1.0));
  box_filter.setMax(Eigen::Vector4f(x2, y2, z2, 1.0));
  box_filter.setInputCloud(input);
  box_filter.setNegative(false);
  box_filter.filter(*output);
}


void intensity_filter(float low_bound, float upper_bound,
                      const PointCloud::ConstPtr& input,
                      const PointCloud::Ptr& output) {
  pcl::PassThrough<pcl::PointXYZI> pass;

  // range settings
  pass.setInputCloud(input);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(low_bound, upper_bound);
  pass.filter(*output);
}


void plane_filter(const PointCloud::ConstPtr& input,
                  const PointCloud::Ptr& output,
                  const double distance_threshold,
                  const bool non_plane) {
  // Plane-Fitting
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);

  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }

  PointCloud::Ptr plane_filtered_points(new PointCloud);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setNegative(non_plane);
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.filter(*output);
}


Eigen::Vector4f Find_Normal(const PointCloud::ConstPtr& input) {
  extern double ce_length;
  Eigen::Vector4f plane_params;
  pcl::PointCloud<pcl::PointXYZ>::Ptr normal_cloud(
                                      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*input, *normal_cloud);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  ne.setInputCloud(normal_cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
                                     new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius $(ce_length)m
  ne.setRadiusSearch(ce_length);

  // Compute the features, and choose only one of normals as output
  ne.compute(*normals);
  plane_params[0] = normals->points[0].normal_x;
  plane_params[1] = normals->points[0].normal_y;
  plane_params[2] = normals->points[0].normal_z;
  plane_params[3] = normals->points[0].curvature;
  return plane_params;
}


void edge_extract(const PointCloud::ConstPtr& input,
                  const PointCloud::Ptr& output,
                  const double resolution) {
  // Estimate rotation matrix such that the plane points facing the x-y plane
  Eigen::Vector4f normal = Find_Normal(input);
  if (!std::isfinite(normal[0])) {
    std::cout << "Cannot esitimate an available normal. Ignore the case!"
              << std::endl;
    return;
  }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*input, centroid);

  Eigen::Vector4f v_Z{0, 0, 1, 0};
  Vector3 normal_(normal[0], normal[1], normal[2]);
  Vector3 v_Z_(v_Z[0], v_Z[1], v_Z[2]);
  Vector3 k_ = normal_.cross(v_Z_);

  double alpha = std::acos(normal_.dot(v_Z_)/(normal_.norm()*v_Z_.norm()));

  // Compute one point at the intersection of board plane and X-Y plane
  Vector3 m_ = normal_.cross(k_);
  double d = (0 - centroid[2])/m_[2];
  Point rot_point(0, 0, 0);
  rot_point[0] = centroid[0] + m_[0] * d;
  rot_point[1] = centroid[1] + m_[1] * d;

  // Rotate an angle alpha alone the intersection of board plane and X-Y plane
  Eigen::Matrix4f rot_matrix = rot_mat(rot_point, k_.normalized(), alpha);

  // Sort points by its azimuth, and choose the farest points as edge points
  PointCloud::Ptr transformed_cloud(new PointCloud);
  pcl::transformPointCloud(*input, *transformed_cloud, rot_matrix);
  Eigen::Vector4f centroid_2d;
  pcl::compute3DCentroid(*transformed_cloud, centroid_2d);
  int div = 360 / resolution;
  double c_x = centroid_2d[0];
  double c_y = centroid_2d[1];
  std::vector<std::pair<int, double>> edge_points;
  double max_distance = 0;
  for (int i = 0; i < div; i++)
    edge_points.push_back(std::make_pair(-1, 0));
  for (int i = 0; i < input->points.size(); i++) {
    double x = transformed_cloud->points[i].x - c_x;
    double y = transformed_cloud->points[i].y - c_y;
    double deg = RadToDeg(std::atan2(y, x)) + 180;
    double distance = sqrt(x*x + y*y);
    int d = deg / resolution;
    if (edge_points[d].first == -1 || edge_points[d].second < distance) {
      edge_points[d].first = i;
      edge_points[d].second = distance;
      max_distance = std::max(max_distance, distance);
    }
  }

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto ep : edge_points) {
    if (ep.first != -1 && ep.second > max_distance/3)
      inliers->indices.push_back(ep.first);
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*output);
}
}  // namespace model_fitting
