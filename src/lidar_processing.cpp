#include <iostream>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <lidar_processing.hpp>

namespace model_fitting {
void pass_filter(const Point& center,
                 const std::vector<float>& cube_side_length,
                 const PointCloud::ConstPtr& input,
                 const PointCloud::Ptr& output) {
  pcl::PassThrough<velodyne_pointcloud::PointXYZIR> pass;

  // range settings
  pass.setInputCloud(input);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(center[0] - cube_side_length[0] / 2,
                       center[0] + cube_side_length[0] / 2);
  pass.filter(*output);

  pass.setInputCloud(output);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(center[1] - cube_side_length[1] / 2,
                       center[1] + cube_side_length[1] / 2);
  pass.filter(*output);

  pass.setInputCloud(output);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(center[2] - cube_side_length[2] / 2,
                       center[2] + cube_side_length[2] / 2);
  pass.filter(*output);
}


void intensity_filter(float low_bound, float upper_bound,
                      const PointCloud::ConstPtr& input,
                      const PointCloud::Ptr& output) {
  pcl::PassThrough<velodyne_pointcloud::PointXYZIR> pass;

  // range settings
  pass.setInputCloud(input);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(low_bound, upper_bound);
  pass.filter(*output);
}


void plane_filter(const PointCloud::ConstPtr& input,
                  const PointCloud::Ptr& output) {
  // Plane-Fitting
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*input, *temp_points);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);

  seg.setInputCloud(temp_points);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }

  PointCloud::Ptr plane_filtered_points(new PointCloud);
  pcl::ExtractIndices<velodyne_pointcloud::PointXYZIR> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.filter(*output);
}


double xy_distance(const Point& p1) {
  const double dx = p1.x();
  const double dy = p1.y();
  return sqrt(dx * dx + dy * dy);
}


std::vector<LineData> line_classifier(const PointCloud::ConstPtr& input) {
  std::vector<LineData> lines;
  for (size_t i = 0; i < input->points.size(); ++i) {
    int pos = 0;
    if (lines.empty()) {
      LineData line;
      line.set_id(input->points[i].ring);
      line.set_stamp(input->header.stamp);
      lines.push_back(line);
      Point p_new(input->points[i].x,
                  input->points[i].y,
                  input->points[i].z);
      lines[pos].add_point(p_new, input->points[i].intensity);
      lines[pos].reserve_point(input->points.size());
      continue;
    }
    // For loop: determine the point shall be classified into which line
    for (size_t j = 0; j < lines.size(); j++) {
      if (input->points[i].ring == lines[j].get_id()) {
        pos = j;
        break;
      }
      if (input->points[i].ring < lines[j].get_id()) {
        pos = j;
        LineData line;
        line.set_id(input->points[i].ring);
        line.set_stamp(input->header.stamp);
        lines.insert(lines.begin() + pos, line);
        lines[pos].reserve_point(input->points.size());
        break;
      }
      if (j == lines.size() - 1) {
        pos = j + 1;
        LineData line;
        line.set_id(input->points[i].ring);
        line.set_stamp(input->header.stamp);
        lines.push_back(line);
        lines[pos].reserve_point(input->points.size());
        break;
      }
    }
    Point p_new(input->points[i].x,
                input->points[i].y,
                input->points[i].z);
    lines[pos].add_point(p_new, input->points[i].intensity);
  }
  return lines;
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


PointCloud::Ptr transform_to_pointcloud(const std::vector<LineData>& lines) {
  PointCloud::Ptr temp(new PointCloud);
  temp->header.frame_id = "velodyne";
  temp->header.stamp = lines[0].get_stamp();

  temp->width = 0;
  temp->height = 1;
  temp->is_dense = false;
  temp->points.resize(temp->width * temp->height);
  int i = 0;
  for (auto l : lines) {
    int j = 0;
    std::vector<std::pair<Point, int>> l_ = l.get_line();
    temp->width += l_.size();
    temp->points.resize(temp->width * temp->height);
    for (i; i < temp->points.size(); i++) {
      temp->points[i].x = l_[j].first.x();
      temp->points[i].y = l_[j].first.y();
      temp->points[i].z = l_[j].first.z();
      temp->points[i].intensity = l_[j].second;
      temp->points[i].ring = l.get_id();
      j++;
    }
  }
  return temp;
}
}  // namespace model_fitting
