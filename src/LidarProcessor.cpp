#include <algorithm>
#include <cmath>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <LidarProcessor.hpp>
#include <ransac.hpp>

namespace model_fitting {

int isTarget2(const std::vector<measurement>& center,
              const std::vector<float>& searching_region,
              const esr_msgs::Track& msg) {
  float a = DegToRad(msg.azimuth);
  float x = msg.range * std::cos(a);
  float y = msg.range * std::sin(a);
  for (auto c : center) {
    if (x <= c.point_.x() + searching_region[0]/2.
       && x >= c.point_.x() - searching_region[0]/2.
       && y <= c.point_.y() + searching_region[1]/2.
       && y >= c.point_.y() - searching_region[1]/2.) {
     return c.id_;
    }
  }
  return -1;
}

LidarProcessor::LidarProcessor(ros::NodeHandle* nh) {
  p_nh = nh;

  filtered_points_pub = p_nh->advertise<PointCloud>("filtered_points", 1, true);
  target_points_pub = p_nh->advertise<PointCloud>("target_points", 1, true);
  edge_points_pub = p_nh->advertise<PointCloud>("edge_points", 1, true);

  centroid_pub = p_nh->advertise<visualization_msgs::Marker>("centroid", 1, true);
  cube_pub = p_nh->advertise<visualization_msgs::Marker>("box_filter_bound", 1, true);
  model_centroid_pub = p_nh->advertise<visualization_msgs::Marker>("model_centroid", 1, true);
  model_pub = p_nh->advertise<visualization_msgs::Marker>("model", 1, true);
  normal_pub = p_nh->advertise<visualization_msgs::Marker>("normal", 1, true);

  // Subscribe lidar topic
  pkg_path = ros::package::getPath("model_fitting");
  p_nh->getParam("/LidarProcessor_node/topic_name_lidar", topic_name_lidar);
  lidar_sub = p_nh->subscribe(topic_name_lidar, 100, &LidarProcessor::cb_lidar, this);

  // Read the specification of target
  std::string str;
  nh->getParam("/LidarProcessor_node/target_size_set/model_type", str);
  m_type = str.c_str();
  nh->getParam("/LidarProcessor_node/target_size_set/center_to_end_length", ce_length);
  nh->getParam("/LidarProcessor_node/target_size_set/reflector_edge_length", cr_length);
  depth = std::sqrt(cr_length*cr_length / 3);

  nh->getParam("/LidarProcessor_node/edge_points_resolution", edge_points_resolution);
  // open file and save the processed target point
  outfile_l.open(pkg_path + "/data/lidar_data_raw.csv");
  outfile_l << "time_stamp, c_x, c_y, c_z, v1_x, v1_y, v1_z, ..., c_xx, c_yy, czz" << std::endl;

  std::vector<float> translation;
  std::vector<float> rotation;
  p_nh->getParam("/LidarProcessor_node/T_lr/translation", translation);
  p_nh->getParam("/LidarProcessor_node/T_lr/rotation", rotation);
  Point p_radar = Point(translation[0], translation[1], translation[2]);
  auto r = AngleAxis(DegToRad(rotation[0]), Vector3::UnitZ()) *
           AngleAxis(DegToRad(rotation[1]), Vector3::UnitY()) *
           AngleAxis(DegToRad(rotation[2]), Vector3::UnitX());
  Orientation o_radar = Orientation(r);
  T_lr = Pose(o_radar, p_radar);
  p_nh->getParam("/LidarProcessor_node/topic_name_radar", topic_name_radar);
  topic_frame_radar = "esr_can1_frame";
  lidar_data.resize(1);
  radar_data.resize(1);
  searching_region = std::vector<float> {3, 3};
  radar_sub = p_nh->subscribe(topic_name_radar, 1000, &LidarProcessor::cb_radar, this);
  rcs_pub = p_nh->advertise<PointCloud>("rcs_points", 1, true);
  p_nh->getParam("/LidarProcessor_node/sid", sid);
  p_nh->getParam("/LidarProcessor_node/box_filter_set/center", center);

  searching_centers.clear();
  Point p_l(center[0], center[1], 0);
  Point p_r = T_lr.inverse() * p_l;

  double rr = p_r.norm();
  double aa = std::atan2(p_r.y(), p_r.x());
  measurement p_c {0,
                   0,
                   Point(rr * std::cos(aa), rr * std::sin(aa), 0)};
  searching_centers.push_back(p_c);
}


// callback function: subscribe lidar pointcloud and process
void LidarProcessor::cb_lidar(const sensor_msgs::PointCloud2& msg) {

  Orientation q_lr = T_lr.unit_quaternion();
  transform.setOrigin(tf::Vector3(T_lr.translation().x(),
                                  T_lr.translation().y(),
                                  T_lr.translation().z()));
  transform.setRotation(tf::Quaternion(q_lr.x(),
                                       q_lr.y(),
                                       q_lr.z(),
                                       q_lr.w()));
  br.sendTransform(tf::StampedTransform(transform,
                                        msg.header.stamp,
                                        msg.header.frame_id,
                                        topic_frame_radar));

  double current_time = msg.header.stamp.toSec();
  // Change point cloud type from sensor_msgs::PointCloud2 to pcl::PointXYZI
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg, pcl_pc2);
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  topic_frame_lidar = msg.header.frame_id;
  // Lidar pointcloud processing
  PointCloud::Ptr non_ground_cloud(new PointCloud);
  plane_filter(cloud, non_ground_cloud, 0.1, true);

  PointCloud::Ptr box_filtered_points(new PointCloud);
  // Use box filter(tracking) to filter pointcloud
  p_nh->getParam("/LidarProcessor_node/box_filter_set/center", center);
  p_nh->getParam("/LidarProcessor_node/box_filter_set/cube_side_length", side_length);
  Vector3 color_white(1, 1, 1);
  visualization_msgs::Marker box_bound = mark_cube(Point(center[0], center[1], center[2]),
                                                   side_length,
                                                   color_white,
                                                   topic_frame_lidar);
  cube_pub.publish(box_bound);
  box_filter(non_ground_cloud,
             box_filtered_points,
             Point(center[0], center[1], center[2]),
             side_length);
  if (box_filtered_points->points.size() == 0) {
    ROS_INFO("All points has been filtered(situation 1): After pass-filtering, no one left.");
    return;
  }

  // Ignore the case that two set of point cloud overlap
  if (cloud_size.size() == 0) {
    cloud_size.push_back(box_filtered_points->points.size());
    cloud_size.push_back(box_filtered_points->points.size());
  } else {
    cloud_size[0] = cloud_size[1];
    cloud_size[1] = box_filtered_points->points.size();
  }

  if (cloud_size[1] > cloud_size[0] * 1.5) {
    std::cerr << "Warning! Two set of target points." << std::endl;
  }

  // Intensity filter
  p_nh->getParam("/LidarProcessor_node/intensity_filter_set/lower_upper_bound", lower_upper_bound);
  PointCloud::Ptr intensity_filtered_points(new PointCloud);

  intensity_filter(lower_upper_bound[0], lower_upper_bound[1],
                   box_filtered_points, intensity_filtered_points);

  if (intensity_filtered_points->points.size() == 0) {
    ROS_INFO("All points has been filtered(situation 2): After filtering by intensity), no one left.");
    return;
  }

  // Warning: filtered cloud size is larger than 3000
  if (intensity_filtered_points->points.size() > 3000
      || intensity_filtered_points->points.size() < 30) {
    std::cout << "The number of target points is "
              << intensity_filtered_points->points.size()
              << "(> 3000 or < 30), please check if the extraction of target points is success."
              << std::endl;
  }
  PointCloud::Ptr plane_cloud(new PointCloud);
  PointCloud::Ptr board_cloud(new PointCloud);
  plane_filter(intensity_filtered_points, plane_cloud, 0.03, false);
  std::vector<Point> data;
  for (int i = 0; i < plane_cloud->points.size(); i++) {
    data.push_back(Point(plane_cloud->points[i].x, plane_cloud->points[i].y, plane_cloud->points[i].z));
  }

  // Outlier rejection
  std::vector<int> inliers;
  ransac_cpp::Ransac ransac(data);
  ransac.setMaxIterations(1000);
  ransac.setDistanceThreshold(ce_length);
  ransac.setProbability(0.99);
  ransac.computeModel();
  inliers = ransac.getInliers();
  pcl::PointIndices::Ptr inliers_(new pcl::PointIndices);
  inliers_->indices = inliers;

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(plane_cloud);
  extract.setIndices(inliers_);
  extract.setNegative(false);
  extract.filter(*board_cloud);

  Eigen::Vector4f normal = Find_Normal(board_cloud);
  if (!std::isfinite(normal[0])) {
    std::cout << "Cannot esitimate an available normal. Ignore the case!"
              << std::endl;
    return;
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*board_cloud, centroid);
  std::vector<float> new_center{static_cast<float>(centroid[0]),
                                static_cast<float>(centroid[1]),
                                static_cast<float>(centroid[2])};
  p_nh->setParam("/LidarProcessor_node/box_filter_set/center", new_center);

  searching_centers.clear();
  Point p_l(centroid[0], centroid[1], centroid[2]);
  Point p_r = T_lr.inverse() * p_l;

  double r = p_r.norm();
  double a = std::atan2(p_r.y(), p_r.x());
  measurement p_c {0,
                   current_time,
                   Point(r * std::cos(a), r * std::sin(a), 0)};
  searching_centers.push_back(p_c);


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

  PointCloud::Ptr transformed_cloud(new PointCloud);
  PointCloud::Ptr edge_points(new PointCloud);
  PointCloud::Ptr transformed_edge_points(new PointCloud);
  std::vector<Vector2> edge_points_;

  edge_extract(board_cloud, edge_points, edge_points_resolution);
  pcl::transformPointCloud(*edge_points, *transformed_edge_points, rot_matrix);

  filtered_points_pub.publish(plane_cloud);
  target_points_pub.publish(board_cloud);
  edge_points_pub.publish(edge_points);

  std::cout << "origin: " << board_cloud->points.size()
            << ", edge: " << edge_points->points.size() << std::endl;

  for (int i = 0; i < edge_points->points.size(); i++) {
    edge_points_.push_back(Vector2(transformed_edge_points->points[i].x,
                                   transformed_edge_points->points[i].y));
  }
  Eigen::Vector4f centroid_xy = rot_matrix * centroid;
  // Model fitting in X-Y plane
  double pre_time = ros::Time::now().toSec();
  Vector3 x_y_phi = model_fitting_2D(edge_points_,
                                     Vector2(centroid_xy[0], centroid_xy[1]),
                                     0);
  std::cout << "Cost time: " << (ros::Time::now().toSec() - pre_time) << std::endl;
  if (!kf.activate()) {
    kf_init(kf, x_y_phi, current_time);
  } else {
    // Predict
    double dt = current_time - kf.get_t();
    Eigen::MatrixXd F(6, 6);
    F << 1, 0, 0, dt, 0, 0,
         0, 1, 0, 0, dt, 0,
         0, 0, 1, 0, 0, dt,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    kf.set_F(F);
    kf.Predict();
    kf.set_t(current_time);
    Eigen::VectorXd pred = kf.get_x();

    if (std::isnan(pred[0]) || std::isnan(pred[1])
        || std::isnan(pred[2]) || std::isnan(pred[3])
        || std::isnan(pred[4]) || std::isnan(pred[5])) {
      kf_init(kf, x_y_phi, current_time);
      std::cout << "Prediction ERROR!!!" << std::endl;
      return;
    }

    // Update
    Eigen::VectorXd z_in(3);
    z_in << x_y_phi[0], x_y_phi[1], x_y_phi[2];
    kf.Update(z_in);
    Eigen::VectorXd update = kf.get_x();

    // std::cout << std::to_string(current_time) << "|"
    //           << "(x, y, phi, vx, vy, vphi) = raw => ("
    //           << x_y_phi[0] << ", "
    //           << x_y_phi[1] << ", "
    //           << x_y_phi[2] << ", none, none, none) v.s. "
    //           << "pred => ("
    //           << pred[0] << ", "
    //           << pred[1] << ", "
    //           << pred[2] << ", "
    //           << pred[3] << ", "
    //           << pred[4] << ", "
    //           << pred[5] << ") v.s. "
    //           << "update => ("
    //           << update[0] << ", "
    //           << update[1] << ", "
    //           << update[2] << ", "
    //           << update[3] << ", "
    //           << update[4] << ", "
    //           << update[5] << ")" << std::endl;
    x_y_phi = Vector3(update[0], update[1], update[2]);
  }

  std::vector<Vector2> tip_points_2D;
  std::string str(m_type);
  if (str.compare("triangle") == 0)
    tip_points_2D = x_y_phi_2_triangle(x_y_phi);
  else if (str.compare("square") == 0)
    tip_points_2D = x_y_phi_2_square(x_y_phi);

  // Transform 2D tip points back to 3D
  std::vector<Point> tip_points_fit;
  Point translation = Point(rot_matrix.cast<double>().block<3, 1>(0, 3));
  Orientation rotation
              = Orientation(rot_matrix.cast<double>().topLeftCorner<3, 3>());

  Pose pose(rotation, translation);

  for (auto tp2 : tip_points_2D) {
    tip_points_fit.push_back(pose.inverse() * Point(tp2[0],
                                                    tp2[1],
                                                    0));
  }

  // visualize triangle model and its centroid on rviz
  visualization_msgs::Marker model = print_Model(tip_points_fit,
                                                 color_white,
                                                 topic_frame_lidar);
  model_pub.publish(model);

  Vector3 color_red(1, 0, 0);  // (r,g,b)
  visualization_msgs::Marker centroid_marker = mark_centroid(Point(centroid[0], centroid[1], centroid[2]),
                                                             color_red,
                                                             topic_frame_lidar);
  centroid_pub.publish(centroid_marker);

  Point model_centroid(0, 0, 0);
  for (auto p : tip_points_fit) {
    model_centroid += p / tip_points_fit.size();
  }

    lidar_data[0].push_back(measurement{0,
                                            current_time,
                                            model_centroid});

  Vector3 color_blue(0, 0, 1);  // (r,g,b)
  visualization_msgs::Marker model_centroid_marker = mark_centroid(model_centroid,
                                                                   color_blue,
                                                                   topic_frame_lidar);
  model_centroid_pub.publish(model_centroid_marker);
  if ((model_centroid - Point(centroid[0], centroid[1], centroid[2])).norm() > 0.2) {
    std::cout << "Difference between centroid and model centroid is larger than 0.2 m, ignore this case." << std::endl;
    return;
  }

  // Model-fitting accomplished. Save the data.
  std::cout << "Good, save lidar data." << std::endl;
  outfile_l << msg.header.stamp
            << ", " << model_centroid[0]
            << ", " << model_centroid[1]
            << ", " << model_centroid[2];
  for (int i = 0; i < tip_points_fit.size(); i++) {
    outfile_l << ", " << tip_points_fit[i][0]
              << ", " << tip_points_fit[i][1]
              << ", " << tip_points_fit[i][2];
  }
  outfile_l << ", " << model_centroid[0] - normal[0] * depth
            << ", " << model_centroid[1] - normal[1] * depth
            << ", " << model_centroid[2] - normal[2] * depth << std::endl;

  Point pc(Point(model_centroid[0], model_centroid[1], model_centroid[2]));
  visualization_msgs::Marker normal_arrow = print_Normal(pc,
                                                         normal,
                                                         topic_frame_lidar,
                                                         depth);
  normal_pub.publish(normal_arrow);
  saveCorrespondences();
}

LidarProcessor::~LidarProcessor() {
  outfile_l.close();
}

void kf_init(KalmanFilter& kf, const Vector3& x_y_phi, const double time) {
  // EKF initialization
  double cov_x = (0.1/3)*(0.1/3);
  double cov_y = (0.1/3)*(0.1/3);
  double cov_phi = (DegToRad(5)/3.)*(DegToRad(5)/3.);
  Eigen::VectorXd x_in(6), u_in(6);
  Eigen::MatrixXd P(6, 6), F(6, 6), B(6, 6),
                  Q(6, 6), H(3, 6), R(3, 3);

  x_in << x_y_phi[0], x_y_phi[1], x_y_phi[2], 0, 0, 0;
  u_in << 0, 0, 0, 0, 0, 0;
  P << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
  F = B = P;

  Eigen::MatrixXd G(6, 3), Qv(3, 3);
  G << 0.125, 0, 0,
       0, 0.125, 0,
       0, 0, 0.125,
       0.5, 0, 0,
       0, 0.5, 0,
       0, 0, 0.5;
  Qv << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;
  Q = G * Qv * G.transpose();

  H << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0;


  R << cov_x, 0, 0,
       0, cov_y, 0,
       0, 0, cov_phi;

  kf.Init(x_in, u_in, P, F, B, Q, H, R, time);
}


void LidarProcessor::cb_radar(const esr_msgs::Track &msg) {
  if (radar_buffer.empty()
      || (msg.header.stamp.toSec() - radar_buffer.back().header.stamp.toSec()) < 1e-5) {
    radar_buffer.push_back(msg);
  } else {
    double current_time = radar_buffer.back().header.stamp.toSec();

    // Target measurement detection and refinement
    for (auto meas : radar_buffer) {
      if (std::fabs(meas.azimuth) > 45)
        continue;
      double range = meas.range;
      double azimuth = DegToRad(meas.azimuth);
      double range_rate = meas.range_rate;

      if (!searching_centers.empty()) {

        int id = isTarget2(searching_centers, searching_region, meas);

        if (id != -1) {
          Vector3 init_v;
          Point cur_location = Point(range * std::cos(azimuth),
                                     range * std::sin(azimuth),
                                     0);
          if (radar_data[id].size() == 0 || (current_time - radar_data[id].back().time_) > 0.1) {
            init_v = Vector3(0, 0, 0);
          } else {
            double time_diff = (current_time - radar_data[id].back().time_);
            init_v = (cur_location - radar_data[id].back().point_) / time_diff;
          }

          measurement pc{id,
                         current_time,
                         cur_location,
                         static_cast<double>(meas.amplitude),
                         range_rate,
                         init_v,
                         true};
          if (radar_data[id].size() == 0
              || (pc.time_ - radar_data[id].back().time_) > 1e-5) {
            radar_data[id].push_back(pc);
          } else if (pc.intensity_ > radar_data[id].back().intensity_) {
            radar_data[id].pop_back();
            radar_data[id].push_back(pc);
          }
        }
      }
    }

    for (int i = 0; i < radar_data.size(); i++) {
      if (radar_data[i].size() == 0)
        continue;
      measurement new_pc{radar_data[i].back().id_,
                         radar_data[i].back().time_,
                         radar_data[i].back().point_,
                         radar_data[i].back().intensity_,
                         radar_data[i].back().range_rate_,
                         radar_data[i].back().v_,
                         true};
      int count = 1;
      for (int j = radar_data[i].size()-2; j >= 0; j--) {
        if ((new_pc.time_- radar_data[i][j].time_) < 1e-5) {
          new_pc.point_ += radar_data[i][j].point_;
          new_pc.v_ += radar_data[i][j].v_;
          count++;
        } else {
          new_pc.point_ /= count;
          new_pc.v_ /= count;
          for (int k = 0; k < count; k++) {
            radar_data[i].pop_back();
          }
          radar_data[i].push_back(new_pc);
          break;
        }
      }
    }
    if (radar_data[0].size() != 0)
      std::cout << "RCS: " << radar_data[0].back().intensity_ << std::endl;; 
    radar_buffer.clear();
    radar_buffer.push_back(msg);
  }
}

void LidarProcessor::saveCorrespondences() {
  // Save and publish radar-LiDAR correspondences
  int r_size = 0;
  for (int i = 0; i < radar_data.size(); i++) {
    r_size += radar_data[i].size();
  }
  PointCloud::Ptr rcs_points(new PointCloud);
  rcs_points->width    = r_size;
  rcs_points->height   = 1;
  rcs_points->is_dense = false;
  rcs_points->points.resize(rcs_points->width * rcs_points->height);
  rcs_points->header.frame_id = topic_frame_radar;
  int k = 0;

  std::ofstream outfile;
  outfile.open(pkg_path+"/data/correspondences.csv");
  for (size_t i = 0; i < radar_data.size(); i++) {
    for (size_t j = 0; j < radar_data[i].size(); j++) {
      double time = radar_data[i][j].time_ - 0.05;
      auto lidar_iter = find_if(lidar_data[i].begin(), lidar_data[i].end(),
                                [time] (const measurement& l_data)
                                {return time <= l_data.time_; });

      if ((lidar_iter == lidar_data[i].begin() && std::fabs(lidar_iter->time_-time) > 1)
          || (lidar_iter == lidar_data[i].end() && std::fabs((lidar_iter-1)->time_-time) > 1)
          || (lidar_iter->time_-time) > 1 || std::fabs((lidar_iter-1)->time_-time) > 1) {
        std::cout << "Data at time : " << std::to_string(time)
                  << " can not matched." << std::endl;
        continue;
      }
      Point pt_lidar, pt_lidar_transformed;
      decltype(lidar_iter) l1, l2;
      if (lidar_iter == lidar_data[i].begin()) {
        l1 = lidar_iter + 1;
        l2 = lidar_iter;
      } else if (lidar_iter == lidar_data[i].end()) {
        l1 = lidar_iter - 1;
        l2 = lidar_iter - 2;
      } else {
        l1 = lidar_iter;
        l2 = lidar_iter - 1;
      }

      pt_lidar = (time-l1->time_)/(l1->time_-l2->time_)*(l1->point_-l2->point_)
                 + l1->point_;
      pt_lidar_transformed = T_lr.inverse() * pt_lidar;
      rcs_points->points[k].x = pt_lidar_transformed.x();
      rcs_points->points[k].y = pt_lidar_transformed.y();
      rcs_points->points[k].z = pt_lidar_transformed.z();
      rcs_points->points[k].intensity = radar_data[i][j].intensity_;
      k++;
      double range = radar_data[i][j].point_.norm();
      double azimuth = RadToDeg(std::atan2(radar_data[i][j].point_.y(),
                                           radar_data[i][j].point_.x()));
      outfile << sid << ", "
              << pt_lidar.x() << ", "
              << pt_lidar.y() << ", "
              << pt_lidar.z() << ", "
              << range << ", "
              << azimuth << ", "
              << radar_data[i][j].intensity_ << std::endl;
    }
  }
  outfile.close();
  rcs_pub.publish(rcs_points);
}


}  // namespace model_fitting
