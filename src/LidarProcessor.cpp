#include <algorithm>
#include <cmath>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <LidarProcessor.hpp>


namespace model_fitting {
LidarProcessor::LidarProcessor(ros::NodeHandle* nh) {
  p_nh = nh;

  filtered_points_pub = p_nh->advertise<PointCloud>("filtered_points", 1, true);
  plane_points_pub = p_nh->advertise<PointCloud>("plane_points", 1, true);
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

  // Read the specification of lidar
  nh->getParam("/LidarProcessor/lidar_resolution_set/vertical_resolution", vertical_resolution);
  nh->getParam("/LidarProcessor/lidar_resolution_set/horizontal_resolution", horizontal_resolution);

  // open file and save the processed target point
  outfile_l.open(pkg_path + "/data/lidar_data_raw.csv");
  outfile_l << "time_stamp, c_x1, c_y1, c_z1, c_x2, c_y2, c_z2,"
            << " t_x, t_y, t_z, r_x, r_y, r_z, l_x, l_y, l_z" << std::endl;
}


// callback function: subscribe lidar pointcloud and process
void LidarProcessor::cb_lidar(const sensor_msgs::PointCloud2& msg) {
  // Change point cloud type from sensor_msgs::PointCloud2 to pcl::PointXYZI
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg, pcl_pc2);
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  topic_frame_lidar = msg.header.frame_id;
  // Lidar pointcloud processing

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
  box_filter(Point(center[0], center[1], center[2]),
              side_length,
              cloud,
              box_filtered_points);
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
    std::cerr << "Two set of target points. Ignore the case!" << std::endl;
    cloud_size[1] = cloud_size[0];
    return;
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
  if (intensity_filtered_points->points.size() > 3000) {
    std::cout << "The number of target points is "
              << intensity_filtered_points->points.size()
              << "(> 3000), please check if the extraction of target points is success."
              << std::endl;
  }

  filtered_points_pub.publish(intensity_filtered_points);

  // Classify pointcloud into different line by using ring information
  std::vector<LineData> lines = line_classifier(intensity_filtered_points);
  std::reverse(lines.begin(), lines.end());
  purify(lines);

  // As the number of lines is less than 3, the program discard the case
  if (lines.size() < 3) {
    ROS_INFO("Line size less than 3. Ignore the case! (cloud size = %lu)",
                                        intensity_filtered_points->points.size());
    return;
  }

  // Perform plane-fitting to extract board points
  PointCloud::Ptr target_points = transform_to_pointcloud(lines);
  target_points_pub.publish(target_points);

  PointCloud::Ptr plane_points(new PointCloud);
  plane_filter(target_points, plane_points);
  plane_points_pub.publish(plane_points);

  lines = line_classifier(plane_points);


  std::reverse(lines.begin(), lines.end());
  purify(lines);
  Eigen::Vector4f normal = Find_Normal(plane_points);
  if (!std::isfinite(normal[0])) {
    std::cout << "Cannot esitimate an available normal. Ignore the case!"
              << std::endl;
    return;
  }
  // Compute average PointCloud centroid
  Point centroid = compute3Dcentroid(lines);
  Vector3 color_red(1, 0, 0);  // (r,g,b)
  visualization_msgs::Marker centroid_marker = mark_centroid(centroid,
                                                             color_red,
                                                             topic_frame_lidar);
  centroid_pub.publish(centroid_marker);


  // As the line size is larger than 10,
  // the program didn't perform model fitting due to the computation load
  if (lines.size() > 12) {
    std::cout << "Line size " << lines.size() << " > 12."
              << "Compute the centroid without model-fitting." << std::endl;
    outfile_l << msg.header.stamp
              << ", " << centroid[0]-normal[0]*depth
              << ", " << centroid[1]-normal[1]*depth
              << ", " << centroid[2]-normal[2]*depth
              << ", " << centroid[0]
              << ", " << centroid[1]
              << ", " << centroid[2]
              << ", " << '0' << ", " << '0' << ", " << '0'
              << ", " << '0' << ", " << '0' << ", " << '0'
              << ", " << '0' << ", " << '0' << ", " << '0' << std::endl;

    Point pc(Point(centroid[0], centroid[1], centroid[2]));
    visualization_msgs::Marker normal_arrow = print_Normal(pc,
                                                           normal,
                                                           topic_frame_lidar,
                                                           depth);
    normal_pub.publish(normal_arrow);

    std::vector<float> new_center{static_cast<float>(centroid[0]),
                                  static_cast<float>(centroid[1]),
                                  static_cast<float>(centroid[2])};
    p_nh->setParam("/LidarProcessor_node/box_filter_set/center", new_center);
    return;
  }

  // Transform the board plane aligning the X-Y plane
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

  // Extract edge points
  std::vector<LineData> edge_lines;
  edge_lines = edge_extract(lines);

  PointCloud::Ptr edge_points = transform_to_pointcloud(edge_lines);
  edge_points_pub.publish(edge_points);

  // Transform to X-Y plane
  edge_lines = transformLines(rot_matrix, edge_lines);

  int origin_point_size = 0;
  int edge_point_size = 0;

  for (auto l : lines) {
    origin_point_size += l.get_size();
  }

  for (auto l : edge_lines) {
    edge_point_size += l.get_size();
  }

  std::cout << "origin: " << origin_point_size
            << ", edge: " << edge_point_size << std::endl;

  // Model fitting in X-Y plane
  lines = transformLines(rot_matrix, lines);
  Point centroid_xy = compute3Dcentroid(lines);
  Vector3 x_y_phi = model_fitting_2D(edge_lines,
                                     Vector2(centroid_xy[0], centroid_xy[1]),
                                     0);

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
                                                 topic_frame_lidar);
  model_pub.publish(model);

  Point model_centroid(0, 0, 0);
  for (auto p : tip_points_fit) {
    model_centroid += p / tip_points_fit.size();
  }
  if ((model_centroid-centroid).norm() > 0.2) {
    std::cout << "Difference between centroid and model centroid is larger than 0.2 m, ignore this case." << std::endl;
    return;
  }
  Vector3 color_blue(0, 0, 1);  // (r,g,b)
  visualization_msgs::Marker model_centroid_marker = mark_centroid(model_centroid,
                                                                   color_blue,
                                                                   topic_frame_lidar);
  model_centroid_pub.publish(model_centroid_marker);

  // Model-fitting accomplished. Save the data.
  std::cout << "Good, save lidar data." << std::endl;
  outfile_l << msg.header.stamp
            << ", " << model_centroid[0]-normal[0]*depth
            << ", " << model_centroid[1]-normal[1]*depth
            << ", " << model_centroid[2]-normal[2]*depth
            << ", " << model_centroid[0]
            << ", " << model_centroid[1]
            << ", " << model_centroid[2]
            << ", " << tip_points_fit[0][0] << ", " << tip_points_fit[0][1]
            << ", " << tip_points_fit[0][2]
            << ", " << tip_points_fit[1][0] << ", " << tip_points_fit[1][1]
            << ", " << tip_points_fit[1][2]
            << ", " << tip_points_fit[2][0] << ", " << tip_points_fit[2][1]
            << ", " << tip_points_fit[2][2] << std::endl;

  Point pc(Point(model_centroid[0], model_centroid[1], model_centroid[2]));
  visualization_msgs::Marker normal_arrow = print_Normal(pc,
                                                         normal,
                                                         topic_frame_lidar,
                                                         depth);
  normal_pub.publish(normal_arrow);

  std::vector<float> new_center{static_cast<float>(model_centroid[0]),
                                static_cast<float>(model_centroid[1]),
                                static_cast<float>(model_centroid[2])};
  p_nh->setParam("/LidarProcessor_node/box_filter_set/center", new_center);
}

LidarProcessor::~LidarProcessor() {
  outfile_l.close();
}
}  // namespace model_fitting
