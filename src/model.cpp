#include <algorithm>
#include <ceres/ceres.h>

#include <model.hpp>

namespace model_fitting {
Eigen::Matrix4f rot_mat(const Point& point,
                        const Vector3& vector,
                        const double t) {
  float u = vector(0);
  float v = vector(1);
  float w = vector(2);
  float a = point(0);
  float b = point(1);
  float c = point(2);

  Eigen::Matrix4f matrix;
  matrix << u*u + (v*v + w*w)*cos(t),
            u*v*(1 - cos(t)) - w*sin(t),
            u*w*(1 - cos(t)) + v*sin(t),
            (a*(v*v + w*w) - u*(b*v + c*w))*(1 - cos(t)) + (b*w - c*v)*sin(t),
            u*v*(1 - cos(t)) + w*sin(t),
            v*v + (u*u + w*w)*cos(t),
            v*w*(1 - cos(t)) - u*sin(t),
            (b*(u*u + w*w) - v*(a*u + c*w))*(1 - cos(t)) + (c*u - a*w)*sin(t),
            u*w*(1 - cos(t)) - v*sin(t),
            v*w*(1 - cos(t)) + u*sin(t),
            w*w + (u*u + v*v)*cos(t),
            (c*(u*u + v*v) - w*(a*u + b*v))*(1 - cos(t)) + (a*v - b*u)*sin(t),
            0, 0, 0, 1;
  return matrix;
}


Vector2 rot_2D(const Vector2& centroid,
               const double phi,
               const Vector2& point) {
  double rot_x, rot_y;
  rot_x = (point[0] - centroid[0]) * cos(phi)
                 - (point[1] - centroid[1]) * sin(phi)
                 + centroid[0];
  rot_y = (point[0] - centroid[0]) * sin(phi)
                  + (point[1] - centroid[1]) * cos(phi)
                  + centroid[1];
  return Vector2{rot_x, rot_y};
}


std::vector<Vector2> x_y_phi_2_triangle(const Vector3& x_y_phi) {
  extern double ce_length;
  Vector2 init_p, p1, p2, p3;
  init_p[0] = x_y_phi[0] + ce_length * 1;
  init_p[1] = x_y_phi[1] + ce_length * 0;
  p1 = rot_2D(Vector2(x_y_phi[0], x_y_phi[1]), x_y_phi[2], init_p);
  p2 = rot_2D(Vector2(x_y_phi[0], x_y_phi[1]), DegToRad(120), p1);
  p3 = rot_2D(Vector2(x_y_phi[0], x_y_phi[1]), DegToRad(120), p2);

  std::vector<Vector2> tri_points;
  tri_points.push_back(p1);
  tri_points.push_back(p2);
  tri_points.push_back(p3);
  return tri_points;
}


std::vector<Vector2> x_y_phi_2_square(const Vector3& x_y_phi) {
  extern double ce_length;
  Vector2 init_p, p1, p2, p3, p4;
  init_p[0] = x_y_phi[0] + ce_length * 1;
  init_p[1] = x_y_phi[1] + ce_length * 0;
  p1 = rot_2D(Vector2(x_y_phi[0], x_y_phi[1]), x_y_phi[2], init_p);
  p2 = rot_2D(Vector2(x_y_phi[0], x_y_phi[1]), DegToRad(90), p1);
  p3 = rot_2D(Vector2(x_y_phi[0], x_y_phi[1]), DegToRad(90), p2);
  p4 = rot_2D(Vector2(x_y_phi[0], x_y_phi[1]), DegToRad(90), p3);

  std::vector<Vector2> tri_points;
  tri_points.push_back(p1);
  tri_points.push_back(p2);
  tri_points.push_back(p3);
  tri_points.push_back(p4);
  return tri_points;
}


bool IsPointInTriangle(const Vector2& point,
                       const std::vector<Vector2>& tri_points) {
  Vector2 PA = tri_points[0] - point;
  Vector2 PB = tri_points[1] - point;
  Vector2 PC = tri_points[2] - point;

  double t1 = PA.x() * PB.y() - PB.x() * PA.y();  // PA X PB
  double t2 = PB.x() * PC.y() - PC.x() * PB.y();
  double t3 = PC.x() * PA.y() - PA.x() * PC.y();
  return t1 * t2 >= 0 && t1 * t3 >= 0;
}


bool IsPointInSquare(const Vector2& point,
                     const std::vector<Vector2>& tri_points) {
  Vector2 PA = tri_points[0] - point;
  Vector2 PB = tri_points[1] - point;
  Vector2 PC = tri_points[2] - point;
  Vector2 PD = tri_points[3] - point;

  double t1 = PA.x() * PB.y() - PB.x() * PA.y();  // PA X PB
  double t2 = PB.x() * PC.y() - PC.x() * PB.y();
  double t3 = PC.x() * PD.y() - PD.x() * PC.y();
  double t4 = PD.x() * PA.y() - PA.x() * PD.y();
  return t1 * t2 >= 0 && t1 * t3 >= 0 && t1 * t4 >= 0;
}


visualization_msgs::Marker mark_centroid(const Point& c,
                                         const Vector3& color,
                                         const std::string& lidar_frame) {
  visualization_msgs::Marker marker;
  // marker setup
  marker.header.frame_id = lidar_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = "vis";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  // marker.pose decide the origin of SPHERE
  marker.pose.position.x = c[0];
  marker.pose.position.y = c[1];
  marker.pose.position.z = c[2];
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  // sphere size
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.lifetime = ros::Duration(0.5);

  return marker;
}


visualization_msgs::Marker print_Model(const std::vector<Point>& pts,
                                       const std::string& lidar_frame) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = lidar_frame;
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "vis";
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;

  line_strip.scale.x = 0.02;

  line_strip.color.r = 1;
  line_strip.color.a = 1;

  geometry_msgs::Point p;
  for (auto pt : pts) {
    p.x = pt[0];
    p.y = pt[1];
    p.z = pt[2];
    line_strip.points.push_back(p);
  }
  p.x = pts[0].x();
  p.y = pts[0].y();
  p.z = pts[0].z();
  line_strip.points.push_back(p);

  line_strip.lifetime = ros::Duration(2);

  return line_strip;
}


double point_to_line_distance(const Point& p, const LineData& line) {
  Point v, w, pb;
  double c1, c2, b;
  double total_distance = 0, count = 0;

  auto l = line.get_line();

  for (int i = 0; i < l.size() - 1; i++) {
    for (int j = i+1; j < l.size() ; j++) {
      v = l[i].first-l[j].first;
      w = p - l[j].first;
      c1 = w.dot(v);
      c2 = v.dot(v);
      b = c1/c2;
      pb = l[j].first+ b*v;
      total_distance += Distance(p, pb);
      count++;
    }
  }
  total_distance /= count;

  return total_distance;
}


double point_to_triangle_distance(const double* const x_y_phi,
                                  const Vector2& point) {
  std::vector<Vector2> tri_points
       = x_y_phi_2_triangle(Vector3(x_y_phi[0], x_y_phi[1], x_y_phi[2]));

  Vector2 v1, v2;
  double h1, h2, h3;
  v1 = point - tri_points[0];
  v2 = tri_points[2] - tri_points[0];
  h1 = std::fabs((v1.x() * v2.y() - v1.y() * v2.x()) / v2.norm());
  v1 = point - tri_points[1];
  v2 = tri_points[0] - tri_points[1];
  h2 = std::fabs((v1.x() * v2.y() - v1.y() * v2.x()) / v2.norm());
  v1 = point - tri_points[2];
  v2 = tri_points[1] - tri_points[2];
  h3 = std::fabs((v1.x() * v2.y() - v1.y() * v2.x()) / v2.norm());

  double min = std::min(std::min(h1, h2), h3);
  return min;
}


double point_to_square_distance(const double* const x_y_phi,
                                  const Vector2& point) {
  std::vector<Vector2> tri_points
       = x_y_phi_2_square(Vector3(x_y_phi[0], x_y_phi[1], x_y_phi[2]));

  Vector2 v1, v2;
  double h1, h2, h3, h4;
  v1 = point - tri_points[0];
  v2 = tri_points[3] - tri_points[0];
  h1 = std::fabs((v1.x() * v2.y() - v1.y() * v2.x()) / v2.norm());
  v1 = point - tri_points[1];
  v2 = tri_points[0] - tri_points[1];
  h2 = std::fabs((v1.x() * v2.y() - v1.y() * v2.x()) / v2.norm());
  v1 = point - tri_points[2];
  v2 = tri_points[1] - tri_points[2];
  h3 = std::fabs((v1.x() * v2.y() - v1.y() * v2.x()) / v2.norm());
  v1 = point - tri_points[3];
  v2 = tri_points[2] - tri_points[3];
  h4 = std::fabs((v1.x() * v2.y() - v1.y() * v2.x()) / v2.norm());

  double min = std::min(std::min(std::min(h1, h2), h3), h4);
  return min;
}


// class ModelErrorTerm is to store a given 2D-point data,
// and then can be used as we compute the residual
class ModelErrorTerm {
 public:
  explicit ModelErrorTerm(const Vector2& point): point_(point) {}

  bool operator()(const double* const x_y_phi, double* residual) const {
    extern char* m_type;
    std::string str(m_type);

    if (str.compare("triangle") == 0) {
      std::vector<Vector2> endpoints
          = x_y_phi_2_triangle(Vector3(x_y_phi[0], x_y_phi[1], x_y_phi[2]));

      if (IsPointInTriangle(point_, endpoints)) {
        residual[0] = 0;
      } else {
        residual[0] = point_to_triangle_distance(x_y_phi, point_);
      }
    } else if (str.compare("square") == 0) {
      std::vector<Vector2> endpoints
          = x_y_phi_2_square(Vector3(x_y_phi[0], x_y_phi[1], x_y_phi[2]));
      if (IsPointInSquare(point_, endpoints)) {
        residual[0] = 0;
      } else {
        residual[0] = point_to_square_distance(x_y_phi, point_);
      }
    }
    return true;
  }

  static ceres::CostFunction* Create(const Vector2& point) {
    return new ceres::NumericDiffCostFunction<ModelErrorTerm, ceres::RIDDERS, 1, 3>(
           new ModelErrorTerm(point));
  }

 private:
  const Vector2 point_;
};  // class Point2LineErrorTerm


// class ModelErrorTerm2 is to store a given 2D-point data (edge points),
// and then can be used as we compute the residual
class ModelErrorTerm2 {
 public:
  explicit ModelErrorTerm2(const Vector2& point): point_(point) {}

  bool operator()(const double* const x_y_phi, double* residual) const {
    extern char* m_type;
    std::string str(m_type);

    if (str.compare("triangle") == 0) {
      std::vector<Vector2> tri_points
           = x_y_phi_2_triangle(Vector3(x_y_phi[0], x_y_phi[1], x_y_phi[2]));
      if (IsPointInTriangle(point_, tri_points)) {
        residual[0] = point_to_triangle_distance(x_y_phi, point_)
                      * point_to_triangle_distance(x_y_phi, point_);
      } else {
        residual[0] = 2 * point_to_triangle_distance(x_y_phi, point_)
                      * point_to_triangle_distance(x_y_phi, point_);
      }
    } else if (str.compare("square") == 0) {
      std::vector<Vector2> endpoints
          = x_y_phi_2_square(Vector3(x_y_phi[0], x_y_phi[1], x_y_phi[2]));
      if (IsPointInSquare(point_, endpoints)) {
        residual[0] = point_to_square_distance(x_y_phi, point_)
                      * point_to_square_distance(x_y_phi, point_);
      } else {
        residual[0] = 2 * point_to_square_distance(x_y_phi, point_)
                      * point_to_square_distance(x_y_phi, point_);
      }
    }
    return true;
  }

  static ceres::CostFunction* Create(const Vector2& point) {
    return new ceres::NumericDiffCostFunction<ModelErrorTerm2, ceres::RIDDERS, 1, 3>(
           new ModelErrorTerm2(point));
  }

 private:
  const Vector2 point_;
};  // class Point2LineErrorTerm


Vector3 model_fitting_2D(const std::vector<LineData>& lines,
                         const Vector2& centroid,
                         double phi) {
  double *x_y_phi = new double[3];
  *x_y_phi = centroid[0];
  *(x_y_phi+1) = centroid[1];
  *(x_y_phi+2) = phi;

  // First stage: fix centroid (x,y), compute phi as initial guess
  ceres::Problem problem;
  problem.AddParameterBlock(x_y_phi, 3);

  std::vector<int> constant_indices{0, 1};
  ceres::SubsetParameterization* subset_parameterization
         = new ceres::SubsetParameterization(3, constant_indices);
  problem.SetParameterization(x_y_phi, subset_parameterization);

  int pre_line_size = 0;
  for (size_t i = 0; i < lines.size(); ++i) {
    std::vector<std::pair<Point, int>> line = lines[i].get_line();
    if (line.size() < pre_line_size) {
      continue;
    }
    for (auto p : line) {
    ceres::CostFunction* cost_function
                         = ModelErrorTerm::Create(Vector2(p.first.x(), p.first.y()));
    problem.AddResidualBlock(cost_function, NULL, x_y_phi);
    }
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.gradient_tolerance = 1e-12;  // * Sophus::Constants<double>::epsilon();
  options.function_tolerance = 1e-12;  // * Sophus::Constants<double>::epsilon();

  options.linear_solver_type = ceres::DENSE_QR;
  options.max_linear_solver_iterations = 30;
  options.max_num_iterations = 30;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Second stage: Compute optimized x_y_phi by giving a enough good
  // initial guess
  ceres::Problem problem2;
  problem2.AddParameterBlock(x_y_phi, 3);
  pre_line_size = 0;

  // Optimize all three parameters x_y_phi
  for (size_t i = 0; i < lines.size(); ++i) {
    std::vector<std::pair<Point, int>> line = lines[i].get_line();
    if (line.size() < pre_line_size) {
      continue;
    }
    for (auto p : line) {
    ceres::CostFunction* cost_function
                         = ModelErrorTerm2::Create(Vector2(p.first.x(), p.first.y()));
    problem2.AddResidualBlock(cost_function, NULL, x_y_phi);
    }
  }
  ceres::Solver::Summary summary2;
  ceres::Solve(options, &problem2, &summary2);
  Vector3 x_y_phi_(x_y_phi[0], x_y_phi[1], x_y_phi[2]);
  delete [] x_y_phi;

  return Vector3(x_y_phi_);
}


visualization_msgs::Marker print_Normal(const Point& pt,
                                        const Eigen::Vector4f& plane_params,
                                        const std::string& lidar_frame,
                                        const double& scale) {
  visualization_msgs::Marker normal_arrow;
  normal_arrow.header.frame_id = lidar_frame;
  normal_arrow.header.stamp = ros::Time::now();
  normal_arrow.ns = "vis";
  normal_arrow.id = 0;
  normal_arrow.type = visualization_msgs::Marker::ARROW;
  normal_arrow.action = visualization_msgs::Marker::ADD;

  normal_arrow.scale.x = 0.05;
  normal_arrow.scale.y = 0.1;
  normal_arrow.color.g = 1;
  normal_arrow.color.a = 1;

  geometry_msgs::Point p;
  p.x = pt[0];
  p.y = pt[1];
  p.z = pt[2];
  normal_arrow.points.push_back(p);

  p.x = pt[0]-plane_params[0]*scale;
  p.y = pt[1]-plane_params[1]*scale;
  p.z = pt[2]-plane_params[2]*scale;

  normal_arrow.points.push_back(p);

  normal_arrow.lifetime = ros::Duration(0.5);

  return normal_arrow;
}
}  // namespace model_fitting
