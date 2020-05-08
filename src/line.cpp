#include <algorithm>
#include <line.hpp>

namespace model_fitting {
LineData::LineData(std::vector<std::pair<Point, int>> line) {
  line_.assign(line.begin(), line.end());
}


Point LineData::get_avg() {
  Point temp(0, 0, 0);
  for (auto p : line_) {
    temp += p.first;
  }
  temp /= line_.size();
  return temp;
}


void LineData::applyTransform(const Eigen::Matrix4f& rot_matrix) {
  for (int i = 0; i < line_.size(); i++) {
    Eigen::Vector4f v(line_[i].first.x(),
                      line_[i].first.y(),
                      line_[i].first.z(),
                      1);
    v = rot_matrix * v;
    line_[i].first = Point(v[0], v[1], v[2]);
  }
}


void LineData::reorder(int option) {
  if (option == 0) {
    std::sort(line_.begin(), line_.end(),
              [](std::pair<Point, int> a, std::pair<Point, int> b)
              {return a.first.x() > b.first.x(); });
  } else if (option == 1) {
    std::sort(line_.begin(), line_.end(),
              [](std::pair<Point, int> a, std::pair<Point, int> b)
              {return a.first.y() > b.first.y(); });
  } else if (option == 2) {
    std::sort(line_.begin(), line_.end(),
              [](std::pair<Point, int> a, std::pair<Point, int> b)
              {return a.first.z() > b.first.z(); });
  } else {
    std::cerr << "Wrong Option." << std::endl;
  }
}


void purify(std::vector<LineData>& lines) {
  extern char* m_type;
  std::string str(m_type);
  // Reorder points in line according to variance of 3 axis
  std::vector<double> sum{0, 0, 0};
  std::vector<double> mean{0, 0, 0};
  std::vector<double> var{0, 0, 0};
  auto line_1 = lines[1].get_line();
  std::for_each(line_1.begin(), line_1.end(),
                [&](const std::pair<Point, int> p) {
                sum[0] += p.first.x();
                sum[1] += p.first.y();
                sum[2] += p.first.z();
                });

  mean[0] = sum[0] / line_1.size();
  mean[1] = sum[1] / line_1.size();
  mean[2] = sum[2] / line_1.size();

  std::for_each(line_1.begin(), line_1.end(),
                [&](const std::pair<Point, int> p) {
    var[0] += (p.first.x() - mean[0]) * (p.first.x() - mean[0]);
    var[1] += (p.first.y() - mean[1]) * (p.first.y() - mean[1]);
    var[2] += (p.first.z() - mean[2]) * (p.first.z() - mean[2]);
  });

  auto max_it = std::max_element(var.begin(), var.end());
  int option = std::distance(var.begin(), max_it);

  for (int idx = 0; idx < lines.size(); idx++) {
    lines[idx].reorder(option);
  }

  // Remove edge outlier
  for (int idx = 0; idx < lines.size(); idx++) {
    if (lines[idx].get_size() > 5) {
      std::vector<std::pair<Point, int>> l_temp = lines[idx].get_line();
      int len = l_temp.size();
      if (Distance(l_temp[len-1].first, l_temp[len-2].first)
          > Distance(l_temp[len-2].first, l_temp[len-3].first) * 2) {
        lines[idx].erase_point(len-1);
      }
      if (Distance(l_temp[0].first, l_temp[1].first)
          > Distance(l_temp[1].first, l_temp[2].first) * 2 ) {
        lines[idx].erase_point(0);
      }
    }
  }

  std::vector<int> erase_set;
  if (str.compare("triangle") == 0) {
    // Remove tube (triangle board)
    for (int i = 1; i < lines.size(); i++) {
      std::vector<std::pair<Point, int>> l1 = lines[i-1].get_line();
      std::vector<std::pair<Point, int>> l2 = lines[i].get_line();
      double len_1 = (l1[l1.size()-1].first - l1[0].first).norm();
      double len_2 = (l2[l2.size()-1].first - l2[0].first).norm();
      if (len_2 < len_1) {
        for (int j = i + 1; j < lines.size(); j++)
          erase_set.push_back(j);
        if ( i > 1 && (Distance(lines[i].get_avg(), lines[i-1].get_avg())
                 > Distance(lines[i-1].get_avg(), lines[i-2].get_avg()) * 2)) {
            erase_set.push_back(i);
          break;
        }
        break;
      }
      if ( i > 1 && (Distance(lines[i].get_avg(), lines[i-1].get_avg())
               > Distance(lines[i-1].get_avg(), lines[i-2].get_avg()) * 2)) {
        for (int j = i; j < lines.size(); j++)
          erase_set.push_back(j);
        break;
      }
    }
  } else if (str.compare("square") == 0) {
    // Remove tube (sqare board)
    for (int i = 1; i < lines.size(); i++) {
      std::vector<std::pair<Point, int>> l1 = lines[i-1].get_line();
      std::vector<std::pair<Point, int>> l2 = lines[i].get_line();
      double len_1 = (l1[l1.size()-1].first - l1[0].first).norm();
      double len_2 = (l2[l2.size()-1].first - l2[0].first).norm();

      if (len_2 < len_1) {
        for (int j = 2 * (i-1) + 1; j < lines.size(); j++)
          erase_set.push_back(j);
        break;
      } else if ( i > 1 && (Distance(lines[i].get_avg(), lines[i-1].get_avg())
               > Distance(lines[i-1].get_avg(), lines[i-2].get_avg()) * 2)) {
        for (int j = i; j < lines.size(); j++)
          erase_set.push_back(j);
        break;
      }
    }
  }
  std::reverse(erase_set.begin(), erase_set.end());
  for (int idx : erase_set) {
    lines.erase(lines.begin() + idx);
  }

  // Remove undesirable lines (tube, ground, etc.)
  for (int i = 2; i < lines.size(); i++) {
    if ((Distance(lines[i].get_avg(), lines[i-1].get_avg())
        > Distance(lines[i-1].get_avg(), lines[i-2].get_avg()) * 2)) {
      int temp = lines.size();
      for (int idx = i; idx < temp; idx++) {
        lines.erase(lines.begin() + idx);
      }
      break;
    }
  }
}


std::vector<LineData> edge_extract(const std::vector<LineData>& lines) {
  std::vector<LineData> new_lines;
  for (auto line : lines) {
    if (line.get_size() <=  2) {
      new_lines.push_back(line);
    } else {
      std::vector<std::pair<Point, int>> line_temp = line.get_line();
      LineData new_line;
      new_line.add_point(line_temp[0].first, line_temp[0].second);
      new_line.add_point(line_temp[line_temp.size()-1].first
                           , line_temp[line_temp.size()-1].second);
      new_lines.push_back(new_line);
    }
  }
  return new_lines;
}


Point compute3Dcentroid(const std::vector<LineData>& lines) {
  Point temp(0, 0, 0);
  int count = 0;
  for (auto l : lines) {
    for (auto p : l.get_line()) {
      temp += p.first;
      count++;
    }
  }
  temp /= count;
  return temp;
}


std::vector<LineData> transformLines(const Eigen::Matrix4f& rot_matrix,
                                     const std::vector<LineData>& lines) {
  std::vector<LineData> new_lines;
  for (int i = 0; i < lines.size(); i++) {
    LineData l = lines[i];
    l.applyTransform(rot_matrix);
    new_lines.push_back(l);
  }
  return new_lines;
}
}  // namespace model_fitting
