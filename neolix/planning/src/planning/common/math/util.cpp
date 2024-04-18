#include "util.h"

namespace neodrive {
namespace planning {

/*Coordiante transformation, x2y,y2(-x)*/
// change into veh coordinate: front x +, left y +
// x_earth: start pos;x_target:end pos; x_target_local:end pos in start ISO coor
bool earth2vehicle(const double x_earth, const double y_earth,
                   const double theta_earth, const double x_target,
                   const double y_target, const double theta_target,
                   double &x_target_local, double &y_target_local,
                   double &theta_target_local) {
  x_target_local = (x_target - x_earth) * std::cos(theta_earth) +
                   (y_target - y_earth) * std::sin(theta_earth);
  y_target_local = (x_earth - x_target) * std::sin(theta_earth) +
                   (y_target - y_earth) * std::cos(theta_earth);
  theta_target_local = theta_target - theta_earth;
  return true;
}

// x_earth: start pos; x_target_local: local coordinate to
// x_earth;x_target_earth: transfer to world coor
bool vehicle2earth(const double x_earth, const double y_earth,
                   const double theta_earth, const double x_target_local,
                   const double y_target_local, const double theta_target_local,
                   double &x_target_earth, double &y_target_earth,
                   double &theta_target_earth) {
  x_target_earth = x_target_local * std::cos(theta_earth) -
                   y_target_local * std::sin(theta_earth) + x_earth;
  y_target_earth = x_target_local * std::sin(theta_earth) +
                   y_target_local * std::cos(theta_earth) + y_earth;
  theta_target_earth = theta_earth + theta_target_local;
  return true;
}

bool PolygonCollision(const std::vector<Vec2d> &self_bound,
                      const std::vector<Vec2d> &other_bound) {
  if (other_bound.size() < 3 || self_bound.size() < 3) return false;
  std::size_t i_vertex = self_bound.size();
  bool bcollide = false;
  for (std::size_t i = 0; i < i_vertex; ++i) {
    bcollide =
        PointInPolygon(self_bound[i].x(), self_bound[i].y(), other_bound);
    if (bcollide) return bcollide;
  }
  for (std::size_t i = 0; i < other_bound.size(); ++i) {
    bcollide =
        PointInPolygon(other_bound[i].x(), other_bound[i].y(), self_bound);
    if (bcollide) return bcollide;
  }
  return bcollide;
}

// bool PointInPolygon(const double &x, const double &y,
//                    const std::vector<Vec2d> &bound) {
//  int inum, icount_vertex, quadrant_1, quadrant_2, sum, f;
//  icount_vertex = bound.size();
//  if (icount_vertex <= 1) return false;
//  std::vector<Vec2d> tmp_vertex;
//  for (int i = 0; i < icount_vertex; ++i) tmp_vertex.push_back(bound[i]);
//  tmp_vertex.push_back(bound[0]);
//  // improved riks method
//  for (inum = 0; inum <= icount_vertex; ++inum) {
//    tmp_vertex[inum].set_x(tmp_vertex[inum].x() - x);
//    tmp_vertex[inum].set_y(tmp_vertex[inum].y() - y);
//  }
//  quadrant_1 = tmp_vertex[0].x() >= 0 ? (tmp_vertex[0].y() >= 0 ? 0 : 3)
//                                      : (tmp_vertex[0].y() >= 0 ? 1 : 2);
//  for (sum = 0, inum = 1; inum <= icount_vertex; ++inum) {
//    if (!tmp_vertex[inum].x() && !tmp_vertex[inum].y()) break;
//    f = (int)(tmp_vertex[inum].y() * tmp_vertex[inum - 1].x() -
//              tmp_vertex[inum].x() * tmp_vertex[inum - 1].y());
//    if (!f && tmp_vertex[inum - 1].x() * tmp_vertex[inum].x() <= 0 &&
//        tmp_vertex[inum - 1].y() * tmp_vertex[inum].y() <= 0)
//      break;
//    quadrant_2 = tmp_vertex[inum].x() >= 0
//                     ? (tmp_vertex[inum].y() >= 0 ? 0 : 3)
//                     : (tmp_vertex[inum].y() >= 0 ? 1 : 2);
//    if (quadrant_2 == (quadrant_1 + 1) % 4)
//      sum += 1;
//    else if (quadrant_2 == (quadrant_1 + 3) % 4)
//      sum -= 1;
//    else if (quadrant_2 == (quadrant_1 + 2) % 4) {
//      if (f > 0)
//        sum += 2;
//      else
//        sum -= 2;
//    }
//    quadrant_1 = quadrant_2;
//  }
//  if (inum <= icount_vertex || sum) return true;
//  return false;
//}

bool PointInPolygon(const double &x, const double &y,
                    const std::vector<Vec2d> &bound) {
  long long int_x = (long long)(x * 100);
  long long int_y = (long long)(y * 100);

  std::vector<std::pair<long long, long long>> bound_int;
  for (std::size_t i = 0; i < bound.size(); ++i) {
    long long temp_x = (long long)(bound[i].x() * 100);
    long long temp_y = (long long)(bound[i].y() * 100);
    bound_int.push_back(std::make_pair(temp_x, temp_y));
  }

  return PointInPolygonINT(int_x, int_y, bound_int);
}

bool PointInPolygonINT(
    const long long &x, const long long &y,
    const std::vector<std::pair<long long, long long>> &bound) {
  long long quadrant_1 = 0, quadrant_2 = 0, sum = 0, f = 0;

  std::size_t icount_vertex = 0;
  icount_vertex = bound.size();
  if (icount_vertex <= 1) return false;

  std::vector<std::pair<long long, long long>> tmp_vertex;
  for (std::size_t i = 0; i < icount_vertex; ++i) {
    tmp_vertex.push_back(bound[i]);
  }
  tmp_vertex.push_back(bound[0]);
  // improved riks method
  std::size_t inum = 0;
  for (inum = 0; inum <= icount_vertex; ++inum) {
    tmp_vertex[inum].first = tmp_vertex[inum].first - x;
    tmp_vertex[inum].second = tmp_vertex[inum].second - y;
  }
  quadrant_1 = tmp_vertex[0].first >= 0 ? (tmp_vertex[0].second >= 0 ? 0 : 3)
                                        : (tmp_vertex[0].second >= 0 ? 1 : 2);
  for (sum = 0, inum = 1; inum <= icount_vertex; ++inum) {
    if (!tmp_vertex[inum].first && !tmp_vertex[inum].second) break;
    f = (long long)(tmp_vertex[inum].second * tmp_vertex[inum - 1].first -
                    tmp_vertex[inum].first * tmp_vertex[inum - 1].second);
    if (!f && tmp_vertex[inum - 1].first * tmp_vertex[inum].first <= 0 &&
        tmp_vertex[inum - 1].second * tmp_vertex[inum].second <= 0)
      break;
    quadrant_2 = tmp_vertex[inum].first >= 0
                     ? (tmp_vertex[inum].second >= 0 ? 0 : 3)
                     : (tmp_vertex[inum].second >= 0 ? 1 : 2);
    if (quadrant_2 == (quadrant_1 + 1) % 4)
      sum += 1;
    else if (quadrant_2 == (quadrant_1 + 3) % 4)
      sum -= 1;
    else if (quadrant_2 == (quadrant_1 + 2) % 4) {
      if (f > 0)
        sum += 2;
      else
        sum -= 2;
    }
    quadrant_1 = quadrant_2;
  }
  if (inum <= icount_vertex || sum) return true;
  return false;
}

bool VectorValidTest(const std::vector<Vec2d> &points) {
  int num_points = points.size();
  if (num_points <= 3) return false;

  // Make sure the points are in ccw order.
  double area = 0.0;
  for (int i = 1; i < num_points; ++i) {
    area += cross_prod(points[0], points[i - 1], points[i]);
  }
  if (area < 0) {
    area = -area;
  }
  area /= 2.0;
  if (area >= 1e-10)
    return true;
  else
    return false;
}

bool LineIntersectWithPolygon(const Vec2d &pt1, const Vec2d &pt2,
                              const std::vector<Vec2d> &bound) {
  Segment2d line_(pt1, pt2);
  // if pt1 or pt2 not in polygon, return true,
  // means has inersection or the line not in polygon.
  bool ret = false;
  if (PointInPolygon(pt1.x(), pt1.y(), bound) ^
      PointInPolygon(pt2.x(), pt2.y(), bound)) {
    return true;
  }
  // traverse all line in polygon
  for (std::size_t i = 0; i < bound.size(); ++i) {
    if (i == (bound.size() - 1)) {
      Segment2d tmp_line(bound[i], bound[0]);
      if (tmp_line.has_intersect(line_)) {
        ret = true;
        break;
      }
    } else {
      Segment2d tmp_line(bound[i], bound[i + 1]);
      if (tmp_line.has_intersect(line_)) {
        ret = true;
        break;
      }
    }
  }
  return ret;
}

bool calc_left_right_dis_to_freespace(const std::vector<Vec2d> &free_space,
                                      const double &free_space_x_range,
                                      const double &free_space_y_range,
                                      const Vec2d &pt, const double &theta,
                                      double *left_dis, double *right_dis) {
  // get freespace max range
  double max_range = 10 * std::hypot(free_space_x_range, free_space_y_range);
  double perpen_theta = theta + M_PI_2;
  Vec2d max_pt(pt.x() + max_range * cos(perpen_theta),
               pt.y() + max_range * sin(perpen_theta));
  Vec2d min_pt(pt.x() - max_range * cos(perpen_theta),
               pt.y() - max_range * sin(perpen_theta));
  // build two line segment
  Segment2d up_line(pt, max_pt);
  Segment2d down_line(pt, min_pt);
  // traverse all line in polygon
  std::vector<Vec2d> intersections;
  for (std::size_t i = 0; i < free_space.size(); ++i) {
    if (i == (free_space.size() - 1)) {
      Segment2d tmp_line(free_space[i], free_space[0]);
      Vec2d tmp_pt;
      if (tmp_line.get_intersect(up_line, &tmp_pt)) {
        intersections.push_back(tmp_pt);
      }
      if (tmp_line.get_intersect(down_line, &tmp_pt)) {
        intersections.push_back(tmp_pt);
      }
    } else {
      Segment2d tmp_line(free_space[i], free_space[i + 1]);
      Vec2d tmp_pt;
      if (tmp_line.get_intersect(up_line, &tmp_pt)) {
        intersections.push_back(tmp_pt);
      }
      if (tmp_line.get_intersect(down_line, &tmp_pt)) {
        intersections.push_back(tmp_pt);
      }
    }
  }
  if (intersections.size() < 2) {
    LOG_ERROR(
        "calc_left_right_dis_to_freespace "
        "get {} intersections!",
        intersections.size());
    // for (auto pt : intersections) {
    //   LOG_ERROR("({}, {})", pt.x(), pt.y());
    // }
    return false;
  }

  // change into local coordinate.
  double loc_x = 0.0, loc_y = 0.0, loc_theta = 0.0;
  double left_min_dis = 10000.0;
  double right_min_dis = 10000.0;
  const double max_dis = 1000.0;
  const double min_dis = 0.0;
  for (std::size_t i = 0; i < intersections.size(); ++i) {
    if (!earth2vehicle(pt.x(), pt.y(), theta, intersections[i].x(),
                       intersections[i].y(), 0.0, loc_x, loc_y, loc_theta)) {
      LOG_ERROR("earth2vehicle err");
      return false;
    }
    double tmp_dis = hypot(loc_x, loc_y);
    // x axis, left positive, right negative.
    if (loc_y >= min_dis && tmp_dis < left_min_dis) {
      left_min_dis = tmp_dis;
    } else if (loc_y < min_dis && tmp_dis < right_min_dis) {
      right_min_dis = tmp_dis;
    }
    // LOG_DEBUG(
    //     "{}, tmp_dis {}, intersections ({}, {}), loc_x/y ({}, {}), left/right
    //     " "min_dis ({}, "
    //     "{})",
    //     i, tmp_dis, intersections[i].x(), intersections[i].y(), loc_x, loc_y,
    //     left_min_dis, right_min_dis);
  }
  *left_dis = left_min_dis;
  *right_dis = right_min_dis;

  // LOG_DEBUG("final left/right dis ({}, {})", *left_dis, *right_dis);
  if (*left_dis > max_dis || *left_dis < min_dis) {
    LOG_DEBUG("*left_dis {} is out of bound ({}, {})", *left_dis, min_dis,
              max_dis);
    *left_dis = 0.0;
    return false;
  } else if (*right_dis > max_dis || *right_dis < min_dis) {
    LOG_DEBUG("*right_dis {} is out of bound ({}, {})", *right_dis, min_dis,
              max_dis);
    *right_dis = 0.0;
    return false;
  } else {
  }

  return true;
}

bool calc_min_dis_from_freespace_to_point(const std::vector<Vec2d> &free_space,
                                          const Vec2d &pt, double *min_dis) {
  // traverse all freespace segment lines and find min dis
  *min_dis = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < free_space.size(); ++i) {
    if (i == (free_space.size() - 1)) {
      Segment2d tmp_line(free_space[i], free_space[0]);
      double dis = tmp_line.distance_to(pt);
      *min_dis = fmin(dis, *min_dis);
    } else {
      Segment2d tmp_line(free_space[i], free_space[i + 1]);
      double dis = tmp_line.distance_to(pt);
      *min_dis = fmin(dis, *min_dis);
    }
  }
  return true;
}

bool calc_min_dis_from_freespace_to_segment(
    const std::vector<Vec2d> &free_space, const Vec2d &pt1, const Vec2d &pt2,
    double *min_dis) {
  Segment2d tmp_line(pt1, pt2);
  // traverse all freespace segment lines and find min dis
  *min_dis = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < free_space.size(); ++i) {
    double dis = tmp_line.distance_to(free_space[i]);
    *min_dis = fmin(dis, *min_dis);
  }
  return true;
}

double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol) {
  static constexpr double gr = 1.618033989;  // (sqrt(5) + 1) / 2

  double a = lower_bound;
  double b = upper_bound;

  double t = (b - a) / gr;
  double c = b - t;
  double d = a + t;

  while (std::abs(c - d) > tol) {
    if (func(c) < func(d)) {
      b = d;
    } else {
      a = c;
    }
    t = (b - a) / gr;
    c = b - t;
    d = a + t;
  }
  return (a + b) * 0.5;
}

}  // namespace planning
}  // namespace neodrive
