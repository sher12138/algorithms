/**
 * @file smooth.cpp
 **/
#include "smooth.h"
#include <cmath>
#include <iostream>
#include "common/planning_map/planning_map.h"

namespace neodrive {
namespace planning_rl {

Smooth::Smooth() {}

bool Smooth::linear_interpolate(
    std::pair<double, double>& start_point,
    std::pair<double, double>& end_point,
    std::vector<std::pair<double, double>>* new_points,
    const double resolution) {
  if (new_points == nullptr) return false;
  double orin_x = start_point.first;
  double orin_y = start_point.second;

  double new_end_x = end_point.first - orin_x;
  double new_end_y = end_point.second - orin_y;
  // double set_orin_x = 0.0;
  // double set_orin_y = 0.0;
  new_points->clear();
  double s = sqrt(pow(new_end_x, 2) + pow(new_end_y, 2));
  if (s <= 2 * resolution) {
    new_points->push_back(start_point);
    new_points->push_back(end_point);
    return true;
  }

  int count = std::floor(s / resolution);
  for (int i = 0; i < count; ++i) {
    double tmp_y = (new_end_y) / count * i;
    double tmp_x = (new_end_x) / count * i;
    new_points->push_back(std::make_pair(tmp_x + orin_x, tmp_y + orin_y));
  }
  return true;
}

bool Smooth::linear_interpolate(PlanningRLMap::MapPoint& start_point,
                                PlanningRLMap::MapPoint& end_point,
                                PlanningRLMap::MapPoint2d& new_points,
                                const double resolution) {
  double orin_x = start_point.x;
  double orin_y = start_point.y;

  double new_end_x = end_point.x - orin_x;
  double new_end_y = end_point.y - orin_y;
  // double set_orin_x = 0.0;
  // double set_orin_y = 0.0;
  new_points.clear();
  double s = sqrt(pow(new_end_x, 2) + pow(new_end_y, 2));
  if (s <= 2 * resolution) {
    new_points.push_back(start_point);
    new_points.push_back(end_point);
    return true;
  }

  int count = std::floor(s / resolution);
  for (int i = 0; i < count; ++i) {
    double tmp_y = (new_end_y) / count * i;
    double tmp_x = (new_end_x) / count * i;
    if (start_point.lane_id.empty()) {
      new_points.push_back(
          PlanningRLMap::MapPoint{tmp_x + orin_x, tmp_y + orin_y, 0.0, 0.0});
    } else {
      new_points.push_back(PlanningRLMap::MapPoint{
          tmp_x + orin_x, tmp_y + orin_y, 0.0, 0.0, 0.0, start_point.lane_id});
    }
  }
  new_points.push_back(end_point);
  return true;
}

PlanningRLMap::MapPoint2d Smooth::line_linear_interpolate(
    PlanningRLMap::MapPoint2d& path) {
  PlanningRLMap::MapPoint2d new_points;
  PlanningRLMap::MapPoint2d tmp_points;
  for (size_t i = 0; i < path.size() - 1; i++) {
    linear_interpolate(path[i], path[i + 1], tmp_points, 0.2);
    for (std::size_t j = 0; j < tmp_points.size(); ++j) {
      new_points.push_back(tmp_points[j]);
    }
    if (i != path.size() - 2) {
      new_points.pop_back();
    }
  }
  return new_points;
}

bool Smooth::smooth_path(PlanningRLMap::MapPoint2d& path,
                         PlanningRLMap::MapPoint2d& new_points,
                         double weight_data = 0.5, double weight_smooth = 0.2,
                         double tolerance = 0.000001) {
  new_points.assign(path.begin(), path.end());

  double change = tolerance;
  while (change >= tolerance) {
    change = 0.0;

    for (size_t i = 1; i < new_points.size() - 1; i++) {
      auto aux = new_points[i].x;
      new_points[i].x += weight_data * (path[i].x - new_points[i].x);
      new_points[i].x +=
          weight_smooth *
          (new_points[i - 1].x + new_points[i + 1].x - (2.0 * new_points[i].x));
      change += fabs(aux - new_points[i].x);

      aux = new_points[i].y;
      new_points[i].y += weight_data * (path[i].y - new_points[i].y);
      new_points[i].y +=
          weight_smooth *
          (new_points[i - 1].y + new_points[i + 1].y - (2.0 * new_points[i].y));
      change += fabs(aux - new_points[i].y);
    }
  }
  return true;
}

bool Smooth::smooth_path(std::vector<std::vector<double>>& path,
                         std::vector<std::vector<double>>& new_points,
                         double weight_data = 0.5, double weight_smooth = 0.2,
                         double tolerance = 0.000001) {
  new_points.assign(path.begin(), path.end());

  double change = tolerance;
  while (change >= tolerance) {
    change = 0.0;

    for (size_t i = 1; i < new_points.size() - 1; i++) {
      for (size_t j = 0; j < 2; j++) {
        auto aux = new_points[i][j];
        new_points[i][j] += weight_data * (path[i][j] - new_points[i][j]);
        new_points[i][j] +=
            weight_smooth * (new_points[i - 1][j] + new_points[i + 1][j] -
                             (2.0 * new_points[i][j]));
        change += fabs(aux - new_points[i][j]);
      }
    }
  }
  return true;
}

}  // namespace planning_rl
}  // namespace neodrive