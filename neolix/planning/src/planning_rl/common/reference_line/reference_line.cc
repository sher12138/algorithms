/**
 * @file reference_line.cpp
 **/

#include "reference_line.h"

namespace neodrive {
namespace planning_rl {

ReferenceLine::ReferenceLine(const ReferencePointVec1d& reference_points)
    : ref_points_(reference_points) {
  init();
}

ReferenceLine::ReferenceLine(const ReferenceLine& line) {
  ref_points_ = line.ref_points();
  init();
}

ReferenceLine::ReferenceLine(const PlanningRLMap::MapPoint3d& map_points3d) {
  auto pairs_map_points3d = pairs_map_points(map_points3d);
  auto curvature_and_theta = cal_map_points_curvature(pairs_map_points3d[1]);
  for (size_t i = 0; i < pairs_map_points3d[1].size(); i++) {
    ReferencePoint reference_point(
        Vec2d(pairs_map_points3d[1][i].x, pairs_map_points3d[1][i].y),
        curvature_and_theta[1][i], curvature_and_theta[0][i],
        Vec2d(pairs_map_points3d[0][i].x, pairs_map_points3d[0][i].y),
        Vec2d(pairs_map_points3d[2][i].x, pairs_map_points3d[2][i].y),
        pairs_map_points3d[1][i].x, pairs_map_points3d[1][i].lane_id);
    ref_points_.push_back(reference_point);
  }
  if (!add_road_boundary_to_reference_points()) {
    LOG_INFO("add_road_boundary_to_reference_points failed!");
  }
  ComputeReferenceLineS(ref_points_);
  // init();
  // Json::Value root = to_json();
  // printf("%s\n", root.toStyledString().c_str());
}

PlanningRLMap::MapPoint3d ReferenceLine::pairs_map_points(
    const PlanningRLMap::MapPoint3d& map_points) {
  PlanningRLMap::MapPoint2d left_map_points;
  PlanningRLMap::MapPoint2d right_map_points;
  PlanningRLMap::MapPoint3d new_map_points;
  for (size_t i = 0; i < map_points[1].size(); i++) {
    auto left_index = PlanningRLMap::Instance().find_closest_map_points_index(
        map_points[1][i], map_points[0]);
    auto right_index = PlanningRLMap::Instance().find_closest_map_points_index(
        map_points[1][i], map_points[2]);
    left_map_points.push_back(map_points[0][left_index]);
    right_map_points.push_back(map_points[2][right_index]);
  }

  new_map_points.push_back(left_map_points);
  new_map_points.push_back(map_points[1]);
  new_map_points.push_back(right_map_points);
  return new_map_points;
}

ReferencePointVec1d ReferenceLine::ComputeReferenceLineS(
    ReferencePointVec1d& refline_points) {
  double s = 0.0;
  int point_num = refline_points.size();
  if (point_num > 0) {
    refline_points[0].set_s(s);
  }
  for (int i = 0; i < point_num - 1; i++) {
    s = s + sqrt(pow(refline_points[i + 1].x() - refline_points[i].x(), 2) +
                 pow(refline_points[i + 1].y() - refline_points[i].y(), 2));
    refline_points[i + 1].set_s(s);
  }
  return refline_points;
}

std::vector<std::vector<double>> ReferenceLine::cal_map_points_curvature(
    const PlanningRLMap::MapPoint2d& map_points) {
  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<double> theta;
  for (size_t i = 0; i < map_points.size(); i++) {
    x_list.push_back(map_points[i].x);
    y_list.push_back(map_points[i].y);
  }
  std::vector<std::vector<double>> curvature_and_theta;
  cal_curvature(x_list, y_list, curvature_and_theta);
  return curvature_and_theta;
}

bool ReferenceLine::get_closest_reference_point(const Vec2d& xy_point,
                                                ReferencePoint& closet_index) {
  if (ref_points_.size() < 1) {
    // LOG_ERROR("ref_points_ is empty");
    return false;
  }

  double dis = 1000.0;

  // pay attention, cannot enlarge this one
  // you can change tol, and dis_tol to reduce the calculation
  for (std::size_t i = 0; i < ref_points_.size() - 1; ++i) {
    double tmp_dis = std::pow(ref_points_[i].x() - xy_point.x(), 2) +
                     std::pow(ref_points_[i].y() - xy_point.y(), 2);
    if (tmp_dis < dis) {
      dis = tmp_dis;
      closet_index = ref_points_[i];
    }
  }
  return true;
}

// void ReferenceLine::cartesian_to_frenet(const ReferencePoint& pt,
//                                         const Vec2d& xy_point, SLPoint&
//                                         sl_pt) {
//   double loc_x{0.0}, loc_y{0.0}, loc_theta{0.0};
//   earth2vehicle(pt.x(), pt.y(), pt.heading(), xy_point.x(), xy_point.y(),
//   0.0,
//                 loc_x, loc_y, loc_theta);
//   sl_pt.set_l(loc_y);
//   sl_pt.set_s(pt.s() + loc_x);

//   return;
// }

// bool ReferenceLine::frenet_to_cartesian(const ReferencePoint& pt,
//                                         const SLPoint& sl_pt, Vec2d&
//                                         xy_point) {
//   double ptr_x{0.0}, ptr_y{0.0}, ptr_theta{0.0}, ptr_kappa{0.0};
//   std::array<double, 3> d_condition{};
//   d_condition[0] = sl_pt.l();
//   d_condition[1] = 0.0;
//   d_condition[2] = 0.0;

//   bool bflag = converter_.frenet_to_cartesian(
//       pt.s(), pt.x(), pt.y(), pt.heading(), pt.kappa(), pt.dkappa(),
//       sl_pt.s(), d_condition, &ptr_x, &ptr_y, &ptr_theta, &ptr_kappa);
//   xy_point.set_x(ptr_x);
//   xy_point.set_y(ptr_y);
//   return bflag;
// }

ReferencePoint ReferenceLine::front_point() const {
  if (ref_points_.empty()) {
    ReferencePoint tmp_pt;
    return tmp_pt;
  }
  return ref_points_.front();
}

ReferencePoint ReferenceLine::back_point() const {
  if (ref_points_.empty()) {
    ReferencePoint tmp_pt;
    return tmp_pt;
  }
  return ref_points_.back();
}

Json::Value ReferenceLine::to_json() const {
  Json::Value root;
  for (std::size_t i = 0; i < ref_points_.size(); ++i) {
    root.append(ref_points_[i].to_json());
  }
  return root;
}

void ReferenceLine::set_ref_points(const ReferencePointVec1d& ref_points) {
  ref_points_ = ref_points;
  init();
}

const ReferencePointVec1d& ReferenceLine::ref_points() const {
  return ref_points_;
}

ReferencePointVec1d* ReferenceLine::mutable_ref_points() {
  return &ref_points_;
}

void ReferenceLine::init() {
  if (ref_points_.size() < 3) {
    LOG_ERROR("The points in ref_line is too few: %d", ref_points_.size());
    return;
  }
}
bool ReferenceLine::add_road_boundary_to_reference_points() {
  if (ref_points_.size() < 3) {
    LOG_ERROR("The points in ref_line is too few: %d", ref_points_.size());
    return false;
  }
  std::string tmp_lane_id;
  PlanningRLMap::MapPoint2d left_road_bound_points;
  PlanningRLMap::MapPoint2d right_road_bound_points;
  for (auto& reference_point : ref_points_) {
    auto lane_id = reference_point.hd_map_lane_id();
    if (tmp_lane_id != lane_id) {
      tmp_lane_id = lane_id;
      PlanningRLMap::Instance().GetRoadBoundPointsByLaneId(
          lane_id, left_road_bound_points, right_road_bound_points);
    }
    if (left_road_bound_points.size() > 0) {
      auto left_index = PlanningRLMap::Instance().find_closest_map_points_index(
          reference_point.x(), reference_point.y(), left_road_bound_points);
      reference_point.set_left_road_bound_point(
          PlanningRLMap::ConverMapPointToVec2d(
              left_road_bound_points[left_index]));
    }
    if (right_road_bound_points.size() > 0) {
      auto right_index =
          PlanningRLMap::Instance().find_closest_map_points_index(
              reference_point.x(), reference_point.y(),
              right_road_bound_points);
      reference_point.set_right_road_bound_point(
          PlanningRLMap::ConverMapPointToVec2d(
              right_road_bound_points[right_index]));
    }
  }
  return true;
}

}  // namespace planning_rl
}  // namespace neodrive
