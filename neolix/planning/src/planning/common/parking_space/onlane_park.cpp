#include "onlane_park.h"

#include "common/data_center/data_center.h"
#include "src/planning/common/parking_space/park_util.h"

namespace neodrive {
namespace planning {

OnlanePark::OnlanePark(cyberverse::ParkingSpaceInfoConstPtr park)
    : ParkingSpace(park) {
  if (!park) return;
  auto &points = park->Points();
  auto hdmap = cyberverse::HDMap::Instance();
  auto overlap_lane_id = park->OverlapLaneIds()[0];
  overlap_lane_ptr_ = hdmap->GetLaneById(overlap_lane_id);
  heading_ = overlap_lane_ptr_ != nullptr
                 ? overlap_lane_ptr_->Heading(3.) + M_PI_2
                 : park->Heading() + M_PI_2;
  origin_pos_ = Vec3d(points[3].x, points[3].y, heading_);
  length_ = std::hypot(points[0].y - points[3].y, points[0].x - points[3].x);
  width_ = std::hypot(points[1].y - points[0].y, points[1].x - points[0].x);
}

double OnlanePark::Length() {
  if (!park_ptr_) return 0.0;
  auto &points = park_ptr_->Points();
  return std::hypot(points[0].y - points[1].y, points[0].x, points[1].x);
}

double OnlanePark::Width() {
  if (!park_ptr_) return 0.0;
  auto &points = park_ptr_->Points();
  return std::hypot(points[1].y - points[2].y, points[1].x, points[2].x);
}

double OnlanePark::GetDistToEnd() {
  if (!park_ptr_) return 0.0;
  return length_;
}

double OnlanePark::Heading() {
  if (!park_ptr_) return 0.0;
  return park_ptr_->Heading();
}

void OnlanePark::Solve() {
  if (is_park_in_) {
    GenerateParkInPath();
  } else {
    GenerateParkOutPath();
  }
}

void OnlanePark::Reset() { ; }

bool OnlanePark::CheckNeedStitchPath() { return false; }

ParkingPath &OnlanePark::GetStitchPath() {
  ParkingPath parking_path{};
  return parking_path;
}

void OnlanePark::GenerateParkInPath() {
  const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
  Vec3d point_a{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
  auto &points = park_ptr_->Points();
  double dis_to_end{config::PlanningConfig::Instance()
                        ->plan_config()
                        .parking.onlane_parking_space.dis_to_end};
  Vec3d point_b_rel{-0.5 * width_,
                    -dis_to_end - ParkingSpace::kParkingPathExternDist,
                    -M_PI_2};
  Vec3d point_a_rel;
  common::ConvertToRelativeCoordinate(point_a, origin_pos_, point_a_rel);
  ParkingPath path_ab;
  QuinticCurveFiting(point_a_rel, point_b_rel, path_ab.path_points, 0.02, true);
  path_ab.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path_ab.check_type = ParkingPath::DistEndCheck;
  path_ab.stop_befor_end = ParkingSpace::kParkingPathExternDist;
  path_ab.is_park_in = true;
  path_ab.is_need_stop = true;
  path_ab.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < path_ab.path_points.size(); ++i) {
    path_ab.point_s.emplace_back(
        path_ab.point_s.back() +
        common::Distance2D(path_ab.path_points[i], path_ab.path_points[i - 1]));
  }
  TransformToWorldCorrdinate(path_ab, origin_pos_);
  park_path_.emplace_back(path_ab);
}

void OnlanePark::GenerateParkOutPath() {
  park_path_.clear();
  if (overlap_lane_ptr_ == nullptr) return;
  auto successor = GetOnlySuccessor(overlap_lane_ptr_);
  auto next_successor = GetOnlySuccessor(successor);
  std::vector<Vec3d> overlap_lane_rel_pts, successor_lane_rel_pts,
      next_successor_lane_rel_pts;
  GetLaneRelativePoints(overlap_lane_ptr_, origin_pos_, overlap_lane_rel_pts);
  GetLaneRelativePoints(successor, origin_pos_, successor_lane_rel_pts);
  GetLaneRelativePoints(next_successor, origin_pos_,
                        next_successor_lane_rel_pts);
  const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
  Vec3d point_a{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
  Vec3d point_a_rel;
  common::ConvertToRelativeCoordinate(point_a, origin_pos_, point_a_rel);
  ParkingPath parking_out_path;
  parking_out_path.path_points.emplace_back(point_a_rel);
  LOG_INFO("point a rel x = {:.2f}, y = {:.2f}, z = {:.2f}", point_a_rel.x(),
           point_a_rel.y(), point_a_rel.z());
  for (const auto &point : overlap_lane_rel_pts) {
    LOG_INFO("overlap lane rel pt x = {:.2f}, y = {:.2f}, z = {:.2f}",
             point.x(), point.y(), point.z());
    if (point.y() > parking_out_path.path_points.back().y() - 0.1) continue;
    std::vector<Vec3d> tmp_path;
    QuinticCurveFiting(parking_out_path.path_points.back(), point, tmp_path,
                       0.02, true);
    parking_out_path.path_points.insert(parking_out_path.path_points.end(),
                                        tmp_path.begin(), tmp_path.end());
  }
  for (const auto &point : successor_lane_rel_pts) {
    if (common::Distance2D(parking_out_path.path_points.back(), point) < 0.1)
      continue;
    std::vector<Vec3d> tmp_path;
    QuinticCurveFiting(parking_out_path.path_points.back(), point, tmp_path);
    parking_out_path.path_points.insert(parking_out_path.path_points.end(),
                                        tmp_path.begin(), tmp_path.end());
  }
  for (const auto &point : next_successor_lane_rel_pts) {
    if (common::Distance2D(parking_out_path.path_points.back(), point) < 0.1)
      continue;
    double extern_dis{config::PlanningConfig::Instance()
                          ->plan_config()
                          .parking.onlane_parking_space.extern_dis};
    if (common::Distance2D(point, next_successor_lane_rel_pts.front()) >=
        extern_dis)
      continue;
    std::vector<Vec3d> tmp_path;
    QuinticCurveFiting(parking_out_path.path_points.back(), point, tmp_path);
    parking_out_path.path_points.insert(parking_out_path.path_points.end(),
                                        tmp_path.begin(), tmp_path.end());
  }
  parking_out_path.point_s.reserve(parking_out_path.path_points.size());
  parking_out_path.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < parking_out_path.path_points.size(); ++i) {
    parking_out_path.point_s.emplace_back(
        parking_out_path.point_s.back() +
        common::Distance2D(parking_out_path.path_points[i - 1],
                           parking_out_path.path_points[i]));
  }
  TransformToWorldCorrdinate(parking_out_path, origin_pos_);
  parking_out_path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  parking_out_path.stop_befor_end = ParkingSpace::kParkingPathExternDist;
  parking_out_path.is_park_in = false;
  parking_out_path.is_need_stop = false;
  park_path_.emplace_back(parking_out_path);
  for (auto &each_path : park_path_) RemoveDuplicates(each_path);
}

}  // namespace planning
}  // namespace neodrive