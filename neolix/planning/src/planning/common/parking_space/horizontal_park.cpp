#include "horizontal_park.h"

#include "common/data_center/data_center.h"
#include "common/parking_space/park_util.h"
#include "common_config/config/common_config.h"
#include "config/planning_config.h"
#include "hdmap/hdmap.h"

namespace neodrive {
namespace planning {
HorizontalPark::HorizontalPark(cyberverse::ParkingSpaceInfoConstPtr park)
    : ParkingSpace(park) {
  if (!park) return;
  auto hdmap = cyberverse::HDMap::Instance();
  auto overlap_lane_id = park->OverlapLaneIds()[0];
  overlap_lane_ptr_ = hdmap->GetLaneById(overlap_lane_id);
  if (!overlap_lane_ptr_) return;
  auto &points = park->Points();
  heading_ = park->Heading();
  origin_pos_ = Vec3d(points[0].x, points[0].y, heading_);
  length_ = std::hypot(points[0].y - points[1].y, points[0].x - points[1].x);
  width_ = std::hypot(points[3].y - points[0].y, points[3].x - points[0].x);
  is_park_in_ = true;
  park_path_.clear();
}

double HorizontalPark::Length() {
  if (!park_ptr_) return 0.0;
  auto &points = park_ptr_->Points();
  return std::hypot(points[0].y - points[1].y, points[0].x - points[1].x);
}

double HorizontalPark::Width() {
  if (!park_ptr_) return 0.0;
  auto &points = park_ptr_->Points();
  return std::hypot(points[1].y - points[2].y, points[1].x - points[2].x);
}

double HorizontalPark::Heading() {
  if (!park_ptr_) return 0.0;
  return park_ptr_->Heading();
}

void HorizontalPark::Solve() {
  if (is_park_in_)
    GenerateParkInPath();
  else
    GenerateParkOutPath();
}

double HorizontalPark::GetDistToEnd() {
  const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
  Vec3d current_pos{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
  common::ConvertToRelativeCoordinate(current_pos, origin_pos_, current_pos);
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.horizontal_parking_space;
  double ret = current_pos.y() - (-length_ + parking_config.dist_to_end);
  return ret;
}

void HorizontalPark::Reset() { ; }

void HorizontalPark::CalculateKeyPoints() {
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.horizontal_parking_space;
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  point_a_rel_.set_x(overlap_lane_ptr_->GetWidth(0.0) / 2.0);
  point_a_rel_.set_y(1.0);
  point_a_rel_.set_z(-M_PI_2);
  point_b_rel_.set_x(-Width() / 2.0 + parking_config.front_offset);
  point_b_rel_.set_y(-Length() + ego_car_config.length);
  point_b_rel_.set_z(-M_PI_2);
  point_c_rel_.set_x(-Width() / 2.0 + parking_config.back_offset);
  point_c_rel_.set_y(-parking_config.back_safe_dist);
  point_c_rel_.set_z(M_PI_2);
  point_d_rel_.set_x(-Width() / 2.0);
  point_d_rel_.set_y(-Length() + ego_car_config.length);
  point_d_rel_.set_z(-M_PI_2);
}

void GenerateABPath(Vec3d &point_a_rel, Vec3d &point_b_rel, ParkingPath &path,
                    double step = 0.1) {
  QuinticCurveFiting(point_a_rel, point_b_rel, path.path_points, 0.01, true);
  for (double i = step; i < ParkingSpace::kParkingPathExternDist; i += step) {
    path.path_points.emplace_back(point_b_rel.x(), point_b_rel.y() - i,
                                  point_b_rel.z());
  }
  path.point_s.reserve(path.path_points.size());
  path.point_s.push_back(0.0);
  for (size_t i = 1; i < path.path_points.size(); ++i) {
    path.point_s.push_back(
        path.point_s.back() +
        common::Distance2D(path.path_points[i - 1], path.path_points[i]));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path.stop_befor_end = ParkingSpace::kParkingPathExternDist;
  path.is_park_in = true;
  path.is_need_stop = true;
}

void GenerateBCPath(Vec3d &point_b_rel, Vec3d &point_c_rel, ParkingPath &path,
                    double step = 0.1) {
  QuinticCurveFiting(point_b_rel, point_c_rel, path.path_points);
  for (double i = step; i < ParkingSpace::kParkingPathExternDist; i += step) {
    path.path_points.emplace_back(point_c_rel.x(), point_c_rel.y() + i,
                                  point_c_rel.z());
  }
  path.point_s.reserve(path.path_points.size());
  path.point_s.push_back(0.0);
  for (size_t i = 1; i < path.path_points.size(); ++i) {
    path.point_s.push_back(
        path.point_s.back() +
        common::Distance2D(path.path_points[i - 1], path.path_points[i]));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_BACKWARD;
  path.stop_befor_end = ParkingSpace::kParkingPathExternDist;
  path.is_park_in = true;
  path.is_need_stop = true;
}

void GenerateCDPath(Vec3d &point_c_rel, Vec3d &point_d_rel, ParkingPath &path,
                    double step = 0.1) {
  QuinticCurveFiting(point_c_rel, point_d_rel, path.path_points);
  for (double i = step; i < ParkingSpace::kParkingPathExternDist; i += step) {
    path.path_points.emplace_back(point_d_rel.x(), point_d_rel.y() - i,
                                  point_d_rel.z());
  }
  path.point_s.reserve(path.path_points.size());
  path.point_s.push_back(0.0);
  for (size_t i = 1; i < path.path_points.size(); ++i) {
    path.point_s.push_back(
        path.point_s.back() +
        common::Distance2D(path.path_points[i - 1], path.path_points[i]));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path.stop_befor_end = ParkingSpace::kParkingPathExternDist + 0.2;
  path.is_park_in = true;
  path.is_need_stop = true;
}

bool HorizontalPark::CheckNeedStitchPath() {
  if (!is_park_in_) return false;
  const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
  Vec3d current_pos{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
  common::ConvertToRelativeCoordinate(current_pos, origin_pos_, current_pos);
  return current_pos.y() - point_a_rel_.y() >= 0.5;
}

ParkingPath &HorizontalPark::GetStitchPath() {
  stitch_path.Reset();
  if (stitch_path.path_points.empty()) {
    const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
    Vec3d current_pos{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
    common::ConvertToRelativeCoordinate(current_pos, origin_pos_, current_pos);
    QuinticCurveFiting(current_pos, point_a_rel_, stitch_path.path_points);
  }
  stitch_path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  stitch_path.stop_befor_end = 0.0;
  stitch_path.is_park_in = true;
  stitch_path.is_need_stop = false;
  stitch_path.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < stitch_path.path_points.size(); ++i) {
    stitch_path.point_s.emplace_back(
        stitch_path.point_s.back() +
        common::Distance2D(stitch_path.path_points[i],
                           stitch_path.path_points[i - 1]));
  }
  TransformToWorldCorrdinate(stitch_path, origin_pos_);
  for (auto &each_s : park_path_[0].point_s)
    each_s += stitch_path.point_s.back();
  park_path_[0].path_points.insert(park_path_[0].path_points.begin(),
                                   stitch_path.path_points.begin(),
                                   stitch_path.path_points.end() - 1);
  park_path_[0].point_s.insert(park_path_[0].point_s.begin(),
                               stitch_path.point_s.begin(),
                               stitch_path.point_s.end() - 1);
  return stitch_path;
}

void HorizontalPark::GenerateParkInPath() {
  CalculateKeyPoints();
  LOG_INFO("point_a_rel_ x:{} y:{} z:{}", point_a_rel_.x(), point_a_rel_.y(),
           point_a_rel_.z());
  LOG_INFO("point_b_rel_ x:{} y:{} z:{}", point_b_rel_.x(), point_b_rel_.y(),
           point_b_rel_.z());
  LOG_INFO("point_c_rel_ x:{} y:{} z:{}", point_c_rel_.x(), point_c_rel_.y(),
           point_c_rel_.z());
  ParkingPath ab_path, bc_path, cd_path;
  GenerateABPath(point_a_rel_, point_b_rel_, ab_path);
  point_b_rel_.set_z(point_b_rel_.z() + M_PI);
  point_b_rel_.set_x(point_c_rel_.x());
  GenerateBCPath(point_b_rel_, point_c_rel_, bc_path);
  point_c_rel_.set_z(point_c_rel_.z() + M_PI);
  GenerateCDPath(point_c_rel_, point_d_rel_, cd_path);
  TransformToWorldCorrdinate(ab_path, origin_pos_);
  TransformToWorldCorrdinate(bc_path, origin_pos_);
  TransformToWorldCorrdinate(cd_path, origin_pos_);
  park_path_.emplace_back(ab_path);
  park_path_.emplace_back(bc_path);
  park_path_.emplace_back(cd_path);
}

void HorizontalPark::GenerateParkOutPath() {
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  auto hdmap = cyberverse::HDMap::Instance();
  if (!park_ptr_ || park_ptr_->OverlapLaneIds().empty()) {
    LOG_ERROR("no park or no overlap lanes");
    return;
  }
  const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();

  // 1. point a
  Vec3d point_a_utm{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
  Vec3d point_a_rel;
  common::ConvertToRelativeCoordinate(point_a_utm, origin_pos_, point_a_rel);

  // 2. point b
  Vec3d parking_point_odom, parking_point_rel;
  Vec3d ego_utm{DataCenter::Instance()->vehicle_state_utm().X(),
                DataCenter::Instance()->vehicle_state_utm().Y(),
                DataCenter::Instance()->vehicle_state_utm().Heading()};
  Vec3d ego_odom{DataCenter::Instance()->vehicle_state_proxy().X(),
                 DataCenter::Instance()->vehicle_state_proxy().Y(),
                 DataCenter::Instance()->vehicle_state_proxy().Heading()};
  common::ConvertToRelativeCoordinate(
      Vec3d{park_ptr_->Points()[1].x, park_ptr_->Points()[1].y,
            utm_pose.Heading()},
      ego_utm, parking_point_rel);
  common::ConvertToWorldCoordinate(parking_point_rel, ego_odom,
                                   parking_point_odom);
  auto &task_info = DataCenter::Instance()->task_info_list().front();
  SLPoint parking_end;
  ReferencePoint dest_point;
  if (!task_info.reference_line()->GetPointInFrenetFrame(
          Vec2d{parking_point_odom.x(), parking_point_odom.y()},
          &parking_end)) {
    LOG_ERROR("GetPointInFrenetFrame failed");
  }
  // TODO: 2.5 to json
  task_info.reference_line()->GetNearestRefPoint(parking_end.s() + 2.5,
                                                 &dest_point);
  Vec3d target_rel, target_utm;
  common::ConvertToRelativeCoordinate(
      Vec3d{dest_point.x(), dest_point.y(), dest_point.heading()}, ego_odom,
      target_rel);
  common::ConvertToWorldCoordinate(target_rel, ego_utm, target_utm);
  Vec3d point_b_utm{target_utm};
  Vec3d point_b_rel;
  common::ConvertToRelativeCoordinate(point_b_utm, origin_pos_, point_b_rel);
  LOG_INFO("point a rel x = {:.2f}, y = {:.2f}, theta = {:.2f}",
           point_a_rel.x(), point_a_rel.y(), point_a_rel.z());
  LOG_INFO("point b rel x = {:.2f}, y = {:.2f}, theta = {:.2f}",
           point_b_rel.x(), point_b_rel.y(), point_b_rel.z());
  LOG_INFO("point a utm x = {:.2f}, y = {:.2f}, theta = {:.2f}",
           point_a_utm.x(), point_a_utm.y(), point_a_utm.z());
  LOG_INFO("point b utm x = {:.2f}, y = {:.2f}, theta = {:.2f}",
           point_b_utm.x(), point_b_utm.y(), point_b_utm.z());

  // Generate path ab rel
  ParkingPath path_ab;
  point_a_rel.set_z(-M_PI_2);
  QuinticCurveFiting(point_a_rel, point_b_rel, path_ab.path_points, 0.01, true);
  std::vector<Vec3d> next_successor_lane_rel_pts, tmp_path,
      successor_lane_rel_pts;
  auto successor = GetOnlySuccessor(overlap_lane_ptr_);
  GetLaneRelativePoints(successor, origin_pos_, successor_lane_rel_pts);
  LOG_INFO("hyh test:{}", successor_lane_rel_pts.front().y());
  if (successor_lane_rel_pts.front().y() <= -8.0) {
    Vec3d target_pt = successor_lane_rel_pts.front();
    target_pt.set_z(-M_PI_2);
    QuinticCurveFiting(path_ab.path_points.back(), target_pt, tmp_path);
    path_ab.path_points.insert(path_ab.path_points.end(), tmp_path.begin(),
                               tmp_path.end());
    tmp_path.clear();
  }
  for (auto i = 1; i < successor_lane_rel_pts.size(); ++i) {
    auto &start_pt = successor_lane_rel_pts[i - 1];
    auto &target_pt = successor_lane_rel_pts[i];
    QuinticCurveFiting(start_pt, target_pt, tmp_path);
    path_ab.path_points.insert(path_ab.path_points.end(), tmp_path.begin(),
                               tmp_path.end());
    tmp_path.clear();
  }
  auto next_successor = GetOnlySuccessor(successor);
  GetLaneRelativePoints(next_successor, origin_pos_,
                        next_successor_lane_rel_pts);
  for (auto i = 0; i < next_successor_lane_rel_pts.size(); ++i) {
    auto &target_pt = next_successor_lane_rel_pts[i];
    QuinticCurveFiting(path_ab.path_points.back(), target_pt, tmp_path);
    path_ab.path_points.insert(path_ab.path_points.end(), tmp_path.begin(),
                               tmp_path.end());
    tmp_path.clear();
  }
  path_ab.point_s.reserve(path_ab.path_points.size());
  path_ab.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < path_ab.path_points.size(); ++i) {
    path_ab.point_s.emplace_back(
        path_ab.point_s.back() +
        common::Distance2D(path_ab.path_points[i - 1], path_ab.path_points[i]));
  }
  TransformToWorldCorrdinate(path_ab, origin_pos_);
  path_ab.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path_ab.stop_befor_end =
      next_successor->TotalLength() - ParkingSpace::kParkingPathExternDist;
  path_ab.is_park_in = false;
  path_ab.is_need_stop = false;
  park_path_.emplace_back(path_ab);
  for (auto &each_path : park_path_) RemoveDuplicates(each_path);
}

}  // namespace planning
}  // namespace neodrive