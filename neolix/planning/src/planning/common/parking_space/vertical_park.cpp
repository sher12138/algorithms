#include "vertical_park.h"

#include "common/data_center/data_center.h"
#include "common/parking_space/park_util.h"
#include "common_config/config/common_config.h"
#include "config/planning_config.h"
#include "hdmap/hdmap.h"
#include "hdmap/topo_map/lane_topomap.h"
#include "src/common/util/hash_util.h"

namespace neodrive {
namespace planning {

VerticalPark::VerticalPark(cyberverse::ParkingSpaceInfoConstPtr park)
    : ParkingSpace(park) {
  if (!park || park->OverlapLaneIds().empty()) return;
  auto hdmap = cyberverse::HDMap::Instance();
  auto overlap_lane_id = park->OverlapLaneIds()[0];
  overlap_lane_ptr_ = hdmap->GetLaneById(overlap_lane_id);
  if (!overlap_lane_ptr_) return;
  auto &points = park->Points();
  heading_ = park->Heading();
  origin_pos_ = Vec3d(points[1].x, points[1].y, heading_);
  on_left_ = std::any_of(special_parking_config_.left_parking_ids.begin(),
                         special_parking_config_.left_parking_ids.end(),
                         [park](const std::string &id) {
                           return park->Id() == common::HashString(id);
                         });
  if (on_left_) {
    heading_ = std::atan2(points[0].y - points[3].y, points[0].x - points[3].x);
    origin_pos_ = {points[0].x, points[0].y, heading_};
  }
  length_ = std::hypot(points[2].y - points[1].y, points[2].x - points[1].x);
  width_ = std::hypot(points[1].y - points[0].y, points[1].x - points[0].x);
  is_park_in_ = true;
  park_path_.clear();
}

double VerticalPark::Length() { return length_; }

double VerticalPark::Width() { return width_; }

double VerticalPark::Heading() { return heading_; }

void CalculateO(const double b, const double r, const double w,
                const double p_w, Vec3d &point_o_rel) {
  point_o_rel.set_y(-b - r - w / 2.0 + p_w / 2.0);
  point_o_rel.set_x(-std::sqrt(r * r - point_o_rel.y() * point_o_rel.y()));
}

void CalculateKeyPoints(const double b, const double r, double &R,
                        const double w, const double p_w, const double theta,
                        const Vec3d &point_o_rel, Vec3d &point_b_rel,
                        Vec3d &point_a_rel, Vec3d &point_c_rel,
                        Vec3d &point_O_rel) {
  double center_r = r + b;
  R = (point_o_rel.x() + center_r * std::sin(theta) - point_a_rel.x()) /
      (1.0 - std::sin(theta));
  point_b_rel.set_x(point_o_rel.x() + center_r * std::sin(theta));
  point_b_rel.set_y(point_o_rel.y() + center_r * std::cos(theta));
  point_b_rel.set_z(-theta);
  point_O_rel.set_x(point_o_rel.x() + (center_r + R) * std::sin(theta));
  point_O_rel.set_y(point_o_rel.y() + (center_r + R) * std::cos(theta));
  point_a_rel.set_y(point_O_rel.y());
  point_a_rel.set_z(-M_PI_2);
  point_c_rel.set_x(point_o_rel.x());
  point_c_rel.set_y(p_w / 2.0);
  point_c_rel.set_z(0.0);
}

void GenerateABPath(const Vec3d &point_O_rel, const Vec3d &point_a_rel,
                    const Vec3d &point_b_rel, const double R, ParkingPath &path,
                    double step = 0.1) {
  common::math::LineSegment2d seg({point_O_rel.x(), point_O_rel.y()},
                                  {point_a_rel.x(), point_a_rel.y()});
  double theta =
      std::asin(seg.distance_to({point_b_rel.x(), point_b_rel.y()}) / R) +
      ParkingSpace::kParkingPathExternDist / R;
  double delta_theta = step / R;
  path.path_points.emplace_back(point_a_rel);
  path.point_s.emplace_back(0.0);
  for (double i = delta_theta; i <= theta; i += delta_theta) {
    Vec3d new_pt({point_a_rel.x() + R * (1 - std::cos(i)),
                  point_a_rel.y() - R * std::sin(i), -M_PI_2 + i});
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(new_pt, path.path_points.back()));
    path.path_points.emplace_back(std::move(new_pt));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path.stop_befor_end = 1.;
  path.is_park_in = true;
  path.is_need_stop = true;
}

void GenerateBCPath(const Vec3d &point_o_rel, const Vec3d &point_b_rel,
                    const Vec3d &point_c_rel, const double r, ParkingPath &path,
                    double step = 0.1) {
  common::math::LineSegment2d seg({point_o_rel.x(), point_o_rel.y()},
                                  {point_c_rel.x(), point_c_rel.y()});
  Vec3d point_B_rel{r + point_o_rel.x(), point_o_rel.y(), M_PI_2};
  common::math::LineSegment2d seg_B({point_o_rel.x(), point_o_rel.y()},
                                    {point_B_rel.x(), point_B_rel.y()});
  double thetaB =
      std::asin(seg_B.distance_to({point_b_rel.x(), point_b_rel.y()}) / r);
  double theta =
      std::asin(seg.distance_to({point_b_rel.x(), point_b_rel.y()}) / r);
  double delta_theta = step / r;
  path.path_points.emplace_back(point_b_rel);
  path.point_s.emplace_back(0.0);
  for (double i = delta_theta; i <= theta; i += delta_theta) {
    Vec3d new_pt({point_B_rel.x() - r * (1 - std::cos(thetaB + i)),
                  point_B_rel.y() + r * std::sin(thetaB + i),
                  M_PI_2 + thetaB + i});
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(new_pt, path.path_points.back()));
    path.path_points.emplace_back(std::move(new_pt));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_BACKWARD;
  path.stop_befor_end = ParkingSpace::kStopDist;
  path.is_park_in = true;
  path.is_need_stop = true;
  path.check_type = ParkingPath::ParallelHeadingDiffCheck;
}

void GenerateCBPath(const Vec3d &point_o_rel, const Vec3d &point_b_rel,
                    const Vec3d &point_c_rel,
                    std::vector<Vec3d> &overlap_lane_pts,
                    std::vector<Vec3d> &successor_lane_pts,
                    std::vector<Vec3d> &next_successor_lane_pts, const double r,
                    ParkingPath &path, double step = 0.1) {
  double theta = M_PI_2;
  double delta_theta = step / r;
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.vertical_parking_space;
  path.path_points.emplace_back(point_c_rel);
  LOG_INFO("overlap_lane_pts.back().y() = {:.2f}", overlap_lane_pts.back().y());
  if (overlap_lane_pts.back().y() >= point_c_rel.y() - r) {
    for (auto &each_pt : next_successor_lane_pts) {
      std::vector<Vec3d> tmp_path;
      Vec3d &start = path.path_points.back();
      QuinticCurveFiting(start, each_pt, tmp_path);
      path.path_points.insert(path.path_points.end(), tmp_path.begin(),
                              tmp_path.end());
    }
  } else {
    for (double i = delta_theta; i <= theta; i += delta_theta) {
      Vec3d new_pt({point_c_rel.x() + r * std::sin(i),
                    point_c_rel.y() - r * (1 - std::cos(i)), -i});
      if (new_pt.y() - overlap_lane_pts.back().y() <= 0.0) break;
      path.path_points.emplace_back(std::move(new_pt));
    }
    if (overlap_lane_pts.back().y() <= path.path_points.back().y() - .5) {
      std::vector<Vec3d> overlap_lane_path;
      QuinticCurveFiting(path.path_points.back(), overlap_lane_pts.back(),
                         overlap_lane_path);
      path.path_points.insert(path.path_points.end(), overlap_lane_path.begin(),
                              overlap_lane_path.end());
    }
    path.path_points.emplace_back(successor_lane_pts.front());
    for (std::size_t i = 1; i < successor_lane_pts.size(); ++i) {
      auto &each_pt = successor_lane_pts[i];
      std::vector<Vec3d> tmp_path;
      Vec3d &start = path.path_points.back();
      QuinticCurveFiting(start, each_pt, tmp_path);
      path.path_points.insert(path.path_points.end(), tmp_path.begin(),
                              tmp_path.end());
    }
    for (auto &each_pt : next_successor_lane_pts) {
      std::vector<Vec3d> tmp_path;
      Vec3d &start = path.path_points.back();
      QuinticCurveFiting(start, each_pt, tmp_path);
      path.path_points.insert(path.path_points.end(), tmp_path.begin(),
                              tmp_path.end());
    }
  }
  Vec3d back_pt = next_successor_lane_pts.back();
  for (double i = step; i < parking_config.park_out_extend_length; i += step) {
    path.path_points.emplace_back(back_pt.x() + i, back_pt.y(), back_pt.z());
  }
  path.point_s.reserve(path.path_points.size());
  path.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < path.path_points.size(); ++i) {
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(path.path_points[i - 1], path.path_points[i]));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path.stop_befor_end = parking_config.park_out_extend_length;
  path.is_park_in = false;
  path.is_need_stop = false;
}

void GenerateCDPath(const Vec3d &point_c_rel, const Vec3d &point_d_rel,
                    ParkingPath &path, double step = 0.1) {
  path.path_points.emplace_back(point_c_rel);
  path.point_s.emplace_back(0.0);
  for (double i = point_c_rel.x() - step;
       i > point_d_rel.x() - ParkingSpace::kParkingPathExternDist; i -= step) {
    Vec3d new_pt({i, point_c_rel.y(), 0.0});
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(new_pt, path.path_points.back()));
    path.path_points.emplace_back(std::move(new_pt));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_BACKWARD;
  path.stop_befor_end = ParkingSpace::kParkingPathExternDist;
  path.is_park_in = true;
  path.is_need_stop = true;
}

void VerticalPark::GenerateBCPathTypeC(const Vec3d &point_b_rel,
                                       const Vec3d &point_c_rel,
                                       ParkingPath &path, double step) {
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.vertical_parking_space;
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  double r{parking_config.c_type_radius + parking_config.soft_bios};
  path.path_points.emplace_back(point_b_rel);
  path.point_s.emplace_back(0.0);
  for (double theta = step / r;
       theta < M_PI_2  + parking_config.degree_buffer;
       theta += step / r) {
    Vec3d new_pt{point_b_rel.x() - r * (1 - std::cos(theta)),
                 point_b_rel.y() + r * std::sin(theta),
                 point_b_rel.z() + theta};
    if (on_left_) {
      new_pt.set_y(point_b_rel.y() - r * std::sin(theta));
      new_pt.set_z(point_b_rel.z() - theta);
    }
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(new_pt, path.path_points.back()));
    path.path_points.emplace_back(std::move(new_pt));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_BACKWARD;
  path.stop_befor_end = ParkingSpace::kStopDist;
  path.is_park_in = true;
  path.is_need_stop = true;
  path.check_type = ParkingPath::ParallelHeadingDiffCheck;
}

void GenerateCDPathTypeC(const Vec3d &point_c_rel, const Vec3d &point_d_rel,
                         ParkingPath &path, double step = 0.05) {
  path.path_points.emplace_back(point_c_rel);
  path.point_s.emplace_back(0.0);
  for (double i = point_c_rel.x() - step;
       i > point_d_rel.x() - ParkingSpace::kParkingPathExternDist; i -= step) {
    Vec3d new_pt{i, point_c_rel.y(), 0.0};
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(new_pt, path.path_points.back()));
    path.path_points.emplace_back(std::move(new_pt));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_BACKWARD;
  path.stop_befor_end = ParkingSpace::kParkingPathExternDist;
  path.is_park_in = true;
  path.is_need_stop = true;
}

void VerticalPark::Reset() {
  park_path_.clear();
  park_ptr_ = nullptr;
}

bool VerticalPark::CheckNeedStitchPath() {
  if (!is_park_in_) return false;
  const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
  Vec3d current_pos{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
  common::ConvertToRelativeCoordinate(current_pos, origin_pos_, current_pos);
  current_pos.set_z(point_a_rel_.z());
  bool need_stitch =
      current_pos.y() - point_a_rel_.y() >= 0.5 || park_in_method_ == TYPE_C;
  LOG_INFO("current pos x:{},y:{},z:{},point a rel x:{},y:{},z:{}",
           current_pos.x(), current_pos.y(), current_pos.z(), point_a_rel_.x(),
           point_a_rel_.y(), point_a_rel_.z());
  LOG_INFO("gap is:{},is need stich:{}", current_pos.y() - point_a_rel_.y(),
           need_stitch);
  return need_stitch;
}

double VerticalPark::GetDistToEnd() {
  const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
  Vec3d current_pos{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
  common::ConvertToRelativeCoordinate(current_pos, origin_pos_, current_pos);
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.vertical_parking_space;
  double ret{0.0};
  switch (park_in_method_) {
    case TYPE_C:
      ret = current_pos.x() - (-length_ + parking_config.c_type_dist_to_end);
      break;
    case TYPE_L:
      ret = current_pos.x() - (-length_ + parking_config.l_type_dist_to_end);
      break;
    default:
      ret = current_pos.x() - (-length_ + parking_config.dist_to_end);
      break;
  }
  LOG_INFO("debug dist to end:{}", ret);
  return ret;
}

ParkingPath &VerticalPark::GetStitchPath() {
  if (stitch_path.path_points.empty()) {
    const auto &utm_pose = DataCenter::Instance()->vehicle_state_utm();
    Vec3d current_pos{utm_pose.X(), utm_pose.Y(), utm_pose.Heading()};
    common::ConvertToRelativeCoordinate(current_pos, origin_pos_, current_pos);
    LOG_INFO("stitch start x:{},y:{},z:{}", current_pos.x(), current_pos.y(),
             current_pos.z());
    LOG_INFO("point_a_rel_ x:{},y:{},z:{}", point_a_rel_.x(), point_a_rel_.y(),
             point_a_rel_.z());
    QuinticCurveFiting(current_pos, point_a_rel_, stitch_path.path_points, 0.01,
                       true);
  }
  stitch_path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  stitch_path.check_type = ParkingPath::DistEndCheck;
  stitch_path.stop_befor_end = 0.0;
  stitch_path.is_park_in = true;
  stitch_path.is_need_stop = false;
  if (park_in_method_ == TYPE_C) {
    stitch_path.is_need_stop = true;
    stitch_path.stop_befor_end = kParkingPathExternDist;
    if (on_left_) {
      for (double i = 0; i < kParkingPathExternDist; i += 0.05) {
        stitch_path.path_points.emplace_back(
            point_a_rel_.x(), point_a_rel_.y() + i, point_a_rel_.z());
      }
    } else {
      for (double i = 0; i < kParkingPathExternDist; i += 0.05) {
        stitch_path.path_points.emplace_back(
            point_a_rel_.x(), point_a_rel_.y() - i, point_a_rel_.z());
      }
    }
  }
  stitch_path.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < stitch_path.path_points.size(); ++i) {
    stitch_path.point_s.emplace_back(
        stitch_path.point_s.back() +
        common::Distance2D(stitch_path.path_points[i],
                           stitch_path.path_points[i - 1]));
  }
  TransformToWorldCorrdinate(stitch_path, origin_pos_);
  if (park_in_method_ == TYPE_C) {
    stitch_path.is_need_stop = true;
    stitch_path.stop_befor_end = ParkingSpace::kStopDist;
    RemoveDuplicates(stitch_path);
    park_path_.insert(park_path_.begin(), stitch_path);
  } else {
    for (auto &each_s : park_path_[0].point_s)
      each_s += stitch_path.point_s.back();
    park_path_[0].path_points.insert(park_path_[0].path_points.begin(),
                                     stitch_path.path_points.begin(),
                                     stitch_path.path_points.end());
    park_path_[0].point_s.insert(park_path_[0].point_s.begin(),
                                 stitch_path.point_s.begin(),
                                 stitch_path.point_s.end());
    RemoveDuplicates(park_path_[0]);
  }
  return stitch_path;
}

void GenerateLABPath(const Vec3d &point_b_rel,
                     std::vector<Vec3d> &successor_lane_pts, ParkingPath &path,
                     double step = 0.1) {
  if (successor_lane_pts.size() <= 2) return;
  for (auto i = 1; i < successor_lane_pts.size(); ++i) {
    std::vector<Vec3d> tmp_path;
    auto &start = successor_lane_pts[i - 1];
    auto &end = successor_lane_pts[i];
    QuinticCurveFiting(start, end, tmp_path);
    path.path_points.insert(path.path_points.end(), tmp_path.begin(),
                            tmp_path.end());
  }
  for (double i = step; i < ParkingSpace::kParkingPathExternDist; i += step) {
    path.path_points.emplace_back(point_b_rel.x() + i, point_b_rel.y(),
                                  point_b_rel.z());
  }
  path.point_s.reserve(path.path_points.size());
  path.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < path.path_points.size(); ++i) {
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(path.path_points[i - 1], path.path_points[i]));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path.stop_befor_end = ParkingSpace::kStopDist;
  path.is_park_in = true;
  path.is_need_stop = true;
}

void GenerateLBCDPath(const Vec3d &point_b_rel, const Vec3d &point_c_rel,
                      const Vec3d &point_d_rel, ParkingPath &path,
                      double step = 0.1) {
  std::vector<Vec3d> temp_pts;
  QuinticCurveFiting(point_b_rel, point_c_rel, temp_pts);
  path.path_points.insert(path.path_points.end(), temp_pts.begin(),
                          temp_pts.end());
  temp_pts.clear();
  QuinticCurveFiting(point_c_rel, point_d_rel, temp_pts);
  path.path_points.insert(path.path_points.end(), temp_pts.begin(),
                          temp_pts.end());
  for (double i = step; i < ParkingSpace::kParkingPathExternDist; i += step) {
    path.path_points.emplace_back(point_d_rel.x() - i, point_d_rel.y(),
                                  point_d_rel.z());
  }
  path.point_s.reserve(path.path_points.size());
  path.point_s.emplace_back(0.0);
  for (std::size_t i = 1; i < path.path_points.size(); ++i) {
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(path.path_points[i - 1], path.path_points[i]));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_BACKWARD;
  path.stop_befor_end = ParkingSpace::kParkingPathExternDist;
  path.is_park_in = true;
  path.is_need_stop = true;
}

void VerticalPark::GenerateParkInPathTypeR() {
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.vertical_parking_space;
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  double r = parking_config.soft_radius;
  double R = parking_config.prepare_radius;
  double b = parking_config.soft_bios;
  point_a_rel_.set_x(overlap_lane_ptr_->GetWidth(0.0) / 2.0);
  point_o_rel_.set_y(-b - r + width_ / 2.0);
  point_o_rel_.set_x(parking_config.front_dis_in_park);
  CalculateKeyPoints(b, r, R, ego_car_config.width, width_,
                     parking_config.theta, point_o_rel_, point_b_rel_,
                     point_a_rel_, point_c_rel_, point_O_rel_);
  LOG_INFO("point_o_rel_ x:{} y:{} z:{}", point_o_rel_.x(), point_o_rel_.y(),
           point_o_rel_.z());
  LOG_INFO("point_a_rel_ x:{} y:{} z:{}", point_a_rel_.x(), point_a_rel_.y(),
           point_a_rel_.z());
  LOG_INFO("point_b_rel_ x:{} y:{} z:{}", point_b_rel_.x(), point_b_rel_.y(),
           point_b_rel_.z());
  LOG_INFO("point_c_rel_ x:{} y:{} z:{}", point_c_rel_.x(), point_c_rel_.y(),
           point_c_rel_.z());
  ParkingPath ab_path, bc_path, cd_path;
  GenerateABPath(point_O_rel_, point_a_rel_, point_b_rel_, R, ab_path);
  point_b_rel_.set_z(point_b_rel_.z() + M_PI);
  GenerateBCPath(point_o_rel_, point_b_rel_, point_c_rel_, r + b, bc_path);
  point_d_rel_.set_x(-length_ + parking_config.dist_to_end);
  point_d_rel_.set_y(width_ / 2.0);
  point_d_rel_.set_z(0.0);
  GenerateCDPath(point_c_rel_, point_d_rel_, cd_path);
  TransformToWorldCorrdinate(ab_path, origin_pos_);
  TransformToWorldCorrdinate(bc_path, origin_pos_);
  TransformToWorldCorrdinate(cd_path, origin_pos_);
  park_path_.emplace_back(ab_path);
  park_path_.emplace_back(bc_path);
  park_path_.emplace_back(cd_path);
}

void VerticalPark::GenerateParkInPathTypeC() {
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.vertical_parking_space;
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  double r = parking_config.c_type_radius;
  double b = parking_config.soft_bios;
  if (on_left_) {
    point_o_rel_ = {parking_config.c_type_front_dis_in_park,
                    -width_ / 2. + r + b, 0.};
    point_b_rel_ = {r + b + point_o_rel_.x(), point_o_rel_.y(), -M_PI_2};
    point_a_rel_ = {point_b_rel_.x(), point_b_rel_.y(), M_PI_2};
    point_c_rel_ = {point_o_rel_.x(), -width_ / 2., M_PI};
    point_d_rel_ = {-length_ + parking_config.c_type_dist_to_end, -width_ / 2.,
                    M_PI};
  } else {
    point_o_rel_ = {parking_config.c_type_front_dis_in_park,
                    -r - b + width_ / 2., 0.};
    point_b_rel_ = {r + b + point_o_rel_.x(), point_o_rel_.y(), M_PI_2};
    point_a_rel_ = {point_b_rel_.x(), point_b_rel_.y(), -M_PI_2};
    point_c_rel_ = {point_o_rel_.x(), width_ / 2., M_PI};
    point_d_rel_ = {-length_ + parking_config.c_type_dist_to_end, width_ / 2.,
                    M_PI};
  }
  LOG_INFO("point_o_rel_ x:{} y:{} z:{}", point_o_rel_.x(), point_o_rel_.y(),
           point_o_rel_.z());
  LOG_INFO("point_a_rel_ x:{} y:{} z:{}", point_a_rel_.x(), point_a_rel_.y(),
           point_a_rel_.z());
  LOG_INFO("point_b_rel_ x:{} y:{} z:{}", point_b_rel_.x(), point_b_rel_.y(),
           point_b_rel_.z());
  LOG_INFO("point_c_rel_ x:{} y:{} z:{}", point_c_rel_.x(), point_c_rel_.y(),
           point_c_rel_.z());
  LOG_INFO("point_d_rel_ x:{} y:{} z:{}", point_d_rel_.x(), point_d_rel_.y(),
           point_d_rel_.z());
  ParkingPath bc_path, cd_path;
  GenerateBCPathTypeC(point_b_rel_, point_c_rel_, bc_path);
  GenerateCDPathTypeC(point_c_rel_, point_d_rel_, cd_path);
  TransformToWorldCorrdinate(bc_path, origin_pos_);
  TransformToWorldCorrdinate(cd_path, origin_pos_);
  park_path_.emplace_back(bc_path);
  park_path_.emplace_back(cd_path);
}

void VerticalPark::GenerateParkInPathTypeL() {
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.vertical_parking_space;
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  auto successor = GetOnlySuccessor(overlap_lane_ptr_);
  auto next_successor = GetOnlySuccessor(successor);
  std::vector<Vec3d> successor_lane_rel_pts;
  ParkingPath ab_path, bc_path;
  GetLaneRelativePoints(successor, origin_pos_, successor_lane_rel_pts);
  point_a_rel_ = successor_lane_rel_pts.front();
  point_b_rel_ = successor_lane_rel_pts.back();
  point_c_rel_.set_x(parking_config.front_dis_in_park);
  point_c_rel_.set_y(width_ / 2.0 + 0.05);
  point_c_rel_.set_z(-M_PI);
  point_d_rel_.set_x(-length_ + parking_config.l_type_dist_to_end);
  point_d_rel_.set_y(width_ / 2.0);
  point_d_rel_.set_z(-M_PI);
  GenerateLABPath(point_b_rel_, successor_lane_rel_pts, ab_path);
  point_b_rel_.set_z(-M_PI);
  GenerateLBCDPath(point_b_rel_, point_c_rel_, point_d_rel_, bc_path);
  TransformToWorldCorrdinate(ab_path, origin_pos_);
  TransformToWorldCorrdinate(bc_path, origin_pos_);
  park_path_.emplace_back(ab_path);
  park_path_.emplace_back(bc_path);
}

void GenerateDCPath(const Vec3d &point_c_rel, const Vec3d &point_d_rel,
                    double pre_stop_dist, ParkingPath &path,
                    double step = 0.1) {
  path.path_points.emplace_back(point_d_rel);
  path.point_s.emplace_back(0.0);
  for (double i = point_d_rel.x() + step; i < point_c_rel.x() + pre_stop_dist;
       i += step) {
    Vec3d new_pt{i, point_c_rel.y(), 0.0};
    path.point_s.emplace_back(
        path.point_s.back() +
        common::Distance2D(new_pt, path.path_points.back()));
    path.path_points.emplace_back(std::move(new_pt));
  }
  path.drive_direction = MasterInfo::DriveDirection::DRIVE_FORWARD;
  path.stop_befor_end = pre_stop_dist;
  path.is_park_in = false;
  path.is_need_stop = false;
}

void VerticalPark::GenerateParkOutPath() {
  auto &parking_config = config::PlanningConfig::Instance()
                             ->plan_config()
                             .parking.vertical_parking_space;
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  auto &lane_points = overlap_lane_ptr_->Points();
  auto successor = GetOnlySuccessor(overlap_lane_ptr_);
  auto next_successor = GetOnlySuccessor(successor);
  double mid_lane_width = overlap_lane_ptr_->GetWidth(0.0) / 2.0;
  std::vector<Vec3d> overlap_lane_rel_pts, successor_lane_rel_pts,
      next_successor_lane_rel_pts;
  GetLaneRelativePoints(overlap_lane_ptr_, origin_pos_, overlap_lane_rel_pts);
  GetLaneRelativePoints(successor, origin_pos_, successor_lane_rel_pts);
  GetLaneRelativePoints(next_successor, origin_pos_,
                        next_successor_lane_rel_pts);
  if (on_left_) {
    point_d_rel_ = {-ego_car_config.length, -width_ / 2., 0.};
    point_c_rel_ = {mid_lane_width - parking_config.park_out_soft_radius,
                    -width_ / 2., 0.};
    point_o_rel_.set_x(point_c_rel_.x());
    point_o_rel_.set_y(point_c_rel_.y() + parking_config.park_out_soft_radius);
    point_b_rel_ = {mid_lane_width, point_o_rel_.y(), M_PI_2};
  } else {
    point_d_rel_ = {-ego_car_config.length, width_ / 2., 0.};
    point_c_rel_ = {mid_lane_width - parking_config.park_out_soft_radius,
                    width_ / 2., 0.};
    point_o_rel_.set_x(point_c_rel_.x());
    point_o_rel_.set_y(point_c_rel_.y() - parking_config.park_out_soft_radius);
    point_b_rel_ = {mid_lane_width, point_o_rel_.y(), -M_PI_2};
  }
  ParkingPath dc_path, cb_path;
  GenerateDCPath(point_c_rel_, point_d_rel_, kParkingPathExternDist, dc_path);
  LOG_INFO("hyh debug point rel b x:{},y:{}", point_b_rel_.x(),
           point_b_rel_.y());
  LOG_INFO("hyh debug point last overlap pt x:{},y:{}",
           overlap_lane_rel_pts.back().x(), overlap_lane_rel_pts.back().y());
  GenerateCBPath(point_o_rel_, point_b_rel_, point_c_rel_, overlap_lane_rel_pts,
                 successor_lane_rel_pts, next_successor_lane_rel_pts,
                 parking_config.park_out_soft_radius, cb_path);
  TransformToWorldCorrdinate(dc_path, origin_pos_);
  TransformToWorldCorrdinate(cb_path, origin_pos_);
  park_path_.emplace_back(dc_path);
  park_path_.emplace_back(cb_path);
}

void VerticalPark::SwitchParkinMethod() {
  auto &overlap_lane_pts = overlap_lane_ptr_->Points();
  auto &last_pt = overlap_lane_pts.back();
  auto &first_pt = overlap_lane_pts.front();
  Vec3d last_rel_pt(last_pt.x(), last_pt.y(), 0.0),
      first_rel_pt(first_pt.x(), first_pt.y(), 0.0);
  common::ConvertToRelativeCoordinate(first_rel_pt, origin_pos_, first_rel_pt);
  common::ConvertToRelativeCoordinate(last_rel_pt, origin_pos_, last_rel_pt);
  LOG_INFO("first_rel_pt x:{},y:{}", first_rel_pt.x(), first_rel_pt.y());
  LOG_INFO("last_rel_pt x:{},y:{}", last_rel_pt.x(), last_rel_pt.y());
  bool c_type =
      std::any_of(special_parking_config_.c_type_parking_ids.begin(),
                  special_parking_config_.c_type_parking_ids.end(),
                  [this](const std::string &id) {
                    return this->park_ptr_->Id() == common::HashString(id);
                  });
  if (first_rel_pt.y() <= 2.0 || c_type)
    park_in_method_ = TYPE_C;
  else if (last_rel_pt.y() >= 3.5)
    park_in_method_ = TYPE_L;
  else
    park_in_method_ = TYPE_R;
}

void VerticalPark::Solve() {
  if (is_park_in_) {
    SwitchParkinMethod();
    switch (park_in_method_) {
      case TYPE_C:
        GenerateParkInPathTypeC();
        break;
      case TYPE_L:
        GenerateParkInPathTypeL();
        break;
      default:
        GenerateParkInPathTypeR();
        break;
    }
  } else {
    GenerateParkOutPath();
  }
  for (auto &each_path : park_path_) RemoveDuplicates(each_path);
}
}  // namespace planning
}  // namespace neodrive