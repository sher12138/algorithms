#include "parking_path_optimizer.h"
#include "proxy/event_report_proxy.h"
#include "src/common/util/hash_util.h"
#include "src/planning/common/math/math_utils.h"
#include "src/planning/common/parking_space/horizontal_park.h"
#include "src/planning/common/parking_space/vertical_park.h"

namespace neodrive {
namespace planning {

ParkingPathOptimizer::ParkingPathOptimizer() { name_ = "ParkingPathOptimizer"; }

ParkingPathOptimizer::~ParkingPathOptimizer() { Reset(); }

void ParkingPathOptimizer::Reset() {
  path_idx_ = 0;
  ego_stop_count_ = 0;
  ego_stuck_count_ = 0;
  stop_and_steer_ = false;
}

void ParkingPathOptimizer::SaveTaskResults(TaskInfo& task_info) {
  task_info.current_frame()
      ->mutable_inside_planner_data()
      ->curr_scenario_state = ScenarioState::PARKING;
}

double GetHeadingDiffTargetSpeed(double ego_heading, double park_heading,
                                 const ParkingPath& path) {
  auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  return parking_config.parking_in_max_speed;

  double heading_diff = std::fabs(ego_heading - park_heading);
  LOG_INFO("heading diff:{}", heading_diff);
  if (path.check_type == ParkingPath::VerticalHeadingDiffCheck)
    heading_diff = std::fabs(heading_diff - M_PI_2);
  if (heading_diff <= parking_config.heading_diff_threshold) heading_diff = 0.0;
  double ret{parking_config.parking_in_max_speed};
  double dis_to_end =
      heading_diff * parking_config.vertical_parking_space.soft_radius;
  double expect_v = dis_to_end * parking_config.heading_slow_down_coef;
  ret = std::max(0.2, std::min(ret, expect_v));
  return path.check_type == ParkingPath::DistEndCheck
             ? parking_config.parking_in_max_speed
             : ret;
}

double GetDistErrorTargetSpeed(double dis_error) {
  auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  double ret;
  if (dis_error >= parking_config.high_dis_error_threshold) {
    ret = parking_config.high_dis_error_speed;
  } else if (dis_error >= parking_config.low_dis_error_threshold) {
    ret = parking_config.low_dis_error_speed;
  } else {
    ret = parking_config.default_speed;
  }
  return ret;
}

ErrorCode ParkingPathOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  auto& frame = task_info.current_frame();
  auto parking_ptr = data_center_->parking_ptr();
  if (parking_ptr == nullptr || parking_ptr->OriginPark() == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("Parking id = {}, is_parking_in = {}, parking_path.size() = {}",
           cyberverse::HDMap::Instance()->GetIdHashString(
               parking_ptr->OriginPark()->Id()),
           parking_ptr->is_park_in(), parking_ptr->ParkPath().size());
  if (parking_ptr->ParkPath().empty()) {
    // replan
    LOG_INFO("replan");
    parking_ptr->Solve();
    if (parking_ptr->CheckNeedStitchPath()) {
      LOG_INFO("stitch");
      parking_ptr->GetStitchPath();
    }
    path_idx_ = RematchPathIndex();
    LOG_INFO("rematch path index = {}", path_idx_);
  }
  auto& parking_path = parking_ptr->ParkPath();
  if (parking_path.empty()) {
    LOG_ERROR("parking path is empty!");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  UpdateDistToEnd();
  const auto& curr_parking_path = parking_path[path_idx_];
  LOG_INFO("path_points.size = {}, point s.size = {}",
           curr_parking_path.path_points.size(),
           curr_parking_path.point_s.size());
  double heading_diff = std::fabs(data_center_->vehicle_state_utm().Heading() -
                                  parking_ptr->Heading());
  std::size_t curr_index = GetNearestPointIndex(curr_parking_path);
  double curr_s = curr_parking_path.point_s.size() > curr_index
                      ? curr_parking_path.point_s[curr_index]
                      : 0.;
  const double stop_check_threshold = data_center_->is_sim() ? 0.5 : 0.12;
  LOG_INFO("curr_s = {:.2f}, point_s.back - stop_befor_end = {:.2f}", curr_s,
           curr_parking_path.point_s.back() - curr_parking_path.stop_befor_end);
  LOG_INFO(
      "{},{},{}",
      (curr_parking_path.check_type == ParkingPath::DistEndCheck &&
       curr_s >=
           curr_parking_path.point_s.back() - curr_parking_path.stop_befor_end),
      (curr_parking_path.check_type == ParkingPath::ParallelHeadingDiffCheck &&
       heading_diff <= parking_config.heading_diff_threshold),
      curr_parking_path.is_need_stop &&
          vehicle_state_.LinearVelocity() < kStopVelThreshold);
  bool pos_check =
      path_idx_ == parking_path.size() - 1 &&
              parking_ptr->OriginPark()->Type() ==
                  global::hdmap::ParkingSpace_ParkingType_VERTICAL &&
              curr_parking_path.is_park_in
          ? parking_ptr->GetDistToEnd() <= 0.0
          : dist_to_end_ <= parking_config.parking_slow_down_dist;
  bool stop_check =
      (curr_parking_path.is_need_stop &&
       vehicle_state_.LinearVelocity() < kStopVelThreshold) ||
      (curr_parking_path.check_type == ParkingPath::ParallelHeadingDiffCheck &&
       heading_diff <= parking_config.heading_diff_threshold) ||
      !curr_parking_path.is_need_stop;
  LOG_INFO("pos_check:{},dist_to_end:{},parking_ptr dist to end:{}", pos_check,
           dist_to_end_, parking_ptr->GetDistToEnd());
  if (pos_check && stop_check) {
    ++path_idx_;
    stop_and_steer_ = parking_ptr->is_park_in();
    LOG_INFO("path idx = {}", path_idx_);
    LOG_INFO("at change point,currs:{},dist_to_end:{}", curr_s,
             curr_parking_path.point_s.back() -
                 curr_parking_path.stop_befor_end - curr_s);
  }
  if (path_idx_ >= parking_path.size()) {
    LOG_INFO("All path has finished");
    if (!parking_path.empty() && parking_path[path_idx_ - 1].is_park_in) {
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::PARK_IN_FINISH, data_center_->parking_ptr()->GetParkId());
    } else if (!parking_path.empty() &&
               !parking_path[path_idx_ - 1].is_park_in) {
      data_center_->mutable_event_report_proxy()->SetEvent(
          EventType::PARK_OUT_FINISH, data_center_->parking_ptr()->GetParkId());
    }
    parking_ptr->set_is_finished(true);
    stop_and_steer_ = false;
    return ErrorCode::PLANNING_OK;
  }
  if (stop_and_steer_) {
    double now_t = cyber::Time::Now().ToSecond();
    if (vehicle_state_.LinearVelocity() >= kStopVelThreshold)
      start_stop_t_ = now_t;
    if (now_t - start_stop_t_ >= parking_config.stop_time_threhold) {
      stop_and_steer_ = false;
      start_stop_t_ = now_t;
      frame->mutable_outside_planner_data()->trajectory_replan = true;
    } else {
      frame->mutable_outside_planner_data()->hold_on = true;
    }
  }

  JudgeIfStuck();

  LOG_INFO("path index is {}", path_idx_);
  std::vector<Vec3d> curr_path;
  curr_path.reserve(curr_parking_path.path_points.size());
  for (std::size_t i = 0; i < curr_parking_path.path_points.size(); ++i) {
    if (curr_parking_path.point_s[i] < curr_s) continue;
    curr_path.push_back(curr_parking_path.path_points[i]);
  }
  bool is_reverse_driving = curr_parking_path.drive_direction ==
                            MasterInfo::DriveDirection::DRIVE_BACKWARD;
  data_center_->mutable_master_info()->set_drive_direction(
      curr_parking_path.drive_direction);
  data_center_->current_frame()
      ->mutable_inside_planner_data()
      ->is_reverse_driving = is_reverse_driving;
  TransformUtmPathToOdom(curr_path);
  std::vector<PathPoint> final_path;
  BuildPathPoint(curr_path, final_path, is_reverse_driving);
  frame->mutable_outside_planner_data()->path_data->set_path(final_path);
  frame->mutable_outside_planner_data()->path_succeed_tasks += 1;
  SetParkingSpeedLimit(task_info, curr_parking_path);
  return ErrorCode::PLANNING_OK;
}

void ParkingPathOptimizer::TransformUtmPathToOdom(std::vector<Vec3d>& path) {
  Vec3d ego_utm{data_center_->vehicle_state_utm().X(),
                data_center_->vehicle_state_utm().Y(),
                data_center_->vehicle_state_utm().Heading()};
  Vec3d ego_odom{vehicle_state_.X(), vehicle_state_.Y(),
                 vehicle_state_.Heading()};
  for (auto& each_pt : path) {
    common::ConvertToRelativeCoordinate(each_pt, ego_utm, each_pt);
    common::ConvertToWorldCoordinate(each_pt, ego_odom, each_pt);
  }
}

std::size_t ParkingPathOptimizer::GetNearestPointIndex(
    const ParkingPath& curr_parking_path) const {
  std::size_t ans{0};
  if (curr_parking_path.path_points.size() !=
      curr_parking_path.point_s.size()) {
    LOG_ERROR("path_points.size {} != point_s.size {}",
              curr_parking_path.path_points.size(),
              curr_parking_path.point_s.size());
    return ans;
  }

  double min_dis{std::numeric_limits<double>::max()};
  for (std::size_t i = 0; i < curr_parking_path.path_points.size(); ++i) {
    double dis = std::hypot(curr_parking_path.path_points[i].x() -
                                data_center_->vehicle_state_utm().X(),
                            curr_parking_path.path_points[i].y() -
                                data_center_->vehicle_state_utm().Y());
    if (dis < min_dis) {
      min_dis = dis;
      ans = i;
    }
  }
  return ans;
}

Vec3d ParkingPathOptimizer::GetPathPointFromS(
    const ParkingPath& curr_parking_path, double s) const {
  if (curr_parking_path.path_points.empty()) return {};
  std::size_t index{0};
  for (std::size_t i = 0; i < curr_parking_path.point_s.size(); ++i) {
    if (curr_parking_path.point_s[i] <= s) {
      index = i;
    } else {
      break;
    }
  }
  return curr_parking_path.path_points[index];
}

void ParkingPathOptimizer::BuildPathPoint(const std::vector<Vec3d>& origin_path,
                                          std::vector<PathPoint>& final_path,
                                          bool is_reverse_driving) const {
  if (origin_path.empty()) return;
  const std::size_t n{origin_path.size()};
  std::vector<double> ds(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    ds[i] = std::hypot(origin_path[i].x() - origin_path[i - 1].x(),
                       origin_path[i].y() - origin_path[i - 1].y());
  }

  double s{0.};
  final_path.reserve(n);
  final_path.emplace_back(Vec2d{origin_path[0].x(), origin_path[0].y()},
                          origin_path[0].z(), 0., 0., 0., 0.);
  for (std::size_t i = 1; i < n; ++i) {
    double kappa =
        ds[i] > 0.01
            ? normalize_angle(origin_path[i].z() - origin_path[i - 1].z()) /
                  ds[i]
            : 0.01;
    if (is_reverse_driving) kappa = -kappa;
    final_path.emplace_back(Vec2d{origin_path[i].x(), origin_path[i].y()},
                            origin_path[i].z(), kappa, 0., 0., s += ds[i]);
  }
  if (n > 1) final_path[0].set_kappa(final_path[1].kappa());
  return;
}

void ParkingPathOptimizer::UpdateDistToEnd() {
  auto parking_ptr = data_center_->parking_ptr();
  if (!parking_ptr) {
    dist_to_end_ = 0.0;
    return;
  }
  auto& parking_path = parking_ptr->ParkPath();
  if (parking_path.empty()) {
    dist_to_end_ = 0.0;
    return;
  }
  const auto& curr_parking_path = parking_path[path_idx_];
  std::size_t curr_index = GetNearestPointIndex(curr_parking_path);
  dist_to_end_ = curr_parking_path.point_s.back() -
                 curr_parking_path.stop_befor_end -
                 curr_parking_path.point_s[curr_index];
  if (path_idx_ == parking_path.size() - 1 &&
      parking_ptr->OriginPark()->Type() ==
          global::hdmap::ParkingSpace_ParkingType_VERTICAL &&
      curr_parking_path.is_park_in)
    dist_to_end_ = parking_ptr->GetDistToEnd();
}

void ParkingPathOptimizer::SetParkingSpeedLimit(
    TaskInfo& task_info, const ParkingPath& curr_parking_path) const {
  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  Vec3d ego_utm{data_center_->vehicle_state_utm().X(),
                data_center_->vehicle_state_utm().Y(),
                data_center_->vehicle_state_utm().Heading()};
  auto& parking_target_speed = task_info.current_frame()
                                   ->mutable_outside_planner_data()
                                   ->parking_outside_data.parking_speed_cmd;
  std::size_t curr_index = GetNearestPointIndex(curr_parking_path);
  if (curr_parking_path.is_park_in) {
    double curr_s = curr_parking_path.point_s.size() > curr_index
                        ? curr_parking_path.point_s[curr_index]
                        : 0.;
    auto projection_pt = GetPathPointFromS(curr_parking_path, curr_s);
    double closest_dis = common::Distance2D(projection_pt, ego_utm);
    double dist_target_speed =
        GetDistTargetSpeed(curr_parking_path, curr_s, dist_to_end_);
    double heading_diff_target_speed = GetHeadingDiffTargetSpeed(
        data_center_->vehicle_state_utm().Heading(),
        data_center_->parking_ptr()->Heading(), curr_parking_path);
    double dist_error_target_speed{
        path_idx_ == data_center_->parking_ptr()->ParkPath().size() - 1
            ? 0.56
            : parking_config.parking_in_max_speed};

    parking_target_speed =
        std::min(std::min(dist_target_speed, heading_diff_target_speed),
                 dist_error_target_speed);
    task_info.current_frame()->mutable_outside_planner_data()->speed_slow_down =
        parking_target_speed <= parking_config.parking_slow_down_dist &&
        path_idx_ == data_center_->parking_ptr()->ParkPath().size() - 1;
    LOG_INFO(
        "dist_target_speed:{},heading_diff_target_speed:{},"
        "dist_error_target_speed:{},parking_target_speed:{}",
        dist_target_speed, heading_diff_target_speed, dist_error_target_speed,
        parking_target_speed);
  } else {
    parking_target_speed =
        data_center_->parking_ptr()->OriginPark()->Type() ==
                global::hdmap::ParkingSpace_ParkingType_HORIZONTAL
            ? parking_config.parking_out_max_speed
            : parking_config.parkint_out_default_speed;
    // TODO: not use id
    if (data_center_->parking_ptr()->OriginPark()->Id() ==
            common::HashString("215299052001") ||
        data_center_->parking_ptr()->OriginPark()->Id() ==
            common::HashString("215299052002") ||
        data_center_->parking_ptr()->OriginPark()->Id() ==
            common::HashString("215299052003")) {
      parking_target_speed = 1.83;
    }

    auto path_data =
        task_info.current_frame()->outside_planner_data().path_data;
    if (path_data && !path_data->path().path_points().empty() &&
        path_data->path().path_points().size() > curr_index &&
        path_data->path().path_points().at(curr_index).kappa() >
            parking_config.parking_out_kappa_threshold) {
      parking_target_speed = parking_config.parking_out_speed_in_turn;
      LOG_INFO("kappa is {:.2f}, speed limit to {:.2f}",
               path_data->path().path_points().at(curr_index).kappa(),
               parking_target_speed);
    }
    LOG_INFO("Parking out speed limit to {:.2f}", parking_target_speed);
  }
}

void ParkingPathOptimizer::JudgeIfStuck() {
  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  if (ego_stuck_count_ >=
      static_cast<int>(kStuckTimeThreshold * FLAGS_planning_period_frequence)) {
    data_center_->mutable_event_report_proxy()->SetEvent(
        EventType::PARK_FAIL, data_center_->parking_ptr()->GetParkId());
    ego_stuck_count_ = 0;
  } else if (vehicle_state_.LinearVelocity() <
             parking_config.parking_stuck_speed_threshold) {
    ++ego_stuck_count_;
  } else {
    ego_stuck_count_ = 0;
  }
}

int ParkingPathOptimizer::RematchPathIndex() const {
  int ans{0};
  if (!data_center_->parking_ptr() ||
      !data_center_->parking_ptr()->OriginPark() ||
      data_center_->parking_ptr()->ParkPath().size() < 2) {
    return ans;
  }
  double min_dis{std::numeric_limits<double>::max()};
  const Vec3d ego_utm{data_center_->vehicle_state_utm().X(),
                      data_center_->vehicle_state_utm().Y(),
                      data_center_->vehicle_state_utm().Heading()};
  for (std::size_t i = 0; i < data_center_->parking_ptr()->ParkPath().size();
       ++i) {
    const auto& path = data_center_->parking_ptr()->ParkPath().at(i);
    auto proj_idx{GetNearestPointIndex(path)};
    const auto& proj_pt = path.path_points.at(proj_idx);
    double dis{common::Distance2D(proj_pt, ego_utm)};
    if (dis < min_dis) {
      min_dis = dis;
      ans = i;
    }
  }
  return ans;
}

double ParkingPathOptimizer::GetDistTargetSpeed(const ParkingPath& current_path,
                                                double current_s,
                                                double dis2end) const {
  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  double ret{parking_config.parking_in_max_speed};
  if (!current_path.is_need_stop) return ret;

  if (dis2end <= parking_config.parking_slow_down_dist) {
    ret = 0.0;
  } else {
    double expect_v = dis2end * parking_config.dist_slow_down_coef;
    LOG_INFO("dis to end:{},expect v:{}", dis2end, expect_v);
    ret = std::fmax(parking_config.vertical_parking_space.min_speed,
                    std::min(ret, expect_v));
  }
  return ret;
}

}  // namespace planning
}  // namespace neodrive
