#include "motorway_speed_limit_path.h"

#include "src/planning/util/speed_limit_trans.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

MotorwaySpeedLimitPath::MotorwaySpeedLimitPath()
    : MotorwaySpeedLimitInterface("motorway_speed_limit_path") {}

void MotorwaySpeedLimitPath::ComputeSpeedLimit(TaskInfo &task_info) {
  KappaLimit(task_info);

  DistanceToRoadLimit(task_info);

  DistanceToStaticObsLimit(task_info);

  LaneBorrowLimit(task_info);
}

void MotorwaySpeedLimitPath::KappaLimit(TaskInfo &task_info) {
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &path_data = outside_data.path_data;
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  /// Limit: path_points' size
  if (path_data->path().path_points().size() <= 3) {
    SaveSpeedLimit(SpeedLimitType::PATH, SpeedLimitType::SOFT, 0.0, 0.0);
    return;
  }
  /// Limit: valid region length
  valid_region_s_ = std::max(outside_data.path_context.valid_region_end_s,
                             outside_data.path_context.valid_backup_end_s) -
                    inside_data.init_sl_point.s();
  if (valid_region_s_ < 5.0) {
    SaveSpeedLimit(SpeedLimitType::PATH, SpeedLimitType::SOFT, 1.5, 0.0);
    return;
  } else if (valid_region_s_ < 10.0) {
    SaveSpeedLimit(SpeedLimitType::PATH, SpeedLimitType::SOFT, 2.5, 0.0);
    return;
  }
  /// Limit: kappa limit = std::min(min_kappa, cloest_kappa)
  double forward_valid_s =
      std::max(1.0, inside_data.init_point.velocity()) * 8.0;
  double centric_accel_limit = config::PlanningConfig::Instance()
                                   ->plan_config()
                                   .common.centric_accel_limit;
  double curr_kappa_limit =
      std::sqrt(centric_accel_limit /
                std::abs(path_data->path().path_points().front().kappa()));
  double min_end_v{2 * max_speed()}, end_v{2 * max_speed()};
  std::array<double, 3> init_state{0.0, inside_data.init_point.velocity(), 0.0};
  std::array<double, 3> min_end_state{}, cloest_end_state{};
  bool cloest_find{false};
  for (const auto &pt : path_data->path().path_points()) {
    if (pt.s() > valid_region_s_) break;
    if (pt.s() > forward_valid_s) break;
    end_v = std::sqrt(centric_accel_limit / std::abs(pt.kappa()));
    if (end_v < min_end_v && end_v < max_speed()) {
      min_end_state[0] = pt.s();
      min_end_state[1] = end_v;
      min_end_state[2] = 0.;
      min_end_v = end_v;
    }
    if (!cloest_find) {
      double cloest_end_v =
          std::sqrt(centric_accel_limit / std::abs(pt.kappa()));
      if (cloest_end_v < max_speed()) {
        cloest_end_state[0] = pt.s();
        cloest_end_state[1] = cloest_end_v;
        cloest_end_state[2] = 0.;
        cloest_find = true;
      }
    }
  }
  if (!cloest_find) return;

  /// state of min_end_v
  double min_end_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "path_min_kappa", init_state, min_end_state, max_speed(), 3.0);
  min_end_limit = std::max(0., std::min(curr_kappa_limit, min_end_limit));

  /// state of cloest_end_s
  double cloest_end_limit = cloest_find
                                ? SpeedLimitTrans::InfiniteLimitToSequenceLimit(
                                      "path_cloest_kappa", init_state,
                                      cloest_end_state, max_speed(), 3.0)
                                : max_speed();
  cloest_end_limit = std::max(0., std::min(curr_kappa_limit, cloest_end_limit));

  double path_limit = std::min(min_end_limit, cloest_end_limit);
  LOG_INFO("path_limit: {:.3f}", path_limit);

  if (std::min(min_end_limit, cloest_end_limit) < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::PATH, SpeedLimitType::SOFT,
                   std::min(min_end_limit, cloest_end_limit), 0.0);
  }
}

void MotorwaySpeedLimitPath::DistanceToRoadLimit(TaskInfo &task_info) {
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &reference_line = task_info.reference_line();
  const auto &path_data = outside_data.path_data;
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  if (path_data->path().path_points().empty()) return;

  // reverse lane detour check
  bool ignore_left_road_limit =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_motorway_lane_borrow_context()
          ->stage == MotorwayDetourStageState::REVERSE_LANE_BORROWING;

  /// Limit : distance to road
  double forward_valid_s = std::max(inside_data.vel_v, 3.0) * 8.0;
  using AD2 = std::array<double, 2>;  /// <longitudinal dis, lateral dis>
  std::vector<AD2> dis_infos{};
  for (std::size_t i = 0; i < path_data->path().path_points().size(); i += 3) {
    const auto &pt = path_data->path().path_points()[i];
    if (pt.s() > valid_region_s_) break;
    if (pt.s() > forward_valid_s) break;

    Box2d adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {pt.x(), pt.y()}, pt.theta(), 0.0, 0.0, 0.0);
    std::vector<Vec2d> tmp_corners;
    adc_box.get_all_corners(&tmp_corners);
    SLPoint sl_pt;
    ReferencePoint tmp_refer_pt;
    double min_dis_to_road{1.e5};
    for (auto corner_pt : tmp_corners) {
      if (!reference_line->GetPointInFrenetFrameWithHeading(
              {corner_pt.x(), corner_pt.y()}, pt.theta(), &sl_pt) ||
          !reference_line->GetNearestRefPointWithHeading(
              {corner_pt.x(), corner_pt.y()}, pt.theta(), &tmp_refer_pt)) {
        LOG_ERROR("get point to frenet coordinate failed");
        continue;
      }
      double dis_to_road;
      if (ignore_left_road_limit) {
        dis_to_road =
            std::min(std::max(tmp_refer_pt.left_road_bound(),
                              tmp_refer_pt.left_reverse_road_bound()) -
                         sl_pt.l(),
                     tmp_refer_pt.right_road_bound() + sl_pt.l());
      } else {
        dis_to_road = std::min(tmp_refer_pt.left_road_bound() - sl_pt.l(),
                               tmp_refer_pt.right_road_bound() + sl_pt.l());
      }

      min_dis_to_road = std::min(dis_to_road, min_dis_to_road);
    }
    dis_infos.push_back({pt.s(), min_dis_to_road});
  }
  AD2 cloest_min_info = {1.e5, 1.e5}, min_info = {1.e5, 1.e5};
  for (const auto &info : dis_infos) {
    if (info[1] < 0.3) {
      cloest_min_info = info;
      break;
    }
  }
  for (const auto &info : dis_infos) {
    if (min_info[1] > info[1] && info[1] < 0.3) {
      min_info = info;
    }
  }
  if (min_info[1] > 0.3) return;
  if (cloest_min_info[1] > 0.3 && min_info[1] > 0.3) return;

  auto limit = [](const auto &info) {
    if (info[1] <= 0.15) {
      return 2.0;
    } else if (info[1] <= 0.25) {
      return 2.8;
    } else if (info[1] <= 0.3) {
      return 3.5;
    }
    return 1.e5;
  };

  std::array<double, 3> init_state{0.0, inside_data.init_point.velocity(), 0.0};
  std::array<double, 3> min_end_state{min_info[0], limit(min_info)};
  std::array<double, 3> cloest_end_state{cloest_min_info[0],
                                         limit(cloest_min_info)};

  /// state of min_end_v
  double min_end_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "road_min_dis", init_state, min_end_state, max_speed(), 3.0);
  min_end_limit = std::max(0., std::min(max_speed(), min_end_limit));

  /// state of cloest_end_s
  double cloest_end_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "road_cloest_dis", init_state, cloest_end_state, max_speed(), 3.0);
  cloest_end_limit = std::max(0., std::min(max_speed(), cloest_end_limit));

  double road_dis_limit = std::min(min_end_limit, cloest_end_limit);
  LOG_INFO("road_dis_limit: {:.3f}", road_dis_limit);

  if (road_dis_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::ROAD_BOUNDARY_DIS, SpeedLimitType::SOFT,
                   road_dis_limit, 0.0);
  }
}

void MotorwaySpeedLimitPath::DistanceToStaticObsLimit(
    TaskInfo &task_info) {  // 1. adc on reference_line's position check
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &reference_line = task_info.reference_line();
  const auto &path_data = outside_data.path_data;
  const auto &decision_data_ptr = task_info.decision_data();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  if (path_data->path().path_points().empty()) return;

  // 2. distance to obs protection
  double dis_obs_limit = max_speed();
  double forward_valid_s = std::max(inside_data.vel_v, 3.0) * 4.0;
  for (const auto &obstacle : decision_data_ptr->static_obstacle()) {
    if (obstacle->is_virtual()) continue;
    if (!obstacle->is_static()) continue;
    if (obstacle->PolygonBoundary().end_s() < inside_data.init_sl_point.s()) {
      continue;
    }
    if (obstacle->PolygonBoundary().start_s() >
        std::max(valid_region_s_, forward_valid_s)) {
      continue;
    }
    double min_dis_to_static_obs(1000.0), min_dis_to_static_obs_s(1000.0);
    for (std::size_t i = 0; i < path_data->path().path_points().size();
         i += 3) {
      const auto &pt = path_data->path().path_points().at(i);
      if (pt.s() > valid_region_s_) break;
      if (pt.s() > forward_valid_s) break;

      // distance to static obs
      Box2d adc_box = VehicleParam::Instance()->get_adc_bounding_box(
          {pt.x(), pt.y()}, pt.theta(), 0.0, 0.0, 0.0);
      Polygon2d adc_polygon(adc_box);
      double dis = adc_polygon.distance_to_2(obstacle->polygon());
      if (dis <= kMathEpsilon) continue;
      if (dis < min_dis_to_static_obs) {
        min_dis_to_static_obs = dis;
        min_dis_to_static_obs_s = pt.s();
      }
    }
    double static_obs_dis_limit = max_speed();
    if (min_dis_to_static_obs <= 0.2) {
      static_obs_dis_limit = 2.0;
    } else if (min_dis_to_static_obs <= 0.3) {
      static_obs_dis_limit = 2.8;
    } else if (min_dis_to_static_obs <= 0.4) {
      static_obs_dis_limit = 3.5;
    }
    dis_obs_limit = std::min(dis_obs_limit, static_obs_dis_limit);
  }

  LOG_INFO("dis_obs_limit: {:.3f}", dis_obs_limit);
  if (dis_obs_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::STATIC_OBS_DIS, SpeedLimitType::SOFT,
                   dis_obs_limit, 0.0);
  }
}

void MotorwaySpeedLimitPath::LaneBorrowLimit(TaskInfo &task_info) {
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  double lane_borrow_limit = max_speed();
  if (inside_data.is_prepare_borrowing) {
    lane_borrow_limit = std::fmin(
        plan_config.speed_limit.prepare_borrowing_limit, lane_borrow_limit);
  }
  if (inside_data.is_lane_borrowing) {
    lane_borrow_limit = std::fmin(plan_config.speed_limit.lane_borrowing_limit,
                                  lane_borrow_limit);
  }
  if (lane_borrow_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::LANE_BORROW, SpeedLimitType::SOFT,
                   lane_borrow_limit, 0.0);
  }
}

}  // namespace planning
}  // namespace neodrive
