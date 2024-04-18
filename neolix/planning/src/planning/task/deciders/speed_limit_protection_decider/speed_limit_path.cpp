#include "speed_limit_path.h"

#include "src/planning/util/speed_limit_trans.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

SpeedLimitPath::SpeedLimitPath() : SpeedLimitInterface("speed_limit_path") {}

void SpeedLimitPath::ComputeSpeedLimit(TaskInfo &task_info) {
  KappaLimit(task_info);

  DistanceToRoadLimit(task_info);

  DistanceToStaticObsLimit(task_info);

  LaneBorrowLimit(task_info);
}

void SpeedLimitPath::KappaLimit(TaskInfo &task_info) {
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
  double centric_accel_limit = inside_data.is_indoor
                                   ? plan_config.indoor.centric_accel_limit
                                   : plan_config.common.centric_accel_limit;
  double curr_kappa_limit =
      std::sqrt(centric_accel_limit /
                std::abs(path_data->path().path_points().front().kappa()));
  LOG_INFO("kappa is {:.2f}, curr_kappa_limit = {:.2f}",
           path_data->path().path_points().front().kappa(), curr_kappa_limit);
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

  if (path_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::PATH, SpeedLimitType::SOFT, path_limit, 0.0);
  }
}

void SpeedLimitPath::DistanceToRoadLimit(TaskInfo &task_info) {
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
          ->mutable_lane_borrow_context()
          ->stage == DetourStageState::REVERSE_LANE_BORROWING;

  /// Limit : distance to road
  double dis_road_limit = max_speed();
  double forward_valid_s = std::max(inside_data.vel_v, 3.0) * 4.0;
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
    double min_dis_to_road{1.e5}, min_dis_to_road_s{1.e5};
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

      if (min_dis_to_road > dis_to_road) {
        min_dis_to_road = dis_to_road;
        min_dis_to_road_s = pt.s();
      }
    }
    double static_road_dis_limit = max_speed();
    if (min_dis_to_road <= 0.15) {
      static_road_dis_limit = 2.0;
    } else if (min_dis_to_road <= 0.25) {
      static_road_dis_limit = 2.8;
    } else if (min_dis_to_road <= 0.3) {
      static_road_dis_limit = 3.5;
    }
    dis_road_limit = std::min(dis_road_limit, static_road_dis_limit);
  }

  LOG_INFO("dis_road_limit: {:.3f}", dis_road_limit);
  if (dis_road_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::ROAD_BOUNDARY_DIS, SpeedLimitType::SOFT,
                   dis_road_limit, 0.0);
  }
}

void SpeedLimitPath::DistanceToStaticObsLimit(
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

void SpeedLimitPath::LaneBorrowLimit(TaskInfo &task_info) {
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &reference_line = task_info.reference_line();
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  // reverse lane detour check
  bool ignore_left_road_limit =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_lane_borrow_context()
          ->stage == DetourStageState::REVERSE_LANE_BORROWING;

  bool consider_limit = false;
  double lane_borrow_limit = max_speed();
  if (inside_data.is_prepare_borrowing) {
    lane_borrow_limit = std::fmin(
        plan_config.speed_limit.prepare_borrowing_limit, lane_borrow_limit);
    consider_limit = true;
  }
  if (inside_data.is_lane_borrowing) {
    lane_borrow_limit = std::fmin(plan_config.speed_limit.lane_borrowing_limit,
                                  lane_borrow_limit);
    consider_limit = true;
  }

  // speed limit for lane borrow back stage
  double road_bound_length{100.0};
  if (inside_data.is_lane_borrowing) {
    double adc_upper_l =
        inside_data.init_sl_point.l() + 0.5 * VehicleParam::Instance()->width();
    double adc_lower_l =
        inside_data.init_sl_point.l() - 0.5 * VehicleParam::Instance()->width();
    double left_lane_bound =
        outside_data.veh_real_reference_point.left_lane_bound();
    double right_lane_bound =
        -outside_data.veh_real_reference_point.right_lane_bound();
    if (adc_upper_l > left_lane_bound || adc_lower_l < right_lane_bound) {
      for (auto &pt : reference_line->ref_points()) {
        if (pt.s() < inside_data.init_sl_point.s()) continue;
        // single road
        if (std::fabs(pt.left_road_bound() - pt.left_lane_bound()) < 0.3 &&
            std::fabs(pt.right_road_bound() - pt.right_lane_bound()) < 0.3) {
          road_bound_length = pt.s() - inside_data.init_sl_point.s();
          LOG_INFO("single road:  road_bound_length:{:.2f}", road_bound_length);
          break;
        }
        // fence
        if ((pt.left_boundary_edge_type() == BoundaryEdgeType::FENCE &&
             pt.left_road_bound() - adc_upper_l <
                 VehicleParam::Instance()->width()) ||
            (pt.right_boundary_edge_type() == BoundaryEdgeType::FENCE &&
             pt.right_road_bound() + adc_lower_l <
                 VehicleParam::Instance()->width())) {
          road_bound_length = pt.s() - inside_data.init_sl_point.s();
          LOG_INFO("fence:  road_bound_length:{:.2f}", road_bound_length);
          break;
        }
      }
      // speed limit
      if (road_bound_length < 7.0) {
        lane_borrow_limit = std::fmin(lane_borrow_limit, 1.5);
        consider_limit = true;
      } else if (road_bound_length < 10.0) {
        lane_borrow_limit = std::fmin(lane_borrow_limit, 2.0);
        consider_limit = true;
      } else if (road_bound_length < 15.0) {
        lane_borrow_limit = std::fmin(lane_borrow_limit, 2.5);
        consider_limit = true;
      }
    }
  }
  if (!ignore_left_road_limit && consider_limit) {
    SaveSpeedLimit(SpeedLimitType::LANE_BORROW, SpeedLimitType::SOFT,
                   lane_borrow_limit, 0.0);
  }
}

}  // namespace planning
}  // namespace neodrive
