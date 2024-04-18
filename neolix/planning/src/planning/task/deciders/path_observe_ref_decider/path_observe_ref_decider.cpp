#include "path_observe_ref_decider.h"

#include <memory>

#include "common/visualizer_event/visualizer_event.h"
#include "observe_ref_for_follow_obs.h"
#include "observe_ref_for_lane_borrow.h"
#include "observe_ref_for_reverse_obs.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/deciders/pilot_state_decider/pilot_state_decider.h"
#include "src/planning/scenario_manager/scenario_common.h"

namespace neodrive {
namespace planning {

namespace {

void VisRefL(const neodrive::planning::ReferenceLinePtr &reference_line,
             const neodrive::planning::SLPoint &sl_pt,
             const std::string &name) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto set_pts = [&](const auto &pts) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pts.x());
    sphere->mutable_center()->set_y(pts.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.5);
  };
  Vec2d xy_point{};
  if (!reference_line->GetPointInCartesianFrame({sl_pt.s(), sl_pt.l()},
                                                &xy_point)) {
    LOG_ERROR("failed get closest point.");
    return;
  }
  set_pts(xy_point);
}

bool IsFarway(const Obstacle &obstacle, const Boundary &adc_boundary,
              const bool &isInMotorWay) {
  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  double observe_back_dis =
      isInMotorWay ? 2 * ref_conf.observe_back_dis : ref_conf.observe_back_dis;
  if (obstacle.min_s() - ref_conf.observe_front_dis > adc_boundary.end_s() ||
      obstacle.max_s() + observe_back_dis < adc_boundary.start_s() ||
      obstacle.min_l() - ref_conf.observe_left_dis > adc_boundary.end_l() ||
      obstacle.max_l() + ref_conf.observe_right_dis < adc_boundary.start_l()) {
    LOG_INFO(
        "skip faraway obs id[{}], front[{}] back[{}] left[{}] right[{}].",
        obstacle.id(),
        obstacle.min_s() - ref_conf.observe_front_dis > adc_boundary.end_s(),
        obstacle.max_s() +
            ref_conf
                .observe_back_dis<adc_boundary.start_s(),
                                  obstacle.min_l() - ref_conf.observe_left_dis>
                    adc_boundary.end_l(),
        obstacle.max_l() + ref_conf.observe_right_dis < adc_boundary.start_l());
    return false;
  } else {
    return true;
  }
}

bool IsInbound(Obstacle &obstacle,
               const std::vector<neodrive::planning::PathRegion::Bound> &bounds,
               double &obs_left_witdh, double &obs_right_witdh,
               const bool &isInMotorWay) {
  if (bounds.empty()) return false;

  double begin_delt_s =
      (obstacle.min_s() - bounds.front().lower_point.s()) / 0.1;
  double end_delt_s = (obstacle.max_s() - bounds.front().lower_point.s()) / 0.1;
  size_t start_index = 0;
  while (start_index < bounds.size()) {
    if (bounds[start_index].lower_type != PathRegion::Bound::BoundType::VIR &&
        bounds[start_index].upper_type != PathRegion::Bound::BoundType::VIR)
      break;
    ++start_index;
  }

  size_t end_index = std::clamp(static_cast<size_t>(std::max(0.0, end_delt_s)),
                                start_index, bounds.size() - 1);
  size_t begin_index =
      std::clamp(static_cast<size_t>(std::max(0.0, begin_delt_s)), start_index,
                 bounds.size() - 1);
  LOG_INFO(
      "start_index: {}, search begin: {}, end: {}, bounds size: {}. bounds s: "
      "{:.4f}, begin_delt_s: {:.4f}, end_delt_s: {:.4f}",
      start_index, begin_index, end_index, bounds.size(),
      bounds.front().lower_point.s(), begin_delt_s, end_delt_s);
  if (begin_index <= end_index) {
    double max_upper_l = 0.0, min_upper_l = 100.0;
    double max_lower_l = -100.0, min_lower_l = 0;
    // Screen obstacles based on the bounds
    for (size_t i = begin_index; i <= end_index; ++i) {
      max_upper_l = std::max(max_upper_l, bounds[i].upper_point.l());
      min_upper_l = std::min(min_upper_l, bounds[i].upper_point.l());
      max_lower_l = std::max(max_lower_l, bounds[i].lower_point.l());
      min_lower_l = std::min(min_lower_l, bounds[i].lower_point.l());
    }
    obs_left_witdh = min_upper_l - obstacle.max_l();
    obs_right_witdh = obstacle.min_l() - max_lower_l;
    LOG_INFO(
        "lower_point l: {:.4f}, upper_point l: {:.4f}, obstacle.min_l: "
        "{:.4f}, obstacle.max_l(): {:.4f}, obs_left_witdh: {:.4f}, "
        "obs_right_witdh: {:.4f}",
        max_lower_l, min_upper_l, obstacle.min_l(), obstacle.max_l(),
        obs_left_witdh, obs_right_witdh);

    auto &ref_conf = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .path_observe_ref_decider_config;
    if (obstacle.min_l() > max_upper_l + ref_conf.extend_buff ||
        obstacle.max_l() < min_lower_l - ref_conf.extend_buff) {
      LOG_INFO("skip obs [{}] not in lane bound", obstacle.id());
      return false;
    }
    if (isInMotorWay) {
      if ((obstacle.min_l() < max_upper_l + ref_conf.extend_buff &&
           obstacle.min_l() > max_upper_l) ||
          (obstacle.max_l() > min_lower_l - ref_conf.extend_buff &&
           obstacle.max_l() < min_lower_l)) {
        obstacle.set_width(ref_conf.virtual_width);
        LOG_INFO(
            "obs[{}] is out current lane. extend_buff: {:.4f}, set virtual "
            "width: {:.4f}",
            obstacle.id(), ref_conf.extend_buff, ref_conf.virtual_width);
      }
    }
  } else {
    LOG_INFO("skip obs [{}] not in lane bound", obstacle.id());
    return false;
  }
  return true;
}

bool ComputeAttentionInfos(TaskInfo &task_info, SLPoint &sl_pt) {
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data == nullptr) {
    LOG_INFO("decision_data null");
    return false;
  }

  auto &ref_ptr = task_info.reference_line();
  if (ref_ptr == nullptr || ref_ptr->ref_points().empty()) {
    return false;
  }

  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &shrink_bounds_info =
      outside_data.path_context.shrink_path_boundary.path_boundary;
  const auto &original_path_boundary =
      outside_data.path_context.original_path_boundary.path_boundary;
  if (shrink_bounds_info.empty() || original_path_boundary.empty()) {
    LOG_INFO("shrink_bounds_info or original_path_boundary is empty");
    return false;
  }
  const auto &adc_boundary = outside_data.path_obstacle_context.adc_boundary;

  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  const float &front_dis = ref_conf.danger_range.front_dis;
  const float &back_dis = ref_conf.danger_range.back_dis;
  const float &left_dis = ref_conf.danger_range.left_dis;
  const float &right_dis = ref_conf.danger_range.right_dis;
  std::vector<Obstacle> near_obstacles{};
  std::vector<Obstacle> far_obstacles{};
  bool find_front_slow_obs = false;
  double front_slow_obs_max_l = -100.0;
  static double veh_v = inside_data.init_point.velocity();
  double k = config::PlanningConfig::Instance()
                 ->planning_research_config()
                 .path_observe_ref_decider_config.vhe_v_filter_ratio;
  veh_v = veh_v * (1.0 - k) + k * task_info.current_frame()
                                      ->inside_planner_data()
                                      .init_point.velocity();
  LOG_INFO(
      "adc min_s: {:.4f}, max_s: {:.4f}, min_l: {:.4f}, max_l: "
      "{:.4f}, v: {:.4f}",
      adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
      adc_boundary.end_l(), veh_v);
  // Screening obstacles
  LOG_INFO("all_obstacle size: {}", decision_data->all_obstacle().size());
  for (size_t i = 0; i < decision_data->dynamic_obstacle().size(); ++i) {
    auto obstacle = *decision_data->dynamic_obstacle()[i];
    LOG_INFO(
        "obstacle[{}] min_s: {:.4f}, max_s: {:.4f}, min_l: {:.4f}, max_l: "
        "{:.4f}, v: {:.4f}",
        obstacle.id(), obstacle.min_s(), obstacle.max_s(), obstacle.min_l(),
        obstacle.max_l(), obstacle.speed());
    // faraway obs
    if (!IsFarway(obstacle, adc_boundary,
                  task_info.curr_referline_pt().lane_type_is_city_driving())) {
      continue;
    }

    // in bound?
    double obs_left_witdh = 0.0;
    double obs_right_witdh = 0.0;
    if (!IsInbound(obstacle, original_path_boundary, obs_left_witdh,
                   obs_right_witdh,
                   task_info.curr_referline_pt().lane_type_is_city_driving())) {
      continue;
    }

    /***/
    ReferencePoint reference_point;
    if (!ref_ptr->GetNearestRefPoint(obstacle.center_sl().s(),
                                     &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double obs_heading = obstacle.velocity_heading();
    double heading_diff = std::abs(normalize_angle(obstacle.velocity_heading() -
                                                   reference_point.heading()));
    if (heading_diff > ref_conf.filter_obs_heading_threshold &&
        M_PI - heading_diff > ref_conf.filter_obs_heading_threshold) {
      LOG_INFO("skip oblique obstacle[{}] heading_diff: {:.4f}", obstacle.id(),
               heading_diff);
      continue;
    }
    if (heading_diff > M_PI / 2.0 && obs_right_witdh < 0.0) {
      LOG_INFO("skip right obverse obstacle[{}] obs_right_witdh: {:.4f}",
               obstacle.id(), obs_right_witdh);
      continue;
    }

    if (!(obstacle.min_s() - front_dis > adc_boundary.end_s() ||
          obstacle.max_s() + back_dis < adc_boundary.start_s() ||
          obstacle.min_l() - left_dis > adc_boundary.end_l() ||
          obstacle.max_l() + right_dis < adc_boundary.start_l())) {
      if (heading_diff > M_PI / 2.0 &&
          obstacle.max_s() + 1.0 < adc_boundary.start_s()) {
        LOG_INFO("skip obverse back obstacle[{}] heading_diff: {:.4f}",
                 obstacle.id(), heading_diff);
        continue;
      }
      LOG_INFO("find near obs[{}], front[{}] back[{}] left[{}] right[{}].",
               obstacle.id(),
               obstacle.min_s() - front_dis > adc_boundary.end_s(),
               obstacle.max_s() +
                   back_dis<adc_boundary.start_s(), obstacle.min_l() - left_dis>
                       adc_boundary.end_l(),
               obstacle.max_l() + right_dis < adc_boundary.start_l());
      near_obstacles.push_back(obstacle);
      continue;
    }

    if (heading_diff < M_PI / 2.0) {
      // Same direction
      // Establishing an artificial potential field
      double max_v = std::max(veh_v, 3.0);
      if (obstacle.min_s() > adc_boundary.end_s() && obstacle.speed() > max_v) {
        LOG_INFO(
            "skip front high speed obs[{}]: obs speed: {:.4f}, veh_v: {:.4f}",
            obstacle.id(), obstacle.speed(), max_v);
        continue;
      }
      if (obstacle.max_s() < adc_boundary.start_s() &&
          obstacle.speed() < veh_v) {
        LOG_INFO(
            "skip back low speed obs[{}]: obs speed: {:.4f}, veh_v: {:.4f}",
            obstacle.id(), obstacle.speed(), veh_v);
        continue;
      }
      if (obstacle.min_s() > adc_boundary.end_s() && obstacle.speed() < max_v) {
        double delt_s = obstacle.min_s() - front_dis - adc_boundary.end_s();
        double delt_v = max_v - obstacle.speed();
        double delt_t = delt_s / delt_v;
        if (delt_t > 8.0 || delt_t < 0.0) {
          LOG_INFO(
              "skip front obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          continue;
        } else {
          if (obs_left_witdh > obs_right_witdh && obs_right_witdh < 2.0 &&
              (far_obstacles.empty() ||
               (!far_obstacles.empty() && find_front_slow_obs))) {
            find_front_slow_obs = true;
            front_slow_obs_max_l =
                std::max(front_slow_obs_max_l, obstacle.max_l());
            LOG_INFO("take over obs[{}], front_slow_obs_max_l: {:.4f}",
                     obstacle.id(), front_slow_obs_max_l);
          } else {
            find_front_slow_obs = false;
          }
          LOG_INFO(
              "init s : {:.4f}, veh_v: {:.4f}, time: {:.4f}, "
              "delt_s: {:.4f}, delt_v: {:.4f}",
              inside_data.init_sl_point.s(), max_v, delt_t, delt_s, delt_v);
        }
      }
      if (obstacle.max_s() < adc_boundary.start_s() &&
          obstacle.speed() > veh_v) {
        double delt_s = adc_boundary.start_s() - back_dis - obstacle.max_s();
        double delt_v = obstacle.speed() - veh_v;
        double delt_t = delt_s / delt_v;
        if (delt_t > 8.0 || delt_t < 0.0) {
          LOG_INFO(
              "skip back obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          continue;
        }
        if (delt_t < 1.0) {
          near_obstacles.push_back(obstacle);
          LOG_INFO(
              "find back dangerous obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
              "delt_t: {:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          continue;
        }

        find_front_slow_obs = false;
        LOG_INFO(
            "init s : {:.4f}, veh_v: {:.4f}, time: {:.4f}, "
            "delt_s: {:.4f}, delt_v: {:.4f}",
            inside_data.init_sl_point.s(), veh_v, delt_t, delt_s, delt_v);
      }
    } else {
      // Opposite direction
      if (obstacle.max_s() < adc_boundary.start_s()) {
        LOG_INFO("reverse back obs[{}]: obs max_s: {:.4f}, veh start_s: {:.4f}",
                 obstacle.id(), obstacle.max_s(), adc_boundary.start_s());
        continue;
      }
      double delt_s = obstacle.min_s() - adc_boundary.end_s() - back_dis;
      double delt_v = obstacle.speed() + veh_v;
      double delt_t = delt_s / delt_v;
      if (delt_t > 8.0 || delt_t < 0.0) {
        LOG_INFO(
            "skip front reverse obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
            "delt_t: {:.4f}",
            obstacle.id(), delt_s, delt_v, delt_t);
        continue;
      }
      if (delt_t < 1.0) {
        near_obstacles.push_back(obstacle);
        LOG_INFO(
            "find back dangerous obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
            "delt_t: {:.4f}",
            obstacle.id(), delt_s, delt_v, delt_t);
        continue;
      }

      find_front_slow_obs = false;
      LOG_INFO(
          "init s : {:.4f}, veh_v: {:.4f}, time: {:.4f}, "
          "delt_s: {:.4f}, delt_v: {:.4f}",
          inside_data.init_sl_point.s(), veh_v, delt_t, delt_s, delt_v);
    }
    LOG_INFO("find far obs[{}]", obstacle.id());
    far_obstacles.push_back(obstacle);
  }
  LOG_INFO("near_obstacles size: {}, far_obstacles: {}", near_obstacles.size(),
           far_obstacles.size());

  // MakeDecision
  double left_l = 100.0;
  double right_l = -100.0;
  for (auto &bound : shrink_bounds_info) {
    if (bound.upper_type != PathRegion::Bound::BoundType::VIR)
      left_l = std::min(left_l, bound.upper_point.l());
    if (bound.lower_type != PathRegion::Bound::BoundType::VIR)
      right_l = std::max(right_l, bound.lower_point.l());
  }
  LOG_INFO("left_l: {:.4f}, right_l: {:.4f}", left_l, right_l);
  auto &observe_ref_l_info = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->path_observe_ref_l_info;
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  double max_width = 0.0;
  for (auto &obs : far_obstacles) {
    max_width = std::max(max_width, obs.width());
  }
  for (auto &obs : near_obstacles) {
    max_width = std::max(max_width, obs.width());
  }
  if (near_obstacles.empty()) {
    if (far_obstacles.empty()) {
      // observe_ref_l = 0.0;
      LOG_INFO("go middle");
    } else {
      if (find_front_slow_obs) {
        observe_ref_l = std::max(
            inside_data.init_sl_point.l(),
            std::min(left_l, front_slow_obs_max_l +
                                 0.5 * VehicleParam::Instance()->width() +
                                 2.0));
        LOG_INFO("go left overtake");
      } else {
        DataCenter *data_center{DataCenter::Instance()};
        if ((data_center->master_info().curr_scenario() ==
                 ScenarioState::DETOUR &&
             data_center->master_info()
                 .lane_borrow_context()
                 .is_refer_lane_static_obs_clear) ||
            (data_center->master_info().curr_scenario() ==
                 ScenarioState::MOTORWAY_DETOUR &&
             data_center->master_info()
                 .motorway_lane_borrow_context()
                 .is_refer_lane_static_obs_clear)) {
          observe_ref_l = std::min(
              std::max(right_l, left_l - max_width -
                                    0.5 * VehicleParam::Instance()->width()),
              std::min(inside_data.init_sl_point.l(), 0.0));
        } else {
          observe_ref_l = std::min(
              std::max(right_l, left_l - max_width -
                                    0.5 * VehicleParam::Instance()->width()),
              inside_data.init_sl_point.l());
        }
        LOG_INFO("go right race");
      }
    }
  } else {
    bool right_find = false;
    for (auto &obs : near_obstacles) {
      if ((obs.min_l() < adc_boundary.start_l() ||
           obs.max_l() < adc_boundary.end_l())) {
        right_find = true;
        break;
      }
    }
    bool only_front = true;
    for (auto &obs : near_obstacles) {
      ReferencePoint init_reference_point;
      if (!ref_ptr->GetNearestRefPoint(obs.center_sl().s(),
                                       &init_reference_point)) {
        LOG_INFO("GetNearestRefPoint fail");
        return false;
      }
      double heading_diff = std::abs(normalize_angle(
          obs.velocity_heading() - init_reference_point.heading()));
      if (!(obs.min_s() > adc_boundary.end_s() && obs.speed() > veh_v &&
            heading_diff < M_PI / 2.0)) {
        only_front = false;
        break;
      }
    }

    bool only_near_right_front = true;
    double near_front_slow_obs_max_l = right_l;
    for (auto &obs : near_obstacles) {
      ReferencePoint init_reference_point;
      ref_ptr->GetNearestRefPoint(obs.center_sl().s(), &init_reference_point);
      double heading_diff = std::abs(normalize_angle(
          obs.velocity_heading() - init_reference_point.heading()));
      if (obs.min_s() > adc_boundary.end_s() &&
          obs.speed() < std::max(veh_v, 3.0) && heading_diff < M_PI / 2.0 &&
          obs.max_l() < inside_data.init_sl_point.l() &&
          (left_l + 0.5 * VehicleParam::Instance()->width() - obs.max_l() >
           2.0)) {
        near_front_slow_obs_max_l =
            std::max(near_front_slow_obs_max_l, obs.max_l());
      } else {
        only_near_right_front = false;
      }
    }

    bool only_near_right = true;
    double near_right_obs_max_l = right_l;
    for (auto &obs : near_obstacles) {
      ReferencePoint init_reference_point;
      ref_ptr->GetNearestRefPoint(obs.center_sl().s(), &init_reference_point);
      double heading_diff = std::abs(normalize_angle(
          obs.velocity_heading() - init_reference_point.heading()));
      if (!(obs.min_s() > adc_boundary.end_s() ||
            obs.max_s() < adc_boundary.start_s()) &&
          heading_diff < M_PI / 2.0 &&
          obs.max_l() < inside_data.init_sl_point.l()) {
        near_right_obs_max_l = std::max(near_right_obs_max_l, obs.max_l());
      } else {
        only_near_right = false;
      }
    }
    if (far_obstacles.empty() && only_near_right) {
      observe_ref_l = std::max(
          inside_data.init_sl_point.l(),
          std::min(left_l, near_right_obs_max_l +
                               0.5 * VehicleParam::Instance()->width() + 1.0));
      LOG_INFO("near obs go left avoid, near_right_obs_max_l: {:.4f}",
               near_right_obs_max_l);
    } else if (far_obstacles.empty() && only_near_right_front) {
      observe_ref_l = std::max(
          inside_data.init_sl_point.l(),
          std::min(left_l, near_front_slow_obs_max_l +
                               0.5 * VehicleParam::Instance()->width() + 2.0));
      LOG_INFO("near obs go left overtake. near_front_slow_obs_max_l: {:.4f}",
               near_front_slow_obs_max_l);
    } else if (right_find || only_front) {
      observe_ref_l = inside_data.init_sl_point.l();
      LOG_INFO(
          "right dangerous[{}], only front high speed obs[{}], keep current l",
          right_find, only_front);
    } else {
      DataCenter *data_center{DataCenter::Instance()};
      if ((data_center->master_info().curr_scenario() ==
               ScenarioState::DETOUR &&
           data_center->master_info()
               .lane_borrow_context()
               .is_refer_lane_static_obs_clear) ||
          (data_center->master_info().curr_scenario() ==
               ScenarioState::MOTORWAY_DETOUR &&
           data_center->master_info()
               .motorway_lane_borrow_context()
               .is_refer_lane_static_obs_clear)) {
        observe_ref_l = std::min(
            std::max(right_l, left_l - max_width -
                                  0.5 * VehicleParam::Instance()->width()),
            std::min(inside_data.init_sl_point.l(), 0.0));
      } else {
        observe_ref_l = std::min(
            std::max(right_l, left_l - max_width -
                                  0.5 * VehicleParam::Instance()->width()),
            inside_data.init_sl_point.l());
      }
      LOG_INFO("go right avoidance");
    }
  }
  sl_pt = SLPoint{inside_data.init_sl_point.s(), observe_ref_l};
  VisRefL(ref_ptr, sl_pt, "ref_l");
  LOG_INFO("target l: {:.4f}", observe_ref_l);

  static char str_buffer[256];
  sprintf(str_buffer,
          "[CRUISE][near obs: %d] [far obs: %d] [target l: %f] "
          "[only_front_slow_obs: %d] [filter veh_v: %f]",
          near_obstacles.size(), far_obstacles.size(), observe_ref_l,
          find_front_slow_obs, veh_v);
  DataCenter::Instance()->SetMonitorString(str_buffer,
                                           MonitorItemSource::CRUISE_STATE);
  return true;
}

bool ComputeAttentionInfos(
    TaskInfo &task_info,
    const config::AutoPlanningResearchConfig::PathObserveRefDeciderConfig::
        RangesPartition &ranges_partition) {
  auto ref_ptr = task_info.reference_line();
  if (ref_ptr == nullptr || ref_ptr->ref_points().empty()) {
    return false;
  }
  const auto &sl_pt =
      task_info.current_frame()->inside_planner_data().init_sl_point;
  auto &observe_ref_l_info = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->path_observe_ref_l_info;
  auto &front_attention = observe_ref_l_info.front_attention;
  auto &left_attention = observe_ref_l_info.left_attention;
  auto &right_attention = observe_ref_l_info.right_attention;
  front_attention.Reset(), left_attention.Reset(), right_attention.Reset();

  /// adc on reference line
  ReferencePoint ref_pt{};
  if (!ref_ptr->GetNearestRefPoint(sl_pt.s(), &ref_pt)) {
    LOG_WARN("could not find nearest ref point.");
    return false;
  }
  double width_threshold = VehicleParam::Instance()->width() * 0.5;
  bool adc_on_refer_lane =
      ((sl_pt.l() > ref_pt.left_lane_bound() - width_threshold) ||
       (sl_pt.l() < -ref_pt.right_lane_bound() + width_threshold))
          ? false
          : true;
  if (!adc_on_refer_lane) {
    LOG_INFO("adc is not on refer lane, skip");
    return false;
  }

  /// single road
  bool is_single_road{false};
  for (double s = sl_pt.s();
       s < sl_pt.s() + ranges_partition.single_road_check_dis; s += 1.0) {
    if (s > ref_ptr->ref_points().back().s()) continue;
    ReferencePoint pt{};
    if (!ref_ptr->GetNearestRefPoint(s, &pt)) {
      LOG_WARN("could not get nearest ref point");
      continue;
    }
    if (std::abs(pt.left_lane_bound() - pt.left_road_bound()) < 0.3 &&
        std::abs(pt.right_lane_bound() - pt.right_road_bound()) < 0.3) {
      is_single_road = true;
      break;
    }
  }
  if (!is_single_road) {
    LOG_INFO("front 15.0 m is not in single road, skip");
    return false;
  }

  /// ranges partition
  const auto &adc_boundary = task_info.adc_boundary_origin();
  if (adc_boundary.start_s() > adc_boundary.end_s()) {
    return false;
  }

  double length = adc_boundary.end_s() - adc_boundary.start_s();
  ReferencePoint start_pt{}, end_pt{};
  // front rangs
  for (double start_s = adc_boundary.start_s(), end_s = start_s + length;
       end_s < adc_boundary.end_s() + ranges_partition.front_longitudinal_range;
       start_s += length, end_s = start_s + length) {
    if (start_s > ref_ptr->ref_points().back().s() ||
        end_s > ref_ptr->ref_points().back().s()) {
      LOG_WARN("start_s/end_s > ref_points.back.s");
      continue;
    }
    if (!ref_ptr->GetNearestRefPoint(start_s, &start_pt) ||
        !ref_ptr->GetNearestRefPoint(end_s, &end_pt)) {
      LOG_WARN("could not get nearest ref point");
      continue;
    }
    double min_l = std::max(
        adc_boundary.start_l() - ranges_partition.front_lateral_range,
        std::min(-start_pt.right_lane_bound(), -end_pt.right_lane_bound()));
    double max_l = std::min(
        adc_boundary.end_l() + ranges_partition.front_lateral_range,
        std::max(start_pt.left_lane_bound(), end_pt.left_lane_bound()));
    front_attention.ranges.emplace_back(Boundary(start_s, end_s, min_l, max_l));
    LOG_INFO("front ranges: {:.3f}, {:.3f}, {:.3f}, {:.3f}", start_s, end_s,
             min_l, max_l);
  }
  // left and right ranges
  for (double end_s = adc_boundary.end_s() +
                      ranges_partition.left_right_front_range,
              start_s = end_s - length;
       start_s >
       adc_boundary.start_s() - ranges_partition.left_right_back_range;
       end_s -= length, start_s = end_s - length) {
    if (start_s < ref_ptr->ref_points().front().s() ||
        end_s < ref_ptr->ref_points().front().s()) {
      LOG_WARN("start_s/end_s < ref_points.front.s");
      continue;
    }
    if (!ref_ptr->GetNearestRefPoint(start_s, &start_pt) ||
        !ref_ptr->GetNearestRefPoint(end_s, &end_pt)) {
      LOG_WARN("could not get nearest ref point");
      continue;
    }
    double min_l = std::max(
        adc_boundary.start_l() - ranges_partition.left_right_lateral_range,
        std::min(-start_pt.right_lane_bound(), -end_pt.right_lane_bound()));
    double max_l = std::min(
        adc_boundary.end_l() + ranges_partition.left_right_lateral_range,
        std::max(start_pt.left_lane_bound(), end_pt.left_lane_bound()));
    right_attention.ranges.emplace_back(
        Boundary(start_s, end_s, min_l,
                 (adc_boundary.start_l() + adc_boundary.end_l()) / 2.));
    left_attention.ranges.emplace_back(
        Boundary(start_s, end_s,
                 (adc_boundary.start_l() + adc_boundary.end_l()) / 2., max_l));
    LOG_INFO("right ranges: {:.3f}, {:.3f}, {:.3f}, {:.3f}", start_s, end_s,
             min_l, (adc_boundary.start_l() + adc_boundary.end_l()) / 2.);
    LOG_INFO("left ranges: {:.3f}, {:.3f}, {:.3f}, {:.3f}", start_s, end_s,
             (adc_boundary.start_l() + adc_boundary.end_l()) / 2., max_l);
  }
  if (front_attention.ranges.empty() && left_attention.ranges.empty() &&
      right_attention.ranges.empty()) {
    LOG_WARN("front/left/right ranges empty.");
    return false;
  }

  /// match obstacle with ranges
  auto min_compute = [](const auto &ranges, double *min_l, double *max_l,
                        double *min_s, double *max_s) {
    for (const auto &range : ranges) {
      *min_l = std::min(range.start_l(), *min_l);
      *max_l = std::max(range.end_l(), *max_l);
      *min_s = std::min(range.start_s(), *min_s);
      *max_s = std::max(range.end_s(), *max_s);
    }
  };
  double min_l{1000.0}, max_l{-1000.};
  double min_s{ref_ptr->ref_points().back().s() + 1000.};
  double max_s{ref_ptr->ref_points().front().s() - 1000.};
  min_compute(front_attention.ranges, &min_l, &max_l, &min_s, &max_s);
  min_compute(left_attention.ranges, &min_l, &max_l, &min_s, &max_s);
  min_compute(right_attention.ranges, &min_l, &max_l, &min_s, &max_s);
  LOG_INFO("min_l, max_l, min_s, max_s: {:.3f}, {:.3f}, {:.3f}, {:.3f}", min_l,
           max_l, min_s, max_s);

  double adc_theta = DataCenter::Instance()->vehicle_state_proxy().Heading();
  for (const auto &obs : task_info.decision_data()->dynamic_obstacle()) {
    double heading_diff = normalize_angle(obs->velocity_heading() - adc_theta);
    double project_vel = obs->speed() * std::cos(heading_diff);
    bool reverse_obs = project_vel < 0.01 ? true : false;
    LOG_INFO("obs[{}], {:.3f}, {:.3f}, {:.3f}, {:.3f}, {}", obs->id(),
             obs->PolygonBoundary().start_s(), obs->PolygonBoundary().end_s(),
             obs->PolygonBoundary().start_l(), obs->PolygonBoundary().end_l(),
             reverse_obs);
    if (obs->PolygonBoundary().end_s() < min_s ||
        obs->PolygonBoundary().start_s() > max_s) {
      continue;
    }
    if (obs->PolygonBoundary().end_l() < min_l ||
        obs->PolygonBoundary().start_l() > max_l) {
      continue;
    }
    // math front rangs
    if (reverse_obs) {
      for (auto &boundary : front_attention.ranges) {
        if (boundary.has_overlap(obs->PolygonBoundary())) {
          front_attention.obstacles.emplace_back(*obs);
          break;
        }
      }
      continue;
    }
    // math left && right ranges
    for (auto &boundary : left_attention.ranges) {
      if (boundary.has_overlap(obs->PolygonBoundary())) {
        left_attention.obstacles.emplace_back(*obs);
        break;
      }
    }
    for (auto &boundary : right_attention.ranges) {
      if (boundary.has_overlap(obs->PolygonBoundary())) {
        right_attention.obstacles.emplace_back(*obs);
        break;
      }
    }
  }
  LOG_INFO("front/left/right obstacles size: {}, {}, {}",
           front_attention.obstacles.size(), left_attention.obstacles.size(),
           right_attention.obstacles.size());

  return !(front_attention.obstacles.empty() &&
           left_attention.obstacles.empty() &&
           right_attention.obstacles.empty());
}

void VisObstacles(std::vector<Box2d> &boxes, const std::string &name) {
  if (!FLAGS_planning_enable_vis_event || boxes.empty()) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto &box : boxes) {
    auto polygon = event->mutable_polygon()->Add();
    std::vector<Vec2d> corners;
    box.get_all_corners(&corners);
    for (auto &pt : corners) {
      set_pt(polygon->add_point(), pt);
    }
  }
}

void VisObstacles(std::vector<Obstacle> &obstacles, const std::string &name) {
  if (!FLAGS_planning_enable_vis_event || obstacles.empty()) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto &obstacle : obstacles) {
    auto polygon = event->mutable_polygon()->Add();
    for (auto &pt : obstacle.polygon_corners())
      set_pt(polygon->add_point(), pt);

    auto text = event->mutable_text()->Add();
    set_pt(text->mutable_position(), obstacle.center());
    text->set_text("id: " + std::to_string(obstacle.id()));
  }
}

bool ComputeAttentionInfos(TaskInfo &task_info) {
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data == nullptr) {
    LOG_INFO("decision_data null");
    return false;
  }

  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  DataCenter *data_center = DataCenter::Instance();
  const auto &prev_trajectory =
      data_center->last_frame()->planning_data().computed_trajectory();
  if (prev_trajectory.num_of_points() < 1 && ref_conf.use_prev_trajectory) {
    LOG_INFO("prev_trajectory.num_of_points() < 1");
    return false;
  }

  auto &ref_ptr = task_info.reference_line();
  if (ref_ptr == nullptr || ref_ptr->ref_points().empty()) {
    return false;
  }
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &outside_data =
      task_info.current_frame()->mutable_outside_planner_data();
  const auto &bounds_info =
      outside_data->path_context.original_path_boundary.path_boundary;
  const auto &adc_boundary = outside_data->path_obstacle_context.adc_boundary;

  auto &attention_dynamic_obstacles =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->path_observe_ref_l_info.attention_dynamic_obstacles;
  std::vector<Obstacle> all_obstacles{};
  std::vector<Obstacle> observe_obstacles{};
  std::vector<Vec2d> points_pre{};

  for (size_t i = 0; i < decision_data->dynamic_obstacle().size(); ++i) {
    auto obstacle = *decision_data->dynamic_obstacle()[i];
    LOG_INFO("XXXdynamic_obstacle size: {}, current id {}",
             decision_data->dynamic_obstacle().size(), obstacle.id());

    // faraway obs
    if (obstacle.min_s() - ref_conf.observe_front_dis > adc_boundary.end_s() ||
        obstacle.max_s() + ref_conf.observe_back_dis < adc_boundary.start_s() ||
        obstacle.min_l() - ref_conf.observe_left_dis > adc_boundary.end_l() ||
        obstacle.max_l() + ref_conf.observe_right_dis <
            adc_boundary.start_l()) {
      LOG_INFO(
          "skip faraway obs id[{}], front[{}] back[{}] left[{}] right[{}].",
          obstacle.id(),
          obstacle.min_s() - ref_conf.observe_front_dis > adc_boundary.end_s(),
          obstacle.max_s() +
              ref_conf
                  .observe_back_dis<adc_boundary.start_s(),
                                    obstacle.min_l() -
                                        ref_conf.observe_left_dis>
                      adc_boundary.end_l(),
          obstacle.max_l() + ref_conf.observe_right_dis <
              adc_boundary.start_l());
      continue;
    }

    if (obstacle.speed() < ref_conf.observe_low_speed ||
        obstacle.speed() > ref_conf.observe_high_speed) {
      LOG_INFO("skip over speed obstacle[{}] speed: {:.4f}", obstacle.id(),
               obstacle.speed());
      continue;
    }

    ReferencePoint reference_point;
    if (!ref_ptr->GetNearestRefPoint(obstacle.center_sl().s(),
                                     &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double obs_heading = obstacle.velocity_heading();
    double veh_v =
        std::max(static_cast<double>(ref_conf.veh_pre_min_v),
                 inside_data.init_point.velocity() + ref_conf.veh_pre_add_v);
    double heading_diff = std::abs(normalize_angle(obstacle.velocity_heading() -
                                                   reference_point.heading()));
    if (heading_diff > ref_conf.filter_obs_heading_threshold &&
        M_PI - heading_diff > ref_conf.filter_obs_heading_threshold) {
      LOG_INFO("skip oblique obstacle[{}] heading_diff: {:.4f}", obstacle.id(),
               heading_diff);
      continue;
    }
    double start_len = std::clamp(3.0 * obstacle.speed(),
                                  static_cast<double>(ref_conf.min_len),
                                  static_cast<double>(ref_conf.max_len));
    double end_len = std::clamp(3.0 * obstacle.speed(),
                                static_cast<double>(ref_conf.min_len),
                                static_cast<double>(ref_conf.max_len));
    double width =
        std::clamp(obstacle.speed(), static_cast<double>(ref_conf.min_wid),
                   static_cast<double>(ref_conf.max_wid));
    // Create dynamic obstacle prediction collision areas
    if (heading_diff < M_PI / 2.0) {
      // Same direction
      // Establishing an artificial potential field
      if ((obstacle.min_s() < adc_boundary.end_s() &&
           obstacle.min_s() > adc_boundary.start_s()) ||
          (obstacle.max_s() < adc_boundary.end_s() &&
           obstacle.max_s() > adc_boundary.start_s())) {
        start_len = start_len * std::cos(heading_diff);
        end_len = ref_conf.max_len * std::cos(heading_diff);
      } else if (obstacle.max_s() < adc_boundary.start_s()) {
        start_len = start_len * std::cos(heading_diff);
        end_len = ref_conf.min_len * std::cos(heading_diff);
      } else if (obstacle.speed() > veh_v &&
                 obstacle.min_s() > adc_boundary.end_s()) {
        start_len = ref_conf.min_len * std::cos(heading_diff);
        end_len = end_len * std::cos(heading_diff);
      } else if (obstacle.speed() < veh_v &&
                 obstacle.max_s() > adc_boundary.start_s()) {
        start_len = ref_conf.max_len * std::cos(heading_diff);
        end_len = ref_conf.max_len * std::cos(heading_diff);
      } else {
        start_len = start_len * std::cos(heading_diff);
        end_len = end_len * std::cos(heading_diff);
      }

      if (obstacle.min_s() - std::min(1.0, start_len) > adc_boundary.end_s() &&
          obstacle.speed() > veh_v) {
        LOG_INFO("front high speed obs[{}]: obs speed: {:.4f}, veh_v: {:.4f}",
                 obstacle.id(), obstacle.speed(), veh_v);
        continue;
      }
      if (obstacle.max_s() +
                  std::min(static_cast<double>(ref_conf.min_len), end_len) <
              adc_boundary.start_s() &&
          obstacle.speed() < veh_v) {
        LOG_INFO("back low speed obs[{}]: obs speed: {:.4f}, veh_v: {:.4f}",
                 obstacle.id(), obstacle.speed(), veh_v);
        continue;
      }
      if (obstacle.max_s() + end_len < adc_boundary.start_s() &&
          obstacle.speed() > veh_v &&
          (obstacle.min_l() < adc_boundary.start_l() ||
           obstacle.max_l() < adc_boundary.end_l())) {
        LOG_INFO(
            "back right high speed obs[{}]: obs speed: {:.4f}, veh_v: "
            "{:.4f}, "
            "obs min_l: {:.4f}, max_l: {:.4f}, veh min_l: {:.4f}, max_l: "
            "{:.4f}",
            obstacle.id(), obstacle.speed(), veh_v, obstacle.min_l(),
            obstacle.max_l(), adc_boundary.start_l(), adc_boundary.end_l());
        continue;
      }
      if (heading_diff < ref_conf.use_reference_heading_threshold) {
        if (obs_heading < 0.0) {
          obs_heading += 2 * M_PI;
        }
        double ref_heading = reference_point.heading();
        if (ref_heading < 0.0) {
          ref_heading += 2 * M_PI;
        }
        obs_heading = normalize_angle(
            obs_heading * (1 - ref_conf.reference_heading_ratio) +
            ref_conf.reference_heading_ratio * ref_heading);
      }
    } else {
      // Opposite direction
      if (obstacle.min_s() > adc_boundary.end_s()) {
        start_len = ref_conf.max_len * std::cos(M_PI - heading_diff);
        end_len = ref_conf.max_len * std::cos(M_PI - heading_diff);
      } else {
        start_len = start_len * std::cos(M_PI - heading_diff);
        end_len = ref_conf.min_len * std::cos(M_PI - heading_diff);
      }

      if (obstacle.max_s() + end_len < adc_boundary.start_s()) {
        LOG_INFO("reverse obs[{}]: obs speed: {:.4f}, veh_v: {:.4f}",
                 obstacle.id(), obstacle.speed(), veh_v);
        continue;
      }
      if (M_PI - heading_diff < ref_conf.use_reference_heading_threshold) {
        obs_heading = normalize_angle(reference_point.heading() + M_PI);
      }
    }

    double v_x = obstacle.speed() * std::cos(obs_heading);
    double v_y = obstacle.speed() * std::sin(obs_heading);
    LOG_INFO(
        "obstacle[{}]: speed: {:.4f}, speed x: {:.4f}, speed y: {:.4f}, "
        "obs_heading: {:.4f}, speed heading: {:.4f}, ref heading: {:.4f}, "
        "heading_diff: {:.4f}, set obs_heading: {:.4f}",
        obstacle.id(), obstacle.speed(), v_x, v_y, obstacle.heading(),
        obstacle.velocity_heading(), reference_point.heading(), heading_diff,
        obs_heading);
    LOG_INFO(
        "obstacle[{}]: min s: {:.4f}, max s: {:.4f}, min l: {:.4f}, max l: "
        "{:.4f}",
        obstacle.id(), obstacle.min_s(), obstacle.max_s(), obstacle.min_l(),
        obstacle.max_l());
    LOG_INFO("veh_v: {:.4f}, adc start s: {:.4f}, end s: {:.4f}", veh_v,
             adc_boundary.start_s(), adc_boundary.end_s());
    LOG_INFO("start_len: {:.4f}, end_len: {:.4f}, width: {:.4f}", start_len,
             end_len, width);

    size_t init_index = 0;
    prev_trajectory.query_relative_time_lower_bound_index(0.1, init_index);
    TrajectoryPoint init_point;
    prev_trajectory.trajectory_point_at(init_index, init_point);

    // Start prediction
    bool found = false;
    for (double t = 0.0; t < ref_conf.predict_total_time; t += 0.1) {
      obstacle.set_center(
          Vec2d{decision_data->dynamic_obstacle()[i]->center().x() + v_x * t,
                decision_data->dynamic_obstacle()[i]->center().y() + v_y * t});

      const auto &points =
          decision_data->dynamic_obstacle()[i]->polygon().points();
      points_pre.clear();
      for (auto &pt : points) {
        points_pre.emplace_back(Vec2d{pt.x() + v_x * t, pt.y() + v_y * t});
      }
      obstacle.mutable_polygon()->Init(points_pre);
      obstacle.init_with_reference_line(ref_ptr);

      double delt_s = 0.0;
      if (ref_conf.use_prev_trajectory) {
        size_t current_index = 0;
        prev_trajectory.query_relative_time_lower_bound_index(0.1 + t,
                                                              current_index);
        TrajectoryPoint current_point;
        prev_trajectory.trajectory_point_at(current_index, current_point);
        delt_s = current_point.s() - init_point.s();
      } else {
        delt_s = veh_v * t;
      }

      if (decision_data->dynamic_obstacle()[i]->min_s() >
          adc_boundary.end_s()) {
        if (obstacle.min_s() - start_len < adc_boundary.end_s() + delt_s) {
          found = true;
          LOG_INFO(
              "front obs[{}]: pre time: {:.3f}, obstacle.min_s():{:.3f}, "
              "adc_boundary.end_s(): {:.3f}",
              obstacle.id(), t, obstacle.min_s(),
              adc_boundary.end_s() + delt_s);
          break;
        }
      } else if (decision_data->dynamic_obstacle()[i]->max_s() <
                 adc_boundary.start_s()) {
        double len_s = obstacle.speed() > veh_v ? 0 : end_len;
        if (obstacle.max_s() + len_s > adc_boundary.start_s() + delt_s) {
          found = true;
          LOG_INFO(
              "back obs[{}]: pre time: {:.3f}, obstacle.max_s():{:.3f}, "
              "adc_boundary.start_s(): {:.3f}",
              obstacle.id(), t, obstacle.max_s(),
              adc_boundary.start_s() + delt_s);
          break;
        }
      } else {
        if ((obstacle.max_s() > adc_boundary.start_s() - start_len + delt_s &&
             obstacle.max_s() < adc_boundary.end_s() + end_len + delt_s) ||
            (obstacle.min_s() > adc_boundary.start_s() - start_len + delt_s &&
             obstacle.min_s() < adc_boundary.end_s() + end_len + delt_s)) {
          found = true;
          LOG_INFO(
              "mid obs[{}]: pre time: {:.3f}, obstacle.max_s():{:.3f}, "
              "adc_boundary.start_s(): {:.3f}",
              obstacle.id(), t, obstacle.max_s(),
              adc_boundary.start_s() - start_len + delt_s);
          break;
        }
      }
    }

    if (found && !bounds_info.empty()) {
      double begin_delt_s =
          (obstacle.min_s() - bounds_info.front().lower_point.s()) / 0.1;
      double end_delt_s =
          (obstacle.max_s() - bounds_info.front().lower_point.s()) / 0.1;
      size_t end_index = static_cast<size_t>(std::max(0.0, end_delt_s));
      size_t begin_index = static_cast<size_t>(std::max(0.0, begin_delt_s));
      LOG_INFO(
          "begin: {}, end: {}. bounds_info s: {:.4f}, begin_delt_s: {:.4f}, "
          "end_delt_s: {:.4f}",
          begin_index, end_index, bounds_info.front().lower_point.s(),
          begin_delt_s, end_delt_s);
      if (begin_index <= end_index && end_index <= bounds_info.size()) {
        double max_upper_l = 0, min_upper_l = 100;
        double max_lower_l = -100, min_lower_l = 0;

        // Screen obstacles based on the bounds
        for (size_t i = begin_index; i <= end_index; ++i) {
          max_upper_l = std::max(max_upper_l, bounds_info[i].upper_point.l());
          min_upper_l = std::min(min_upper_l, bounds_info[i].upper_point.l());
          max_lower_l = std::max(max_lower_l, bounds_info[i].lower_point.l());
          min_lower_l = std::min(min_lower_l, bounds_info[i].lower_point.l());
        }
        LOG_INFO(
            "lower_point l: {:.4f}, upper_point l: {:.4f}, obstacle.min_l: "
            "{:.4f}, obstacle.max_l(): {:.4f}",
            max_lower_l, min_upper_l, obstacle.min_l(), obstacle.max_l());
        if ((obstacle.min_l() - max_lower_l > 1.0 &&
             obstacle.min_l() - min_upper_l < 0.0) ||
            (obstacle.max_l() - min_upper_l < -1.0 &&
             obstacle.max_l() - max_lower_l > 0.0)) {
          Box2d box(
              Vec2d{
                  obstacle.center().x() + std::cos(reference_point.heading()) *
                                              (end_len - start_len) / 2.0,
                  obstacle.center().y() + std::sin(reference_point.heading()) *
                                              (end_len - start_len) / 2.0},
              reference_point.heading(),
              obstacle.length() + start_len + end_len,
              obstacle.width() + width);
          auto obs = obstacle;
          std::vector<Vec2d> points_pre;
          box.get_all_corners(&points_pre);
          obs.set_center(box.center());
          obs.set_length(obstacle.length() + start_len + end_len);
          obs.set_width(obstacle.width() + width);
          obs.set_heading(reference_point.heading());
          obs.mutable_polygon()->Init(points_pre);
          obs.init_with_reference_line(ref_ptr);

          observe_obstacles.push_back(obs);
          LOG_INFO("push obs[{}]", obs.id());
        }
        all_obstacles.push_back(obstacle);
      }
    }

    ReferencePoint pre_pt;
    ref_ptr->GetNearestRefPoint(
        Vec2d{obstacle.center().x(), obstacle.center().y()}, &pre_pt);
  }

  std::sort(observe_obstacles.begin(), observe_obstacles.end(),
            [](auto &a, auto &b) { return a.min_s() < b.min_s(); });
  int num = !observe_obstacles.empty() &&
                    observe_obstacles.front().min_s() < adc_boundary.start_s()
                ? 0
                : 1;
  for (size_t i = 0; i < observe_obstacles.size(); ++i) {
    if (num >= ref_conf.observe_obs_num) break;
    if (i < observe_obstacles.size() - 1 &&
        observe_obstacles[i].min_s() < adc_boundary.start_s() &&
        observe_obstacles[i + 1].min_s() < adc_boundary.start_s())
      continue;
    attention_dynamic_obstacles.push_back(observe_obstacles[i]);
    ++num;
  }
  LOG_INFO("attention_dynamic_obstacles size: {}",
           attention_dynamic_obstacles.size());
  if (!attention_dynamic_obstacles.empty()) {
    VisObstacles(attention_dynamic_obstacles, "attention_dynamic_obstacles");
  }
  if (!all_obstacles.empty()) {
    VisObstacles(all_obstacles, "all obstacles");
  }
  return true;
}

}  // namespace

PathObserveRefDecider::PathObserveRefDecider() {
  name_ = "PathObserveRefDecider";
}

PathObserveRefDecider::~PathObserveRefDecider() { Reset(); }

ErrorCode PathObserveRefDecider::Execute(TaskInfo &task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto &observe_ref_l_info = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->path_observe_ref_l_info;
  if (!data_center_->is_auto_driving()) {
    observe_ref_l_info.observe_ref_l = task_info.curr_sl().l();
    return ErrorCode::PLANNING_OK;
  }
  auto &drive_strategy_config =
      common::config::CommonConfig::Instance()->drive_strategy_config();
  bool enable_bias_drive =
      drive_strategy_config.enable_motorway
          ? (task_info.curr_referline_pt().lane_type_is_city_driving()
                 ? drive_strategy_config.motor_way.enable_bias_drive
                 : drive_strategy_config.non_motorway.enable_bias_drive)
          : drive_strategy_config.non_motorway.enable_bias_drive;
  scenario_common::ComputeRefL(task_info, enable_bias_drive);
  auto &init_sl_point =
      task_info.current_frame()->inside_planner_data().init_sl_point;
  SLPoint sl_pt{init_sl_point.s(), 0.0};
  if (drive_strategy_config.non_motorway.enable_dynamic_obs_detour &&
      data_center_->master_info().enable_static_detour() &&
      !task_info.current_frame()->inside_planner_data().is_in_the_park) {
    ComputeAttentionInfos(task_info, sl_pt);
  }
  scenario_common::RefLFilter(task_info);
  LOG_INFO("observe_ref_l: {:.4f}", observe_ref_l_info.observe_ref_l);
  return ErrorCode::PLANNING_OK;

  /// compute observe_ref L of lane borrow scenario
  std::shared_ptr<ObserveRef> lane_borrow_observe =
      std::make_shared<ObserveRefForLaneBorrow>();
  if (!lane_borrow_observe->ComputePathObserveRefLInfo(task_info,
                                                       &observe_ref_l_info)) {
    observe_ref_l_info.observe_ref_l =
        (1 - config::PlanningConfig::Instance()
                 ->planning_research_config()
                 .path_observe_ref_decider_config.lane_borrow
                 .lane_borrow_observe_l_filter_ratio) *
        task_info.last_frame()
            ->outside_planner_data()
            .path_observe_ref_l_info.observe_ref_l;
  }
  if (observe_ref_l_info.type == PathObserveRefLInfo::RefLType::LANEBORROW) {
    LOG_INFO("observe_ref L is LANEBORROW");
    return ErrorCode::PLANNING_OK;
  }

  /// compute attention infos only for single road scenario
  if (!ComputeAttentionInfos(
          task_info, config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .path_observe_ref_decider_config.ranges_partition)) {
    LOG_WARN("compute attention_infos are empty, skip");
    return ErrorCode::PLANNING_OK;
  }

  /// compute observe_ref L of reverse scenario
  std::shared_ptr<ObserveRef> reverse_obs_observe =
      std::make_shared<ObserveRefForReverseObs>();
  reverse_obs_observe->ComputePathObserveRefLInfo(task_info,
                                                  &observe_ref_l_info);
  if (observe_ref_l_info.type == PathObserveRefLInfo::RefLType::REVERSE) {
    LOG_INFO("observe_ref L is REVERSE");
    return ErrorCode::PLANNING_OK;
  }

  /// compute observe_ref L of follow scenario
  std::shared_ptr<ObserveRef> follow_obs_observe =
      std::make_shared<ObserveRefForFollowObs>();
  follow_obs_observe->ComputePathObserveRefLInfo(task_info,
                                                 &observe_ref_l_info);
  if (observe_ref_l_info.type == PathObserveRefLInfo::RefLType::FOLLOW) {
    LOG_INFO("observe_ref L is FOLLOW");
    return ErrorCode::PLANNING_OK;
  }

  LOG_INFO("observe_ref L is NONE");
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
