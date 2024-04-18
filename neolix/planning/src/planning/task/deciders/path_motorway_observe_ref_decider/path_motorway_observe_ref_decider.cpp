#include "path_motorway_observe_ref_decider.h"

#include <memory>

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/deciders/pilot_state_decider/pilot_state_decider.h"
#include "src/planning/scenario_manager/scenario_common.h"

namespace neodrive {
namespace planning {

static constexpr double kLaneBuffer = 0.2;

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
    sphere->set_radius(0.3);
  };
  Vec2d xy_point{};
  if (!reference_line->GetPointInCartesianFrame({sl_pt.s(), sl_pt.l()},
                                                &xy_point)) {
    LOG_ERROR("failed get closest point.");
    return;
  }
  set_pts(xy_point);
}

void VisCheckWindow(const ReferenceLinePtr &reference_line,
                    std::vector<SLPoint> &corner_pts) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("dy_obs check window");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  auto polygon = event->mutable_polygon()->Add();
  for (auto &sl_pt : corner_pts) {
    Vec2d pt;
    reference_line->GetPointInCartesianFrame(sl_pt, &pt);
    set_pt(polygon->add_point(), pt);
  }
}

void UpdateRefLByDynamicObs(TaskInfo &task_info, char *monitor_str_buffer) {
  const auto &decision_data = task_info.decision_data();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  const auto &adc_speed =
      task_info.current_frame()->inside_planner_data().vel_v;
  const auto &shrink_boundries = task_info.current_frame()
                                     ->outside_planner_data()
                                     .road_obs_path_shrink_boundries;
  double cur_left_bound = task_info.curr_referline_pt().left_lane_bound() -
                          VehicleParam::Instance()->width() * 0.5,
         cur_right_bound = -task_info.curr_referline_pt().right_lane_bound() +
                           VehicleParam::Instance()->width() * 0.5;
  if (!shrink_boundries.empty()) {
    cur_left_bound = shrink_boundries.front().left_bound;
    cur_right_bound = shrink_boundries.front().right_bound;
  }

  // check dynamic window & vis
  auto adc_boundary = task_info.adc_boundary();
  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  double left_limit_l =
             task_info.curr_referline_pt().left_lane_bound() + kLaneBuffer,
         right_limit_l =
             -task_info.curr_referline_pt().right_lane_bound() - kLaneBuffer;
  double back_s = adc_boundary.start_s() - ref_conf.danger_range.back_dis,
         front_s = adc_boundary.end_s() + ref_conf.danger_range.front_dis;
  double left_l = std::min(left_limit_l, adc_boundary.end_l() +
                                             ref_conf.danger_range.left_dis),
         right_l = std::max(right_limit_l, adc_boundary.start_l() -
                                               ref_conf.danger_range.right_dis);

  std::vector<SLPoint> corner_pts{
      SLPoint{front_s, left_l}, SLPoint{back_s, left_l},
      SLPoint{back_s, right_l}, SLPoint{front_s, right_l}};
  VisCheckWindow(task_info.reference_line(), corner_pts);

  // dynamic obs discussion
  std::vector<Obstacle *> left_obs_vec{}, right_obs_vec{};
  for (auto obs_ptr : decision_data->dynamic_obstacle()) {
    if (obs_ptr->max_s() < back_s || obs_ptr->min_s() > front_s ||
        obs_ptr->max_l() < right_l || obs_ptr->min_l() > left_l) {
      LOG_DEBUG("ignore obs[{}] outside dynamic window", obs_ptr->id());
      continue;
    }
    ReferencePoint obs_ref_pt;
    if (!task_info.reference_line()->GetNearestRefPoint(
            obs_ptr->center_sl().s(), &obs_ref_pt)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double heading_diff = std::abs(
        normalize_angle(obs_ptr->velocity_heading() - obs_ref_pt.heading()));
    if (heading_diff >= M_PI / 2.0 &&
        obs_ptr->max_s() < adc_boundary.start_s()) {
      LOG_INFO("ignore reverse obs[{}] behind adc", obs_ptr->id());
      continue;
    }
    double delt_t_thresh = 1.0;
    if (obs_ptr->max_s() < adc_boundary.start_s()) {
      if (obs_ptr->speed() > adc_speed) {
        double delt_s = adc_boundary.start_s() - obs_ptr->max_s();
        double delt_v = obs_ptr->speed() - adc_speed;
        double delt_t = delt_s / delt_v;
        if (delt_t > delt_t_thresh) {
          LOG_INFO(
              "ignore obs[{}] that cannot catching up with adc, delt_t:{:.4f}",
              obs_ptr->id(), delt_t);
          continue;
        }
      } else {
        LOG_INFO("ignore low speed obs[{}] behind adc", obs_ptr->id());
        continue;
      }
    }
    if (obs_ptr->min_s() > adc_boundary.end_s()) {
      if (obs_ptr->speed() < adc_speed) {
        double delt_s = obs_ptr->min_s() - adc_boundary.end_s();
        double delt_v = adc_speed - obs_ptr->speed();
        double delt_t = delt_s / delt_v;
        if (delt_t > delt_t_thresh) {
          LOG_INFO(
              "ignore obs[{}] that adc cannot catching up with, delt_t:{:.4f}",
              obs_ptr->id(), delt_t);
          continue;
        }
      } else {
        LOG_INFO("ignore high speed obs[{}] in front of adc", obs_ptr->id());
        continue;
      }
    }

    if (obs_ptr->max_l() > task_info.curr_sl().l() &&
        obs_ptr->min_l() < task_info.curr_sl().l()) {
      left_obs_vec.emplace_back(obs_ptr);
      right_obs_vec.emplace_back(obs_ptr);
      LOG_INFO("dynamic obs[{}] in both side", obs_ptr->id());
      break;
    } else {
      if (obs_ptr->max_l() > right_l &&
          obs_ptr->max_l() < task_info.curr_sl().l()) {
        right_obs_vec.emplace_back(obs_ptr);
        LOG_INFO("dynamic obs[{}] only in right side", obs_ptr->id());
      }
      if (obs_ptr->min_l() < left_l &&
          obs_ptr->min_l() > task_info.curr_sl().l()) {
        left_obs_vec.emplace_back(obs_ptr);
        LOG_INFO("dynamic obs[{}] only in left side", obs_ptr->id());
      }
    }
  }

  // dynamic obs decision
  std::pair<std::string, double> final_decision;
  std::pair<std::string, double> keep_decision = {"KEEP",
                                                  task_info.curr_sl().l()},
                                 center_decision = {"CENTER", observe_ref_l},
                                 avoid_decision = {"AVOID", 0.0};
  if (!left_obs_vec.empty() && !right_obs_vec.empty()) {
    final_decision = keep_decision;
  } else if (!left_obs_vec.empty()) {
    Obstacle *most_right_obs_ptr = *std::min_element(
        left_obs_vec.begin(), left_obs_vec.end(),
        [](const auto a, const auto b) { return a->min_l() < b->min_l(); });
    double obs_nudge_l =
        most_right_obs_ptr->min_l() - 2 * VehicleParam::Instance()->width();
    avoid_decision.second = std::max(obs_nudge_l, cur_right_bound);
    LOG_INFO(
        "most_right_obs:{}, obs_min_l:{}, obs_nudge_l:{:.4f}, "
        "right_limit:{:.4f}, avoid_decision_l:{:.4f}",
        most_right_obs_ptr->id(), most_right_obs_ptr->min_l(), obs_nudge_l,
        cur_right_bound, avoid_decision.second);

    auto decisions = {keep_decision, center_decision, avoid_decision};
    final_decision = *std::min_element(
        decisions.begin(), decisions.end(),
        [](const auto &a, const auto &b) { return a.second < b.second; });
  } else if (!right_obs_vec.empty()) {
    Obstacle *most_left_obs_ptr = *std::max_element(
        right_obs_vec.begin(), right_obs_vec.end(),
        [](const auto a, const auto b) { return a->max_l() < b->max_l(); });
    double obs_nudge_l =
        most_left_obs_ptr->max_l() + 2 * VehicleParam::Instance()->width();
    avoid_decision.second = std::min(obs_nudge_l, cur_left_bound);
    LOG_INFO(
        "most_left_obs:{}, obs_max_l:{}, obs_nudge_l:{:.4f}, "
        "left_limit:{:.4f}, avoid_decision_l:{:.4f}",
        most_left_obs_ptr->id(), most_left_obs_ptr->max_l(), obs_nudge_l,
        cur_left_bound, avoid_decision.second);

    auto decisions = {keep_decision, center_decision, avoid_decision};
    final_decision = *std::max_element(
        decisions.begin(), decisions.end(),
        [](const auto &a, const auto &b) { return a.second < b.second; });
  } else {
    final_decision = center_decision;
  }

  // update
  observe_ref_l = final_decision.second;

  // log
  char dynamic_obs_check_str[256];
  sprintf(dynamic_obs_check_str,
          "[exist_dynamic_obs: %d %d][KEEP/CENTER/AVOID ref_l: "
          "%.3f, %.3f, %.3f][dynamic obs decision: %s, %.3f]",
          !left_obs_vec.empty(), !right_obs_vec.empty(), keep_decision.second,
          center_decision.second, avoid_decision.second,
          final_decision.first.c_str(), final_decision.second);
  std::strcat(monitor_str_buffer, dynamic_obs_check_str);
}

PathMotorwayObserveRefDecider::PathMotorwayObserveRefDecider() {
  name_ = "PathMotorwayObserveRefDecider";
}

PathMotorwayObserveRefDecider::~PathMotorwayObserveRefDecider() { Reset(); }

ErrorCode PathMotorwayObserveRefDecider::Execute(TaskInfo &task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  char monitor_str_buffer[256] = "[MotorwayCruise]";

  auto &observe_ref_l_info = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->path_observe_ref_l_info;

  //  non auto drive
  if (data_center_->vehicle_state_proxy().chassis().driving_mode() !=
      neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
    observe_ref_l_info.observe_ref_l =
        task_info.current_frame()->inside_planner_data().init_sl_point.l();
    return ErrorCode::PLANNING_OK;
  }

  // main process
  if (data_center_->master_info().enable_static_detour() &&
      !task_info.current_frame()->inside_planner_data().is_in_the_park) {
    UpdateRefLByDynamicObs(task_info, monitor_str_buffer);
  }

  // filter
  scenario_common::RefLFilter(task_info);

  // log & vis
  char filter_ref_l_str[256];
  sprintf(filter_ref_l_str, "[filter ref_l: %.3f]",
          observe_ref_l_info.observe_ref_l);
  std::strcat(monitor_str_buffer, filter_ref_l_str);
  DataCenter::Instance()->SetMonitorString(
      monitor_str_buffer, MonitorItemSource::MOTORWAY_CRUISE_STATE);
  LOG_INFO(monitor_str_buffer);
  VisRefL(task_info.reference_line(),
          SLPoint{task_info.curr_sl().s(), observe_ref_l_info.observe_ref_l},
          "ref_l motorway");

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
