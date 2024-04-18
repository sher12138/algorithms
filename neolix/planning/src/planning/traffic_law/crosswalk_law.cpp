#include "crosswalk_law.h"

#include "common/util/time_logger.h"
#include "common_config/config/common_config.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

namespace {

void AttentionSpace(const TaskInfo& task_info, const Boundary& adc_boundary,
                    const CrosswalkSignal& curr_crosswalk, Boundary* valid_area,
                    Boundary* left_zone, Boundary* right_zone) {
  LOG_INFO(
      "curr_crosswalk id : {}, start_s/end_s: {:3f}, {:.3f}",
      PlanningMap::Instance()->GetHashIdString(curr_crosswalk.crosswalk_id),
      curr_crosswalk.start_route_s, curr_crosswalk.end_route_s);
  valid_area->set_end_l(adc_boundary.end_l() +
                        FLAGS_planning_crosswalk_watch_area_l_expand_buffer);
  valid_area->set_start_l(adc_boundary.start_l() -
                          FLAGS_planning_crosswalk_watch_area_l_expand_buffer);
  valid_area->set_end_s(curr_crosswalk.end_route_s +
                        FLAGS_planning_crosswalk_crosswalk_s_expand_buffer);
  valid_area->set_start_s(curr_crosswalk.end_route_s -
                          FLAGS_planning_crosswalk_crosswalk_s_expand_buffer);
  // road bound
  ReferencePoint tmp_point;
  const auto& reference_line = task_info.reference_line();
  for (double s = valid_area->start_s(); s < valid_area->end_s(); s += 2.0) {
    if (!reference_line->GetNearestRefPoint(s, &tmp_point)) {
      LOG_ERROR("GetNearestRefPoint failed, {:.4f}", s);
      continue;
    }
    valid_area->set_start_l(
        std::max(valid_area->start_l(),
                 -tmp_point.right_road_bound() -
                     FLAGS_planning_crosswalk_roadbound_l_expand_buffer));
    valid_area->set_end_l(
        std::min(valid_area->end_l(),
                 tmp_point.left_road_bound() +
                     FLAGS_planning_crosswalk_roadbound_l_expand_buffer));
  }
  LOG_INFO(
      "crosswalk zone: s_s, e_s, s_l, e_l [{:.4f}, {:.4f}, {:.4f}, {:.4f}]",
      valid_area->start_s(), valid_area->end_s(), valid_area->start_l(),
      valid_area->end_l());

  left_zone->set_start_s(valid_area->start_s());
  left_zone->set_end_s(valid_area->end_s());
  left_zone->set_start_l(std::min(
      adc_boundary.end_l() + FLAGS_planning_crosswalk_adc_l_expand_buffer,
      valid_area->end_l()));
  left_zone->set_end_l(valid_area->end_l());
  LOG_INFO(
      "crosswalk left zone: s_s, e_s, s_l, e_l [{:.4f}, {:.4f}, {:.4f}, "
      "{:.4f}]",
      left_zone->start_s(), left_zone->end_s(), left_zone->start_l(),
      left_zone->end_l());

  right_zone->set_start_s(valid_area->start_s());
  right_zone->set_end_s(valid_area->end_s());
  right_zone->set_start_l(valid_area->start_l());
  right_zone->set_end_l(std::max(
      adc_boundary.start_l() - FLAGS_planning_crosswalk_adc_l_expand_buffer,
      valid_area->start_l()));
  LOG_INFO(
      "crosswalk right zone: s_s, e_s, s_l, e_l [{:.4f}, {:.4f}, {:.4f}, "
      "{:.4f}]",
      right_zone->start_s(), right_zone->end_s(), right_zone->start_l(),
      right_zone->end_l());
}

}  // namespace

// Crosswalk law
CrosswalkLaw::CrosswalkLaw() : TrafficLaw("CrosswalkLaw") {}

ErrorCode CrosswalkLaw::Apply(TaskInfo& task_info,
                              const InsidePlannerData& inside_data,
                              const OutsidePlannerData& outside_data,
                              const Boundary& adc_boundary,
                              DecisionData* const decision_data,
                              TrafficLawContext* const traffic_law_context) {
  ErrorCode ret = ErrorCode::PLANNING_OK;
  if (decision_data == nullptr || traffic_law_context == nullptr) {
    LOG_ERROR("input is null");
    return ret;
  }
  auto& config = config::PlanningConfig::Instance()->plan_config();
  double crosswalk_speed_limit =
      DataCenter::Instance()->drive_strategy_max_speed();
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->traffic_law_speed_limit.crosswalk_speed_limit = crosswalk_speed_limit;

  // find front crosswalk
  std::vector<CrosswalkSignal> front_crosswalks;
  CrosswalkLawContext* crosswalk_context =
      traffic_law_context->MutableCrosswalklawcontext();
  FindFrontCrosswalk(task_info, adc_boundary, front_crosswalks);
  if (front_crosswalks.empty()) {
    crosswalk_context->Reset();
    LOG_INFO("front_crosswalks is empty");
    return ret;
  }
  std::sort(front_crosswalks.begin(), front_crosswalks.end(),
            [](const auto& a, const auto& b) {
              return a.start_route_s < b.start_route_s;
            });
  LOG_INFO("front crosswalk size: {}", front_crosswalks.size());

  // process each crosswalk
  for (std::size_t i = 0; i < front_crosswalks.size(); ++i) {
    CrosswalkDecisionContext context{};
    if (!ComputeDecisionInfo(task_info, adc_boundary, decision_data,
                             front_crosswalks[i], &context)) {
      LOG_WARN("failed to process crosswalk: {}",
               PlanningMap::Instance()->GetHashIdString(
                   front_crosswalks[i].crosswalk_id));
      continue;
    }
    if (!context.need_speed_limit) {
      continue;
    }
    LOG_INFO("lateral_dis: {:.3f}, {:.3f}", context.static_lateral_min_dis,
             context.dynamic_lateral_min_dis);
    LOG_INFO("longitudinal_dis: {:.3f}, {:.3f}",
             context.static_longitudinal_min_dis,
             context.dynamic_longitudinal_min_dis);
    double lateral_dis = std::min(context.static_lateral_min_dis,
                                  context.dynamic_lateral_min_dis);
    double longitudinal_dis = std::min(context.static_longitudinal_min_dis,
                                       context.dynamic_longitudinal_min_dis);
    crosswalk_speed_limit = std::min(
        crosswalk_speed_limit,
        std::min(1.0, longitudinal_dis /
                          config.traffic_law.crosswalk_longitudinal_max_dis) *
            (static_cast<double>(
                config.traffic_law.crosswalk_min_speed +
                (lateral_dis < config.traffic_law.crosswalk_lateral_min_dis
                     ? 0.0
                     : std::pow((lateral_dis -
                                 config.traffic_law.crosswalk_lateral_min_dis),
                                2)))));
    crosswalk_speed_limit =
        std::max(static_cast<double>(config.traffic_law.crosswalk_min_speed),
                 crosswalk_speed_limit);

    LOG_INFO("crosswalk_speed_limit: {:.3f}", crosswalk_speed_limit);
  }
  task_info.current_frame()
      ->mutable_outside_planner_data()
      ->traffic_law_speed_limit.crosswalk_speed_limit = crosswalk_speed_limit;

  return ret;
}

void CrosswalkLaw::FindFrontCrosswalk(
    const TaskInfo& task_info, const Boundary& adc_boundary,
    std::vector<CrosswalkSignal>& crosswalks) {
  crosswalks.clear();
  double look_ahead_dis = std::max(
      FLAGS_planning_crosswalk_look_forward_distance,
      DataCenter::Instance()->vehicle_state_proxy().LinearVelocity() * 8.0);
  for (const auto& cross_overlap :
       task_info.reference_line()->crosswalk_overlaps()) {
    if (cross_overlap.end_s <=
        adc_boundary.end_s() - FLAGS_planning_crosswalk_passing_line_buffer) {
      LOG_INFO("cross_overlap.end_s < adc_boundary.end_s");
      continue;
    }
    if (adc_boundary.end_s() + look_ahead_dis < cross_overlap.start_s) {
      LOG_INFO("adc_boundary.end_s + look_ahead_dis < cross_overlap.start_s");
      continue;
    }
    CrosswalkSignal cross_signal{.start_route_s = cross_overlap.start_s,
                                 .end_route_s = cross_overlap.end_s,
                                 .crosswalk_id = cross_overlap.object_id,
                                 .valid = true};
    crosswalks.emplace_back(cross_signal);
  }
}

bool CrosswalkLaw::ComputeDecisionInfo(const TaskInfo& task_info,
                                       const Boundary& adc_boundary,
                                       DecisionData* const decision_data,
                                       const CrosswalkSignal& curr_crosswalk,
                                       CrosswalkDecisionContext* context) {
  Boundary valid_area{}, left_zone{}, right_zone{};
  AttentionSpace(task_info, adc_boundary, curr_crosswalk, &valid_area,
                 &left_zone, &right_zone);

  context->is_all_obs_static = true;
  for (const auto* obstacle : decision_data->all_obstacle()) {
    if (obstacle == nullptr) continue;
    if (obstacle->is_virtual()) continue;
    if (!(obstacle->type() == Obstacle::ObstacleType::PEDESTRIAN ||
          obstacle->type() == Obstacle::ObstacleType::BICYCLE)) {
      continue;
    }
    const auto& obs_boundary = obstacle->PolygonBoundary();
    if (obs_boundary.start_s() > valid_area.end_s() ||
        obs_boundary.end_s() < task_info.curr_sl().s() ||
        obs_boundary.start_l() > valid_area.end_l() ||
        obs_boundary.end_l() < valid_area.start_l()) {
      continue;
    }
    if (obs_boundary.end_s() < adc_boundary.end_s()) {
      continue;
    }
    // for static
    if (obstacle->is_static()) {
      context->need_speed_limit = true;
      context->static_lateral_min_dis =
          std::min(context->static_lateral_min_dis,
                   std::min(std::abs(obs_boundary.start_l()),
                            std::abs(obs_boundary.end_l())));
      context->static_longitudinal_min_dis =
          std::min(context->static_longitudinal_min_dis,
                   obs_boundary.end_s() - task_info.curr_sl().s());
      continue;
    }
    // for dynamic
    ReferencePoint obs_refer_pt;
    if (!task_info.reference_line()->GetNearestRefPoint(obs_boundary.start_s(),
                                                        &obs_refer_pt)) {
      LOG_ERROR("GetNearestRefPoint failed, {:.4f}", obs_boundary.start_s());
      continue;
    }
    double delta_heading =
        obstacle->velocity_heading() - obs_refer_pt.heading();
    delta_heading = normalize_angle(delta_heading);
    double obs_v_project_to_l = obstacle->speed() * std::sin(delta_heading);
    double obs_v_project_to_s = obstacle->speed() * std::cos(delta_heading);
    LOG_INFO(
        "obs: {}, obs_velocity_heading: {:.2f}, obs_speed: {:.2f}, "
        "speed_proj_to_s: {:.2f}, speed_proj_to_l: {:.2f}, refer heading: "
        "{:.2f}, delta: {:.2f}",
        obstacle->id(), obstacle->velocity_heading(), obstacle->speed(),
        obs_v_project_to_s, obs_v_project_to_l, obs_refer_pt.heading(),
        delta_heading);

    // if dynamic obs runs faster than ego car in s direction, ignore it
    double vel = DataCenter::Instance()->vehicle_state_proxy().LinearVelocity();
    if (obs_v_project_to_s - vel >
        FLAGS_planning_crosswalk_adc_obs_s_speed_safe_delta) {
      LOG_INFO("skip obs {} due to obs is faster on s direction:{:.2f}>{:.2f}.",
               obstacle->id(), obs_v_project_to_s, vel);
      continue;
    }
    // left zone & away ?
    if (!(obs_boundary.start_s() > left_zone.end_s() ||
          obs_boundary.end_s() < left_zone.start_s() ||
          obs_boundary.start_l() > left_zone.end_l() ||
          obs_boundary.end_l() < left_zone.start_l())) {
      // away? if not move to right, ignore it
      // ||||<-----||||||||||||||||||||
      if (obs_v_project_to_l > -FLAGS_planning_adc_stop_velocity_threshold) {
        LOG_INFO("left_zone: skip obs {} due to away from current pos",
                 obstacle->id());
        continue;
      }
      LOG_INFO("left_zone: {}, need to speed limit", obstacle->id());
      context->need_speed_limit = true;
      context->is_all_obs_static = false;
      context->static_lateral_min_dis =
          std::min(context->dynamic_lateral_min_dis,
                   std::min(std::abs(obs_boundary.start_l()),
                            std::abs(obs_boundary.end_l())));
      context->dynamic_longitudinal_min_dis =
          std::min(context->dynamic_longitudinal_min_dis,
                   obs_boundary.start_s() - task_info.curr_sl().s());
      continue;
    }
    // right zone & away ?
    if (!(obs_boundary.start_s() > right_zone.end_s() ||
          obs_boundary.end_s() < right_zone.start_s() ||
          obs_boundary.start_l() > right_zone.end_l() ||
          obs_boundary.end_l() < right_zone.start_l())) {
      // away? if not move to left, ignore it
      // ||||||||||||||||||------>||||
      if (obs_v_project_to_l < FLAGS_planning_adc_stop_velocity_threshold) {
        LOG_INFO("right_zone: skip obs {} due to away from current pos",
                 obstacle->id());
        continue;
      }
      LOG_INFO("right_zone: {}, need to speed limit", obstacle->id());
      context->need_speed_limit = true;
      context->is_all_obs_static = false;
      context->static_lateral_min_dis =
          std::min(context->dynamic_lateral_min_dis,
                   std::min(std::abs(obs_boundary.start_l()),
                            std::abs(obs_boundary.end_l())));
      context->dynamic_longitudinal_min_dis =
          std::min(context->dynamic_longitudinal_min_dis,
                   obs_boundary.start_s() - task_info.curr_sl().s());
      continue;
    }
    // center zone
    LOG_INFO("center_zone: {}, need to speed limit", obstacle->id());
    context->need_speed_limit = true;
    context->is_all_obs_static = false;
    context->static_lateral_min_dis =
        std::min(context->dynamic_lateral_min_dis,
                 std::min(std::abs(obs_boundary.start_l()),
                          std::abs(obs_boundary.end_l())));
    context->dynamic_longitudinal_min_dis =
        std::min(context->dynamic_longitudinal_min_dis,
                 obs_boundary.start_s() - task_info.curr_sl().s());
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive
