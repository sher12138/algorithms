#include "clear_zone_law.h"

#include "common/util/time_logger.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

// Clear zone decider
ClearZoneDecider::ClearZoneDecider(double start_s, double end_s)
    : start_route_s_(start_s),
      end_route_s_(end_s),
      check_point_s_(end_s +
                     FLAGS_planning_distance_from_check_point_to_clear_zone) {}

bool ClearZoneDecider::NeedStop(const Obstacle& object,
                                const Boundary& object_boundary) const {
  // TODO(wyc): may be shorter:
  // return object_boundary.start_s() < _check_point_s && object.is_static() &&
  //        object_boundary.start_s() > _start_route_s;
  if (object_boundary.start_s() > check_point_s_) {
    return false;
  }
  if (object.is_static()) {
    if (object_boundary.start_s() > start_route_s_) {
      return true;
    } else {
      return false;
    }
  } else {
    LOG_INFO("object is dynamic, igore. object_id: {}", object.id());
    return false;
  }
}

void ClearZoneDecider::SetStartS(const double& start_s) {
  start_route_s_ = start_s;
}

void ClearZoneDecider::SetEndS(const double& end_s) {
  end_route_s_ = end_s;
  check_point_s_ =
      end_s + FLAGS_planning_distance_from_check_point_to_clear_zone;
}

double ClearZoneDecider::StartS() const { return start_route_s_; }

double ClearZoneDecider::EndS() const { return end_route_s_; }

// Clear zone law
ClearZoneLaw::ClearZoneLaw() : TrafficLaw("ClearZoneLaw") {}

ErrorCode ClearZoneLaw::Apply(TaskInfo& task_info,
                              const InsidePlannerData& inside_data,
                              const OutsidePlannerData& outside_data,
                              const Boundary& adc_boundary,
                              DecisionData* const decision_data,
                              TrafficLawContext* const traffic_law_context) {
  if (!FLAGS_planning_enable_clear_zone_law) {
    LOG_INFO("clearzone is not enabled, skip");
    return ErrorCode::PLANNING_OK;
  }
  ErrorCode ret = ErrorCode::PLANNING_OK;
  if (decision_data == nullptr || traffic_law_context == nullptr) {
    LOG_ERROR("input is null");
    return ret;
  }

  const auto& refer_line = task_info.reference_line();
  if (refer_line == nullptr) {
    LOG_ERROR("refer_line is null");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  ClearZoneDeciderArray clear_zone_deciders;
  FindFrontClearZone(refer_line, adc_boundary, &clear_zone_deciders);
  // crosswalk is also clearzone
  FindFrontCrosswalkClearZone(refer_line, adc_boundary, &clear_zone_deciders);
  LOG_INFO("clear zone deciders size:{}", clear_zone_deciders.size());

  // GeoSpeedConfig geo_speed_config;
  // GeoSpeedBoundaryMapper geo_speed_bd_mp;
  // if (!geo_speed_bd_mp.MapBoundary(inside_data, outside_data,
  //                                  &geo_speed_config)) {
  //   LOG_INFO("MapBoundary failed, ignore front obstacles");
  //   return ret;
  // }

  // bool obs_collide_flag = geo_speed_config.obs_collide_flag_;
  bool obs_collide_flag = false;
  double veh_stop_s = 0.0;
  if (obs_collide_flag) {
    LOG_INFO("obstacle block ahead.");
    // 5.0m is a temperature value, best to vehicle stop distance to obstacle
    // veh_stop_s = geo_speed_config.obs_distance_ - 5.0;
    veh_stop_s = 5.0;
  } else {
    // if trajectory is no collision, set veh_stop_s to destination positon
    // which can also be in clearzone area;
    LOG_INFO("no block obstacle, judge destination in clearzone or not");
    // veh_stop_s = geo_speed_config.stop_line_distance_;
    veh_stop_s = 5.0;
  }

  for (ClearZoneDecider& clear_zone_decider : clear_zone_deciders) {
    if (NeedStop(clear_zone_decider, veh_stop_s)) {
      double virtual_obstacle_route_s =
          std::max(clear_zone_decider.StartS(),
                   adc_boundary.end_s() +
                       FLAGS_planning_min_distance_between_wall_and_front_edge);
      int32_t obstacle_id = 0;
      ret = decision_data->create_virtual_obstacle(
          virtual_obstacle_route_s, VirtualObstacle::CLEAR_ZONE, &obstacle_id);
      if (ret != ErrorCode::PLANNING_OK) {
        LOG_ERROR("Failed to create virtual obstacle.");
        return ret;
      }
      LOG_INFO("Created virtual obstacle id:{} for clearzone.", obstacle_id);
    }
  }
  return ret;
}

void ClearZoneLaw::FindFrontClearZone(
    const ReferenceLinePtr& reference_line, const Boundary& adc_boundary,
    ClearZoneDeciderArray* const clear_zone_deciders) {
  if (clear_zone_deciders == nullptr) {
    LOG_ERROR("clear_zone_deciders == nullptr");
    return;
  }
  double front_edge_s = adc_boundary.end_s();
  for (const auto& clearzone_overlap : reference_line->clearzone_overlaps()) {
    if (clearzone_overlap.start_s < front_edge_s) {
      LOG_INFO("skip clearzone_overlap, id:{}, start_s:{:.2f}, end_s:{:.2f}",
               clearzone_overlap.object_id, clearzone_overlap.start_s,
               clearzone_overlap.end_s);
      continue;
    }
    LOG_INFO(
        "find clear zone. clearzone_overlap.start_s:{:.2f}, "
        "clearzone_overlap.end_s:{:.2f}",
        clearzone_overlap.start_s, clearzone_overlap.end_s);
    ClearZoneDecider clear_zone_decider(clearzone_overlap.start_s,
                                        clearzone_overlap.end_s);
    MergeWithExistClearZoneDecider(clear_zone_decider, clear_zone_deciders);
  }
}

void ClearZoneLaw::FindFrontCrosswalkClearZone(
    const ReferenceLinePtr& reference_line, const Boundary& adc_boundary,
    ClearZoneDeciderArray* const clear_zone_deciders) {
  if (clear_zone_deciders == nullptr) {
    LOG_ERROR("clear_zone_deciders == nullptr");
    return;
  }
  double front_edge_s = adc_boundary.end_s();
  for (const auto& crosswalk_overlap : reference_line->crosswalk_overlaps()) {
    if (crosswalk_overlap.start_s < front_edge_s) {
      LOG_INFO("skip crosswalk, id:{}, start_s:{:.2f}, end_s:{:.2f}",
               crosswalk_overlap.object_id, crosswalk_overlap.start_s,
               crosswalk_overlap.end_s);
      continue;
    }
    LOG_INFO(
        "find crosswalk clear zone. crosswalk.start_s:{:.2f}, "
        "crosswalk.end_s:{:.2f}",
        crosswalk_overlap.start_s, crosswalk_overlap.end_s);
    ClearZoneDecider clear_zone_decider(crosswalk_overlap.start_s,
                                        crosswalk_overlap.end_s);
    MergeWithExistClearZoneDecider(clear_zone_decider, clear_zone_deciders);
  }
}

void ClearZoneLaw::MergeWithExistClearZoneDecider(
    ClearZoneDecider& clear_zone_decider,
    ClearZoneDeciderArray* const clear_zone_deciders) {
  if (clear_zone_deciders == nullptr) return;
  double adc_length = VehicleParam::Instance()->length();

  // TODO(wyc): N^2 time complexity
  ClearZoneDeciderArray::iterator iter = clear_zone_deciders->begin();
  while (iter != clear_zone_deciders->end()) {
    double start_s = clear_zone_decider.StartS();
    double end_s = clear_zone_decider.EndS();

    double keep_clear_start_s = iter->StartS();
    double keep_clear_end_s = iter->EndS();

    bool merged = false;

    // (keep_clear_start_s) |---------------| (keep_clear_end_s)
    //                         o---------o
    if (start_s >= keep_clear_start_s && end_s <= keep_clear_end_s) {
      clear_zone_decider.SetStartS(keep_clear_start_s);
      clear_zone_decider.SetEndS(keep_clear_end_s);
      merged = true;
    }

    // (keep_clear_start_s) |---------------| (keep_clear_end_s)
    //                 o---------o
    //                 o------------------------o
    //                 o--o
    if (start_s < keep_clear_start_s &&
        (keep_clear_start_s - end_s) <= adc_length) {
      clear_zone_decider.SetStartS(start_s);
      if (end_s < keep_clear_end_s) {
        clear_zone_decider.SetEndS(keep_clear_end_s);
      }
      merged = true;
    }

    // (keep_clear_start_s) |---------------| (keep_clear_end_s)
    //                                  o---------o
    //                   o------------------------o
    //                                         o--o
    if (end_s > keep_clear_end_s &&
        (start_s - keep_clear_end_s) <= adc_length) {
      clear_zone_decider.SetEndS(end_s);
      if (start_s > keep_clear_start_s) {
        clear_zone_decider.SetStartS(keep_clear_start_s);
      }
      // TODO(wyc): merged = true;
      if (!merged) {
        merged = true;
      }
    }

    if (merged) {
      iter = clear_zone_deciders->erase(iter);
    } else {
      ++iter;
    }
  }
  clear_zone_deciders->push_back(clear_zone_decider);
}

bool ClearZoneLaw::NeedStop(ClearZoneDecider& clear_zone_decider,
                            const double veh_stop_s) {
  double clear_zone_end_s = clear_zone_decider.EndS();

  // TODO(wyc): return veh_stop_s < clear_zone_end_s;
  if (veh_stop_s < clear_zone_end_s) {
    return true;
  }

  return false;
}

}  // namespace planning
}  // namespace neodrive
