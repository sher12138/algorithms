#include "traffic_law_manager.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

TrafficLawManager::TrafficLawManager() {}

int TrafficLawManager::ApplyLaw(TaskInfo* const task_info,
                                const InsidePlannerData& inside_planner_data,
                                const OutsidePlannerData& outside_data) {
  LOG_INFO("apply traffic law");

  Boundary adc_boundary;
  CalcAdcBoundary(*task_info, &adc_boundary);

  TrafficLightLaw traffic_light_law;
  CrosswalkLaw crosswalk_law;
  ClearZoneLaw clear_zone_law;
  StopSignLaw stop_sign_law;
  YJunctionLaw y_junction_law;
  RestrictedAreaLaw restricted_area_law;

  std::vector<TrafficLaw*> law_recorder;

  // apply law
  if (FLAGS_planning_enable_traffic_light_law) {
    law_recorder.emplace_back(&traffic_light_law);
  }
  if (config::PlanningConfig::Instance()
          ->planning_research_config()
          .traffic_light_y_law_config.enable_traffic_light_y) {
    law_recorder.emplace_back(&y_junction_law);
  }
  if (config::PlanningConfig::Instance()
          ->planning_research_config()
          .restricted_area_law_config.enable_restricted_area) {
    law_recorder.emplace_back(&restricted_area_law);
  }
  if (FLAGS_planning_enable_crosswalk_law) {
    law_recorder.emplace_back(&crosswalk_law);
  }

  if (FLAGS_planning_enable_clear_zone_law) {
    law_recorder.emplace_back(&clear_zone_law);
  }

  if (FLAGS_planning_enable_stop_sign_law) {
    law_recorder.emplace_back(&stop_sign_law);
  }

  for (std::size_t i = 0; i < law_recorder.size(); ++i) {
    law_recorder[i]->Apply(
        *task_info, inside_planner_data, outside_data, adc_boundary,
        task_info->current_frame()
            ->mutable_planning_data()
            ->mutable_decision_data(),
        task_info->current_frame()->mutable_traffic_law_context());
  }

  return 0;
}

int TrafficLawManager::CalcAdcBoundary(const TaskInfo& task_info,
                                       Boundary* const adc_boundary) const {
  const Environment& environment = DataCenter::Instance()->environment();
  const auto& vehicle_state = DataCenter::Instance()->vehicle_state_proxy();
  double adc_heading = vehicle_state.Heading();
  double pose_x = vehicle_state.X();
  double pose_y = vehicle_state.Y();

  Vec2d position(pose_x, pose_y);
  Vec2d back_right_pt(-VehicleParam::Instance()->back_edge_to_center(),
                      -VehicleParam::Instance()->right_edge_to_center());
  Vec2d front_right_pt(VehicleParam::Instance()->front_edge_to_center(),
                       -VehicleParam::Instance()->right_edge_to_center());
  Vec2d front_left_pt(VehicleParam::Instance()->front_edge_to_center(),
                      VehicleParam::Instance()->left_edge_to_center());
  Vec2d back_left_pt(-VehicleParam::Instance()->back_edge_to_center(),
                     VehicleParam::Instance()->left_edge_to_center());
  std::vector<Vec2d> points;
  points.emplace_back(position + Rotate(back_right_pt, adc_heading));
  points.emplace_back(position + Rotate(front_right_pt, adc_heading));
  points.emplace_back(position + Rotate(front_left_pt, adc_heading));
  points.emplace_back(position + Rotate(back_left_pt, adc_heading));
  Polygon2d adc_polygon(points);
  double x_mean = 0.0;
  double y_mean = 0.0;
  for (const auto& pt : adc_polygon.points()) {
    x_mean += pt.x();
    y_mean += pt.y();
  }
  x_mean /= adc_polygon.points().size();
  y_mean /= adc_polygon.points().size();

  Vec2d center(x_mean, y_mean);
  double adc_length = VehicleParam::Instance()->length();
  double adc_width = VehicleParam::Instance()->width();
  Box2d adc_bounding_box(center, adc_heading, adc_length, adc_width);

  Utility::CalcBoundary(task_info.reference_line(), adc_polygon,
                        adc_bounding_box, adc_boundary);

  LOG_INFO(
      "adc_boundary: start_s:{:.3f}, end_s:{:.3f}, start_l:{:.3f}, "
      "end_l:{:.3f}",
      adc_boundary->start_s(), adc_boundary->end_s(), adc_boundary->start_l(),
      adc_boundary->end_l());
  return 0;
}

Vec2d TrafficLawManager::Rotate(const Vec2d& vec, const double theta) const {
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);
  return Vec2d(vec.x() * cos_theta - vec.y() * sin_theta,
               vec.x() * sin_theta + vec.y() * cos_theta);
}

}  // namespace planning
}  // namespace neodrive
