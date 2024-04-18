#include "speed_cipv_on_path_decider.h"

#include <unordered_map>

#include "reference_line/reference_line_util.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

SpeedCipvOnPathDecider::SpeedCipvOnPathDecider() {
  name_ = "SpeedCipvOnPathDecider";
}

void SpeedCipvOnPathDecider::GetCipvCheckArea(TaskInfo& task_info) {
  const auto& speed_worm_follow_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_worm_follow;
  auto outside_planner_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  for (std::size_t i = 0; i < task_info.current_frame()
                                  ->outside_planner_data()
                                  .path_data->frenet_path()
                                  .num_of_points();
       i++) {
    const auto& veh_pt = task_info.current_frame()
                             ->outside_planner_data()
                             .path_data->path()
                             .path_points()[i];
    Polygon2d veh_polygon = VehicleParam::Instance()->get_adc_polygon(
        {veh_pt.x(), veh_pt.y()}, veh_pt.theta(), 0, 0.1, 0.1);
    worm_follow_path_check_area_polygen_.push_back(veh_polygon);
  }
}

SpeedCipvOnPathDecider::~SpeedCipvOnPathDecider() {}

ErrorCode SpeedCipvOnPathDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    LOG_INFO("path successed tasks is 0, skip rest tasks.");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  LOG_INFO(">>>> Check Cipv on Path decider work normal");
  if (!Init(task_info)) {
    LOG_ERROR("Check Cipv on Path deciderr Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Check Cipv on Path decider Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void SpeedCipvOnPathDecider::Reset() {
  dynamic_cipv_.Reset();
  static_cipv_.Reset();
  worm_follow_path_check_area_polygen_.clear();
};

bool SpeedCipvOnPathDecider::Init(TaskInfo& task_info) {
  LOG_INFO(">>>> step into Check CIPV Decider");
  // adc_boundary_origin_ = task_info.adc_boundary_origin();
  adc_current_s_ =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  // front_edge_to_center=1.54
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  Reset();
  GetCipvCheckArea(task_info);
  return true;
}

bool SpeedCipvOnPathDecider::Process(TaskInfo& task_info) {
  if (!ProcessCipvCheck(task_info)) {
    LOG_INFO("ProcessCipvCheck error.");
    return false;
  }

  return true;
}

bool SpeedCipvOnPathDecider::ProcessCipvCheck(TaskInfo& task_info) {
  CheckDynamicObsInCheckArea(task_info);
  CheckStaticObsInCheckArea(task_info);
  JudgeCipvIsConcerned(task_info);
  return true;
}
void SpeedCipvOnPathDecider::SaveTaskResults(TaskInfo& task_info) {}
// filter dynamic obs
bool SpeedCipvOnPathDecider::CheckDynamicObsInCheckArea(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();

  LOG_INFO("check all dynamic obstacle num: [{}].", dynamic_obs_vec.size());
  auto& path_data = task_info.current_frame()->outside_planner_data().path_data;
  if (path_data == nullptr) {
    LOG_ERROR("path_data == nullptr.");
    return false;
  }

  std::vector<double> record_dynamic_obs_relative_dis;
  record_dynamic_obs_relative_dis.clear();
  std::unordered_map<double, size_t> worm_area_dynamic;
  worm_area_dynamic.clear();

  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (nullptr == dynamic_obs_vec[i]) {
      LOG_DEBUG("dynamic obstacle is nullptr .");
      continue;
    }
    bool has_overlap_dynamic_use_polgen{false};
    for (const auto& polgen : worm_follow_path_check_area_polygen_) {
      if (polgen.has_overlap(dynamic_obs_vec[i]->polygon())) {
        has_overlap_dynamic_use_polgen = true;
      }
    }
    if (!has_overlap_dynamic_use_polgen) {
      continue;
    }
    record_dynamic_obs_relative_dis.push_back(
        std::min(dynamic_obs_vec[i]->PolygonBoundary().start_s(),
                 dynamic_obs_vec[i]->PolygonBoundary().end_s()) -
        adc_front_edge_s_);
    worm_area_dynamic.insert(
        std::make_pair(std::min(dynamic_obs_vec[i]->PolygonBoundary().start_s(),
                                dynamic_obs_vec[i]->PolygonBoundary().end_s()) -
                           adc_front_edge_s_,
                       i));
  }
  if (record_dynamic_obs_relative_dis.size() == 0) {
    return false;
  }
  std::sort(record_dynamic_obs_relative_dis.begin(),
            record_dynamic_obs_relative_dis.end());
  std::size_t dynamic_need_worm_index{100};
  auto it = worm_area_dynamic.find(record_dynamic_obs_relative_dis[0]);
  if (it != worm_area_dynamic.end()) {
    dynamic_need_worm_index = it->second;
  }
  dynamic_cipv_.cipv_obsulte_distance =
      dynamic_obs_vec[dynamic_need_worm_index]->PolygonBoundary().start_s();
  dynamic_cipv_.if_has_dynamic_obs = true;
  dynamic_cipv_.cipv_speed = dynamic_obs_vec[dynamic_need_worm_index]->speed();
  dynamic_cipv_.heading_diff_dynamic = normalize_angle(
      dynamic_obs_vec[dynamic_need_worm_index]->velocity_heading() -
      inside_data.vel_heading);
  dynamic_cipv_.cipv_id = dynamic_obs_vec[dynamic_need_worm_index]->id();
  return true;
}

bool SpeedCipvOnPathDecider::CheckStaticObsInCheckArea(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& static_obs_vec = task_info.current_frame()
                                   ->planning_data()
                                   .decision_data()
                                   .static_obstacle();

  LOG_INFO("static obstacle num: [{}].", static_obs_vec.size());

  std::vector<double> record_static_obs_relative_dis;
  record_static_obs_relative_dis.clear();

  std::unordered_map<double, size_t> worm_area_static;
  worm_area_static.clear();
  for (std::size_t i = 0; i < static_obs_vec.size(); ++i) {
    if (nullptr == static_obs_vec[i]) {
      LOG_DEBUG("static obstacle is nullptr.");
      continue;
    }
    bool has_overlap_static_use_polgen{false};
    for (const auto& polgen : worm_follow_path_check_area_polygen_) {
      if (polgen.has_overlap(static_obs_vec[i]->polygon())) {
        has_overlap_static_use_polgen = true;
      }
    }
    if (!has_overlap_static_use_polgen) {
      LOG_DEBUG("static obstacle [{}] is not in worm check area.",
                static_obs_vec[i]->id());
      continue;
    }

    record_static_obs_relative_dis.push_back(
        std::min(static_obs_vec[i]->PolygonBoundary().start_s(),
                 static_obs_vec[i]->PolygonBoundary().end_s()) -
        adc_front_edge_s_);
    worm_area_static.emplace(
        std::min(static_obs_vec[i]->PolygonBoundary().start_s(),
                 static_obs_vec[i]->PolygonBoundary().end_s()) -
            adc_front_edge_s_,
        i);
  }
  if (record_static_obs_relative_dis.size() == 0) {
    return false;
  }
  std::sort(record_static_obs_relative_dis.begin(),
            record_static_obs_relative_dis.end());
  std::size_t static_need_worm_index{100};

  bool has_real_static_cipv{false};
  for (int i = 0; i < record_static_obs_relative_dis.size(); i++) {
    auto it = worm_area_static.find(record_static_obs_relative_dis[i]);
    if (it != worm_area_static.end()) {
      static_need_worm_index = it->second;
      if (!static_obs_vec[static_need_worm_index]->is_virtual()) {
        has_real_static_cipv = true;
        break;
      }
    }
  }

  static_cipv_.cipv_obsulte_distance =
      static_obs_vec[static_need_worm_index]->PolygonBoundary().start_s();
  static_cipv_.cipv_speed = static_obs_vec[static_need_worm_index]->speed();
  static_cipv_.if_has_static_obs = has_real_static_cipv;
  static_cipv_.heading_diff_static = normalize_angle(
      static_obs_vec[static_need_worm_index]->velocity_heading() -
      inside_data.vel_heading);
  static_cipv_.if_is_virtual =
      static_obs_vec[static_need_worm_index]->is_virtual();
  static_cipv_.cipv_id = static_obs_vec[static_need_worm_index]->id();
  return true;
}
void SpeedCipvOnPathDecider::JudgeCipvIsConcerned(TaskInfo& task_info) {
  double dynamic_cipv_dis{};
  double static_cipv_dis{};
  auto& cipv_decision_data = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->speed_obstacle_context.cipv_decision;
  cipv_decision_data.Reset();
  dynamic_cipv_.if_has_dynamic_obs
      ? dynamic_cipv_dis = dynamic_cipv_.cipv_obsulte_distance
      : dynamic_cipv_dis = 10000;
  static_cipv_.if_has_static_obs
      ? static_cipv_dis = static_cipv_.cipv_obsulte_distance
      : static_cipv_dis = 10000;

  if (dynamic_cipv_dis <= static_cipv_dis && dynamic_cipv_.if_has_dynamic_obs) {
    cipv_decision_data.has_cipv = true;
    cipv_decision_data.cipv_id = dynamic_cipv_.cipv_id;
    LOG_INFO("dynamic obs is ego's cipv , id is : {}", dynamic_cipv_.cipv_id);
  }
  if (static_cipv_dis < dynamic_cipv_dis && static_cipv_.if_has_static_obs) {
    cipv_decision_data.has_cipv = true;
    cipv_decision_data.cipv_id = static_cipv_.cipv_id;
    LOG_INFO("static obs is ego's cipv , id is : {}", static_cipv_.cipv_id);
  }
  if (dynamic_cipv_.if_has_dynamic_obs == false &&
      static_cipv_.if_has_static_obs == false) {
    LOG_INFO("There has no cipv");
  }
}
// CipvDecision SpeedCipvOnPathDecider::ReturnResult() { return cipv_decision_;
// }

}  // namespace planning
}  // namespace neodrive
