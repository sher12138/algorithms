#include "motorway_speed_lanechange_decider.h"

#include <unordered_map>

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {

namespace {
void VisLaneChangeCheckBoundary(
    const ReferenceLinePtr ref_line,
    const LaneChangeCheckAreaBoundary& check_lane_change_check_area,
    const std::string event_name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(event_name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (const auto& vec : {check_lane_change_check_area.ll_check_boundarys,
                          check_lane_change_check_area.lr_check_boundarys,
                          check_lane_change_check_area.rl_check_boundarys,
                          check_lane_change_check_area.rr_check_boundarys}) {
    for (const auto& bdry : vec) {
      Vec2d pt{};
      auto polygon = event->mutable_polygon()->Add();
      ref_line->GetPointInCartesianFrame({bdry.start_s(), bdry.end_l()}, &pt);
      set_pt(polygon->add_point(), pt);
      ref_line->GetPointInCartesianFrame({bdry.start_s(), bdry.start_l()}, &pt);
      set_pt(polygon->add_point(), pt);
      ref_line->GetPointInCartesianFrame({bdry.end_s(), bdry.start_l()}, &pt);
      set_pt(polygon->add_point(), pt);
      ref_line->GetPointInCartesianFrame({bdry.end_s(), bdry.end_l()}, &pt);
      set_pt(polygon->add_point(), pt);
    }
  }
}

}  // namespace

MotorwaySpeedLaneChangeDecider::MotorwaySpeedLaneChangeDecider() {
  name_ = "MotorwaySpeedLaneChangeDecider";
}
/**
 * @brief Divide obstacles into the interior of the Nine Palace Grid as bellows
                  0 | 1 | 2
                  3 | * | 4
                  5 | 6 | 7

 */

bool MotorwaySpeedLaneChangeDecider::ObsNinePalaceGridDivision(
    TaskInfo& task_info) {
  check_areas_.Reset();
  GetLaneBounds(task_info, left_lc_lat_dis_, right_lc_lat_dis_, check_areas_);

  LOG_INFO("check left LC lat dis : {}", left_lc_lat_dis_);

  LOG_INFO("check right LC lat dis : {}", right_lc_lat_dis_);

  VisLaneChangeCheckBoundary(task_info.reference_line(), check_areas_,
                             "LaneChangeCheckArea");

  ObstacleClassification(task_info, check_areas_);

  return true;
}

void MotorwaySpeedLaneChangeDecider::GenerateCherkAreaBoundary(
    TaskInfo& task_info, const LaneChangeBoundPara& lane_change_bound_para,
    const LaneChangeBoundPara& back_lane_change_bound_para,
    LaneChangeCheckAreaBoundary& check_area, const double& expend_index) {
  // check_area.Reset();
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;

  auto adc_boundary = task_info.adc_boundary_origin();
  check_area.ll_check_boundarys.emplace_back(
      Boundary(adc_boundary.end_s() + expend_index * 3,
               adc_boundary.end_s() + (expend_index + 1) * 3,
               lane_change_bound_para.left_bound_start,
               lane_change_bound_para.left_bound_end));

  check_area.lr_check_boundarys.emplace_back(
      Boundary(adc_boundary.end_s() - (expend_index + 1) * 3,
               adc_boundary.end_s() - expend_index * 3,
               back_lane_change_bound_para.left_bound_start,
               back_lane_change_bound_para.left_bound_end));

  check_area.rl_check_boundarys.emplace_back(
      Boundary(adc_boundary.end_s() + expend_index * 3,
               adc_boundary.end_s() + (expend_index + 1) * 3,
               -lane_change_bound_para.right_bound_end,
               -lane_change_bound_para.right_bound_start));
  check_area.rr_check_boundarys.emplace_back(
      Boundary(adc_boundary.end_s() - (expend_index + 1) * 3,
               adc_boundary.end_s() - expend_index * 3,
               -back_lane_change_bound_para.right_bound_end,
               -back_lane_change_bound_para.right_bound_start));
}

void MotorwaySpeedLaneChangeDecider::GetLaneBounds(
    TaskInfo& task_info, double& left_lanechange_dis,
    double& right_lanechage_dis, LaneChangeCheckAreaBoundary& check_area) {
  auto CalFrontCheckArea = [&](double expend_index) {
    Vec2d utm_pt{};
    task_info.reference_line_raw()->GetPointInCartesianFrame(
        {task_info.curr_sl().s() + expend_index * 3, 0.0}, &utm_pt);
    uint64_t left_lane_id, right_lane_id;

    double cal_left_boud{}, cal_right_boud{}, left_dis{}, right_dis{};
    ReferencePoint ref_pt;
    task_info.reference_line()->GetNearestRefPoint(
        task_info.curr_sl().s() + expend_index * 3.0, &ref_pt);
    double lane_left_bound = ref_pt.left_lane_bound();
    double lane_right_bound = ref_pt.right_lane_bound();

    if (PlanningMap::Instance()->GetNearestLeftLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, left_lane_id)) {
      double lane_s, lane_l;
      if (!PlanningMap::Instance()->GetSLWithLane(left_lane_id, utm_pt.x(),
                                                  utm_pt.y(), lane_s, lane_l)) {
        lane_s = task_info.curr_sl().s();
      }
      std::pair<double, double> left_lane_width{0.0, 0.0};
      left_lane_width = PlanningMap::Instance()->GetLaneDistanceWidth(
          left_lane_id, lane_s + expend_index * 3);

      cal_left_boud =
          lane_left_bound + left_lane_width.first + left_lane_width.second - 1;
      left_dis = (left_lane_width.first + left_lane_width.second) / 2 +
                 lane_left_bound - task_info.curr_sl().l();

      // LOG_INFO(
      //     "ego left change dis a : {} , b : {} , leftbound : {} ,ego l : {}",
      //     left_lane_width.first, left_lane_width.second, lane_left_bound,
      //     task_info.curr_sl().l());

    } else {
      cal_left_boud = lane_left_bound + 3.5 - 1;
      left_dis = 3.5;
    }

    if (PlanningMap::Instance()->GetNearestRightLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, right_lane_id)) {
      double lane_s, lane_l;
      if (!PlanningMap::Instance()->GetSLWithLane(right_lane_id, utm_pt.x(),
                                                  utm_pt.y(), lane_s, lane_l)) {
        lane_s = task_info.curr_sl().s();
      }
      std::pair<double, double> right_lane_width{0.0, 0.0};
      right_lane_width = PlanningMap::Instance()->GetLaneDistanceWidth(
          right_lane_id, lane_s + expend_index * 3);

      cal_right_boud = lane_right_bound + right_lane_width.first +
                       right_lane_width.second - 1;

      right_dis = lane_right_bound + task_info.curr_sl().l() +
                  (right_lane_width.first + right_lane_width.second) / 2;
      // LOG_INFO(
      //     "ego left change dis a : {} , b : {} , leftbound : {} ,ego l : {}",
      //     right_lane_width.first, right_lane_width.second, lane_right_bound,
      //     task_info.curr_sl().l());

    } else {
      cal_right_boud = lane_right_bound + 3.5 - 1;
      right_dis = 3.5;
    }
    LaneChangeBoundPara para{cal_left_boud,    cal_right_boud, lane_left_bound,
                             lane_right_bound, left_dis,       right_dis};
    return para;
  };

  auto CalBackCheckArea = [&](double expend_index) {
    Vec2d utm_pt{};
    task_info.reference_line_raw()->GetPointInCartesianFrame(
        {task_info.curr_sl().s() - expend_index * 3, 0.0}, &utm_pt);
    uint64_t left_lane_id, right_lane_id;

    double cal_left_boud_back{}, cal_right_boud_back{}, left_dis_non{},
        right_dis_non{};
    ReferencePoint ref_pt;
    task_info.reference_line()->GetNearestRefPoint(
        task_info.curr_sl().s() - expend_index * 3.0, &ref_pt);
    double lane_left_bound_back = ref_pt.left_lane_bound();
    double lane_right_bound_back = ref_pt.right_lane_bound();

    if (PlanningMap::Instance()->GetNearestLeftLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, left_lane_id)) {
      double lane_s, lane_l;
      if (!PlanningMap::Instance()->GetSLWithLane(left_lane_id, utm_pt.x(),
                                                  utm_pt.y(), lane_s, lane_l)) {
        lane_s = task_info.curr_sl().s();
      }
      std::pair<double, double> left_lane_width{0.0, 0.0};
      left_lane_width = PlanningMap::Instance()->GetLaneDistanceWidth(
          left_lane_id, lane_s - expend_index * 3);

      cal_left_boud_back = lane_left_bound_back + left_lane_width.first +
                           left_lane_width.second - 1;
      left_dis_non = (left_lane_width.first + left_lane_width.second) / 2 +
                     lane_left_bound_back - task_info.curr_sl().l();

    } else {
      cal_left_boud_back = lane_left_bound_back + 3.5 - 1;
      left_dis_non = 3.5;
    }
    if (PlanningMap::Instance()->GetNearestRightLane(
            task_info.curr_referline_pt().hd_map_lane_id(),
            {utm_pt.x(), utm_pt.y()}, right_lane_id)) {
      double lane_s, lane_l;
      if (!PlanningMap::Instance()->GetSLWithLane(right_lane_id, utm_pt.x(),
                                                  utm_pt.y(), lane_s, lane_l)) {
        lane_s = task_info.curr_sl().s();
      }
      std::pair<double, double> right_lane_width{0.0, 0.0};
      right_lane_width = PlanningMap::Instance()->GetLaneDistanceWidth(
          right_lane_id, lane_s - expend_index * 3);

      cal_right_boud_back = lane_right_bound_back + right_lane_width.first +
                            right_lane_width.second - 1;

      right_dis_non = lane_right_bound_back + task_info.curr_sl().l() +
                      (right_lane_width.first + right_lane_width.second) / 2;

    } else {
      cal_right_boud_back = lane_right_bound_back + 3.5 - 1;
      right_dis_non = 3.5;
    }
    LaneChangeBoundPara para{cal_left_boud_back,   cal_right_boud_back,
                             lane_left_bound_back, lane_right_bound_back,
                             left_dis_non,         right_dis_non};
    return para;
  };

  size_t expend_time = 15;
  for (size_t expend_index = 0; expend_index < expend_time; ++expend_index) {
    LaneChangeBoundPara lane_change_lat_para_front =
        CalFrontCheckArea(expend_index);
    LaneChangeBoundPara lane_change_lat_para_back =
        CalBackCheckArea(expend_index);
    if (expend_index == 0) {
      left_lanechange_dis = lane_change_lat_para_front.left_lane_change_dis;
      right_lanechage_dis = lane_change_lat_para_front.right_lane_change_dis;
      GenerateCherkAreaBoundary(task_info, lane_change_lat_para_front,
                                lane_change_lat_para_back, check_area,
                                expend_index);
      LOG_INFO("Left Lane change dis : {}.", left_lanechange_dis);
      LOG_INFO("Right Lane change dis : {}.", right_lanechage_dis);
    } else {
      GenerateCherkAreaBoundary(task_info, lane_change_lat_para_front,
                                lane_change_lat_para_back, check_area,
                                expend_index);
    }
  }
}

void MotorwaySpeedLaneChangeDecider::ObstacleClassification(
    TaskInfo& task_info, const LaneChangeCheckAreaBoundary& check_area) {
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();

  const auto& dynamic_obs_vec = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .dynamic_obstacle();
  LOG_INFO("check all dynamic obstacle num: [{}], donot care static obs.",
           dynamic_obs_vec.size());
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data->path();

  auto OverlapCheck = [](std::vector<Boundary> pos_check_area,
                         Boundary obs_boundary) {
    for (const auto& bdry : pos_check_area) {
      if (bdry.has_overlap(obs_boundary)) {
        return true;
      }
    }
    return false;
  };

  auto CalcLatDis = [&](Obstacle* dynamic_obs) {
    double rel_lon_dis =
        dynamic_obs->PolygonBoundary().start_s() - adc_current_s_;
    // lat_dis_obs2adc always > 0
    double lat_dis_obs2adc = std::max(dynamic_obs->max_l(), adc_end_l_) -
                             std::min(dynamic_obs->min_l(), adc_start_l_) -
                             adc_width_ - dynamic_obs->width();
    // cal lat dis between obs and closest_pt
    double lat_dis_obs2path{};
    double rel_lat_dis = speed_planner_common::GetObsToPathLatDis(
                             task_info.current_frame()
                                 ->outside_planner_data()
                                 .speed_obstacle_context.adc_sl_boundaries,
                             *dynamic_obs, lat_dis_obs2path)
                             ? lat_dis_obs2path
                             : lat_dis_obs2adc;
    rel_lat_dis =
        rel_lat_dis + std::abs(dynamic_obs->PolygonBoundary().start_l() -
                               dynamic_obs->PolygonBoundary().end_l()) /
                          2;
    return rel_lat_dis;
  };

  std::vector<Obstacle*> ll_obs{}, lr_obs{}, rl_obs{}, rr_obs{};
  for (std::size_t i = 0; i < dynamic_obs_vec.size(); ++i) {
    if (nullptr == dynamic_obs_vec[i]) {
      LOG_DEBUG("dynamic obstacle is nullptr .");
      continue;
    }
    if (dynamic_obs_vec[i]->type() == Obstacle::ObstacleType::PEDESTRIAN ||
        dynamic_obs_vec[i]->type() == Obstacle::ObstacleType::UNKNOWN_MOVABLE ||
        dynamic_obs_vec[i]->type() ==
            Obstacle::ObstacleType::UNKNOWN_UNMOVABLE ||
        dynamic_obs_vec[i]->type() == Obstacle::ObstacleType::UNKNOWN) {
      continue;
    }
    PathPoint closest_pt{};
    double path_heading_near_obs =
        path.query_closest_point(dynamic_obs_vec[i]->center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff = normalize_angle(
        dynamic_obs_vec[i]->velocity_heading() - path_heading_near_obs);
    if (std::abs(heading_diff) > 0.78539815 ||
        std::abs(dynamic_obs_vec[i]->speed() * std::cos(heading_diff)) <
            speed_lane_change_config.min_obs_speed) {
      continue;
    }
    if (OverlapCheck(check_area.ll_check_boundarys,
                     dynamic_obs_vec[i]->PolygonBoundary())) {
      ll_obs.emplace_back(dynamic_obs_vec[i]);
    }
    if (OverlapCheck(check_area.lr_check_boundarys,
                     dynamic_obs_vec[i]->PolygonBoundary())) {
      lr_obs.emplace_back(dynamic_obs_vec[i]);
    }
    if (OverlapCheck(check_area.rl_check_boundarys,
                     dynamic_obs_vec[i]->PolygonBoundary())) {
      rl_obs.emplace_back(dynamic_obs_vec[i]);
    }
    if (OverlapCheck(check_area.rr_check_boundarys,
                     dynamic_obs_vec[i]->PolygonBoundary())) {
      rr_obs.emplace_back(dynamic_obs_vec[i]);
    }
  }

  if (!ll_obs.empty()) {
    std::sort(ll_obs.begin(), ll_obs.end(),
              [](const Obstacle* a, const Obstacle* b) {
                return a->PolygonBoundary().start_s() <
                       b->PolygonBoundary().start_s();
              });
    PathPoint closest_pt{};
    double path_heading_near_obs =
        path.query_closest_point(ll_obs.front()->center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff = normalize_angle(ll_obs.front()->velocity_heading() -
                                          path_heading_near_obs);
    LeftChangeZeroObsInfo zero_obs_info{
        true,
        ll_obs.front()->id(),
        ll_obs.front()->PolygonBoundary().start_s() - adc_current_s_,
        CalcLatDis(ll_obs.front()),
        std::abs(ll_obs.front()->speed() * std::cos(heading_diff)),
        false};
    left_lanechange_info_.left_change_zero_obs_info = zero_obs_info;
  }
  // right front
  if (!rl_obs.empty()) {
    std::sort(rl_obs.begin(), rl_obs.end(),
              [](const Obstacle* a, const Obstacle* b) {
                return a->PolygonBoundary().start_s() <
                       b->PolygonBoundary().start_s();
              });
    PathPoint closest_pt{};
    double path_heading_near_obs =
        path.query_closest_point(rl_obs.front()->center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff = normalize_angle(rl_obs.front()->velocity_heading() -
                                          path_heading_near_obs);
    RightChangeTwoObsInfo two_obs_info{
        true,
        rl_obs.front()->id(),
        rl_obs.front()->PolygonBoundary().start_s() - adc_current_s_,
        CalcLatDis(rl_obs.front()),
        std::abs(rl_obs.front()->speed() * std::cos(heading_diff)),
        false};
    right_lanechange_info_.right_change_two_obs_info = two_obs_info;
  }
  // 5
  if (!lr_obs.empty()) {
    std::sort(lr_obs.begin(), lr_obs.end(),
              [](const Obstacle* a, const Obstacle* b) {
                return a->PolygonBoundary().start_s() <
                       b->PolygonBoundary().start_s();
              });

    PathPoint closest_pt{};
    double path_heading_near_obs =
        path.query_closest_point(lr_obs.back()->center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff = normalize_angle(lr_obs.back()->velocity_heading() -
                                          path_heading_near_obs);
    LeftChangeFiveObsInfo five_obs_info{
        true,
        lr_obs.back()->id(),
        lr_obs.back()->PolygonBoundary().start_s() - adc_current_s_,
        CalcLatDis(lr_obs.front()),
        std::abs(lr_obs.back()->speed() * std::cos(heading_diff)),
        false};

    left_lanechange_info_.left_change_five_obs_info = five_obs_info;
  }
  // 7
  if (!rr_obs.empty()) {
    std::sort(rr_obs.begin(), rr_obs.end(),
              [](const Obstacle* a, const Obstacle* b) {
                return a->PolygonBoundary().start_s() <
                       b->PolygonBoundary().start_s();
              });
    PathPoint closest_pt{};
    double path_heading_near_obs =
        path.query_closest_point(rr_obs.back()->center(), closest_pt)
            ? closest_pt.theta()
            : inside_data.vel_heading;
    double heading_diff = normalize_angle(rr_obs.back()->velocity_heading() -
                                          path_heading_near_obs);
    RightChangeSevenObsInfo seven_obs_info{
        true,
        rr_obs.back()->id(),
        rr_obs.back()->PolygonBoundary().start_s() - adc_current_s_,
        CalcLatDis(rr_obs.front()),
        std::abs(rr_obs.back()->speed() * std::cos(heading_diff)),
        false};
    right_lanechange_info_.right_change_seven_obs_info = seven_obs_info;
  }

  LOG_INFO(
      "has left front obs ? : [{}] , relative long : {} , relative lat : {} , "
      "obs "
      "speed : {} ,ego speed : {}",
      left_lanechange_info_.left_change_zero_obs_info.has_pos_zero_obs,
      left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_lon_dis,
      left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_lat_dis,
      left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_lon_speed,
      adc_current_v_);

  LOG_INFO(
      "has left rear obs ? : [{}] , relative long : {} , relative lat : {} , "
      "obs "
      "speed : {} ,ego speed : {}",
      left_lanechange_info_.left_change_five_obs_info.has_pos_five_obs,
      left_lanechange_info_.left_change_five_obs_info.pos_five_obs_lon_dis,
      left_lanechange_info_.left_change_five_obs_info.pos_five_obs_lat_dis,
      left_lanechange_info_.left_change_five_obs_info.pos_five_obs_lon_speed,
      adc_current_v_);

  LOG_INFO(
      "has right front obs ? : [{}] , relative long : {} , relative lat : {} , "
      "obs "
      "speed : {} ,ego speed : {}",
      right_lanechange_info_.right_change_two_obs_info.has_pos_two_obs,
      right_lanechange_info_.right_change_two_obs_info.pos_two_obs_lon_dis,
      right_lanechange_info_.right_change_two_obs_info.pos_two_obs_lat_dis,
      right_lanechange_info_.right_change_two_obs_info.pos_two_obs_lon_speed,
      adc_current_v_);
  ///////////right lane change
  LOG_INFO(
      "has right rear obs ? : [{}] , relative long : {} , relative lat : {} , "
      "obs "
      "speed : {} ,ego speed : {}",
      right_lanechange_info_.right_change_seven_obs_info.has_pos_seven_obs,
      right_lanechange_info_.right_change_seven_obs_info.pos_seven_obs_lon_dis,
      right_lanechange_info_.right_change_seven_obs_info.pos_seven_obs_lat_dis,
      right_lanechange_info_.right_change_seven_obs_info
          .pos_seven_obs_lon_speed,
      adc_current_v_);
}

MotorwaySpeedLaneChangeDecider::~MotorwaySpeedLaneChangeDecider() {}

ErrorCode MotorwaySpeedLaneChangeDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    LOG_INFO("path successed tasks is 0, skip rest tasks.");
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  LOG_INFO(">>>>LANE CHANGE speed decider work normal");
  if (!Init(task_info)) {
    LOG_ERROR("LANE CHANGE speed decider Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  LOG_INFO(" pre_stage is {}",
           neodrive::global::planning::MotorwayLaneChangeStageState_State_Name(
               pre_stage_));

  SetVirtualObsForTerminal(task_info);

  pre_stage_ = data_center_->master_info().motorway_lane_change_context().stage;

  if (!Process(task_info)) {
    LOG_ERROR("Check LaneChange Speed decider Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedLaneChangeDecider::SetVirtualObsForTerminal(
    TaskInfo& task_info) {
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;

  auto& lane_change_end_point =
      data_center_->navigation_result().lane_change_end_point;
  ReferencePoint utm_lane_change_end_point;
  ReferencePoint odom_lane_change_end_point;
  ReferencePoint odom_lane_change_end_point_xy;

  if (!task_info.reference_line_raw()->GetNearestRefPoint(
          Vec2d{lane_change_end_point.x(), lane_change_end_point.y()},
          &utm_lane_change_end_point)) {
    return;
  }

  if (pre_stage_ !=
      data_center_->master_info().motorway_lane_change_context().stage) {
    double dis_to_lanechange_require_point =
        utm_lane_change_end_point.s() - task_info.curr_sl().s();
    if (dis_to_lanechange_require_point <
        2 * speed_lane_change_config.dis_to_end_triger) {
      if (data_center_->master_info().motorway_lane_change_context().stage ==
          MotorwayLaneChangeStageState::WAITING)
        lane_change_end_point_s_ =
            std::max(utm_lane_change_end_point.s() -
                         speed_lane_change_config.dis_to_end_triger,
                     task_info.curr_sl().s() + adc_current_v_ * 4.0);
      else
        lane_change_end_point_s_ = std::max(
            utm_lane_change_end_point.s(),
            task_info.curr_sl().s() + std::max(adc_current_v_ * 4.0, 20.0));
    } else {  // utm to odom
      if (data_center_->master_info().motorway_lane_change_context().stage ==
          MotorwayLaneChangeStageState::WAITING)
        lane_change_end_point_s_ = utm_lane_change_end_point.s() -
                                   speed_lane_change_config.dis_to_end_triger;
      else
        lane_change_end_point_s_ = utm_lane_change_end_point.s();
    }
  }
  LOG_INFO(
      "first change waiting. end_point_s: {:.4f}, dis_to_end_triger: "
      "{:.4f}, lane_change_end_point_s_: {:.4f}, "
      "adc_current_v: {:.4f}, curr_s: {:.4f}",
      utm_lane_change_end_point.s(), speed_lane_change_config.dis_to_end_triger,
      lane_change_end_point_s_, adc_current_v_, task_info.curr_sl().s());

  double length = 10.0;
  task_info.reference_line()->GetNearestRefPoint(
      lane_change_end_point_s_ + length / 2.0, &odom_lane_change_end_point_xy);
  task_info.reference_line()->GetNearestRefPoint(lane_change_end_point_s_,
                                                 &odom_lane_change_end_point);
  LOG_INFO("odom_lane_change_end_point s: {:.4f}, curr_s: {:.4f}",
           odom_lane_change_end_point.s(), task_info.curr_sl().s());
  if (decision_data->create_virtual_obstacle(
          Vec2d{odom_lane_change_end_point_xy.x(),
                odom_lane_change_end_point_xy.y()},
          length, FLAGS_planning_virtual_obstacle_height,
          odom_lane_change_end_point.left_lane_bound() +
              odom_lane_change_end_point.right_lane_bound() - 1.0,
          odom_lane_change_end_point.heading(),
          VirtualObstacle::LANECHANGE) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create lane change virtual_obstacle.");
    return;
  }
  // vis func
  auto vis_obs = [](const DecisionData* decision_data) {
    if (!FLAGS_planning_enable_vis_event) return;
    auto event =
        vis::EventSender::Instance()->GetEvent("virtual_obstacles_lane_change");
    event->set_type(visualizer::Event::k3D);
    event->add_attribute(visualizer::Event::kOdom);
    auto set_pt = [](auto ans, auto& p) {
      ans->set_x(p.x()), ans->set_y(p.y()), ans->set_z(0);
    };
    std::vector<Obstacle*> virtual_obstacle_vector{};
    auto ret = decision_data->get_virtual_obstacle_by_type(
        VirtualObstacle::LANECHANGE, virtual_obstacle_vector);
    if (ret != ErrorCode::PLANNING_OK || virtual_obstacle_vector.empty()) {
      LOG_INFO("get virtual obstacle failed.");
      return;
    }
    for (auto obstacle : virtual_obstacle_vector) {
      std::vector<Vec2d> pts{};
      auto polygon = event->add_polygon();
      obstacle->bounding_box().get_all_corners(&pts);
      for (auto& p : pts) set_pt(polygon->add_point(), p);
    }
  };
  vis_obs(decision_data);
}

void MotorwaySpeedLaneChangeDecider::Reset() {
  adj_acc_ = 0;
  check_areas_.Reset();
};

bool MotorwaySpeedLaneChangeDecider::Init(TaskInfo& task_info) {
  LOG_INFO(">>>> step into Lane Change Speed Decider");
  adc_current_s_ =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  adc_current_l_ =
      task_info.current_frame()->inside_planner_data().init_sl_point.l();
  adc_width_ = VehicleParam::Instance()->width();
  adc_length_ = VehicleParam::Instance()->length();
  adc_back_edge_to_center_ = VehicleParam::Instance()->back_edge_to_center();

  adc_front_edge_s_ = adc_current_s_ + adc_length_ - adc_back_edge_to_center_;
  adc_start_l_ = task_info.curr_sl().l() - adc_width_ / 2;
  adc_end_l_ = task_info.curr_sl().l() + adc_width_ / 2;
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  auto& lane_change_enable = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->lane_change_enable_info;
  adj_type_ = LaneChangeAdjType::NONE;
  adj_acc_ = 0;
  lane_change_enable.Reset();

  left_lanechange_info_.Reset();
  right_lanechange_info_.Reset();

  if (data_center_->lastframe_state() !=
      neodrive::global::planning::ScenarioState::State::
          ScenarioState_State_MOTORWAY_LANE_CHANGE)
    pre_stage_ = MotorwayLaneChangeStageState::PREPARE;
  if (!ObsNinePalaceGridDivision(task_info)) {
    LOG_INFO("dynamic obstacle Grid Division failed !");
    return false;
  }

  return true;
}

bool MotorwaySpeedLaneChangeDecider::Process(TaskInfo& task_info) {
  if (!MssModelJudgeSafty(task_info)) {
    LOG_INFO("Mss Model Judge has error.");
    return false;
  }

  if (!ProcessLaneChangeSpeedAdjust(task_info)) {
    LOG_INFO("Speed Adjust Method error.");
    return false;
  }

  return true;
}

bool MotorwaySpeedLaneChangeDecider::ProcessLaneChangeSpeedAdjust(
    TaskInfo& task_info) {
  return true;
}

bool MotorwaySpeedLaneChangeDecider::MssModelJudgeSafty(TaskInfo& task_info) {
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;
  auto& lane_change_enable = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->lane_change_enable_info;
  lane_change_enable.Reset();
  LOG_INFO("Starting MSS Safty check.");

  auto navigator_request = data_center_->navigation_result().navigator_request;

  if (navigator_request ==
      NavigatorLaneChangeRequest::REQUEST_LEFT_LANE_CHANGE) {
    CalTcPointForLeftLaneChange();
    SetMssMapLeftLaneChange(task_info);

    if (!left_lanechange_info_.left_change_zero_obs_info
             .pos_zero_obs_mss_judge ||
        !left_lanechange_info_.left_change_five_obs_info
             .pos_five_obs_mss_judge) {
      lane_change_enable.left_lane_change_enable = false;
      // ll first , lr second
      AdjustForLeftChange(task_info);
    } else {
      lane_change_enable.left_lane_change_enable = true;
    }
    LOG_INFO("Can left lane change ?   :  [{}]",
             lane_change_enable.left_lane_change_enable);
    return true;
  }
  if (navigator_request ==
      NavigatorLaneChangeRequest::REQUEST_RIGHT_LANE_CHANGE) {
    CalTcPointForRightLaneChange();
    SetMssMapRightLaneChange(task_info);

    if (!right_lanechange_info_.right_change_two_obs_info
             .pos_two_obs_mss_judge ||
        !right_lanechange_info_.right_change_seven_obs_info
             .pos_seven_obs_mss_judge) {
      lane_change_enable.right_lane_change_enable = false;
      // ll first , lr second
      AdjustForRightChange(task_info);
    } else {
      lane_change_enable.right_lane_change_enable = true;
    }
    LOG_INFO("Can right lane change ?   :  [{}]",
             lane_change_enable.right_lane_change_enable);
    return true;
  }

  return true;
}

void MotorwaySpeedLaneChangeDecider::AdjustForRightChange(TaskInfo& task_info) {
  if (!right_lanechange_info_.right_change_two_obs_info.pos_two_obs_mss_judge) {
    LOG_INFO(
        "MSS check, Right now ego pos is unsafe to lane change for rf , need "
        "process rf");
    LongtitudeMethodForRf();
  }

  if (!right_lanechange_info_.right_change_seven_obs_info
           .pos_seven_obs_mss_judge) {
    LOG_INFO(
        "MSS check, Ego pos is unsafe to lane change for rr ,need process "
        "rr.");
    LongtitudeMethodForRr();
  }
}

void MotorwaySpeedLaneChangeDecider::AdjustForLeftChange(TaskInfo& task_info) {
  if (!left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_mss_judge) {
    LOG_INFO(
        "MSS check, Right now ego pos is unsafe to lane change for ll , need "
        "process ll");
    LongtitudeMethodForLl();
  }

  if (!left_lanechange_info_.left_change_five_obs_info.pos_five_obs_mss_judge) {
    LOG_INFO(
        "MSS check, Ego pos is unsafe to lane change for lr ,need process "
        "lr.");
    LongtitudeMethodForLr();
  }
}

void MotorwaySpeedLaneChangeDecider::LongtitudeMethodForRr() {
  // check 5 first

  adj_acc_ = -0.4;
  adj_type_ = LaneChangeAdjType::DECELERATION;
  LOG_INFO(
      "Iter Deduction can not speed up, so wait for five obs -> zero pos , a "
      "is {}",
      adj_acc_);
}

void MotorwaySpeedLaneChangeDecider::LongtitudeMethodForLr() {
  // check 5 first

  adj_acc_ = -0.4;
  adj_type_ = LaneChangeAdjType::DECELERATION;
  LOG_INFO(
      "Iter Deduction can not speed up, so wait for five obs -> zero pos , a "
      "is {}",
      adj_acc_);
}

void MotorwaySpeedLaneChangeDecider::LongtitudeMethodForRf() {
  // checkout init x > 0?
  LOG_INFO("Left Lane Change, Step into process LL obs ");
  bool x_positive_negative =
      right_change_mss_map_point_.rf_mss_init_point.first >= 0 ? true : false;
  bool has_solution_for_rf{false};
  if (!x_positive_negative) {
    LOG_INFO(
        "Right Lane Change rf MSS x point < 0 , means ego speed < obs speed");
    int t_roll_all = 9.;
    std::pair<double, double> final_coordinate_for_rf_neg;
    for (size_t const_v_t = 0; const_v_t <= t_roll_all; ++const_v_t) {
      final_coordinate_for_rf_neg = std::make_pair(
          right_change_mss_map_point_.rf_mss_init_point.first,
          right_change_mss_map_point_.rf_mss_init_point.second -
              const_v_t * right_change_mss_map_point_.rf_mss_init_point.first);
      if (right_change_mss_map_point_.rf_mss_init_point.second >
          right_change_mss_map_point_.rf_mss_slope.second *
              right_change_mss_map_point_.rf_mss_init_point.first) {
        has_solution_for_rf = true;
        adj_type_ = LaneChangeAdjType::SLIGHTDECELERATION;
        LOG_INFO(
            "Right Lane Change keep right now speed can slowly trun to safe "
            "area");
      }
    }
  } else {
    LOG_INFO(
        "Right Lane Change MSS x point > 0 , means ego speed > obs speed, need "
        "deceleration");
    std::pair<double, double> final_coordinate_for_rf_pos;
    for (size_t i = 0; i <= 7; ++i) {
      LOG_INFO("Traverse deceleration points");
      double a_adj = -0.5 - 0.05 * i;
      double brake_time =
          1 +
          right_change_mss_map_point_.rf_mss_init_point.first / std::abs(a_adj);
      double new_cooridinate_x =
          right_change_mss_map_point_.rf_mss_init_point.first +
          brake_time * a_adj;
      double new_cooridinate_y =
          right_change_mss_map_point_.rf_mss_init_point.second +
          std::pow(right_change_mss_map_point_.rf_mss_init_point.first, 2) /
              (2 * a_adj) -
          new_cooridinate_x / (2 * a_adj);
      final_coordinate_for_rf_pos =
          std::make_pair(new_cooridinate_x, new_cooridinate_y);
      if ((final_coordinate_for_rf_pos.first >= 0 &&
           final_coordinate_for_rf_pos.second >=
               right_change_mss_map_point_.rf_mss_slope.first *
                   final_coordinate_for_rf_pos.first) ||
          (final_coordinate_for_rf_pos.first < 0 &&
           final_coordinate_for_rf_pos.second >
               right_change_mss_map_point_.rf_mss_slope.second *
                   final_coordinate_for_rf_pos.first)) {
        adj_acc_ = a_adj;
        has_solution_for_rf = true;
        adj_type_ = LaneChangeAdjType::DECELERATION;
        LOG_INFO(
            "after Traverse deceleration can step into safe area , a is : {}",
            adj_acc_);
        break;
      }
    }
  }
  if (!has_solution_for_rf) {
    adj_type_ = LaneChangeAdjType::DECELERATION;
    adj_acc_ = -0.4;
    LOG_INFO(
        "there is no solution to process rf, Forced deceleration! acc is : {}",
        adj_acc_);
  }
}

void MotorwaySpeedLaneChangeDecider::LongtitudeMethodForLl() {
  // checkout init x > 0?
  LOG_INFO("Left Lane Change, Step into process LL obs ");
  bool x_positive_negative =
      left_change_mss_map_point_.ll_mss_init_point.first >= 0 ? true : false;
  bool has_solution_for_ll{false};
  if (!x_positive_negative) {
    LOG_INFO(
        "Left Lane Change ll MSS x point < 0 , means ego speed < obs speed");
    int t_roll_all = 9.;
    std::pair<double, double> final_coordinate_for_ll_neg;
    for (size_t const_v_t = 0; const_v_t <= t_roll_all; ++const_v_t) {
      final_coordinate_for_ll_neg = std::make_pair(
          left_change_mss_map_point_.ll_mss_init_point.first,
          left_change_mss_map_point_.ll_mss_init_point.second -
              const_v_t * left_change_mss_map_point_.ll_mss_init_point.first);
      if (left_change_mss_map_point_.ll_mss_init_point.second >
          left_change_mss_map_point_.ll_mss_slope.second *
              left_change_mss_map_point_.ll_mss_init_point.first) {
        has_solution_for_ll = true;
        adj_type_ = LaneChangeAdjType::SLIGHTDECELERATION;
        LOG_INFO(
            "Left Lane Change keep right now speed can slowly trun to safe "
            "area");
      }
    }
  } else {
    LOG_INFO(
        "Left Lane Change MSS x point > 0 , means ego speed > obs speed, need "
        "deceleration");
    std::pair<double, double> final_coordinate_for_ll_pos;
    for (size_t i = 0; i <= 7; ++i) {
      LOG_INFO("Traverse deceleration points");
      double a_adj = -0.5 - 0.05 * i;
      double brake_time =
          1 +
          left_change_mss_map_point_.ll_mss_init_point.first / std::abs(a_adj);
      double new_cooridinate_x =
          left_change_mss_map_point_.ll_mss_init_point.first +
          brake_time * a_adj;
      double new_cooridinate_y =
          left_change_mss_map_point_.ll_mss_init_point.second +
          std::pow(left_change_mss_map_point_.ll_mss_init_point.first, 2) /
              (2 * a_adj) -
          new_cooridinate_x / (2 * a_adj);
      final_coordinate_for_ll_pos =
          std::make_pair(new_cooridinate_x, new_cooridinate_y);
      if ((final_coordinate_for_ll_pos.first >= 0 &&
           final_coordinate_for_ll_pos.second >=
               left_change_mss_map_point_.ll_mss_slope.first *
                   final_coordinate_for_ll_pos.first) ||
          (final_coordinate_for_ll_pos.first < 0 &&
           final_coordinate_for_ll_pos.second >
               left_change_mss_map_point_.ll_mss_slope.second *
                   final_coordinate_for_ll_pos.first)) {
        adj_acc_ = a_adj;
        has_solution_for_ll = true;
        adj_type_ = LaneChangeAdjType::DECELERATION;
        LOG_INFO(
            "after Traverse deceleration can step into safe area , a is : {}",
            adj_acc_);
        break;
      }
    }
  }
  if (!has_solution_for_ll) {
    adj_type_ = LaneChangeAdjType::DECELERATION;
    adj_acc_ = -0.4;
    LOG_INFO(
        "there is no solution to process ll, Forced deceleration! acc is : {}",
        adj_acc_);
  }
}

void MotorwaySpeedLaneChangeDecider::CalTcPointForRightLaneChange() {
  // build lane change model
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;
  double lane_change_use_time = speed_lane_change_config.lane_change_use_time;
  /**
   * @brief lat a formula : a = 2 pi /(t_lc*t_lc) * sin (2 * pi * t / t_lc)
   *                        v = H / t_lc  - H / t_lc * cos (2 * pi * t /t_lc)
   *                        L = H / t_lc * t - H / (2 * pi) * sin(2 * pi * t /
   *                            t_lc)
   *
   *     Due to the complexity of solving linear inequalities and the fact
   * that the collision points are in an approximately linear region,
   * linear fitting is used as a rough solution, which is then iteratively
   * solved
   */
  auto cal_tc_para_rf = [=]() {
    double lc =
        right_lanechange_info_.right_change_two_obs_info.pos_two_obs_lat_dis;

    double t1 = speed_lane_change_config.lane_change_sample_t_s;
    double y1 = right_lc_lat_dis_ / lane_change_use_time * t1 -
                right_lc_lat_dis_ / (2 * M_PI) *
                    std::sin(M_PI * 2 * t1 / lane_change_use_time);
    double t2 = speed_lane_change_config.lane_change_sample_t_e;
    double y2 = right_lc_lat_dis_ / lane_change_use_time * t2 -
                right_lc_lat_dis_ / (2 * M_PI) *
                    std::sin(M_PI * 2 * t2 / lane_change_use_time);
    auto tc_init =
        (lc - (t1 * y2 - t2 * y1) / (t2 - t1)) * (t2 - t1) / (y2 - y1);

    auto v_tc_lat = right_lc_lat_dis_ / lane_change_use_time -
                    right_lc_lat_dis_ / lane_change_use_time *
                        std::cos(2 * M_PI * tc_init / lane_change_use_time);

    LOG_INFO("check time tc {}", tc_init);

    auto res = std::make_pair(tc_init, v_tc_lat);
    return res;
  };
  auto para_rf = cal_tc_para_rf();

  right_lc_tc_para_.rf_tc_para_ = para_rf;

  auto ca_sr_rf = [=]() {
    double collision_theta =
        std::atan2(std::abs(para_rf.second), adc_current_v_);

    double sr0 =
        right_lanechange_info_.right_change_two_obs_info.pos_two_obs_lon_dis -
        adc_width_ / 2 * std::sin(collision_theta) -
        adc_length_ * std::cos(collision_theta) -
        speed_lane_change_config.lane_change_ld_buffer -
        speed_lane_change_config.ego_ld_thw * adc_current_v_ -
        speed_lane_change_config.ld_ego_ttw *
            right_lanechange_info_.right_change_two_obs_info
                .pos_two_obs_lon_speed;
    return sr0;
  };

  right_change_mss_map_point_.rf_mss_init_point = std::make_pair(
      (adc_current_v_ -
       right_lanechange_info_.right_change_two_obs_info.pos_two_obs_lon_speed),
      ca_sr_rf());

  LOG_INFO("check ! right front MSS init point {} , {}",
           right_change_mss_map_point_.rf_mss_init_point.first,
           right_change_mss_map_point_.rf_mss_init_point.second);

  auto cal_tc_para_rr = [=]() {
    // if obs 0 is far from ego , target speed is ego , else has target acc
    double v_for_rf =
        (!right_lanechange_info_.right_change_two_obs_info.has_pos_two_obs ||
         right_lanechange_info_.right_change_two_obs_info.pos_two_obs_lon_dis >
             speed_lane_change_config.ignore_ld_dis)
            ? adc_current_v_
            : right_lanechange_info_.right_change_two_obs_info
                  .pos_two_obs_lon_speed;

    double a_for_rf = (v_for_rf - adc_current_v_) / 6;
    LOG_INFO("check v_for_rf :  {} , a_for_rf : {}", v_for_rf, a_for_rf);

    auto res = std::make_pair(
        right_lanechange_info_.right_change_seven_obs_info
                .pos_seven_obs_lon_speed -
            adc_current_v_,
        0.5 * a_for_rf * std::pow(lane_change_use_time, 2) +
            std::abs(right_lanechange_info_.right_change_seven_obs_info
                         .pos_seven_obs_lon_dis) -
            adc_width_ - speed_lane_change_config.lane_change_fd_buffer -
            speed_lane_change_config.ego_fd_thw_ttw * adc_current_v_ -
            speed_lane_change_config.ego_fd_thw_ttw *
                right_lanechange_info_.right_change_seven_obs_info
                    .pos_seven_obs_lon_speed);

    LOG_INFO("RR-- A : {} B: {} C : {}",
             0.5 * a_for_rf * std::pow(lane_change_use_time, 2),
             std::abs(right_lanechange_info_.right_change_seven_obs_info
                          .pos_seven_obs_lon_dis),
             -adc_width_ - speed_lane_change_config.lane_change_fd_buffer -
                 speed_lane_change_config.ego_fd_thw_ttw * adc_current_v_ -
                 speed_lane_change_config.ego_fd_thw_ttw *
                     right_lanechange_info_.right_change_seven_obs_info
                         .pos_seven_obs_lon_speed);

    return res;
  };
  right_change_mss_map_point_.rr_mss_init_point = std::move(cal_tc_para_rr());
  LOG_INFO("check ! LR MSS init point {} , {}",
           right_change_mss_map_point_.rr_mss_init_point.first,
           right_change_mss_map_point_.rr_mss_init_point.second);
}

void MotorwaySpeedLaneChangeDecider::CalTcPointForLeftLaneChange() {
  // build lane change model
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;
  double lane_change_use_time = speed_lane_change_config.lane_change_use_time;
  /**
   * @brief lat a formula : a = 2 pi /(t_lc*t_lc) * sin (2 * pi * t / t_lc)
   *                        v = H / t_lc  - H / t_lc * cos (2 * pi * t /t_lc)
   *                        L = H / t_lc * t - H / (2 * pi) * sin(2 * pi * t /
   *                            t_lc)
   *
   *     Due to the complexity of solving linear inequalities and the fact
   * that the collision points are in an approximately linear region,
   * linear fitting is used as a rough solution, which is then iteratively
   * solved
   */
  auto cal_tc_para_ll = [=]() {
    double lc =
        left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_lat_dis;
    double t1 = speed_lane_change_config.lane_change_sample_t_s;
    double y1 = left_lc_lat_dis_ / lane_change_use_time * t1 -
                left_lc_lat_dis_ / (2 * M_PI) *
                    std::sin(M_PI * 2 * t1 / lane_change_use_time);
    double t2 = speed_lane_change_config.lane_change_sample_t_e;
    double y2 = left_lc_lat_dis_ / lane_change_use_time * t2 -
                left_lc_lat_dis_ / (2 * M_PI) *
                    std::sin(M_PI * 2 * t2 / lane_change_use_time);
    auto tc_init =
        (lc - (t1 * y2 - t2 * y1) / (t2 - t1)) * (t2 - t1) / (y2 - y1);

    // LOG_INFO("check time tc first {}", tc_init);
    // equation : H / 6 * t - H / (2 * pi) * sin(pi * t /3)-lc
    // for (size_t i = 0;
    //      i < speed_lane_change_config.iter_size_for_nonlinear_solve; ++i) {
    //   auto f_x = left_lc_lat_dis_ / lane_change_use_time * tc_init -
    //              left_lc_lat_dis_ / (2 * M_PI) *
    //                  std::sin(2 * M_PI * tc_init / lane_change_use_time) -
    //              lc;
    //   auto f_prime_x = left_lc_lat_dis_ / lane_change_use_time -
    //                    left_lc_lat_dis_ / (2 * M_PI) * 2 * M_PI /
    //                        lane_change_use_time *
    //                        std::cos(2 * M_PI * tc_init /
    //                        lane_change_use_time);
    //   auto x1 = tc_init - f_x / f_prime_x;

    //   if (std::abs(x1 - tc_init) <
    //       speed_lane_change_config.iter_max_threshold) {
    //     tc_init = x1;
    //     break;
    //   }
    //   tc_init = x1;
    // }
    auto v_tc_lat = left_lc_lat_dis_ / lane_change_use_time -
                    left_lc_lat_dis_ / lane_change_use_time *
                        std::cos(2 * M_PI * tc_init / lane_change_use_time);

    LOG_INFO("check time tc {}", tc_init);

    auto res = std::make_pair(tc_init, v_tc_lat);
    return res;
  };
  auto para_ll = cal_tc_para_ll();
  left_lc_tc_para_.ll_tc_para_ = para_ll;
  auto ca_sr = [=]() {
    double collision_theta =
        std::atan2(std::abs(para_ll.second), adc_current_v_);

    double sr0 =
        left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_lon_dis -
        adc_width_ / 2 * std::sin(collision_theta) -
        adc_length_ * std::cos(collision_theta) -
        speed_lane_change_config.lane_change_ld_buffer -
        speed_lane_change_config.ego_ld_thw * adc_current_v_ -
        speed_lane_change_config.ld_ego_ttw *
            left_lanechange_info_.left_change_zero_obs_info
                .pos_zero_obs_lon_speed;
    return sr0;
  };

  left_change_mss_map_point_.ll_mss_init_point = std::make_pair(
      (adc_current_v_ -
       left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_lon_speed),
      ca_sr());
  LOG_INFO("check ! LL MSS init point {} , {}",
           left_change_mss_map_point_.ll_mss_init_point.first,
           left_change_mss_map_point_.ll_mss_init_point.second);

  auto cal_tc_para_lr = [=]() {
    // if obs 0 is far from ego , target speed is ego , else has target acc
    double v_for_ld =
        (!left_lanechange_info_.left_change_zero_obs_info.has_pos_zero_obs ||
         left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_lon_dis >
             speed_lane_change_config.ignore_ld_dis)
            ? adc_current_v_
            : left_lanechange_info_.left_change_zero_obs_info
                  .pos_zero_obs_lon_speed;

    double a_for_ld = (v_for_ld - adc_current_v_) / 6;
    LOG_INFO("check v_for_ld :  {} ,a_for_ld : {}", v_for_ld, a_for_ld);

    auto res = std::make_pair(
        left_lanechange_info_.left_change_five_obs_info.pos_five_obs_lon_speed -
            adc_current_v_,
        0.5 * a_for_ld * std::pow(lane_change_use_time, 2) +
            std::abs(left_lanechange_info_.left_change_five_obs_info
                         .pos_five_obs_lon_dis) -
            adc_width_ - speed_lane_change_config.lane_change_fd_buffer -
            speed_lane_change_config.ego_fd_thw_ttw * adc_current_v_ -
            speed_lane_change_config.ego_fd_thw_ttw *
                left_lanechange_info_.left_change_five_obs_info
                    .pos_five_obs_lon_speed);

    LOG_INFO("LR A : {} B: {} C : {}",
             0.5 * a_for_ld * std::pow(lane_change_use_time, 2),
             std::abs(left_lanechange_info_.left_change_five_obs_info
                          .pos_five_obs_lon_dis),
             -adc_width_ - speed_lane_change_config.lane_change_fd_buffer -
                 speed_lane_change_config.ego_fd_thw_ttw * adc_current_v_ -
                 speed_lane_change_config.ego_fd_thw_ttw *
                     left_lanechange_info_.left_change_five_obs_info
                         .pos_five_obs_lon_speed);

    return res;
  };
  left_change_mss_map_point_.lr_mss_init_point = std::move(cal_tc_para_lr());
  LOG_INFO("check ! LR MSS init point {} , {}",
           left_change_mss_map_point_.lr_mss_init_point.first,
           left_change_mss_map_point_.lr_mss_init_point.second);
}

void MotorwaySpeedLaneChangeDecider::SetMssMapRightLaneChange(
    TaskInfo& task_info) {
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;
  if (right_change_mss_map_point_.rf_mss_init_point.first >= 0.) {
    right_lanechange_info_.right_change_two_obs_info.pos_two_obs_mss_judge =
        right_change_mss_map_point_.rf_mss_init_point.second >
                speed_lane_change_config.lane_change_use_time / 2 *
                    right_change_mss_map_point_.rf_mss_init_point.first
            ? true
            : false;
  } else {
    right_lanechange_info_.right_change_two_obs_info.pos_two_obs_mss_judge =
        right_change_mss_map_point_.rf_mss_init_point.second >
                right_lc_tc_para_.rf_tc_para_.first *
                    right_change_mss_map_point_.rf_mss_init_point.first
            ? true
            : false;
  }
  right_change_mss_map_point_.rf_mss_slope =
      std::make_pair(speed_lane_change_config.lane_change_use_time / 2,
                     right_lc_tc_para_.rf_tc_para_.first);

  LOG_INFO("RF MSS slope : {} , {}",
           right_change_mss_map_point_.rf_mss_slope.first,
           right_change_mss_map_point_.rf_mss_slope.second);

  if (right_change_mss_map_point_.rr_mss_init_point.first >= 0.) {
    right_lanechange_info_.right_change_seven_obs_info.pos_seven_obs_mss_judge =
        right_change_mss_map_point_.rr_mss_init_point.second >
                speed_lane_change_config.lane_change_use_time *
                    right_lc_lat_dis_ / 3.5 *
                    right_change_mss_map_point_.rr_mss_init_point.first
            ? true
            : false;
  } else {
    right_lanechange_info_.right_change_seven_obs_info.pos_seven_obs_mss_judge =
        right_change_mss_map_point_.rr_mss_init_point.second >
                speed_lane_change_config.lane_change_use_time * 0.5 *
                    right_lc_lat_dis_ / 3.5 *
                    right_change_mss_map_point_.rr_mss_init_point.first
            ? true
            : false;
  }
  right_change_mss_map_point_.rr_mss_slope =
      std::make_pair(speed_lane_change_config.lane_change_use_time,
                     speed_lane_change_config.lane_change_use_time * 0.75);

  LOG_INFO("RR MSS slope : {} , {}",
           right_change_mss_map_point_.rr_mss_slope.first,
           right_change_mss_map_point_.rr_mss_slope.second);
}

void MotorwaySpeedLaneChangeDecider::SetMssMapLeftLaneChange(
    TaskInfo& task_info) {
  const auto& speed_lane_change_config = config::PlanningConfig::Instance()
                                             ->planning_research_config()
                                             .speed_lane_change;
  if (left_change_mss_map_point_.ll_mss_init_point.first >= 0.) {
    left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_mss_judge =
        left_change_mss_map_point_.ll_mss_init_point.second >
                speed_lane_change_config.lane_change_use_time / 2 *
                    left_change_mss_map_point_.ll_mss_init_point.first
            ? true
            : false;
  } else {
    left_lanechange_info_.left_change_zero_obs_info.pos_zero_obs_mss_judge =
        left_change_mss_map_point_.ll_mss_init_point.second >
                left_lc_tc_para_.ll_tc_para_.first *
                    left_change_mss_map_point_.ll_mss_init_point.first
            ? true
            : false;
  }
  left_change_mss_map_point_.ll_mss_slope =
      std::make_pair(speed_lane_change_config.lane_change_use_time / 2,
                     left_lc_tc_para_.ll_tc_para_.first);

  LOG_INFO("LL MSS slope : {} , {}",
           left_change_mss_map_point_.ll_mss_slope.first,
           left_change_mss_map_point_.ll_mss_slope.second);

  if (left_change_mss_map_point_.lr_mss_init_point.first >= 0.) {
    left_lanechange_info_.left_change_five_obs_info.pos_five_obs_mss_judge =
        left_change_mss_map_point_.lr_mss_init_point.second >
                speed_lane_change_config.lane_change_use_time *
                    left_lc_lat_dis_ / 3.5 *
                    left_change_mss_map_point_.lr_mss_init_point.first
            ? true
            : false;
  } else {
    left_lanechange_info_.left_change_five_obs_info.pos_five_obs_mss_judge =
        left_change_mss_map_point_.lr_mss_init_point.second >
                speed_lane_change_config.lane_change_use_time * 0.5 *
                    left_lc_lat_dis_ / 3.5 *
                    left_change_mss_map_point_.lr_mss_init_point.first
            ? true
            : false;
  }
  left_change_mss_map_point_.lr_mss_slope =
      std::make_pair(speed_lane_change_config.lane_change_use_time,
                     speed_lane_change_config.lane_change_use_time * 0.75);

  LOG_INFO("LL MSS slope : {} , {}",
           left_change_mss_map_point_.lr_mss_slope.first,
           left_change_mss_map_point_.lr_mss_slope.second);
}

void MotorwaySpeedLaneChangeDecider::SaveTaskResults(TaskInfo& task_info) {
  VisLaneChangePara(task_info);
  // default 3s
  LOG_INFO("check adj type : {}", int(adj_type_));
  const auto& adc_boundary = task_info.last_frame()
                                 ->outside_planner_data()
                                 .path_obstacle_context.adc_boundary;
  bool if_is_in_lane_bound{(adc_boundary.end_l() <
                            task_info.curr_referline_pt().left_lane_bound()) &&
                           (adc_boundary.start_l() >
                            -task_info.curr_referline_pt().right_lane_bound())};
  LOG_INFO("check ego is in origin lane boundary : [{}]", if_is_in_lane_bound);

  if (adj_type_ == LaneChangeAdjType::DECELERATION && if_is_in_lane_bound) {
    double limited_speed = (adc_current_v_ + adj_acc_ * 1.5) > 0
                               ? (adc_current_v_ + adj_acc_ * 1.5)
                               : 0;
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::LANE_CHANGE);
    internal_speed_limit.add_upper_bounds(limited_speed);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(adj_acc_);
    LOG_INFO(
        "LANE_CHANGE {} limit speed: speed = {:.2f}, acc = {:.2f}, current "
        "ego "
        "speed = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed, adj_acc_, adc_current_v_);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
  if (adj_type_ == LaneChangeAdjType::SLIGHTDECELERATION &&
      if_is_in_lane_bound) {
    double limited_speed =
        (adc_current_v_ - 0.2 * 1.5) > 0 ? (adc_current_v_ - 0.2 * 1.5) : 0;
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::LANE_CHANGE);
    internal_speed_limit.add_upper_bounds(limited_speed);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(-0.2);
    LOG_INFO(
        "LANE_CHANGE {} limit speed: speed = {:.2f}, current "
        "ego "
        "speed = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed, adc_current_v_);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

void MotorwaySpeedLaneChangeDecider::VisLaneChangePara(TaskInfo& task_info) {
  auto& lane_change_enable = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->lane_change_enable_info;
  if (!FLAGS_planning_enable_vis_event) return;

  auto e_CanLaneChange =
      vis::EventSender::Instance()->GetEvent("CanLaneChange");
  e_CanLaneChange->set_type(visualizer::Event::k2D);
  e_CanLaneChange->mutable_color()->set_r(0.8);
  e_CanLaneChange->mutable_color()->set_g(0.0);
  e_CanLaneChange->mutable_color()->set_b(0.0);
  e_CanLaneChange->mutable_color()->set_a(0.6);
  auto CanLaneChange = e_CanLaneChange->add_polyline();

  auto set_pt = [](auto ans, auto x, auto y) {
    ans->set_x(x), ans->set_y(y), ans->set_z(0);
  };
  for (size_t i = 0; i < 80; ++i) {
    set_pt(CanLaneChange->add_point(), i * 0.1,
           lane_change_enable.left_lane_change_enable);
  }

  if (adj_type_ == LaneChangeAdjType::DECELERATION) {
    double limited_speed = adc_current_v_ + adj_acc_ * 1.5;

    auto lanechangeacc =
        vis::EventSender::Instance()->GetEvent("lanechangeacc");
    lanechangeacc->set_type(visualizer::Event::k2D);
    lanechangeacc->mutable_color()->set_r(0.8);
    lanechangeacc->mutable_color()->set_g(0.0);
    lanechangeacc->mutable_color()->set_b(0.0);
    lanechangeacc->mutable_color()->set_a(0.6);
    auto lanechangeacc_ = lanechangeacc->add_polyline();

    for (size_t i = 0; i < 80; ++i) {
      set_pt(lanechangeacc_->add_point(), i * 0.1, adj_acc_);
    }
  }
}

}  // namespace planning
}  // namespace neodrive
