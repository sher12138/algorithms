#include "motorway_speed_stop_to_go_freespace_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {
namespace {

const auto& speed_stop_to_go_freespace_decider_config_{
    config::PlanningConfig::Instance()
        ->planning_research_config()
        .speed_stop_to_go_freespace_decider_config};
}

MotorwaySpeedStopToGoFreespaceDecider::MotorwaySpeedStopToGoFreespaceDecider() {
  name_ = "MotorwaySpeedStopToGoFreespaceDecider";
}

MotorwaySpeedStopToGoFreespaceDecider::
    ~MotorwaySpeedStopToGoFreespaceDecider() {
  Reset();
}

ErrorCode MotorwaySpeedStopToGoFreespaceDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!DataCheck(task_info)) {
    LOG_ERROR("DataCheck failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedStopToGoFreespaceDecider::SaveTaskResults(
    TaskInfo& task_info) {
  if (update_limited_speed_) {
    UpdatedLimitedSpeed();
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::STOP_TO_GO);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(limited_deceleration_);
    LOG_INFO(
        "STOP_TO_GO {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, limited_deceleration_);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    last_limited_speed_ = limited_speed_;
  }
}

void MotorwaySpeedStopToGoFreespaceDecider::UpdatedLimitedSpeed() {
  if ((adc_current_v_ < target_speed_) ||
      (std::abs(limited_deceleration_) < 1e-2)) {
    limited_speed_ = target_speed_;
    limited_deceleration_ = 0.0;
    return;
  }

  limited_speed_ =
      std::min(limited_speed_, last_limited_speed_ +
                                   kPlanningCycleTime * limited_deceleration_);
  limited_speed_ = std::max(limited_speed_, target_speed_);

  LOG_INFO(
      "target_speed_ {:.2f}, set limited_speed_ {:.2f}, limited_deceleration_ "
      "{:.2f}",
      target_speed_, limited_speed_, limited_deceleration_);
}

bool MotorwaySpeedStopToGoFreespaceDecider::Init(TaskInfo& task_info) {
  adc_current_l_ = task_info.curr_sl().l();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  limited_deceleration_ = 0.0;
  target_speed_ = 1e5;

  return true;
}

bool MotorwaySpeedStopToGoFreespaceDecider::DataCheck(TaskInfo& task_info) {
  if (task_info.current_frame() == nullptr) {
    LOG_ERROR("current_frame is nullptr.");
    return false;
  }

  if (task_info.last_frame() == nullptr) {
    LOG_ERROR("last_frame is nullptr.");
    return false;
  }
  if (task_info.current_frame()->mutable_outside_planner_data() == nullptr) {
    LOG_ERROR("mutable_outside_planner_data() is nullptr.");
    return false;
  }

  return true;
}

bool MotorwaySpeedStopToGoFreespaceDecider::Process(TaskInfo& task_info) {
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }

  if (adc_current_v_ >
      speed_stop_to_go_freespace_decider_config_.adc_speed_threshold) {
    LOG_INFO("adc speed faster than {} m/s, not deal!",
             speed_stop_to_go_freespace_decider_config_.adc_speed_threshold);
    return true;
  }

  GetAdcCheckPolygons(task_info);

  auto lidar_points = speed_planner_common::GetOdoLidarPoints(
      *(data_center_->lidar_freespace_msg.ptr));
  if (lidar_points.empty()) {
    LOG_INFO("lidar freespace is empty, not deal!");
    return true;
  }
  Polygon2d freespace_polygon(lidar_points);
  ProcessFreespaceCollisionRisk(task_info, freespace_polygon);

  return true;
}

void MotorwaySpeedStopToGoFreespaceDecider::GetAdcCheckPolygons(
    TaskInfo& task_info) {
  auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& outside_data =
      task_info.current_frame()->mutable_outside_planner_data();
  const auto& path_accumulated_s =
      outside_data->motorway_speed_obstacle_context.path_accumulated_s;
  const auto& path_points = outside_data->path_data->path().path_points();
  adc_check_polygons_.clear();
  if ((path_points.size() < 3) ||
      (path_points.size() != path_accumulated_s.size())) {
    LOG_INFO(
        "path related info size error, get adc_polygon_: path_points, "
        "path_accumulated_s: {}, {}",
        path_points.size(), path_accumulated_s.size());
    adc_check_polygons_.emplace_back(
        std::move(VehicleParam::Instance()->get_adc_polygon(
            {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading,
            0.0, 0.0, 0.0)));
  } else {
    for (int i = 0;
         i < path_accumulated_s.size() &&
         path_accumulated_s[i] <
             speed_stop_to_go_freespace_decider_config_.freespace_check_dis;
         i++) {
      const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
          {path_points[i].x(), path_points[i].y()}, path_points[i].theta(), 0.0,
          0.0, 0.0);
      if (!Utility::check_area(adc_box)) {
        LOG_ERROR("build adc bounding box from path point at s[{}] failed",
                  path_points[i].s());
        continue;
      }
      adc_check_polygons_.emplace_back(std::move(Polygon2d(adc_box)));
    }
  }
  LOG_INFO("get adc_check_polygons_ size: {}", adc_check_polygons_.size());
}

bool MotorwaySpeedStopToGoFreespaceDecider::SegmentCutByPolygon(
    const Polygon2d& polygon, const Segment2d& segment) {
  const auto& points = polygon.points();
  for (size_t i = 0; i < points.size(); ++i) {
    Segment2d edge{points[i], points[(i + 1) % points.size()]};
    Vec2d intersect;
    if (edge.get_intersect(segment, &intersect)) {
      LOG_DEBUG(
          "freespace edge: start({:.2f}, {:.2f}),  end({:.2f}, {:.2f}) "
          "intersect with adc segment: start({:.2f}, {:.2f}),  end({:.2f}, "
          "{:.2f}) at point:({:.2f}, {:.2f}) ",
          edge.start().x(), edge.start().y(), edge.end().x(), edge.end().y(),
          segment.start().x(), segment.start().y(), segment.end().x(),
          segment.end().y(), intersect.x(), intersect.y());
      return true;
    }
  }

  return false;
}

void MotorwaySpeedStopToGoFreespaceDecider::ProcessFreespaceCollisionRisk(
    TaskInfo& task_info, const Polygon2d& freespace_polygon) {
  bool cut_by_freespace_edge{false};
  for (const auto& check_polygon : adc_check_polygons_) {
    for (const auto& seg : check_polygon.segments()) {
      if (cut_by_freespace_edge = SegmentCutByPolygon(freespace_polygon, seg)) {
        break;
      }
    }

    if (cut_by_freespace_edge) {
      obs_keep_cnt_ = speed_stop_to_go_freespace_decider_config_.obs_keep_cnt;
      request_sd_cnt_++;
      break;
    }
  }

  if (obs_keep_cnt_ > 0) {
    obs_keep_cnt_--;
    const auto& path =
        task_info.current_frame()->outside_planner_data().path_data->path();
    const double init_s =
        task_info.current_frame()->inside_planner_data().init_sl_point.s();
    task_info.current_frame()
        ->mutable_planning_data()
        ->mutable_decision_data()
        ->create_virtual_obstacle(path, init_s, adc_front_edge_s_,
                                  VirtualObstacle::FREESPACE_STATION);

    update_limited_speed_ = true;
    target_speed_ = 0.0;
    limited_deceleration_ = -2.0;
    LOG_INFO(
        "limit speed and create stop to go freespace virtual obstacle, "
        "init_s, adc_front_edge_s_:{:.2f}, {:.2f}",
        init_s, adc_front_edge_s_);
  } else {
    request_sd_cnt_ = 0;
  }

  if (request_sd_cnt_ >=
      speed_stop_to_go_freespace_decider_config_.request_sd_cnt) {
    LOG_INFO("fresspace block adc long time, request SD !");
    request_sd_cnt_ = speed_stop_to_go_freespace_decider_config_.request_sd_cnt;
    data_center_->mutable_event_report_proxy()->SetEvent(
        EventType::OBSTACLE_BLOCK_TIMEOUT);
  }
}

}  // namespace planning
}  // namespace neodrive
