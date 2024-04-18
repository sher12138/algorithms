#include "motorway_speed_path_info_decider.h"

#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

namespace {
void VisAdcCornerPoint(const Vec2d& corner_point,
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

  auto sphere = event->mutable_sphere()->Add();
  sphere->mutable_center()->set_x(corner_point.x());
  sphere->mutable_center()->set_y(corner_point.y());
  sphere->mutable_center()->set_z(0);
  sphere->set_radius(0.05);
}

}  // namespace
MotorwaySpeedPathInfoDecider::MotorwaySpeedPathInfoDecider() {
  name_ = "MotorwaySpeedPathInfoDecider";
}

MotorwaySpeedPathInfoDecider::~MotorwaySpeedPathInfoDecider() { Reset(); }

ErrorCode MotorwaySpeedPathInfoDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  auto outside_data_ptr =
      task_info.current_frame()->mutable_outside_planner_data();
  auto inside_data = task_info.current_frame()->inside_planner_data();
  if (outside_data_ptr == nullptr) return ErrorCode::PLANNING_ERROR_FAILED;

  GetAdcCornerPointCoordinate(inside_data,
                              outside_data_ptr->motorway_speed_obstacle_context
                                  .adc_corner_pt_coordinate);

  auto& accumulated_s =
      outside_data_ptr->motorway_speed_obstacle_context.path_accumulated_s;
  if (!ComputePathAccumulatedS(*(outside_data_ptr->path_data),
                               &accumulated_s)) {
    LOG_ERROR("compute path accumulated_s failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  enlarge_buffer_ = FLAGS_planning_speed_plan_enlarge_self_buffer;
  front_enlarge_buffer_ = enlarge_buffer_;
  back_enlarge_buffer_ = enlarge_buffer_;
  if (!speed_planner_common::GetAdcEnlargeBuffer(
          inside_data, inside_data.curr_multi_level, &enlarge_buffer_,
          &front_enlarge_buffer_, &back_enlarge_buffer_)) {
    LOG_WARN("get adc self en_large buffer failed.");
  }
  if (inside_data.curr_scenario_state == ScenarioState::BACK_OUT) {
    enlarge_buffer_ = front_enlarge_buffer_ = back_enlarge_buffer_ = 0.;
  }
  LOG_INFO("current_level, front_buffer, back_buffer: {}, {:.3f}, {:.3f}",
           inside_data.curr_multi_level, front_enlarge_buffer_,
           back_enlarge_buffer_);

  auto& path_adc_boxes =
      outside_data_ptr->motorway_speed_obstacle_context.adc_boundaries;
  auto& path_adc_sl_boundaries =
      outside_data_ptr->motorway_speed_obstacle_context.adc_sl_boundaries;
  if (!BuildPathAdcBoundingBoxes(
          task_info.current_frame()->inside_planner_data(),
          *(outside_data_ptr->path_data), &path_adc_boxes)) {
    LOG_ERROR("BuildPathAdcBoundingBoxes failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!BuildPathAdcAABoxes(task_info.reference_line(),
                           *(outside_data_ptr->path_data),
                           &path_adc_sl_boundaries)) {
    LOG_WARN("BuildPathAdcAABoxes failed.");
  }

  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedPathInfoDecider::ComputePathAccumulatedS(
    const PathData& path_data, std::vector<double>* accumulated_s) const {
  if (accumulated_s == nullptr) return false;
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }

  accumulated_s->clear();
  accumulated_s->push_back(0.);
  for (std::size_t i = 1; i < path_data.path().path_points().size(); ++i) {
    auto& curr_pt = path_data.path().path_points()[i];
    auto& last_pt = path_data.path().path_points()[i - 1];
    accumulated_s->push_back(std::sqrt(std::pow(curr_pt.x() - last_pt.x(), 2) +
                                       std::pow(curr_pt.y() - last_pt.y(), 2)) +
                             accumulated_s->back());
  }
  return true;
}

bool MotorwaySpeedPathInfoDecider::BuildPathAdcBoundingBoxes(
    const InsidePlannerData& inside_data, const PathData& path_data,
    std::vector<Box2d>* path_adc_boxes) const {
  if (path_adc_boxes == nullptr) return false;
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }

  const auto& veh_path = path_data.path().path_points();
  path_adc_boxes->clear();
  for (const auto& path_point : veh_path) {
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {path_point.x(), path_point.y()}, path_point.theta(), enlarge_buffer_,
        front_enlarge_buffer_, back_enlarge_buffer_);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build adc bounding box from path point at s[{}] failed",
                path_point.s());
      return false;
    }
    path_adc_boxes->emplace_back(adc_box);
  }
  if (path_adc_boxes->size() != veh_path.size()) {
    LOG_ERROR("path_adc_boxes_.size({}) != veh_path.size({})",
              path_adc_boxes->size(), veh_path.size());
    return false;
  }
  return true;
}

bool MotorwaySpeedPathInfoDecider::BuildPathAdcAABoxes(
    const ReferenceLinePtr ref_ptr, const PathData& path_data,
    std::vector<Boundary>* adc_sl_boundaries) const {
  if (adc_sl_boundaries == nullptr) return false;
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }

  const auto& veh_path = path_data.path().path_points();
  adc_sl_boundaries->clear();
  for (const auto& path_point : veh_path) {
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {path_point.x(), path_point.y()}, path_point.theta(), enlarge_buffer_,
        front_enlarge_buffer_, back_enlarge_buffer_);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build adc bounding box from path point at s[{}] failed",
                path_point.s());
      continue;
    }
    Boundary boundary{};
    if (!ref_line_util::ComputePolygonBoundary(ref_ptr, Polygon2d(adc_box),
                                               &boundary)) {
      LOG_WARN("compute boundary failed.");
      continue;
    }
    adc_sl_boundaries->emplace_back(boundary);
  }
  if (adc_sl_boundaries->size() != veh_path.size()) {
    LOG_WARN("adc_sl_boundaries->size({}) != veh_path.size({})",
             adc_sl_boundaries->size(), veh_path.size());
  }
  return true;
}

void MotorwaySpeedPathInfoDecider::GetAdcCornerPointCoordinate(
    const InsidePlannerData& inside_data,
    std::vector<Vec2d>& adc_corner_pt_coordinate) const {
  adc_corner_pt_coordinate.clear();
  adc_corner_pt_coordinate.resize(
      static_cast<int>(AdcCollideCornerPoint::NONE));
  double x_g = 0.0;
  double y_g = 0.0;
  double theta_g = 0.0;
  std::vector<Vec2d> adc_points{};
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                -VehicleParam::Instance()->back_edge_to_center(),
                VehicleParam::Instance()->left_edge_to_center(), 0.0, x_g, y_g,
                theta_g);
  adc_corner_pt_coordinate[static_cast<int>(AdcCollideCornerPoint::LEFT_REAR)] =
      std::move(Vec2d(x_g, y_g));
  adc_points.emplace_back(x_g, y_g);
  VisAdcCornerPoint(Vec2d(x_g, y_g), "left_rear");
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                -VehicleParam::Instance()->back_edge_to_center(),
                -VehicleParam::Instance()->right_edge_to_center(), 0.0, x_g,
                y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_REAR)] = std::move(Vec2d(x_g, y_g));
  adc_points.emplace_back(x_g, y_g);
  VisAdcCornerPoint(Vec2d(x_g, y_g), "right_rear");
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                VehicleParam::Instance()->front_edge_to_center(),
                -VehicleParam::Instance()->right_edge_to_center(), 0.0, x_g,
                y_g, theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::RIGHT_FRONT)] = std::move(Vec2d(x_g, y_g));
  adc_points.emplace_back(x_g, y_g);
  VisAdcCornerPoint(Vec2d(x_g, y_g), "right_front");
  vehicle2earth(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                VehicleParam::Instance()->front_edge_to_center(),
                VehicleParam::Instance()->left_edge_to_center(), 0.0, x_g, y_g,
                theta_g);
  adc_corner_pt_coordinate[static_cast<int>(
      AdcCollideCornerPoint::LEFT_FRONT)] = std::move(Vec2d(x_g, y_g));
  adc_points.emplace_back(x_g, y_g);
  VisAdcCornerPoint(Vec2d(x_g, y_g), "left_front");
}

}  // namespace planning
}  // namespace neodrive
