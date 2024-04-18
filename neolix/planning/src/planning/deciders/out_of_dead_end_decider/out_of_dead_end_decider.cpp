#include "src/planning/deciders/out_of_dead_end_decider/out_of_dead_end_decider.h"

#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double kHalfSafeBound = 1.8;
constexpr double kSafeLengthFront = 11;
constexpr double kSafeLengthBack = 3;

using AD2 = std::array<double, 2>;

std::vector<AD2> BuildPolygonWithBounds(ReferenceLinePtr ref_line,
                                        const double start_s,
                                        const double end_s,
                                        const bool is_motor_way) {
  auto& ref_pts = ref_line->ref_points();
  std::vector<AD2> ans{};
  auto si = ref_line_util::BinarySearchIndex(ref_pts, start_s);
  auto ei = ref_line_util::BinarySearchIndex(ref_pts, end_s);
  Vec2d tmp_pt{};
  for (auto i = si; i <= ei; ++i) {  // upper bound
    auto& ref_pt = ref_pts[i];
    if (is_motor_way)
      ref_line->GetPointInCartesianFrame({ref_pt.s(), ref_pt.left_lane_bound()},
                                         &tmp_pt);
    else
      ref_line->GetPointInCartesianFrame({ref_pt.s(), ref_pt.left_road_bound()},
                                         &tmp_pt);
    ans.push_back({tmp_pt.x(), tmp_pt.y()});
  }
  std::reverse(ans.begin(), ans.end());

  double last_road_bound = 0.0;
  for (auto i = si; i <= ei; ++i) {  // lower bound
    auto& ref_pt = ref_pts[i];
    if (is_motor_way && i != si &&
        last_road_bound - ref_pt.right_road_bound() > 1.5)
      ref_line->GetPointInCartesianFrame({ref_pt.s(), -last_road_bound},
                                         &tmp_pt);
    else {
      last_road_bound = ref_pt.right_road_bound();
      ref_line->GetPointInCartesianFrame(
          {ref_pt.s(), -ref_pt.right_road_bound()}, &tmp_pt);
    }
    ans.push_back({tmp_pt.x(), tmp_pt.y()});
  }

  return ans;
}

std::vector<AD2> TransObsToPolygon(const Obstacle& obs) {
  std::vector<AD2> ans{};
  for (auto& p : obs.polygon_corners()) {
    ans.push_back({p.x(), p.y()});
  }
  return ans;
}

void VisPolygons(const std::vector<std::vector<AD2>>& polygons,
                 const std::string& name, const std::array<double, 4>& col) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  event->mutable_color()->set_r(col[0]);
  event->mutable_color()->set_g(col[1]);
  event->mutable_color()->set_b(col[2]);
  event->mutable_color()->set_a(col[3]);
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p[0]), ans->set_y(p[1]), ans->set_z(0);
  };

  for (auto& polygon : polygons) {
    auto plg = event->add_polygon();
    for (auto& p : polygon) {
      set_pt(plg->add_point(), p);
    }
  }
}

void VisPose(const std::array<double, 3> p3, const std::string& name,
             const std::array<double, 4>& col) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  event->mutable_color()->set_r(col[0]);
  event->mutable_color()->set_g(col[1]);
  event->mutable_color()->set_b(col[2]);
  event->mutable_color()->set_a(col[3]);
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p[0]), ans->set_y(p[1]), ans->set_z(0);
  };
  auto txt = event->add_text();
  set_pt(txt->mutable_position(), p3);
  txt->set_text("target point: ( " + std::to_string(p3[0]) + ", " +
                std::to_string(p3[1]) + ")");

  set_pt(event->add_sphere()->mutable_center(), p3);
  auto pl = event->add_polyline();
  set_pt(pl->add_point(), p3);
  AD2 pp{p3[0] + 0.5 * std::cos(p3[2]), p3[1] + 0.5 * std::sin(p3[2])};
  set_pt(pl->add_point(), pp);
}

bool IsMotorway(const ReferenceLinePtr& reference_line, const double s) {
  ReferencePoint pt{};
  if (!reference_line->GetNearestRefPoint(s, &pt)) {
    LOG_INFO("Skip BACK_OUT: GetNearestRefPoint fail");
    return false;
  }
  return pt.lane_type_is_pure_city_driving();
}

}  // namespace

void OutOfDeadEndDecider::SaveTaskResults(TaskInfo& task_info) {
  const auto& back_out_config =
      config::PlanningConfig::Instance()->plan_config().back_out;
  neodrive::global::planning::SpeedLimit internal_speed_limit{};
  internal_speed_limit.set_source_type(SpeedLimitType::FREESPACE);
  internal_speed_limit.add_upper_bounds(back_out_config.max_limit_speed);
  internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
  internal_speed_limit.set_acceleration(0.0);
  LOG_INFO(
      "OutOfDeadEndDecider {} limit speed: speed = "
      "{:.2f}, acc = {:.2f}",
      SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
      back_out_config.max_limit_speed, 0.0);

  data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
      internal_speed_limit);
}

void OutOfDeadEndDecider::Reset() {}

OutOfDeadEndDecider::OutOfDeadEndDecider() { name_ = "OutOfDeadEndDecider"; }

ErrorCode OutOfDeadEndDecider::Execute(TaskInfo& task_info) {
  LOG_INFO("XXX " + name_);
  auto ref_line_ptr = task_info.reference_line();
  auto& obstacles = task_info.decision_data()->static_obstacle();
  const auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  double curr_s = task_info.curr_sl().s(), curr_l = task_info.curr_sl().l();

  std::vector<int> obs_idxes_front{};
  std::vector<int> obs_idxes_back{};
  for (int i = 0, n = obstacles.size(); i < n; ++i) {
    if (obstacles[i]->speed() > 0.01) continue;
    if (!(obstacles[i]->max_l() < -kHalfSafeBound ||
          obstacles[i]->min_l() > kHalfSafeBound)) {
      if (obstacles[i]->max_s() >= task_info.curr_sl().s() &&
          obstacles[i]->max_s() - task_info.curr_sl().s() < 30) {
        obs_idxes_front.push_back(i);
        LOG_INFO("obs id : {}", obstacles[i]->id());
      }
      if (obstacles[i]->min_s() < task_info.curr_sl().s() &&
          obstacles[i]->min_s() - task_info.curr_sl().s() > -30) {
        obs_idxes_back.push_back(i);
      }
    }
  }

  std::sort(obs_idxes_front.begin(), obs_idxes_front.end(),
            [&obstacles](auto& a, auto& b) {
              return obstacles[a]->min_s() < obstacles[b]->min_s();
            });
  std::sort(obs_idxes_back.begin(), obs_idxes_back.end(),
            [&obstacles](auto& a, auto& b) {
              return obstacles[a]->max_s() > obstacles[b]->max_s();
            });

  double safe_s_front = task_info.curr_sl().s();
  for (auto i : obs_idxes_front) {
    if (obstacles[i]->min_s() - safe_s_front < kSafeLengthFront) {
      safe_s_front = obstacles[i]->max_s();
    } else {
      break;
    }
  }

  double safe_s_back = task_info.curr_sl().s();
  for (auto i : obs_idxes_back) {
    if (obstacles[i]->max_s() - safe_s_back > -kSafeLengthBack) {
      safe_s_back = obstacles[i]->min_s();
    } else {
      break;
    }
  }

  const double start_s = safe_s_back - kSafeLengthBack;
  const double end_s = safe_s_front + kSafeLengthFront;
  bool is_motor_way = IsMotorway(ref_line_ptr, curr_s);
  auto polygon =
      BuildPolygonWithBounds(ref_line_ptr, start_s, end_s, is_motor_way);
  VisPolygons({polygon}, "dead_end_freespace", {0.2, 0.3, 0.7, 0.7});

  std::vector<int> valid_idxes{};
  int front_obs_id = -1;
  double lat_buffer = ego_car_config.width / 2 + 0.2;
  double lon_buffer = ego_car_config.front_edge_to_base_link;
  double obs_dis = curr_s + lon_buffer + 8.0;

  LOG_INFO("obs_idxes_front size: {}", obs_idxes_front.size());
  for (auto i : obs_idxes_front) {
    if (obstacles[i]->min_s() > end_s || obstacles[i]->max_s() < start_s)
      continue;
    valid_idxes.push_back(i);
    LOG_INFO("obs min l: {:.4f}, max l: {:.4f}, min s: {:.4f}",
             obstacles[i]->min_l(), obstacles[i]->max_l(),
             obstacles[i]->min_s());
    if (!(obstacles[i]->min_l() > curr_l + lat_buffer ||
          obstacles[i]->max_l() < curr_l - lat_buffer ||
          obstacles[i]->min_s() < curr_s ||
          obstacles[i]->min_s() > curr_s + lon_buffer + 1.5)) {
      if (obstacles[i]->min_s() < obs_dis) {
        obs_dis = obstacles[i]->min_s();
        front_obs_id = i;
      }
    }
    LOG_INFO("front valid_idxes id: {} get", obstacles[i]->id());
  }
  if (front_obs_id >= 0)
    LOG_INFO("front_obs_id: {} get", obstacles[front_obs_id]->id());
  else
    LOG_INFO("no front_obs_id get");

  LOG_INFO("obs_idxes_back size: {}", obs_idxes_back.size());
  for (auto i : obs_idxes_back) {
    if (obstacles[i]->min_s() > end_s || obstacles[i]->max_s() < start_s)
      continue;
    valid_idxes.push_back(i);
    LOG_INFO("back valid_idxes idex: {} get", i);
  }
  std::vector<std::vector<AD2>> holes{};
  for (auto i : valid_idxes) holes.push_back(TransObsToPolygon(*obstacles[i]));
  VisPolygons(holes, "dead_end_holes", {0.8, 0.0, 0.3, 0.9});

  const auto& back_out_config =
      config::PlanningConfig::Instance()->plan_config().back_out;
  double target_length = back_out_config.end_length;
  double tar_s = curr_s + target_length;

  if (target_length > 0) {
    tar_s = std::min(safe_s_front + target_length, end_s - 3);
  } else {
    tar_s = std::max(safe_s_back + target_length, start_s + 1);
  }

  ReferencePoint tar_pt{};
  SLPoint new_target{};
  bool left_pass_obs = true;
  double back_out_end_s = curr_s + target_length;

  if (is_motor_way) {
    auto& ref_pts = ref_line_ptr->ref_points();
    auto& adc_ref_pt =
        ref_pts[ref_line_util::BinarySearchIndex(ref_pts, curr_s)];
    tar_pt = ref_pts[ref_line_util::BinarySearchIndex(ref_pts, tar_s)];
    auto right_road_bound = adc_ref_pt.right_road_bound();
    auto right_lane_bound = adc_ref_pt.right_lane_bound();

    new_target.set_s(tar_s);
    new_target.set_l(-(right_road_bound + right_lane_bound) / 2);
    back_out_end_s = std::max(curr_s + 2.0, tar_s - 2.0);
    left_pass_obs = false;

    Vec2d tmp_pt{};
    ref_line_ptr->GetPointInCartesianFrame(new_target, &tmp_pt);
    tar_pt.set_x(tmp_pt.x());
    tar_pt.set_y(tmp_pt.y());
    tar_pt.set_heading(
        ref_pts[ref_line_util::BinarySearchIndex(ref_pts, new_target.s())]
            .heading());

    data_center_->set_motorway_back_out(true);

    LOG_INFO("Motorway Back Out! target_length: {:.4f}", target_length);
    LOG_INFO("cur_s: {:.4f},safe_s_front: {:.4f}, safe_s_back: {:.4f}", curr_s,
             safe_s_front, safe_s_back);
    LOG_INFO("tar_s: {:.4f}, start_s: {:.4f}, end_s: {:.4f}", tar_s, start_s,
             end_s);
  } else if (obs_idxes_front.empty() || front_obs_id == -1) {
    // No obs ahead, set reference line pt as target
    LOG_INFO("No obs Back Out! target_length: {:.4f}", target_length);
    LOG_INFO("cur_s: {:.4f},safe_s_front: {:.4f}, safe_s_back: {:.4f}", curr_s,
             safe_s_front, safe_s_back);
    LOG_INFO("tar_s: {:.4f}, start_s: {:.4f}, end_s: {:.4f}", tar_s, start_s,
             end_s);
    auto& ref_pts = ref_line_ptr->ref_points();
    tar_pt = ref_pts[ref_line_util::BinarySearchIndex(ref_pts, tar_s)];
    new_target.set_s(tar_pt.s());
    new_target.set_l(0);
    back_out_end_s = std::max(curr_s + 2.0, tar_s - 2.0);
  } else {
    // Set target behind the side of first obs
    SLPoint front_obs = obstacles[front_obs_id]->center_sl();
    new_target.set_s(obstacles[front_obs_id]->max_s() +
                     back_out_config.tar_s_bias);
    back_out_end_s = std::max(
        curr_s + 2.0, std::min(obstacles[front_obs_id]->min_s() +
                                   ego_car_config.back_edge_to_base_link,
                               obstacles[front_obs_id]->center_sl().s()));

    LOG_INFO("obs min s = {:.4f}, center s = {:.4f}",
             obstacles[front_obs_id]->min_s(),
             obstacles[front_obs_id]->center_sl().s());
    double target_l = 0.0;
    auto& ref_pts = ref_line_ptr->ref_points();
    auto& obs_ref_pt =
        ref_pts[ref_line_util::BinarySearchIndex(ref_pts, front_obs.s())];
    auto& adc_ref_pt =
        ref_pts[ref_line_util::BinarySearchIndex(ref_pts, curr_s)];
    double left_freespace_width =
        obs_ref_pt.left_road_bound() - obstacles[front_obs_id]->max_l();
    double right_freespace_width =
        obs_ref_pt.right_road_bound() + obstacles[front_obs_id]->min_l();
    double heading_diff = task_info.adc_point().theta() - adc_ref_pt.heading();

    if (obstacles[front_obs_id]->min_l() > 0) {
      target_l = std::max(std::min((-obs_ref_pt.right_road_bound() +
                                    obstacles[front_obs_id]->min_l()),
                                   -obs_ref_pt.right_road_bound()) /
                              2.0,
                          obstacles[front_obs_id]->min_l() - 1);
      left_pass_obs = false;
    } else if (obstacles[front_obs_id]->max_l() < 0) {
      target_l = std::min(std::max((obs_ref_pt.left_road_bound() +
                                    obstacles[front_obs_id]->max_l()),
                                   obs_ref_pt.left_road_bound()) /
                              2.0,
                          obstacles[front_obs_id]->max_l() + 1);
      left_pass_obs = true;
    } else {
      auto& ref_pts = ref_line_ptr->ref_points();
      tar_pt = ref_pts[ref_line_util::BinarySearchIndex(ref_pts, tar_s)];
      new_target.set_s(tar_s);
      target_l = 0;
    }

    LOG_INFO(
        "obs min l: {:.4f}, max l: {:.4f}, right bound: {:.4f}, "
        "left bound: {:.4f}",
        obstacles[front_obs_id]->min_l(), obstacles[front_obs_id]->max_l(),
        -obs_ref_pt.right_road_bound(), obs_ref_pt.left_road_bound());
    new_target.set_l(target_l);

    Vec2d tmp_pt{};
    ref_line_ptr->GetPointInCartesianFrame(new_target, &tmp_pt);
    tar_pt.set_x(tmp_pt.x());
    tar_pt.set_y(tmp_pt.y());
    tar_pt.set_heading(
        ref_pts[ref_line_util::BinarySearchIndex(ref_pts, new_target.s())]
            .heading());
    LOG_INFO("set plan target near the front obs, s: {:.4f}, l: {:.4f}",
             new_target.s(), new_target.l());
    LOG_INFO("cur_s: {:.4f},safe_s_front: {:.4f}, safe_s_back: {:.4f}", curr_s,
             safe_s_front, safe_s_back);
    LOG_INFO("tar_s: {:.4f}, start_s: {:.4f}, end_s: {:.4f}", tar_s, start_s,
             end_s);
  }

  /// Save result
  auto data = task_info.current_frame()->mutable_inside_planner_data();
  data->curr_scenario_state = ScenarioState::BACK_OUT;
  data->left_pass_obs = left_pass_obs;
  std::swap(data->polygon, polygon);
  std::swap(data->holes, holes);
  auto init_point =
      task_info.current_frame()->planning_data().init_planning_point();
  static int v_stop_cnt = 0;
  v_stop_cnt = std::abs(init_point.velocity()) < 0.1 ? v_stop_cnt + 1 : 0;
  LOG_INFO("velocity: {:.4f}, v_stop_cnt: {}", init_point.velocity(),
           v_stop_cnt);
  auto control_command = data_center_->control_command_msg.ptr;
  static double lateral_error = 0.0;
  static double station_error = 0.0;
  static double heading_error = 0.0;
  if (control_command->has_contrl_context() &&
      control_command->contrl_context().has_lat_ctrl_ctx() &&
      control_command->contrl_context().has_long_ctrl_ctx()) {
    lateral_error = std::abs(
        control_command->contrl_context().lat_ctrl_ctx().lateral_error());
    heading_error = std::abs(
        control_command->contrl_context().lat_ctrl_ctx().heading_error());
    station_error = std::abs(
        control_command->contrl_context().long_ctrl_ctx().station_error());
  } else {
    lateral_error = 0.0;
    station_error = 0.0;
    heading_error = 0.0;
  }

  if (auto& d = task_info.last_frame()->inside_planner_data();
      d.curr_scenario_state == ScenarioState::BACK_OUT &&
      lateral_error < back_out_config.lat_err &&
      heading_error < back_out_config.heading_err &&
      // station_error < back_out_config.station_err &&
      v_stop_cnt < back_out_config.stop_cnt_threshold / 2) {
    data->target_point.set_x(d.target_point.x());
    data->target_point.set_y(d.target_point.y());
    data->target_point.set_theta(d.target_point.theta());
    data->target_sl_point.set_s(d.target_sl_point.s());
    data->target_sl_point.set_l(d.target_sl_point.l());
    data->back_out_end_s = d.back_out_end_s;
    LOG_INFO("XXX Use target last");
  } else {
    data->target_point.set_x(tar_pt.x());
    data->target_point.set_y(tar_pt.y());
    data->target_point.set_theta(tar_pt.heading());
    data->target_sl_point.set_s(new_target.s());
    data->target_sl_point.set_l(new_target.l());
    data->back_out_end_s = back_out_end_s;
    data->is_replan = true;
    LOG_INFO("XXX Use target current");
    v_stop_cnt = 0;
  }
  LOG_INFO(
      "XXX replan.is_replan: {}, v_stop_cnt: {}, "
      "lateral_error: {:.4f}/{:.4f}, heading_error: "
      "{:.4f}/{:.4f}, station_error: {:.4f}/{:.4f}",
      data->is_replan, v_stop_cnt, lateral_error, back_out_config.lat_err,
      heading_error, back_out_config.heading_err, station_error,
      back_out_config.station_err);

  VisPose({data->target_point.x(), data->target_point.y(),
           data->target_point.theta()},
          "dead_end_tar", {1, 0, 1, 1});
  LOG_INFO("target pt ({:.4f}, {:.4f}, {:.4f})", data->target_point.x(),
           data->target_point.y(), data->target_point.theta());
  LOG_INFO("end_s: {:.4f}, target_sl_point ({:.4f}, {:.4f})",
           data->back_out_end_s, data->target_sl_point.s(),
           data->target_sl_point.l());
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
