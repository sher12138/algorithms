#include "speed_unprotected_turn_decider.h"
#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {

namespace {
constexpr double kParamA = 1.0;
constexpr double kParamC = 1.0;
constexpr double kParamMargin = 0.2;

void VisCollisionPoint(double x, double y) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("collision_info");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto polyline = event->mutable_sphere()->Add();
  polyline->mutable_center()->set_x(x);
  polyline->mutable_center()->set_y(y);
  polyline->mutable_center()->set_z(0);
  polyline->set_radius(0.3);
}

void VisEgoLine(const std::vector<double>& x, const std::vector<double>& y) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("collision_info");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto polyline = event->mutable_polyline()->Add();
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  for (int i = 0; i < x.size(); ++i) {
    Vec2d point{x[i], y[i]};
    set_pt(polyline->add_point(), point);
  }
}

void VisObsLine(const ReferenceLinePtr& ref_line, const std::vector<double>& s,
                const std::vector<double>& l) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("collision_info");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto polyline = event->mutable_polyline()->Add();
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  for (int i = 0; i < s.size(); ++i) {
    Vec2d point;
    ref_line->GetPointInCartesianFrame(SLPoint(s[i], l[i]), &point);
    set_pt(polyline->add_point(), point);
  }
}

void VisValidRegion(const TurnRegionInfo& turn_region_info_,
                    const ReferenceLinePtr& ref_line) {
  if (!FLAGS_planning_enable_vis_event) return;
  auto event = vis::EventSender::Instance()->GetEvent("valid_region");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  if (turn_region_info_.turn_direction == 0) {
    return;
  }

  std::vector<Vec2d> points;

  std::vector<double> s_l;
  for (const auto& ref_pt : ref_line->ref_points()) {
    if (ref_pt.s() < turn_region_info_.start_s) continue;
    if (ref_pt.s() > turn_region_info_.end_s) break;
    s_l.push_back(ref_pt.s());
    points.push_back({ref_pt.x(), ref_pt.y()});
  }

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto& p : points) {
    auto polyline = event->mutable_sphere()->Add();
    set_pt(polyline->mutable_center(), p);
    polyline->set_radius(0.3);
  }
}

bool PayOffSolver(const CalAfterCollisionInfo& ego_collision_info,
                  const CalAfterCollisionInfo& obs_collision_info,
                  double& rush_diff) {
  double thw_ego = ego_collision_info.attain_time;

  double thw_agent = obs_collision_info.attain_time;

  double t_pass_ego =
      (ego_collision_info.collision_point_sl.s() -
       ego_collision_info.current_point_sl.s() + ego_collision_info.length) /
      ego_collision_info.speed;
  double t_pass_obs =
      (std::sqrt(std::pow(obs_collision_info.collision_point_xy.x() -
                              obs_collision_info.current_point_xy.x(),
                          2) +
                 std::pow(obs_collision_info.collision_point_xy.y() -
                              obs_collision_info.current_point_xy.y(),
                          2)) +
       obs_collision_info.length) /
      obs_collision_info.speed;
  double ego_rush_and_agent_rush_gains =
      -1.0 / thw_ego - kParamA * kParamC * t_pass_ego;
  double ego_rush_and_agent_yield_gains = 1.0 / thw_ego + kParamA * t_pass_ego;
  double ego_yield_and_agent_rush_gains = 1.0 / thw_ego - kParamA * t_pass_ego;
  double ego_yield_and_agent_yield_gains = 1.0 / thw_ego - kParamA * t_pass_ego;

  double agent_rush_and_ego_rush_gains =
      -1.0 / thw_agent - kParamA * kParamC * t_pass_obs;
  double agent_rush_and_ego_yield_gains =
      1.0 / thw_agent + kParamA * t_pass_obs;
  double agent_yield_and_ego_rush_gains =
      1.0 / thw_agent - kParamA * t_pass_obs;
  double agent_yield_and_ego_yield_gains =
      1.0 / thw_agent - kParamA * t_pass_obs;

  double agent_rush_prob =
      (ego_rush_and_agent_yield_gains - ego_yield_and_agent_yield_gains) /
      (ego_rush_and_agent_yield_gains + ego_yield_and_agent_rush_gains -
       ego_rush_and_agent_rush_gains - ego_yield_and_agent_yield_gains);
  double ego_rush_prob =
      (agent_rush_and_ego_yield_gains - agent_yield_and_ego_yield_gains) /
      (agent_rush_and_ego_yield_gains + agent_yield_and_ego_rush_gains -
       agent_rush_and_ego_rush_gains - agent_yield_and_ego_yield_gains);
  if (agent_rush_prob < 0.0 || ego_rush_prob < 0.0) {
    LOG_ERROR("Game Theory Failed!");
    return false;
  }
  LOG_INFO("ego_rush_prob:{:.3f}, agent_rush_prob:{:.3f}", ego_rush_prob,
           agent_rush_prob);

  rush_diff = (ego_rush_prob - agent_rush_prob - kParamMargin);
  return true;
}

bool GenerateRDecisionSuccess(const CalAfterCollisionInfo& ego_collision_info,
                              const CalAfterCollisionInfo& obs_collision_info) {
  // decision system
  //
  if (ego_collision_info.attain_time > 5 &&
      obs_collision_info.attain_time > 5) {
    LOG_INFO("ego and obs attain time is more than 5s, don't handle.");
    return true;
  }
  if (std::fabs(ego_collision_info.attain_time -
                obs_collision_info.attain_time) > 5) {
    LOG_INFO("ego attain time is more than 5s than obs, don't handle.");
    return true;
  }
  double obs_dis = obs_collision_info.speed * obs_collision_info.attain_time;
  if ((ego_collision_info.collision_point_sl.s() -
       ego_collision_info.current_point_sl.s()) < 0.5 &&
      (obs_dis > 5)) {
    LOG_INFO("obs is rush ego, don't handle.");
    return true;
  }
  double rush_diff = 0.0;
  if (PayOffSolver(ego_collision_info, obs_collision_info, rush_diff)) {
    if (rush_diff > 0.0) {
      LOG_INFO("ego rush prob is more than agent, don't handle.");
      return true;
    }
  }

  return false;
};

bool GenerateObsLine(const Obstacle& obs, const ReferenceLinePtr& ref_line,
                     tk::spline& tk_sp, const double start_x,
                     const double start_y, double& start_s, double& end_s) {
  std::vector<double> s;
  std::vector<double> l;

  for (double i = 0.; i < 8; i += 0.5) {
    SLPoint obs_sl;
    ref_line->GetPointInFrenetFrame(
        Vec2d(start_x + i * std::cos(obs.velocity_heading()) * obs.speed(),
              start_y + i * std::sin(obs.velocity_heading()) * obs.speed()),
        &obs_sl);

    s.push_back(obs_sl.s());
    l.push_back(obs_sl.l());
  }
  if (s.size() < 3) {
    LOG_INFO("s.size() < 2, don't handle.");
    return false;
  }
  tk_sp.set_points(s, l);
  start_s = s.front(), end_s = s.back();
  VisObsLine(ref_line, s, l);
  return true;
};

bool ExtractObsStartX(const TurnRegionInfo& turn_region_info,
                      const Obstacle& obs, double& start_x, double& start_y) {
  const double cos_heading = std::cos(obs.velocity_heading());
  const double sin_heading = std::sin(obs.velocity_heading());
  const double half_length = obs.length() / 2;
  const double half_width = obs.width() / 2;
  const double dx1 = cos_heading * half_length;
  const double dy1 = sin_heading * half_length;
  const double dx2 = sin_heading * half_width;
  const double dy2 = -cos_heading * half_width;
  if (turn_region_info.turn_direction < 0) {
    start_x = obs.center().x() + dx1 + dx2;
    start_y = obs.center().y() + dy1 + dy2;

  } else if (turn_region_info.turn_direction > 0) {
    start_x = obs.center().x() + dx1 - dx2;
    start_y = obs.center().y() + dy1 - dy2;
  } else {
    LOG_INFO("ego turn direction is 0, don't handle.");
    return false;
  }
  return true;
}

bool CalCOllisionPoint(const tk::spline& obs_curva_line_,
                       const ReferenceLinePtr& ref_line,
                       const double ego_start_s_, const double ego_end_s_,
                       const double ego_half_vel_width, SLPoint& collision_sl_p,
                       Vec2d& collision_xy_p, const double turn_direction,
                       const double& start_s, const double& end_s) {
  bool update_left = false, update_right = false;
  double left = -1., right = -1.;

  left = std::min(ego_start_s_, ego_end_s_);
  right = std::max(ego_start_s_, ego_end_s_);
  if (left < start_s) {
    left == start_s;
  }
  if (right > end_s) {
    right == end_s;
  }
  double left_value =
      obs_curva_line_(left) + ego_half_vel_width * turn_direction;
  double right_value =
      obs_curva_line_(right) + ego_half_vel_width * turn_direction;
  if ((left_value) * (right_value) > 0) {
    LOG_INFO("no root, no collision.");
    return false;
  }
  while (right - left > 1e-4) {
    double mid = (left + right) / 2;
    if ((obs_curva_line_(mid) + ego_half_vel_width * turn_direction) *
            left_value >
        0) {
      left = mid;
    } else {
      right = mid;
    }
  }
  collision_sl_p.set_s((left + right) / 2);
  collision_sl_p.set_l(-ego_half_vel_width * turn_direction);
  LOG_INFO(" collision s :{:.3f}", (left + right) / 2);
  if (!ref_line->GetPointInCartesianFrame(collision_sl_p, &collision_xy_p)) {
    LOG_ERROR("can't get collision point sl from ref_line!");
    return false;
  };
  VisCollisionPoint(collision_xy_p.x(), collision_xy_p.y());
  return true;
};

bool GenerateSpeedLimit(const CalAfterCollisionInfo& ego_collision_info,
                        const CalAfterCollisionInfo& obs_collision_info,
                        double& limit_speed) {
  double delta_s = ego_collision_info.collision_point_sl.s() -
                   ego_collision_info.current_point_sl.s() -
                   ego_collision_info.length;
  if (delta_s < 0) {
    LOG_INFO("delta_s {:.3f} is less than 0, don't handle.", delta_s);
    return false;
  }
  if (0. < delta_s && delta_s < 3.) {
    double a =
        0.5 * (ego_collision_info.speed * ego_collision_info.speed) / (delta_s);
    limit_speed = std::sqrt(ego_collision_info.speed - std::min(a * 0.75, 0.3));
    return true;
  }

  double buffer = obs_collision_info.speed * 0.5;
  LOG_INFO("buffer is {:.3f},delta_s - buffer = {:.3f}", buffer,
           delta_s - buffer);

  limit_speed =
      std::max((delta_s - buffer) / obs_collision_info.attain_time, 1.0);
  return true;
}
}  // namespace

SpeedUnprotectedTurnDecider::SpeedUnprotectedTurnDecider() {
  name_ = "SpeedUnprotectedTurnDecider";
}

SpeedUnprotectedTurnDecider::~SpeedUnprotectedTurnDecider() {}

ErrorCode SpeedUnprotectedTurnDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute SpeedUnprotectedTurnDecider");
  if (!Init(task_info)) {
    LOG_INFO("SpeedUnprotectedTurnDecider init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    LOG_INFO("SpeedUnprotectedTurnDecider process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void SpeedUnprotectedTurnDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::UNPROTECTED_TURN);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "LEFT_TURN_AREA {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

bool SpeedUnprotectedTurnDecider::Init(TaskInfo& task_info) {
  adc_current_s_ = task_info.curr_sl().s();
  adc_current_x_ = task_info.adc_point().x();
  adc_current_y_ = task_info.adc_point().y();
  adc_heading_ = task_info.current_frame()->inside_planner_data().vel_heading;
  adc_current_l_ = task_info.curr_sl().l();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  ego_vel_width_ = VehicleParam::Instance()->width();
  ego_to_head_ = VehicleParam::Instance()->front_edge_to_center();
  ego_to_tail_ = VehicleParam::Instance()->back_edge_to_center();

  limited_speed_ = std::numeric_limits<double>::infinity();

  double consider_junctin_start_s = 0.0;
  double consider_junction_end_s = 0.0;

  weather_turn_region_info_available_ = false;
  bool valid = false;
  const auto& ref_line = task_info.reference_line();
  if (last_ref_start_s_ != ref_line->ref_points().front().s()) {
    LOG_INFO("ref_line skip, re-map");
    ReMapBaseRefLine(ref_line);
  }

  if (std::abs(turn_region_info_.start_s - 0.0) < 1e-3 &&
      std::abs(turn_region_info_.end_s - 0.0) < 1e-3) {
    consider_junctin_start_s = adc_current_s_;
    consider_junction_end_s = adc_current_s_ + config_->max_front_distance;
  } else {
    LOG_INFO(
        "history info. turn_start_s:{:.3f}, turn_end_s:{:.3f}, turn "
        "direction:{}",
        turn_region_info_.start_s, turn_region_info_.end_s,
        turn_region_info_.turn_direction);
    if (turn_region_info_.end_s >= adc_current_s_ + 5 &&
        turn_region_info_.turn_direction != 0) {
      weather_turn_region_info_available_ = true;
      LOG_INFO("history region is retional, coninue to use it.");
      LOG_INFO("turn_start_s:{:.3f}, turn_end_s:{:.3f}",
               turn_region_info_.start_s, turn_region_info_.end_s);
      VisValidRegion(turn_region_info_, ref_line);
      last_ref_start_s_ = ref_line->ref_points().front().s();
      return true;
    } else {
      consider_junctin_start_s = adc_current_s_;
      consider_junction_end_s = adc_current_s_ + config_->max_front_distance;
    }
  }

  double max_curva_pos = 0.0;
  double max_curva_nag = 0.0;
  double curva_change_positive = 0.0;
  double curva_change_nagative = 0.0;

  double expend_s_start_pos = 0.0;
  bool s_start_update_pos = false;
  double expend_s_end_pos = 0.0;

  double expend_s_start_nag = 0.0;
  bool s_start_update_nag = false;
  double expend_s_end_nag = 0.0;

  const auto& ref_pts = ref_line->ref_points();
  int n = ref_line->ref_points().size();
  LOG_INFO("consider start s:{:.3f}, end s:{:.3f},adc_current_s:{:.3f}",
           consider_junctin_start_s, consider_junction_end_s, adc_current_s_);
  for (int i = 1; i < n; ++i) {
    if (ref_pts[i].s() < consider_junctin_start_s - 5) {
      continue;
    }
    if (ref_pts[i].s() > consider_junction_end_s + 5) {
      break;
    }

    max_curva_pos = std::max(max_curva_pos, ref_pts[i].kappa());

    if (!s_start_update_pos && ref_pts[i].kappa() > config_->min_curvachange) {
      LOG_INFO("s_start_update_pos:{:.3f}, ref_pts[i].kappa():{:.3f}",
               s_start_update_pos, ref_pts[i].kappa());
      expend_s_start_pos = ref_pts[i].s();
      s_start_update_pos = true;
    }

    if (s_start_update_pos && ref_pts[i].kappa() < config_->min_curvachange) {
      LOG_INFO("s_start_update_pos:{:.3f}, ref_pts[i].kappa():{:.3f}",
               s_start_update_pos, ref_pts[i].kappa());
      expend_s_end_pos = ref_pts[i].s();
      break;
    }

    max_curva_nag = std::min(max_curva_nag, ref_pts[i].kappa());
    if (!s_start_update_nag && ref_pts[i].kappa() < -config_->min_curvachange) {
      LOG_INFO("s_start_update_nag:{:.3f}, ref_pts[i].kappa():{:.3f}",
               s_start_update_nag, ref_pts[i].kappa());
      expend_s_start_nag = ref_pts[i].s();
      s_start_update_nag = true;
    }

    if (s_start_update_nag && ref_pts[i].kappa() > -config_->min_curvachange) {
      LOG_INFO("s_start_update_nag:{:.3f}, ref_pts[i].kappa():{:.3f}",
               s_start_update_nag, ref_pts[i].kappa());
      expend_s_end_nag = ref_pts[i].s();
      break;
    }
  }

  LOG_INFO("max_curva_pos:{:.3f}, max_curva_nag:{:.3f}", max_curva_pos,
           max_curva_nag);

  if (max_curva_pos > config_->kappa_sign) {
    // left
    turn_region_info_.start_s = expend_s_start_pos - 5;
    turn_region_info_.end_s = expend_s_end_pos < expend_s_start_pos
                                  ? consider_junction_end_s
                                  : expend_s_end_pos + 5;
    turn_region_info_.turn_direction = -1;
    turn_region_info_.max_curva = max_curva_pos;
    weather_turn_region_info_available_ = true;

  } else if (max_curva_nag < -config_->kappa_sign) {
    // right
    turn_region_info_.start_s = expend_s_start_nag - 5;
    turn_region_info_.end_s = expend_s_end_nag < expend_s_start_nag
                                  ? consider_junction_end_s
                                  : expend_s_end_nag + 5;
    turn_region_info_.turn_direction = 1;
    turn_region_info_.max_curva = max_curva_nag;
    weather_turn_region_info_available_ = true;
  } else {
    turn_region_info_.turn_direction = 0;
  }

  ref_line->GetPointInCartesianFrame(SLPoint(turn_region_info_.start_s, 0.),
                                     &turn_region_info_.start_vec2d);
  ref_line->GetPointInCartesianFrame(SLPoint(turn_region_info_.end_s, 0.),
                                     &turn_region_info_.end_vec2d);

  LOG_INFO(
      "valid region is available:{}, turn_start_s:{:.3f}, turn_end_s:{:.3f}, "
      "turn_max_curva:{:.3f}",
      weather_turn_region_info_available_, turn_region_info_.start_s,
      turn_region_info_.end_s, turn_region_info_.max_curva);

  VisValidRegion(turn_region_info_, ref_line);
  last_ref_start_s_ = ref_line->ref_points().front().s();
  return true;
}

bool SpeedUnprotectedTurnDecider::Process(TaskInfo& task_info) {
  if (!weather_turn_region_info_available_) {
    LOG_INFO("ego not in turn region, don't handle.");
    return true;
  }
  const auto& curr_stage = DataCenter::Instance()
                               ->master_info()
                               .motorway_intersection_context()
                               .stage;

  if (adc_current_s_ < turn_region_info_.start_s ||
      adc_current_s_ > turn_region_info_.end_s) {
    LOG_INFO("adc_current_s is not in turn region, don't handle.{:.3f}",
             adc_current_s_);
    return true;
  }

  const auto& all_obstacle =
      task_info.current_frame()->planning_data().decision_data().all_obstacle();
  const auto& adc_boundary = task_info.current_frame()
                                 ->outside_planner_data()
                                 .speed_obstacle_context.adc_boundaries;
  const auto& ref_line = task_info.reference_line();

  ego_start_s_ = adc_current_s_;
  ego_end_s_ = adc_current_s_ + config_->max_front_distance;

  std::unordered_set<std::size_t> obs_dealt_set{};
  for (const auto& obs_decision : task_info.current_frame()
                                      ->outside_planner_data()
                                      .motorway_speed_obstacle_context
                                      .multi_cipv_dynamic_obstacles_decision) {
    obs_dealt_set.insert(obs_decision.obstacle.id());
  }

  for (const auto obs : all_obstacle) {
    if (obs == nullptr) {
      continue;
    }
    if (obs->is_static() || obs->is_virtual()) {
      continue;
    }
    double heading_diff =
        normalize_angle(obs->velocity_heading() - adc_heading_);
    if (obs->center_sl().s() > adc_current_s_) {
      LOG_INFO("obs {} is from ego", obs->id());
      continue;
    }
    if (obs_dealt_set.find(obs->id()) != obs_dealt_set.end()) {
      LOG_INFO("obs {} is already dealt", obs->id());
      continue;
    }

    if (!ObsIsInEgoTurnDirection(*obs)) {
      LOG_INFO("obs {} is not in ego turn direction", obs->id());
      continue;
    }

    if (GenerateDecision(ref_line, *obs)) {
      LOG_INFO("Obs {} is in ego turn direction, generate decision.",
               obs->id());
    }
  }

  return true;
}

bool SpeedUnprotectedTurnDecider::ObsIsInEgoTurnDirection(const Obstacle& obs) {
  return (adc_current_l_ - obs.center_sl().l()) *
             turn_region_info_.turn_direction >
         0;
}

bool SpeedUnprotectedTurnDecider::GenerateDecision(
    const ReferenceLinePtr& ref_line, const Obstacle& obs) {
  LOG_INFO("obs id:{} is process !", obs.id());
  if (obs.length() < 1e-4 || obs.width() < 1e-4) {
    LOG_INFO(
        "Obs {} length or width is too small, ignore it.length({:.3f}) < "
        "1e-4 "
        "|| width({:.3f}) < 1e-4,",
        obs.id(), obs.length(), obs.width());
    return false;
  }
  if (obs.speed() < 1e-1) {
    LOG_INFO("Obs {} speed is too small, ignore it.speed({:.3f}) < 1e-1,",
             obs.id(), obs.speed());
    return false;
  }
  double start_x = 0.0, start_y = 0.0;
  if (!ExtractObsStartX(turn_region_info_, obs, start_x, start_y)) {
    LOG_INFO("ExtractObsStartX failed.");
    return false;
  };

  double start_s, end_s;
  if (!GenerateObsLine(obs, ref_line, obs_curva_line_, start_x, start_y,
                       start_s, end_s)) {
    return false;
  }

  SLPoint collision_sl_p;
  Vec2d collision_xy_p;

  if (!CalCOllisionPoint(obs_curva_line_, ref_line, ego_start_s_, ego_end_s_,
                         ego_vel_width_ / 2, collision_sl_p, collision_xy_p,
                         turn_region_info_.turn_direction, start_s, end_s)) {
    LOG_INFO("no root ,skip obs:{}.", obs.id());
    return false;
  };

  double collision_x = collision_xy_p.x();
  double collision_y = collision_xy_p.y();

  double collision_time = std::hypot(collision_x - obs.center().x(),
                                     collision_y - obs.center().y()) /
                          obs.speed();
  double ego_attain_time =
      (collision_sl_p.s() - adc_current_s_ - ego_to_head_) /
      (adc_current_v_ + 1e-6);
  LOG_INFO("ego attain time:{:.3f}", ego_attain_time);
  if (collision_time < 0) {
    LOG_INFO("{},collision_time is less than 0, don't handle.", obs.id());
    return false;
  }

  if ((collision_x - adc_current_x_) * adc_add_x_dir_ < 0) {
    LOG_INFO("{},collision_x is not in front of adc_current_x, don't handle.",
             obs.id());
    return false;
  }
  double collision_dis = std::sqrt(
      (collision_x - adc_current_x_) * (collision_x - adc_current_x_) +
      (collision_y - adc_current_l_) * (collision_y - adc_current_l_));

  CalAfterCollisionInfo ego_collision_info, obs_collision_info;
  ego_collision_info.speed = adc_current_v_;
  ego_collision_info.attain_time = ego_attain_time;
  ego_collision_info.speed_heading = adc_heading_;
  ego_collision_info.width = ego_vel_width_;
  ego_collision_info.length = VehicleParam::Instance()->length();
  ego_collision_info.current_point_xy = {adc_current_x_, adc_current_y_};
  ego_collision_info.current_point_sl = {adc_current_s_, adc_current_l_};
  ego_collision_info.collision_point_sl = collision_sl_p;
  ego_collision_info.collision_point_xy = collision_xy_p;

  obs_collision_info.speed = obs.speed();
  obs_collision_info.attain_time = collision_time;
  obs_collision_info.speed_heading = obs.velocity_heading();
  obs_collision_info.width = obs.width();
  obs_collision_info.length = obs.length();
  obs_collision_info.current_point_xy = {obs.center().x(), obs.center().y()};
  obs_collision_info.current_point_sl = {obs.center_sl().s(),
                                         obs.center_sl().l()};
  obs_collision_info.collision_point_sl = collision_sl_p;
  obs_collision_info.collision_point_xy = collision_xy_p;

  if (GenerateRDecisionSuccess(ego_collision_info, obs_collision_info)) {
    LOG_INFO("ego rush to collision point, don't handle.");
    return true;
  };

  LOG_INFO("EGO INFO:{}", ego_collision_info.Print());
  LOG_INFO("OBS INFO:{}", obs_collision_info.Print());

  double gen_speed{std::numeric_limits<double>::infinity()};
  bool sign =
      GenerateSpeedLimit(ego_collision_info, obs_collision_info, gen_speed);
  if (sign) {
    update_limited_speed_ = true;
    limited_speed_ = std::min(limited_speed_, gen_speed);
    LOG_INFO("generate speed limit:{:.3f}", limited_speed_);
  }

  LOG_INFO("don't update speed:{:.3f}", limited_speed_);
  return true;
}

bool SpeedUnprotectedTurnDecider::VehicleCoordinationBehind(
    const Obstacle& obs) {
  double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};

  earth2vehicle(adc_current_x_, adc_current_y_, adc_heading_, obs.center().x(),
                obs.center().y(), obs.velocity_heading(), obs_local_x,
                obs_local_y, obs_local_heading);
  return obs_local_x < 0.;
}

void SpeedUnprotectedTurnDecider::ReMapBaseRefLine(
    const ReferenceLinePtr& ref_line) {
  auto start_sl = SLPoint();
  auto end_sl = SLPoint();
  ref_line->GetPointInFrenetFrame(turn_region_info_.start_vec2d, &start_sl);
  ref_line->GetPointInFrenetFrame(turn_region_info_.end_vec2d, &end_sl);
  double orin_start_s = turn_region_info_.start_s;
  double orin_end_s = turn_region_info_.end_s;
  turn_region_info_.start_s = start_sl.s();
  turn_region_info_.end_s = end_sl.s();
  LOG_INFO(
      "ref_line re-map,from -->{:.3f}, {:.3f},  to --> turn_start_s:{:.3f}, "
      "turn_end_s:{:.3f}",
      orin_start_s, orin_end_s, turn_region_info_.start_s,
      turn_region_info_.end_s);
}

}  // namespace planning
}  // namespace neodrive
