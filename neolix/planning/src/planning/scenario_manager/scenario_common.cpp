#include "planning/scenario_manager/scenario_common.h"

namespace neodrive {
namespace planning {
namespace scenario_common {

template <typename T>
void MultiFrameCheck(T &frame_check_info, bool use_multi_frame, int horizon,
                     double exp_param) {
  frame_check_info.frame_final_ans = frame_check_info.frame_ans;
  double cur_frame_time = cyber::Time::Now().ToSecond();
  frame_check_info.frame_time = cur_frame_time;
  if (kMultiFrameLogPrint) {
    LOG_INFO("cur_frame_time:{}", frame_check_info.frame_time);
  }
  if (!use_multi_frame) {
    frame_check_info.PrintFrameCheckInfo(frame_check_info.frame_info_sign +
                                         " check result:");
    return;
  }

  // queue design
  static std::deque<T> frame_check_info_vec_static;
  if (!frame_check_info_vec_static.empty()) {
    double rel_time =
        cur_frame_time - frame_check_info_vec_static.back().frame_time;
    if (rel_time > kFrameTimeHighLim) {
      frame_check_info_vec_static.clear();
      LOG_INFO(
          "frame discontinuity, clear queue! last frame time:{}, cur frame "
          "time:{}, relative time:{}",
          frame_check_info_vec_static.back().frame_time, cur_frame_time,
          rel_time);
    } else if (rel_time < kFrameTimeLowLim) {
      LOG_INFO("current frame has already been saved");
      return;
    }
  }
  frame_check_info_vec_static.emplace_back(frame_check_info);
  if (frame_check_info_vec_static.size() > kFrameNums) {
    frame_check_info_vec_static.erase(frame_check_info_vec_static.begin());
  }
  // horizon
  int frame_size = frame_check_info_vec_static.size();
  horizon = std::min(std::clamp(horizon, kMinHorizon, kFrameNums), frame_size);
  if (kMultiFrameLogPrint) {
    LOG_INFO("{} horizon:{}, frame_size:{} ", frame_check_info.frame_info_sign,
             horizon, frame_size);
  }
  // check
  std::vector<double> frame_ans_scores{}, frame_ans_weights{};
  std::vector<bool> frame_ans_bool{};
  for (int i = 0; i < frame_size; ++i) {
    auto &frame_check_info_tmp =
        frame_check_info_vec_static.at(frame_size - 1 - i);
    if (i < horizon) {
      // decision 1
      double frame_score = static_cast<double>(frame_check_info_tmp.frame_ans);
      double frame_weight = std::pow(exp_param, i + 1);
      frame_ans_scores.emplace_back(frame_score);
      frame_ans_weights.emplace_back(frame_weight);
      // decision 2
      frame_ans_bool.emplace_back(frame_check_info_tmp.frame_ans);
    }
  }

  bool use_weight_method = false;
  if (use_weight_method) {
    // decision 1
    double sum_scores = 0.0, sum_weights = 0.0;
    for (int i = 0; i < frame_ans_scores.size(); ++i) {
      if (kMultiFrameLogPrint) {
        LOG_INFO("{} frame {} score:{:.4f}, weight:{:.4f}",
                 frame_check_info.frame_info_sign, i + 1, frame_ans_scores[i],
                 frame_ans_weights[i]);
      }
      sum_scores += frame_ans_scores[i] * frame_ans_weights[i];
      sum_weights += frame_ans_weights[i];
    }
    double final_score = (sum_weights > 1e-5) ? (sum_scores / sum_weights) : 0.;
    int frame_index{0};
    for (int i = 0; i < horizon; ++i) {
      if ((final_score > 0.5 && frame_ans_scores.at(i) > 0.5) ||
          (final_score <= 0.5 && frame_ans_scores.at(i) <= 0.5)) {
        if (kMultiFrameLogPrint) {
          LOG_INFO("{} get frame {} for output",
                   frame_check_info.frame_info_sign, i + 1,
                   frame_check_info.frame_info_sign);
        }
        frame_index = frame_size - 1 - i;
        break;
      }
    }
    frame_check_info = frame_check_info_vec_static.at(frame_index);
    frame_check_info.frame_final_ans = final_score > 0.5;
    frame_check_info_vec_static.back().frame_final_ans =
        frame_check_info.frame_final_ans;
    if (kMultiFrameLogPrint) {
      LOG_INFO("{} current frame score:{:.4f}, final ans:{}",
               frame_check_info.frame_info_sign, final_score,
               frame_check_info.frame_final_ans);
    }
  } else {
    // decision 2
    bool all_true = std::all_of(frame_ans_bool.begin(), frame_ans_bool.end(),
                                [](bool val) { return val; });
    bool all_false = std::all_of(frame_ans_bool.begin(), frame_ans_bool.end(),
                                 [](bool val) { return !val; });
    if (all_true || all_false) {
      frame_check_info.frame_final_ans = all_true;
    } else {
      frame_check_info.frame_final_ans =
          (frame_check_info_vec_static.rbegin() + 1)->frame_final_ans;
    }
    frame_check_info_vec_static.back() = frame_check_info;
    if (kMultiFrameLogPrint) {
      LOG_INFO("{} final ans:{}", frame_check_info.frame_info_sign,
               frame_check_info.frame_final_ans);
    }
  }

  // print
  if (kMultiFrameLogPrint) {
    for (int i = 0; i < frame_size; ++i) {
      auto &frame_check_info_tmp =
          frame_check_info_vec_static.at(frame_size - 1 - i);
      frame_check_info_tmp.PrintFrameCheckInfo(
          frame_check_info.frame_info_sign + " frame " + std::to_string(i + 1));
    }
  }
  frame_check_info_vec_static.back().PrintFrameCheckInfo(
      frame_check_info.frame_info_sign + " check result:");
}

bool FrontMinLaneBound(const ReferencePointVec1d &ref_points,
                       double preview_distance, const SLPoint &curr_sl,
                       double &left_min_bound, double &right_min_bound) {
  left_min_bound = 1000.0;
  right_min_bound = 1000.0;
  auto curr_s = curr_sl.s();
  for (std::size_t i = 0; i < ref_points.size(); ++i) {
    auto ref_pt_s = ref_points[i].s();
    if (ref_pt_s > preview_distance + curr_s) break;
    if (ref_pt_s < curr_s) continue;
    left_min_bound = std::fmin(left_min_bound, ref_points[i].left_lane_bound());
    right_min_bound =
        std::fmin(right_min_bound, ref_points[i].right_lane_bound());
  }
  right_min_bound = -right_min_bound;
  return !(left_min_bound > 100.0 || right_min_bound < -100.0);
}

bool IsFrontLaneTurning(const TaskInfo &task_info) {
  if (task_info.last_frame() == nullptr) return false;

  static constexpr double kLaneTurnKappaThreshold = 0.01;
  const auto &shrink_bounds_info =
      task_info.last_frame()
          ->outside_planner_data()
          .path_context.shrink_path_boundary.path_boundary;
  if (shrink_bounds_info.empty()) return false;
  for (const auto &pt : task_info.reference_line()->ref_points()) {
    if (pt.s() > shrink_bounds_info.back().lower_point.s()) {
      break;
    }
    if (pt.s() >
        task_info.curr_sl().s() +
            task_info.last_frame()->inside_planner_data().distance_to_end) {
      break;
    }
    if (pt.s() <= shrink_bounds_info.front().lower_point.s()) continue;

    if (std::abs(pt.kappa()) >= kLaneTurnKappaThreshold) {
      LOG_INFO("front lane is turning.");
      return true;
    }
  }
  return false;
}

bool IsInJunction(const TaskInfo &task_info) {
  if (task_info.last_frame() == nullptr) return false;

  // need exit bias driving in junction or junction close front.
  const auto &shrink_bounds_info =
      task_info.last_frame()
          ->outside_planner_data()
          .path_context.shrink_path_boundary.path_boundary;
  if (shrink_bounds_info.empty()) return false;
  for (const auto &junction_overlap :
       task_info.reference_line()->junction_overlaps()) {
    if (junction_overlap.start_s > shrink_bounds_info.back().lower_point.s()) {
      break;
    }
    if (shrink_bounds_info.front().lower_point.s() > junction_overlap.start_s &&
        shrink_bounds_info.back().lower_point.s() < junction_overlap.end_s) {
      LOG_INFO("Is In Junction");
      return true;
    }
  }
  return false;
}

void ComputeRefL(TaskInfo &task_info, bool enable_bias_drive) {
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  if (task_info.last_frame() == nullptr) return;

  // get most boundary point
  double preview_distance =
      fmax(FLAGS_planning_trajectory_min_length,
           task_info.last_frame()->inside_planner_data().vel_v * 5.0);
  auto &init_sl_point =
      task_info.last_frame()->inside_planner_data().init_sl_point;
  auto &shrink_bounds_info =
      task_info.last_frame()->outside_planner_data().shrink_half_ego_boundaries;
  if (shrink_bounds_info.empty()) return;
  auto right_most_boundary_point = shrink_bounds_info.front();
  auto left_most_boundary_point = shrink_bounds_info.front();
  for (auto &point : shrink_bounds_info) {
    if (point.lower_point.s() > init_sl_point.s() + preview_distance) break;
    if (point.lower_type != PathRegion::Bound::BoundType::VIR) {
      if (right_most_boundary_point.lower_type ==
              PathRegion::Bound::BoundType::VIR ||
          point.lower_point.l() > right_most_boundary_point.lower_point.l()) {
        right_most_boundary_point = point;
      }
    }
    if (point.upper_type != PathRegion::Bound::BoundType::VIR) {
      if (left_most_boundary_point.upper_type ==
              PathRegion::Bound::BoundType::VIR ||
          point.upper_point.l() < left_most_boundary_point.upper_point.l()) {
        left_most_boundary_point = point;
      }
    }
  }

  // design
  struct MostBoundPair {
    PathRegion::Bound most_bound_pair{};
    int lower_id{-9999}, upper_id{-9999};
    double s{0.0}, lower_l{0.0}, upper_l{0.0};
    double width{0.0};
    bool is_lower_obs{false}, is_upper_obs{false};
    double bias_l{0.0};
    double obs_nudge_dis{0.0};   // only dir side use
    double lane_nudge_dis{0.0};  // both side use
    bool dir;                    // left 0 right 1
    MostBoundPair() = default;
    MostBoundPair(PathRegion::Bound bound_pair, bool consider_dir) {
      most_bound_pair = bound_pair;
      lower_id = bound_pair.lower_id;
      upper_id = bound_pair.upper_id;
      s = bound_pair.lower_point.s();
      lower_l = bound_pair.lower_point.l();
      upper_l = bound_pair.upper_point.l();
      width = upper_l - lower_l;
      is_lower_obs = bound_pair.lower_type == PathRegion::Bound::BoundType::OBS;
      is_upper_obs = bound_pair.upper_type == PathRegion::Bound::BoundType::OBS;
      dir = consider_dir;
    }
  };
  MostBoundPair right_pair(right_most_boundary_point, true);
  MostBoundPair left_pair(left_most_boundary_point, false);

  // get conf
  const auto &conf = config::PlanningConfig::Instance()->plan_config().common;
  bool motorway_driving =
      common::config::CommonConfig::Instance()
          ->drive_strategy_config()
          .enable_motorway &&
      task_info.curr_referline_pt().lane_type_is_pure_city_driving();
  std::unordered_map<Obstacle::ObstacleType, double> motorway_bias_map{
      {Obstacle::ObstacleType::UNKNOWN, conf.motorway_bias_to_obs_bound},
      {Obstacle::ObstacleType::UNKNOWN_MOVABLE,
       conf.motorway_bias_to_obs_bound},
      {Obstacle::ObstacleType::UNKNOWN_UNMOVABLE,
       conf.motorway_bias_to_unmovable_obs_bound},
      {Obstacle::ObstacleType::PEDESTRIAN, conf.motorway_bias_to_obs_bound},
      {Obstacle::ObstacleType::BICYCLE, conf.motorway_bias_to_obs_bound},
      {Obstacle::ObstacleType::VEHICLE, conf.motorway_bias_to_obs_bound}};
  std::unordered_map<Obstacle::ObstacleType, double> non_motorway_bias_map{
      {Obstacle::ObstacleType::UNKNOWN, conf.bias_to_right_obs_bound},
      {Obstacle::ObstacleType::UNKNOWN_MOVABLE, conf.bias_to_right_obs_bound},
      {Obstacle::ObstacleType::UNKNOWN_UNMOVABLE, conf.bias_to_right_obs_bound},
      {Obstacle::ObstacleType::PEDESTRIAN, conf.bias_to_right_obs_bound},
      {Obstacle::ObstacleType::BICYCLE, conf.bias_to_right_obs_bound},
      {Obstacle::ObstacleType::VEHICLE, conf.bias_to_right_obs_bound}};

  // obs bias
  auto get_obs_nudge = [&](int obs_id, bool dir) {
    double basic_bias =
        motorway_driving
            ? motorway_bias_map[Obstacle::ObstacleType::UNKNOWN]
            : non_motorway_bias_map[Obstacle::ObstacleType::UNKNOWN];
    if (obs_id <= 0) {
      LOG_INFO("obs {} is not valid.", obs_id);
      return basic_bias;
    }
    Obstacle *obstacle = nullptr;
    auto ret =
        task_info.decision_data()->get_obstacle_by_id(obs_id, false, &obstacle);
    if (ret != ErrorCode::PLANNING_OK || obstacle == nullptr) {
      LOG_INFO("get obs {} failed.", obs_id);
      return basic_bias;
    }
    double width_ratio = std::clamp(
        obstacle->width() / VehicleParam::Instance()->width(), 1.0, 1.5);
    double bias = motorway_driving ? motorway_bias_map[obstacle->type()]
                                   : non_motorway_bias_map[obstacle->type()];
    double final_bias = bias * width_ratio;
    LOG_INFO(
        "get_{}_obs_nudge: is_motorway:{}, obs {} type:{}, bias:{:.4f}, "
        "width:obs/adc/ratio:{:.4f}/{:.4f}/{:.4f}, final_bias:{:.4f}",
        dir ? "right" : "left", motorway_driving, obs_id,
        static_cast<int>(obstacle->type()), bias, obstacle->width(),
        VehicleParam::Instance()->width(), width_ratio, final_bias);
    return final_bias;
  };
  right_pair.obs_nudge_dis = get_obs_nudge(right_pair.lower_id, right_pair.dir);
  left_pair.obs_nudge_dis = get_obs_nudge(left_pair.upper_id, left_pair.dir);

  // lane bias
  auto get_lane_nudge = [&](MostBoundPair &pair) {
    double cur_space_width = shrink_bounds_info.front().upper_point.l() -
                             shrink_bounds_info.front().lower_point.l();
    double width_ratio = std::clamp(pair.width / cur_space_width, 1.0, 1.5);
    double bias = conf.bias_to_right_lane_bound;
    double final_bias = bias * width_ratio;
    LOG_INFO(
        "get_{}_lane_nudge: bias:{:.4f}, "
        "width:most/cur/ratio:{:.4f}/{:.4f}/{:.4f}, final_bias:{:.4f}",
        pair.dir ? "right" : "left", bias, pair.width, cur_space_width,
        width_ratio, final_bias);
    return final_bias;
  };
  right_pair.lane_nudge_dis = get_lane_nudge(right_pair);
  left_pair.lane_nudge_dis = get_lane_nudge(left_pair);

  // process
  if (enable_bias_drive) {
    if (IsFrontLaneTurning(task_info)) {
      observe_ref_l =
          std::min(0.0, right_pair.lower_l + conf.bias_to_right_in_turn);
    } else if (IsInJunction(task_info)) {
      double init_l = std::min(init_sl_point.l(), 0.0);
      observe_ref_l =
          std::max(init_l, right_pair.lower_l + right_pair.lane_nudge_dis);
    } else {
      if (right_pair.is_lower_obs) {
        observe_ref_l =
            std::clamp(right_pair.lower_l + right_pair.obs_nudge_dis,
                       right_pair.lower_l, right_pair.upper_l);
        LOG_INFO("right side has obs.");
      } else {
        observe_ref_l =
            std::min(right_pair.lower_l + right_pair.lane_nudge_dis, 0.0);
        LOG_INFO("right side doesn't have obs, enable bias drive.");
      }
    }
  } else {
    right_pair.bias_l = std::clamp(
        right_pair.lower_l + right_pair.obs_nudge_dis, 0.0, right_pair.upper_l);
    left_pair.bias_l = std::clamp(left_pair.upper_l - left_pair.obs_nudge_dis,
                                  left_pair.lower_l, 0.0);
    bool right_near_more = (right_pair.s < left_pair.s);
    MostBoundPair pair;
    if (right_pair.is_lower_obs && left_pair.is_upper_obs) {
      pair = right_near_more ? right_pair : left_pair;
      LOG_INFO("both sides has obs, right side near:{}, final choose {}_pair",
               right_near_more, pair.dir ? "right" : "left");
    } else if (right_pair.is_lower_obs) {
      pair = right_pair;
      LOG_INFO("only right side has obs.");
    } else if (left_pair.is_upper_obs) {
      pair = left_pair;
      LOG_INFO("only left side has obs.");
    } else {
      pair = right_near_more ? right_pair : left_pair;
      LOG_INFO("both sides don't have obs, right side near:{}",
               right_near_more);
      if (pair.width < (right_pair.lane_nudge_dis + left_pair.lane_nudge_dis)) {
        pair.bias_l = 0.5 * (right_pair.lower_l + left_pair.upper_l);
        LOG_INFO("\tboth sides are close.");
      } else {
        if (right_pair.lower_l + right_pair.lane_nudge_dis > 0.0) {
          pair.bias_l = right_pair.lower_l + right_pair.lane_nudge_dis;
          LOG_INFO("\tonly right side is close.");
        } else if (left_pair.upper_l - left_pair.lane_nudge_dis < 0.0) {
          pair.bias_l = left_pair.upper_l - left_pair.lane_nudge_dis;
          LOG_INFO("\tonly left side is close.");
        } else {
          pair.bias_l = 0.0;
          LOG_INFO("\tboth sides aren't close, center drive!");
        }
      }
    }
    observe_ref_l = pair.bias_l;
  }

  LOG_INFO(
      "enable bias drive: {}, motorway: {}, r/l pair_obs_nudge_dis: "
      "{:.4f}/{:.4f}, r/l pair_lane_nudge_dis: {:.4f}/{:.4f}, r/l pair_most_l: "
      "{:.4f}/{:.4f}, observe_ref_l: {:.4f}",
      enable_bias_drive, motorway_driving, right_pair.obs_nudge_dis,
      left_pair.obs_nudge_dis, right_pair.lane_nudge_dis,
      left_pair.lane_nudge_dis, right_pair.lower_l, left_pair.upper_l,
      observe_ref_l);
}

void ComputeTargetPoint(TaskInfo &task_info) {
  const auto &observe_ref_l = task_info.current_frame()
                                  ->outside_planner_data()
                                  .path_observe_ref_l_info.observe_ref_l;
  auto &observe_ref_pt = task_info.current_frame()
                             ->mutable_outside_planner_data()
                             ->path_observe_ref_l_info.observe_ref_pt;

  task_info.reference_line()->GetPointInCartesianFrame(
      {task_info.curr_sl().s(), observe_ref_l}, &observe_ref_pt);
  LOG_INFO(
      "adc x: {:.4f}, y: {:.4f}. target sl: s: {:.4f}, l: {:.4f}, "
      "target point: x: {:.4f}, y: {:.4f}",
      task_info.adc_point().x(), task_info.adc_point().y(),
      task_info.curr_sl().s(), observe_ref_l, observe_ref_pt.x(),
      observe_ref_pt.y());
}

void RefLFilter(TaskInfo &task_info) {
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  if (task_info.last_frame() != nullptr) {
    double k = config::PlanningConfig::Instance()
                   ->planning_research_config()
                   .path_observe_ref_decider_config.lane_borrow
                   .lane_borrow_observe_l_filter_ratio;
    double filter_vel =
        config::PlanningConfig::Instance()
            ->planning_research_config()
            .path_observe_ref_decider_config.filter_vhe_v_threshold;
    auto &vel_v = task_info.last_frame()->inside_planner_data().vel_v;
    if (vel_v > filter_vel) {
      k = k / (vel_v / filter_vel);
    }

    bool is_swap_ref_line =
        DataCenter::Instance()->navigation_result().is_swap_ref_line;
    LOG_INFO("is_swap_ref_line: {}", is_swap_ref_line);
    if (is_swap_ref_line) {
      // ref_line swap
      auto &last_ref_pt = task_info.last_frame()
                              ->outside_planner_data()
                              .path_observe_ref_l_info.observe_ref_pt;
      SLPoint sl_pt{};
      task_info.reference_line()->GetPointInFrenetFrame(last_ref_pt, &sl_pt);
      observe_ref_l = (1 - k) * sl_pt.l();
      LOG_INFO(
          "is new ref line. ref l: {:.4f}, target s: {:.4f}, l: {:.4f}, k: "
          "{:.4f}, vel_v: {:.4f}, filter_vel: {:.4f}",
          observe_ref_l, sl_pt.s(), sl_pt.l(), k, vel_v, filter_vel);

    } else {
      // filter
      auto &last_observe_ref_l = task_info.last_frame()
                                     ->outside_planner_data()
                                     .path_observe_ref_l_info.observe_ref_l;
      observe_ref_l = observe_ref_l * k + (1 - k) * last_observe_ref_l;
      LOG_INFO(
          "curr l: {:.4f}, last l: {:.4f}, k: {:.4f}, vel_v: {:.4f}, "
          "filter_vel: {:.4f}",
          observe_ref_l, last_observe_ref_l, k, vel_v, filter_vel);
    }
  }
  ComputeTargetPoint(task_info);
}

bool NearbyMinLaneBound(const ReferencePointVec1d &ref_points,
                        double preview_front_distance,
                        double preview_back_distance, const SLPoint &curr_sl,
                        double &left_min_bound, double &right_min_bound) {
  left_min_bound = 1000.0;
  right_min_bound = 1000.0;
  auto curr_s = curr_sl.s();
  for (std::size_t i = 0; i < ref_points.size(); ++i) {
    auto ref_pt_s = ref_points[i].s();
    if (ref_pt_s > preview_front_distance + curr_s) break;
    if (ref_pt_s < curr_s - preview_back_distance) continue;
    left_min_bound = std::fmin(left_min_bound, ref_points[i].left_lane_bound());
    right_min_bound =
        std::fmin(right_min_bound, ref_points[i].right_lane_bound());
  }
  right_min_bound = -right_min_bound;
  return !(left_min_bound > 100.0 || right_min_bound > 100.0);
}

bool IsDynamicObsClear(const TaskInfo &task_info,
                       const TargetLaneCheckInfo &target_lane_check_info,
                       const double &left_bound, const double &right_bound,
                       std::vector<int> &dynamic_obs_ids,
                       std::vector<Obstacle *> &front_reverse_obs) {
  LOG_INFO("dynamic obs clear check: left_bound:{:.4f}, right_bound:{:.4f}",
           left_bound, right_bound);
  // prepare data
  dynamic_obs_ids.clear();
  front_reverse_obs.clear();
  const auto &decision_data = task_info.decision_data();
  auto &reference_line = task_info.reference_line();
  if (decision_data == nullptr || reference_line == nullptr ||
      reference_line->ref_points().empty()) {
    return false;
  }
  // auto &target_reference_line = task_info.target_ref();
  auto &preview_front_distance = target_lane_check_info.preview_front_distance;
  auto &preview_back_distance = target_lane_check_info.preview_back_distance;
  auto &near_front_distance = target_lane_check_info.near_front_distance;
  auto &near_back_distance = target_lane_check_info.near_back_distance;
  auto &preview_time = target_lane_check_info.preview_time;
  auto &k = config::PlanningConfig::Instance()
                ->planning_research_config()
                .path_observe_ref_decider_config.vhe_v_filter_ratio;

  static double veh_v =
      task_info.last_frame()
          ? task_info.last_frame()->inside_planner_data().vel_v
          : 0.0;
  veh_v = veh_v * (1.0 - k) +
          k * (task_info.last_frame()
                   ? task_info.last_frame()->inside_planner_data().vel_v
                   : 0.0);
  auto &adc_boundary = task_info.adc_boundary();
  for (auto obs_ptr : decision_data->dynamic_obstacle()) {
    if (obs_ptr == nullptr) {
      continue;
    }
    if (obs_ptr->max_s() < adc_boundary.start_s() - preview_back_distance ||
        obs_ptr->min_s() > adc_boundary.end_s() + preview_front_distance) {
      LOG_INFO("obs[{}] too far. max s: {:.4f}, min s: {:.4f}", obs_ptr->id(),
               obs_ptr->max_s(), obs_ptr->min_s());
      LOG_INFO("adc start_s:{} end_s:{}", adc_boundary.start_s(),
               adc_boundary.end_s());
      LOG_INFO("preview_back_distance:{} preview_front_distance:{}",
               preview_back_distance, preview_front_distance);
      continue;
    }
    if (obs_ptr->center_sl().l() < right_bound + 0.1 ||
        obs_ptr->center_sl().l() > left_bound - 0.1) {
      LOG_INFO(
          "obs[{}] not in target lane bound. max l: {:.4f}, min l: "
          "{:.3f},center l: {:.3f}",
          obs_ptr->id(), obs_ptr->max_l(), obs_ptr->min_l(),
          obs_ptr->center_sl().l());
      continue;
    }

    ReferencePoint reference_point;
    if (!reference_line->GetNearestRefPoint(obs_ptr->center_sl().s(),
                                            &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double heading_diff = std::abs(normalize_angle(obs_ptr->velocity_heading() -
                                                   reference_point.heading()));
    if (heading_diff < M_PI / 2.0) {
      if (obs_ptr->max_s() > adc_boundary.start_s() - near_back_distance &&
          obs_ptr->min_s() < adc_boundary.end_s() + near_front_distance) {
        dynamic_obs_ids.push_back(obs_ptr->id());
        LOG_INFO(
            "find near dynamic obs[{}],max l: {:.3f}, min l: {:.3f},center l: "
            "{:.3f}",
            obs_ptr->id(), obs_ptr->max_l(), obs_ptr->min_l(),
            obs_ptr->center_sl().l());
        continue;
      }
      if (heading_diff > config::PlanningConfig::Instance()
                             ->plan_config()
                             .detour_scenario.filter_obs_heading_threshold) {
        LOG_INFO("skip traverse dynamic obs:{},heading diff is{:.2f}",
                 obs_ptr->id(), heading_diff);
        continue;
      }
      if (obs_ptr->min_s() > adc_boundary.end_s() + near_front_distance &&
          obs_ptr->speed() > veh_v) {
        LOG_INFO(
            "skip front high speed obs[{}]: obs speed: {:.2f}, veh_v: "
            "{:.4f}",
            obs_ptr->id(), obs_ptr->speed(), veh_v);
        continue;
      }
      if (obs_ptr->max_s() < adc_boundary.start_s() - near_back_distance &&
          obs_ptr->speed() < veh_v) {
        LOG_INFO(
            "skip back low speed obs[{}]: obs speed: {:.2f}, veh_v: {:.2f}",
            obs_ptr->id(), obs_ptr->speed(), veh_v);
        continue;
      }
      if (obs_ptr->min_s() > adc_boundary.end_s() + near_front_distance &&
          obs_ptr->speed() < veh_v) {
        double delt_s =
            obs_ptr->min_s() - near_front_distance - adc_boundary.end_s();
        double delt_v = veh_v - obs_ptr->speed();
        double delt_t = delt_s / delt_v;
        if (delt_t < preview_time) {
          dynamic_obs_ids.push_back(obs_ptr->id());
          LOG_INFO(
              "find front obs[{}]: delt_s: {:.3f}, delt_v: {:.3f}, delt_t: "
              "{:.3f},max l: {:.3f}, min l: {:.3f},center l: {:.3f}",
              obs_ptr->id(), delt_s, delt_v, delt_t, obs_ptr->max_l(),
              obs_ptr->min_l(), obs_ptr->center_sl().l());
          continue;
        }
      }
      if (obs_ptr->max_s() < adc_boundary.start_s() - near_back_distance &&
          obs_ptr->speed() > veh_v) {
        double delt_s =
            adc_boundary.start_s() - near_back_distance - obs_ptr->max_s();
        double delt_v = obs_ptr->speed() - veh_v;
        double delt_t = delt_s / delt_v;
        if (delt_t < preview_time) {
          dynamic_obs_ids.push_back(obs_ptr->id());
          LOG_INFO(
              "find back dangerous obs[{}]: delt_s: {:.3f}, delt_v: {:.2f}, "
              "delt_t: "
              "{:.3f},max l: {:.3f}, min l: {:.3f}, center l: {:.3f}",
              obs_ptr->id(), delt_s, delt_v, delt_t, obs_ptr->max_l(),
              obs_ptr->min_l(), obs_ptr->center_sl().l());
          continue;
        }
      }
    } else {
      LOG_INFO("Find reverse obs[{}]", obs_ptr->id());
      if (obs_ptr->min_s() < adc_boundary.end_s() + near_front_distance &&
          obs_ptr->max_s() > adc_boundary.start_s()) {
        front_reverse_obs.push_back(obs_ptr);
        dynamic_obs_ids.push_back(obs_ptr->id());
        LOG_INFO(
            "find near reversed dynamic obs[{}],max l: {:.3f}, min l: "
            "{:.3f},,center l: {:.3f}",
            obs_ptr->id(), obs_ptr->max_l(), obs_ptr->min_l(),
            obs_ptr->center_sl().l());
        continue;
      }
      if (M_PI - heading_diff >
          config::PlanningConfig::Instance()
              ->plan_config()
              .detour_scenario.filter_obs_heading_threshold) {
        LOG_INFO("skip reverse traverse dynamic obs:{},heading diff is{:.2f}",
                 obs_ptr->id(), heading_diff);
        continue;
      }
      if (obs_ptr->max_s() < adc_boundary.start_s()) {
        LOG_INFO(
            "reverse obs :[{}] has already passed by: obs speed: {:.3f}, "
            "veh_v: "
            "{:.3f}",
            obs_ptr->id(), obs_ptr->speed(), veh_v);
        continue;
      }
      if (obs_ptr->max_s() > adc_boundary.start_s()) {
        dynamic_obs_ids.push_back(obs_ptr->id());
        front_reverse_obs.push_back(obs_ptr);
        LOG_INFO(
            "front has a reverse obs :[{}], obs speed: {:.3f}, veh_v: "
            "{:.3f},max l: {:.3f}, min l: {:.3f},center l: {:.3f}",
            obs_ptr->id(), obs_ptr->speed(), veh_v, obs_ptr->max_l(),
            obs_ptr->min_l(), obs_ptr->center_sl().l());
        continue;
      }
    }
  }
  if (!dynamic_obs_ids.empty()) {
    return false;
  }
  LOG_INFO("target lane is clear.");
  return true;
}

bool IsLeftDynamicObsClear(const TaskInfo &task_info,
                           const TargetLaneCheckInfo &target_lane_check_info,
                           const double &left_bound, const double &right_bound,
                           std::vector<int> &dynamic_obs_ids,
                           std::vector<Obstacle *> &front_reverse_obs,
                           bool use_multi_frame) {
  std::string info_sign = "LEFT dynamic obs clear";
  LOG_INFO("*** {} check: left_bound:{:.4f}, right_bound:{:.4f}", info_sign,
           left_bound, right_bound);
  // reset output
  dynamic_obs_ids.clear();
  front_reverse_obs.clear();

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    std::vector<int> frame_dynamic_obs_ids;
    FRAME_OBS frame_front_reverse_obs{};

    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
      if (frame_dynamic_obs_ids.empty()) {
        LOG_DEBUG("\t check dynamic obs is empty!");
      } else {
        for (auto obs_id : frame_dynamic_obs_ids) {
          LOG_INFO("\t check dynamic obs: id:{}", obs_id);
        }
      }
      if (frame_front_reverse_obs.empty()) {
        LOG_DEBUG("\t check front reverse obs is empty!");
      } else {
        for (auto obs_ptr : frame_front_reverse_obs) {
          LOG_INFO(
              "\t check front reverse obs: id:{}, type:{}, speed:{:.4f}, "
              "time_stamp:{}, frame_cnt:{}, center_sl:({:.4f},{:.4f})",
              obs_ptr->id(), static_cast<int>(obs_ptr->type()),
              obs_ptr->speed(), obs_ptr->get_time_stamp(), obs_ptr->frame_cnt(),
              obs_ptr->center_sl().s(), obs_ptr->center_sl().l());
        }
      }
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsDynamicObsClear(task_info, target_lane_check_info, left_bound,
                        right_bound, dynamic_obs_ids, front_reverse_obs);
  frame_check_info.frame_dynamic_obs_ids = dynamic_obs_ids;
  frame_check_info.frame_front_reverse_obs = front_reverse_obs;

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_dynamic_obs_clear_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  dynamic_obs_ids = frame_check_info.frame_dynamic_obs_ids;
  front_reverse_obs = frame_check_info.frame_front_reverse_obs;
  return frame_check_info.frame_final_ans;
}

bool IsRightDynamicObsClear(const TaskInfo &task_info,
                            const TargetLaneCheckInfo &target_lane_check_info,
                            const double &left_bound, const double &right_bound,
                            std::vector<int> &dynamic_obs_ids,
                            std::vector<Obstacle *> &front_reverse_obs,
                            bool use_multi_frame) {
  std::string info_sign = "RIGHT dynamic obs clear";
  LOG_INFO("*** {} check: left_bound:{:.4f}, right_bound:{:.4f}", info_sign,
           left_bound, right_bound);
  // reset output
  dynamic_obs_ids.clear();
  front_reverse_obs.clear();

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    std::vector<int> frame_dynamic_obs_ids;
    FRAME_OBS frame_front_reverse_obs{};

    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
      if (frame_dynamic_obs_ids.empty()) {
        LOG_DEBUG("\t check dynamic obs is empty!");
      } else {
        for (auto obs_id : frame_dynamic_obs_ids) {
          LOG_INFO("\t check dynamic obs: id:{}", obs_id);
        }
      }
      if (frame_front_reverse_obs.empty()) {
        LOG_DEBUG("\t check front reverse obs is empty!");
      } else {
        for (auto obs_ptr : frame_front_reverse_obs) {
          LOG_INFO(
              "\t check front reverse obs: id:{}, type:{}, speed:{:.4f}, "
              "time_stamp:{}, frame_cnt:{}, center_sl:({:.4f},{:.4f})",
              obs_ptr->id(), static_cast<int>(obs_ptr->type()),
              obs_ptr->speed(), obs_ptr->get_time_stamp(), obs_ptr->frame_cnt(),
              obs_ptr->center_sl().s(), obs_ptr->center_sl().l());
        }
      }
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsDynamicObsClear(task_info, target_lane_check_info, left_bound,
                        right_bound, dynamic_obs_ids, front_reverse_obs);
  frame_check_info.frame_dynamic_obs_ids = dynamic_obs_ids;
  frame_check_info.frame_front_reverse_obs = front_reverse_obs;

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_dynamic_obs_clear_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  dynamic_obs_ids = frame_check_info.frame_dynamic_obs_ids;
  front_reverse_obs = frame_check_info.frame_front_reverse_obs;
  return frame_check_info.frame_final_ans;
}

bool CanCrossLane(const DividerFeature::DividerType &type,
                  const DividerFeature::DividerColor &color) {
  bool is_cross = false;
  switch (type) {
    case DividerFeature::DividerType::LONG_DASHED_LINE:
    case DividerFeature::DividerType::DOUBLE_SOLID_LINE:
    case DividerFeature::DividerType::SINGLE_SOLID_LINE:
    case DividerFeature::DividerType::SOLID_LINE_DASHED_LINE:
    case DividerFeature::DividerType::DASHED_LINE_SOLID_LINE:
    case DividerFeature::DividerType::SHORT_DASHED_LINE:
    case DividerFeature::DividerType::DOUBLE_DASHED_LINE:
      if (color == DividerFeature::DividerColor::WHITE ||
          color == DividerFeature::DividerColor::YELLOW) {
        is_cross = true;
      }
      break;
    default:
      break;
  }
  return is_cross;
}

bool ComputePreviewRoadWidth(const TaskInfo &task_info,
                             const double preview_distance, double &width) {
  // check forward distance.
  std::size_t start_index{0}, end_index{0};
  if (!task_info.reference_line()->GetStartEndIndexBySLength(
          task_info.curr_referline_pt().s(), preview_distance, &start_index,
          &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return false;
  }
  // compute average width.
  double average_width = 0;
  for (std::size_t i = start_index; i <= end_index; ++i) {
    average_width +=
        task_info.reference_line()->ref_points()[i].left_lane_bound() +
        task_info.reference_line()->ref_points()[i].right_lane_bound();
  }
  average_width /= (std::max(1., 1. * (end_index - start_index)));
  width = average_width;
  LOG_INFO("road width: {}", average_width);
  return true;
}

double ComputeObstacleSpeedDecision(double veh_v, bool motorway_driving) {
  double fast_vel = 0.0;
  if (motorway_driving) {
    fast_vel = (DataCenter::Instance()->master_info().curr_scenario() ==
                    ScenarioState::DETOUR ||
                DataCenter::Instance()->master_info().curr_scenario() ==
                    ScenarioState::MOTORWAY_DETOUR)
                   ? 0.5
                   : 0.1;
  } else {
    fast_vel = DataCenter::Instance()->master_info().curr_scenario() ==
                       ScenarioState::DETOUR
                   ? 1.0
                   : 0.5;
  }
  fast_vel = std::min(veh_v, fast_vel);
  LOG_INFO("filt obs velocity: {:.3f}, veh_v: {:.3f}", fast_vel, veh_v);
  return fast_vel;
}

bool IsCloseStaticDetour(const TaskInfo &task_info) {
  auto &decision_data = task_info.decision_data();
  auto &adc_boundary = task_info.adc_boundary();
  double vel_v = task_info.last_frame()
                     ? task_info.last_frame()->inside_planner_data().vel_v
                     : 0.0;
  double back_attention_distance = 10.0;
  double front_attention_distance = std::max(vel_v * 5.0, 30.0);
  double follow_dis = 0.3, width_buffer = 0.4;
  std::vector<Obstacle *> front_potential_obs;
  std::vector<Obstacle *> back_potential_obs;
  bool mid_drive = std::abs(task_info.curr_sl().l()) <
                   VehicleParam::Instance()->width() * 0.5;
  bool left_half_road = task_info.curr_sl().l() > 0;
  for (auto &obs_ptr : decision_data->all_obstacle()) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual()) {
      continue;
    }
    if ((obs_ptr->max_s() + back_attention_distance < adc_boundary.start_s()) ||
        adc_boundary.end_s() + front_attention_distance < obs_ptr->min_s()) {
      continue;
    }
    if (obs_ptr->type() == Obstacle::ObstacleType::UNKNOWN_UNMOVABLE) {
      continue;
    }
    auto boundary = obs_ptr->PolygonBoundary();
    if ((left_half_road &&
         (boundary.start_l() > adc_boundary.end_l() + width_buffer ||
          boundary.end_l() <
              -VehicleParam::Instance()->width() - width_buffer)) ||
        (!left_half_road &&
         (boundary.end_l() < adc_boundary.start_l() - width_buffer ||
          boundary.start_l() >
              VehicleParam::Instance()->width() + width_buffer))) {
      continue;
    }
    if (obs_ptr->max_s() < adc_boundary.start_s()) {
      LOG_INFO("take obs [{}] into back consideration.", obs_ptr->id());
      back_potential_obs.push_back(obs_ptr);
    } else if (obs_ptr->min_s() > adc_boundary.end_s()) {
      LOG_INFO("take obs [{}] into front consideration.", obs_ptr->id());
      front_potential_obs.push_back(obs_ptr);
    } else {
      LOG_INFO("not close static detour due to nearby obs [{}].",
               obs_ptr->id());
      return false;
    }
  }
  auto IsReverseObstacle = [&](Obstacle *obs) {
    if (obs->is_static()) return false;
    ReferencePoint reference_point;
    task_info.reference_line()->GetNearestRefPoint(obs->center_sl().s(),
                                                   &reference_point);
    double heading_diff = std::abs(
        normalize_angle(obs->velocity_heading() - reference_point.heading()));
    return heading_diff > M_PI / 2.0;
  };
  std::sort(front_potential_obs.begin(), front_potential_obs.end(),
            [&](Obstacle *a, Obstacle *b) { return a->min_s() < b->min_s(); });
  std::sort(back_potential_obs.begin(), back_potential_obs.end(),
            [&](Obstacle *a, Obstacle *b) { return a->max_s() > b->max_s(); });
  if (!front_potential_obs.empty()) {
    double range_end_s = front_potential_obs.front()->max_s();
    for (auto &obs : front_potential_obs) {
      if (obs->min_s() > range_end_s) break;
      range_end_s = std::max(range_end_s, obs->max_s());
      auto boundary = obs->PolygonBoundary();
      if (obs->is_static() &&
          ((boundary.start_l() > 0.0 &&
            boundary.start_l() <
                0.5 * VehicleParam::Instance()->width() + width_buffer) ||
           (boundary.end_l() < 0.0 &&
            boundary.end_l() >
                -0.5 * VehicleParam::Instance()->width() - width_buffer))) {
        LOG_INFO("not close static detour due to front close obs [{}]",
                 obs->id());
        return false;
      }
      if (mid_drive) continue;
      // ttc check
      if (obs->speed() > vel_v) continue;
      double delta_s = obs->min_s() - adc_boundary.end_s();
      double deceleration = -1.0, buffer_t = 0.2;
      if (delta_s < follow_dis) {
        LOG_INFO("not close static detour due to front near obs [{}].",
                 obs->id());
        return false;
      }
      if (IsReverseObstacle(obs)) {
        // oppsite direction
        double veh_s =
            vel_v * buffer_t +
            (std::pow(0.0, 2) - std::pow(vel_v, 2)) / (2 * deceleration);
        double obs_s = obs->speed() * (-vel_v / deceleration + buffer_t);
        if (veh_s + obs_s > delta_s - follow_dis) {
          LOG_INFO(
              "not close static detour due to front quick reverse obs [{}].",
              obs->id());
          return false;
        }
      } else {
        // same direction
        double veh_s = vel_v * buffer_t +
                       (std::pow(obs->speed(), 2) - std::pow(vel_v, 2)) /
                           (2 * deceleration);
        double obs_s =
            obs->speed() * ((obs->speed() - vel_v) / deceleration + buffer_t);
        if (veh_s - obs_s > delta_s - follow_dis) {
          LOG_INFO(
              "vehicle v: {:.3f}, s: {:.3f}, obstalce v: {:.3f}, s: {:.3f}, "
              "delta "
              "s: {:.3f}",
              vel_v, veh_s, obs->speed(), obs_s, delta_s);
          LOG_INFO("not close static detour due to front slow obs [{}].",
                   obs->id());
          return false;
        }
      }
    }
  }
  if (!back_potential_obs.empty()) {
    double range_start_s = back_potential_obs.front()->min_s();
    for (auto &obs : back_potential_obs) {
      if (obs->max_s() > range_start_s) break;
      range_start_s = std::min(range_start_s, obs->min_s());
      if (mid_drive) continue;
      // ttc check
      if (obs->is_static() || obs->speed() < vel_v || IsReverseObstacle(obs))
        continue;
      if ((task_info.curr_sl().l() > 0 &&
           obs->max_l() < adc_boundary.start_l()) ||
          (task_info.curr_sl().l() < 0 &&
           obs->min_l() > adc_boundary.end_l())) {
        double delta_s = adc_boundary.start_s() - obs->max_s();
        if (delta_s < follow_dis) {
          LOG_INFO("not close static detour due to back near obs [{}].",
                   obs->id());
          return false;
        }
        double predict_t = 5.0;
        if ((obs->speed() - vel_v) * predict_t > delta_s - follow_dis) {
          LOG_INFO("not close static detour due to back high speed obs [{}].",
                   obs->id());
          return false;
        }
      }
    }
  }
  return true;
}

bool IsJunction(const TaskInfo &task_info, uint64_t &junction_id,
                const double preview_distance) {
  for (const auto &junction_overlap :
       task_info.reference_line()->junction_overlaps()) {
    if (junction_overlap.object_id != 0 &&
        (task_info.curr_sl().s() + preview_distance >
             junction_overlap.start_s &&
         task_info.curr_sl().s() < junction_overlap.end_s)) {
      junction_id = junction_overlap.object_id;
      return true;
    }
  }
  return false;
}

bool IsCertainJunction(const TaskInfo &task_info, const uint64_t junction_id,
                       const double preview_distance) {
  for (const auto &junction_overlap :
       task_info.reference_line()->junction_overlaps()) {
    if (junction_overlap.object_id == junction_id &&
        (task_info.curr_sl().s() + preview_distance >
             junction_overlap.start_s &&
         task_info.curr_sl().s() < junction_overlap.end_s)) {
      return true;
    }
  }
  return false;
}

bool IsFrontHasTrafficLight(const TaskInfo &task_info,
                            double preview_distance) {
  const auto &signals = task_info.reference_line()->signal_overlaps();
  if (signals.empty()) {
    LOG_INFO("front has not signals.");
    return false;
  }
  for (const auto &signal : signals) {
    if (task_info.curr_sl().s() > signal.end_s ||
        task_info.curr_sl().s() + preview_distance < signal.start_s) {
      continue;
    }
    LOG_INFO("front has signal. signal id: {}, start_s: {:.4f}, curr_s: {:.4f}",
             signal.object_id, signal.start_s, task_info.curr_sl().s());
    return true;
  }
  LOG_INFO("front has not signals.");
  return false;
}

bool IsRightTurnHasTrafficLight(const TaskInfo &task_info,
                                const double preview_distance,
                                bool road_queued) {
  if (!road_queued) {
    LOG_INFO("Go through right turn directly.");
    return false;
  }
  if (!task_info.curr_referline_pt().is_no_signal()) {
    return false;
  }
  bool is_right_turn{false};
  ReferencePoint right_turn_pt;
  const auto &ref_points = task_info.reference_line()->ref_points();
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
    if (ref_points[i].is_right_signal()) {
      is_right_turn = true;
      break;
    }
    right_turn_pt = ref_points[i];
  }
  if (is_right_turn) {
    std::vector<uint64_t> next_lanes;
    PlanningMap::Instance()->GetSuccessorLanes(right_turn_pt.hd_map_lane_id(),
                                               next_lanes);
    auto curr_lane_info = cyberverse::HDMap::Instance()->GetLaneById(
        right_turn_pt.hd_map_lane_id());
    double lane_end_s = right_turn_pt.s() + (curr_lane_info->TotalLength() -
                                             right_turn_pt.hd_map_lane_s());
    for (auto &lane : next_lanes) {
      auto lane_info = cyberverse::HDMap::Instance()->GetLaneById(lane);
      if (static_cast<Lane::TurningType>(lane_info->TurnType()) ==
          Lane::TurningType::RIGHT_TURN) {
        continue;
      }
      auto &signal_overlap = lane_info->Signals();
      if (signal_overlap.empty()) {
        continue;
      }
      for (const auto &signal : signal_overlap) {
        double start_s = lane_end_s + signal.start_s;
        double end_s = lane_end_s + signal.end_s;
        if (task_info.curr_sl().s() > end_s ||
            task_info.curr_sl().s() + preview_distance < start_s) {
          continue;
        }
        LOG_INFO(
            "right turn has straight signal. signal id: {}, start_s: {:.4f}, "
            "curr_s: "
            "{:.4f}",
            signal.object_id, start_s, task_info.curr_sl().s());
        return true;
      }
    }
  }
  LOG_INFO("right turn front has no straight signals.");
  return false;
}
bool IsFrontHasTrafficLight(const TaskInfo &task_info, double preview_distance,
                            bool use_multi_frame) {
  std::string info_sign = "front traffic light";
  LOG_INFO("*** {} check: preview_distance:{:.4f}", info_sign,
           preview_distance);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsFrontHasTrafficLight(task_info, preview_distance);

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_front_has_traffic_light_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsNearTrafficJunction(const ReferenceLinePtr &reference_line,
                           const double preview_distance, const double curr_s) {
  using JunctionType = autobot::cyberverse::Junction::JunctionType;

  for (const auto &overlap : reference_line->junction_overlaps()) {
    cyberverse::JunctionInfoConstPtr junction_ptr;
    PlanningMap::Instance()->GetJunctionById(overlap.object_id, junction_ptr);
    // if (junction_ptr->Type() !=
    // static_cast<uint32_t>(JunctionType::CROSS_ROAD) &&
    //     junction_ptr->Type() !=
    //     static_cast<uint32_t>(JunctionType::IN_ROAD))
    if (junction_ptr->Type() != static_cast<uint32_t>(JunctionType::CROSS_ROAD))
      continue;
    if (overlap.start_s <= curr_s + preview_distance &&
        curr_s <= overlap.end_s) {
      return true;
    }
  }
  return false;
}

bool IsTargetLineAdjacent() { return true; }  // todo

bool IsAdcOnRefLine(const TaskInfo &task_info) {
  double width_threshold = VehicleParam::Instance()->width() * 0.5;
  return ((task_info.curr_sl().l() >
           task_info.curr_referline_pt().left_lane_bound() - width_threshold) ||
          (task_info.curr_sl().l() <
           -task_info.curr_referline_pt().right_lane_bound() + width_threshold))
             ? false
             : true;
}

bool IsTCrossRoadJunctionScenario(const TaskInfo &task_info,
                                  double preview_distance) {
  const auto &junction_list = task_info.reference_line()->junctions();
  double adc_current_s_ = task_info.curr_sl().s();
  for (const auto &[junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }
    if (junction_ptr->Type() ==
        static_cast<uint32_t>(
            autobot::cyberverse::Junction::JunctionType::T_CROSS_ROAD)) {
      if (task_info.curr_sl().s() + preview_distance > overlap.start_s &&
          task_info.curr_sl().s() < overlap.end_s + preview_distance) {
        return true;
        LOG_INFO("T_CROSS_ROAD junction found");
      }
    }
  }
  return false;
}

bool IsFrontRoadQueued(const TaskInfo &task_info, double preview_distance,
                       double left_bound, double right_bound) {
  std::vector<Obstacle *> front_blocked_obs;
  if (scenario_common::IsFrontBlockingObsClear(
          task_info, left_bound, right_bound,
          task_info.curr_sl().s() + preview_distance, front_blocked_obs)) {
    LOG_INFO("front queued obs empty.");
    return false;
  }
  return scenario_common::IsRoadQueued(task_info, front_blocked_obs);
}
bool IsFrontHasCrossRoad(const TaskInfo &task_info, double preview_distance,
                         double ignore_distance) {
  const auto &junction_list = task_info.reference_line()->junctions();
  if (junction_list.empty()) {
    LOG_INFO("front has not junction_list.");
    return false;
  }
  for (const auto &[junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }
    if (junction_ptr->Type() ==
        static_cast<uint32_t>(
            autobot::cyberverse::Junction::JunctionType::CROSS_ROAD)) {
      ComputeCrossRoadIgnoreDistance(task_info, overlap.start_s, overlap.end_s,
                                     ignore_distance);
      if (task_info.curr_sl().s() +
                  std::min(ignore_distance,
                           (overlap.end_s - overlap.start_s) / 2.0) >
              overlap.end_s ||
          task_info.curr_sl().s() + preview_distance < overlap.start_s) {
        continue;
      }
      bool is_ignore{false};
      if (!common::config::CommonConfig::Instance()
               ->drive_strategy_config()
               .enable_motorway ||
          task_info.curr_referline_pt().lane_type_is_biking()) {
        for (double start_s = overlap.start_s; start_s <= overlap.end_s;
             start_s += kMinDistance) {
          ReferencePoint ref_pt;
          task_info.reference_line()->GetNearestRefPoint(start_s, &ref_pt);
          if (ref_pt.is_right_signal()) {
            LOG_INFO("right turn in cross road.");
            is_ignore = true;
            break;
          }
        }
      }
      if (is_ignore) continue;
      LOG_INFO("find cross road, id: {}. start s: {:.4f}, end s: {:.4f}",
               junction_ptr->Id(), overlap.start_s, overlap.end_s);
      return true;
    }
  }
  return false;
}

bool IsFrontHasCrossRoad(const TaskInfo &task_info, double preview_distance,
                         double ignore_distance, bool use_multi_frame) {
  std::string info_sign = "front cross road";
  LOG_INFO("*** {} check: preview_distance:{:.4f}, ignore_distance:{:.4f}",
           info_sign, preview_distance, ignore_distance);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsFrontHasCrossRoad(task_info, preview_distance, ignore_distance);

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_front_has_cross_road_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsFrontHasRoadBoundary(const TaskInfo &task_info, double preview_distance,
                            double ignore_distance) {
  auto &ref_points = task_info.reference_line()->ref_points();
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    // ignore too close.
    if (ref_points[i].s() < task_info.curr_sl().s() + ignore_distance) {
      continue;
    }
    if (std::fabs(ref_points[i].left_road_bound() -
                  ref_points[i].left_lane_bound()) < 0.3 &&
        std::fabs(ref_points[i].right_road_bound() -
                  ref_points[i].right_lane_bound()) < 0.3) {
      LOG_INFO("front has road boundary.");
      return true;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
  }
  LOG_INFO("front has not road boundary.");
  return false;
}

bool IsFrontHasRoadBoundary(const TaskInfo &task_info, double preview_distance,
                            double ignore_distance, bool use_multi_frame) {
  std::string info_sign = "front road boundary";
  LOG_INFO("*** {} check: preview_distance:{:.4f}, ignore_distance:{:.4f}",
           info_sign, preview_distance, ignore_distance);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsFrontHasRoadBoundary(task_info, preview_distance, ignore_distance);

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_front_has_cross_road_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsLeftFrontHasRoadBoundary(const TaskInfo &task_info,
                                double preview_distance,
                                double ignore_distance) {
  auto &ref_points = task_info.reference_line()->ref_points();
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    // ignore too close.
    if (ref_points[i].s() < task_info.curr_sl().s() + ignore_distance) {
      continue;
    }
    if (std::fabs(ref_points[i].left_road_bound() -
                  ref_points[i].left_lane_bound()) < 0.3) {
      LOG_INFO("left front has road boundary.");
      return true;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
  }
  LOG_INFO("left front has not road boundary.");
  return false;
}

bool IsLeftFrontHasRoadBoundary(const TaskInfo &task_info,
                                double preview_distance, double ignore_distance,
                                bool use_multi_frame) {
  std::string info_sign = "left front road boundary";
  LOG_INFO("*** {} check: preview_distance:{:.4f}, ignore_distance:{:.4f}",
           info_sign, preview_distance, ignore_distance);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsLeftFrontHasRoadBoundary(task_info, preview_distance, ignore_distance);

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_left_front_has_road_boundary_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsRightFrontHasRoadBoundary(const TaskInfo &task_info,
                                 double preview_distance,
                                 double ignore_distance) {
  auto &ref_points = task_info.reference_line()->ref_points();

  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    // ignore too close.
    if (ref_points[i].s() < task_info.curr_sl().s() + ignore_distance) {
      continue;
    }
    if (std::fabs(ref_points[i].right_road_bound() -
                  ref_points[i].right_lane_bound()) < 0.3) {
      LOG_INFO("right front has road boundary.");
      return true;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
  }
  LOG_INFO("right front has not road boundary.");
  return false;
}

bool IsRightFrontHasRoadBoundary(const TaskInfo &task_info,
                                 double preview_distance,
                                 double ignore_distance, bool use_multi_frame) {
  std::string info_sign = "right front road boundary";
  LOG_INFO("*** {} check: preview_distance:{:.4f}, ignore_distance:{:.4f}",
           info_sign, preview_distance, ignore_distance);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsRightFrontHasRoadBoundary(task_info, preview_distance, ignore_distance);

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_right_front_has_road_boundary_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsRightFirstLane(const TaskInfo &task_info, double preview_distance,
                      double ignore_distance) {
  auto &ref_points = task_info.reference_line()->ref_points();

  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    // ignore too close.
    if (ref_points[i].s() < task_info.curr_sl().s() + ignore_distance) {
      continue;
    }
    if (ref_points[i].is_right_lane() ||
        (std::fabs(ref_points[i].right_road_bound() -
                   ref_points[i].right_lane_bound()) < 0.3)) {
      LOG_INFO("current lane is right lane {}.", ref_points[i].is_right_lane());
      return true;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
  }
  LOG_INFO("current lane is not right lane.");
  return false;
}

bool IsRightFirstLane(const TaskInfo &task_info, double preview_distance,
                      double ignore_distance, bool use_multi_frame) {
  std::string info_sign = "right first lane";
  LOG_INFO("*** {} check: preview_distance:{:.4f}, ignore_distance:{:.4f}",
           info_sign, preview_distance, ignore_distance);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsRightFirstLane(task_info, preview_distance, ignore_distance);

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_in_right_first_line_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsFrontHasLaneTurn(const TaskInfo &task_info, double preview_distance,
                        double ignore_distance, bool ignore_right_turn) {
  auto &ref_points = task_info.reference_line()->ref_points();
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    if (ref_points[i].s() < task_info.curr_sl().s() + ignore_distance) {
      continue;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
    if (ref_points[i].is_uturn_signal() || ref_points[i].is_left_signal() ||
        (!ignore_right_turn && ref_points[i].is_right_signal())) {
      LOG_INFO("front has lane turn, could not lane borrow.");
      return true;
    }
  }
  LOG_INFO("front has not lane turn.");
  return false;
}

bool IsFrontHasLaneTurn(const TaskInfo &task_info, double preview_distance,
                        double ignore_distance, bool ignore_right_turn,
                        bool use_multi_frame) {
  std::string info_sign = "front lane turn";
  LOG_INFO(
      "*** {} check: preview_distance:{:.4f}, ignore_distance:{:.4f}, "
      "ignore_right_turn:{}",
      info_sign, preview_distance, ignore_distance, ignore_right_turn);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO("{} frame_ans:{}, frame_time:{}, final_ans:{}", sign, frame_ans,
               frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans = IsFrontHasLaneTurn(
      task_info, preview_distance, ignore_distance, ignore_right_turn);

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_front_has_lane_turn_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsFrontHasUTurn(const TaskInfo &task_info, double preview_distance,
                     double ignore_distance) {
  auto &ref_points = task_info.reference_line()->ref_points();
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    if (ref_points[i].s() < task_info.curr_sl().s() + ignore_distance) {
      continue;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
    if (ref_points[i].is_uturn_signal()) {
      LOG_INFO("front has u-turn.");
      return true;
    }
  }
  LOG_INFO("front has not u-turn.");
  return false;
}

bool IsFrontStaticObsClear(const TaskInfo &task_info, const double left_bound,
                           const double right_bound, const double preview_s,
                           double &detour_obs_end_s, int &obs_id,
                           std::vector<Obstacle *> &detour_obs) {
  LOG_INFO(
      "front static obs clear check: left_bound:{:.4f}, right_bound:{:.4f}",
      left_bound, right_bound);
  detour_obs.clear();
  auto &reference_line = task_info.reference_line();
  obs_id = -1;
  std::vector<ObstacleBoundary> obs_valid_boundary;
  double fast_vel = ComputeObstacleSpeedDecision(
      task_info.last_frame() == nullptr
          ? 0.0
          : task_info.last_frame()->inside_planner_data().vel_v,
      task_info.curr_referline_pt().lane_type_is_pure_city_driving());
  auto &decision_data = task_info.decision_data();
  for (auto &obs_ptr : decision_data->all_obstacle()) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual()) {
      // invalid static obstacle.
      continue;
    }
    if (!obs_ptr->is_static() && obs_ptr->speed() > fast_vel + 1e-10) {
      // high speed obstacle
      LOG_DEBUG("skip highspeed obs:{}", obs_ptr->id());
      continue;
    }
    auto &tmp_boundary = obs_ptr->PolygonBoundary();
    if (tmp_boundary.end_s() <
            task_info.adc_boundary().start_s() + kMathEpsilon ||
        tmp_boundary.start_s() > preview_s) {
      continue;
    }
    if (tmp_boundary.end_l() < right_bound ||
        tmp_boundary.start_l() > left_bound) {
      continue;
    }
    ReferencePoint reference_point;
    if (!reference_line->GetNearestRefPoint(obs_ptr->center_sl().s(),
                                            &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double heading_diff = std::abs(normalize_angle(obs_ptr->velocity_heading() -
                                                   reference_point.heading()));
    if (heading_diff > config::PlanningConfig::Instance()
                           ->plan_config()
                           .detour_scenario.filter_obs_heading_threshold &&
        M_PI - heading_diff >
            config::PlanningConfig::Instance()
                ->plan_config()
                .detour_scenario.filter_obs_heading_threshold &&
        !obs_ptr->is_static()) {
      LOG_INFO("skip traverse obs:{}", obs_ptr->id());
      continue;
    }
    obs_valid_boundary.emplace_back(
        ObstacleBoundary{obs_ptr->PolygonBoundary(), obs_ptr});

    // TEST
    LOG_DEBUG(
        "tmp_boundary, id[{}], s_s, e_s, s_l, e_l: {:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}",
        obs_ptr->id(), tmp_boundary.start_s(), tmp_boundary.end_s(),
        tmp_boundary.start_l(), tmp_boundary.end_l());
  }
  if (obs_valid_boundary.empty()) {
    LOG_INFO("obs_valid_boundary is empty.");
    return true;
  }

  std::sort(obs_valid_boundary.begin(), obs_valid_boundary.end(),
            [](const auto &a, const auto &b) {
              return a.first.start_s() < b.first.start_s();
            });

  std::vector<std::pair<double, double>> spaces{};
  if (!Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound, right_bound,
                                   spaces,
                                   VehicleParam::Instance()->width() + 0.2,
                                   VehicleParam::Instance()->length() + 0.2)) {
    LOG_ERROR("CalcLeftRightSpace failed.");
    return false;
  }
  if (spaces.size() != obs_valid_boundary.size()) {
    LOG_ERROR("spaces.size({}) != obs_valid_boundary.size({})", spaces.size(),
              obs_valid_boundary.size());
    return false;
  }
  const double remain_dis =
      task_info.curr_referline_pt().lane_type_is_pure_city_driving()
          ? VehicleParam::Instance()->width() +
                config::PlanningConfig::Instance()
                    ->plan_config()
                    .motorway_detour_scenario.detour_triggered_space
          : VehicleParam::Instance()->width() +
                config::PlanningConfig::Instance()
                    ->plan_config()
                    .detour_scenario.detour_triggered_space;
  for (std::size_t i = 0; i < spaces.size(); ++i) {
    LOG_DEBUG("obs {}, right/left space: {:.4f}, {:.4f}",
              obs_valid_boundary[i].second->id(), spaces[i].first,
              spaces[i].second);
    if (spaces[i].first >= remain_dis || spaces[i].second >= remain_dis) {
      continue;
    }
    obs_id = obs_valid_boundary[i].second->id();
    detour_obs_end_s = obs_valid_boundary[i].first.end_s();
    detour_obs.push_back(obs_valid_boundary[i].second);
    LOG_INFO(
        "obs {} is too large to nudge, right/left space: {:.4f} / {:.4f}, "
        "threshold: {:.4f}",
        obs_valid_boundary[i].second->id(), spaces[i].first, spaces[i].second,
        remain_dis);
  }
  LOG_INFO("obs {} is end to detour, end_s: {:.4f}", obs_id, detour_obs_end_s);
  return obs_id < 0;
}

bool IsFrontBlockingObsClear(const TaskInfo &task_info, const double left_bound,
                             const double right_bound, const double preview_s,
                             std::vector<Obstacle *> &detour_obs) {
  LOG_INFO(
      "front blocking obs clear check: left_bound:{:.4f}, right_bound:{:.4f}",
      left_bound, right_bound);
  detour_obs.clear();
  auto &reference_line = task_info.reference_line();
  std::vector<ObstacleBoundary> obs_valid_boundary;
  double fast_vel = ComputeObstacleSpeedDecision(
      task_info.last_frame() == nullptr
          ? 0.0
          : task_info.last_frame()->inside_planner_data().vel_v,
      task_info.curr_referline_pt().lane_type_is_pure_city_driving());
  auto &decision_data = task_info.decision_data();
  for (auto &obs_ptr : decision_data->all_obstacle()) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual()) {
      // invalid static obstacle.
      continue;
    }
    if (obs_ptr->type() != Obstacle::ObstacleType::VEHICLE) {
      continue;
    }
    auto &tmp_boundary = obs_ptr->PolygonBoundary();
    if (tmp_boundary.end_s() <
            task_info.adc_boundary().start_s() + kMathEpsilon ||
        tmp_boundary.start_s() > preview_s) {
      continue;
    }
    if (tmp_boundary.end_l() < right_bound ||
        tmp_boundary.start_l() > left_bound) {
      continue;
    }
    if (!obs_ptr->is_static() &&
        (!IsNonMovableObstacle(task_info, obs_ptr, 10.0) &&
         obs_ptr->speed() > fast_vel + 1e-10)) {
      // high speed obstacle
      LOG_DEBUG("skip highspeed obs:{}", obs_ptr->id());
      continue;
    }
    ReferencePoint reference_point;
    if (!reference_line->GetNearestRefPoint(obs_ptr->center_sl().s(),
                                            &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double heading_diff = std::abs(normalize_angle(obs_ptr->velocity_heading() -
                                                   reference_point.heading()));
    if (heading_diff > config::PlanningConfig::Instance()
                           ->plan_config()
                           .detour_scenario.filter_obs_heading_threshold &&
        M_PI - heading_diff >
            config::PlanningConfig::Instance()
                ->plan_config()
                .detour_scenario.filter_obs_heading_threshold &&
        !obs_ptr->is_static()) {
      LOG_INFO("skip traverse obs:{}", obs_ptr->id());
      continue;
    }
    obs_valid_boundary.emplace_back(
        ObstacleBoundary{obs_ptr->PolygonBoundary(), obs_ptr});

    // TEST
    LOG_DEBUG(
        "tmp_boundary, id[{}], s_s, e_s, s_l, e_l: {:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}",
        obs_ptr->id(), tmp_boundary.start_s(), tmp_boundary.end_s(),
        tmp_boundary.start_l(), tmp_boundary.end_l());
  }
  if (obs_valid_boundary.empty()) {
    LOG_INFO("obs_valid_boundary is empty.");
    return true;
  }

  std::sort(obs_valid_boundary.begin(), obs_valid_boundary.end(),
            [](const auto &a, const auto &b) {
              return a.first.start_s() < b.first.start_s();
            });

  std::vector<std::pair<double, double>> spaces{};
  if (!Utility::CalcLeftRightSpace(obs_valid_boundary, left_bound, right_bound,
                                   spaces,
                                   VehicleParam::Instance()->width() + 0.2,
                                   VehicleParam::Instance()->length() + 0.2)) {
    LOG_ERROR("CalcLeftRightSpace failed.");
    return false;
  }
  if (spaces.size() != obs_valid_boundary.size()) {
    LOG_ERROR("spaces.size({}) != obs_valid_boundary.size({})", spaces.size(),
              obs_valid_boundary.size());
    return false;
  }
  const double remain_dis =
      task_info.curr_referline_pt().lane_type_is_pure_city_driving()
          ? VehicleParam::Instance()->width() +
                config::PlanningConfig::Instance()
                    ->plan_config()
                    .motorway_detour_scenario.detour_triggered_space
          : VehicleParam::Instance()->width() +
                config::PlanningConfig::Instance()
                    ->plan_config()
                    .detour_scenario.detour_triggered_space;
  for (std::size_t i = 0; i < spaces.size(); ++i) {
    LOG_DEBUG("blocking obs {}, right/left space: {:.4f}, {:.4f}",
              obs_valid_boundary[i].second->id(), spaces[i].first,
              spaces[i].second);
    if (spaces[i].first >= remain_dis || spaces[i].second >= remain_dis) {
      continue;
    }
    detour_obs.push_back(obs_valid_boundary[i].second);
    LOG_INFO(
        "blocking obs {} is too large to nudge, right/left space: {:.4f} / "
        "{:.4f}, "
        "threshold: {:.4f}",
        obs_valid_boundary[i].second->id(), spaces[i].first, spaces[i].second,
        remain_dis);
  }
  return detour_obs.empty();
}

bool IsMiddleFrontStaticObsClear(
    const TaskInfo &task_info, const double left_bound,
    const double right_bound, const double preview_s, double &detour_obs_end_s,
    int &obs_id, std::vector<Obstacle *> &detour_obs, bool use_multi_frame) {
  std::string info_sign = "MIDDLE front static obs clear";
  LOG_INFO("*** {} check: left_bound:{:.4f}, right_bound:{:.4f}", info_sign,
           left_bound, right_bound);

  // reset output
  detour_obs_end_s = -1.;
  obs_id = -1;
  detour_obs.clear();

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    double frame_detour_obs_end_s{-1.};
    int frame_obs_id{-1};
    FRAME_OBS frame_detour_obs{};

    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO(
          "{} frame_obs_id:{}, frame_detour_obs_end_s:{:.4f}, frame_ans:{}, "
          "frame_time:{}, final_ans:{}",
          sign, frame_obs_id, frame_detour_obs_end_s, frame_ans, frame_time,
          frame_final_ans);
      if (frame_detour_obs.empty()) {
        LOG_DEBUG("\t check obs is empty!");
      } else {
        for (auto obs_ptr : frame_detour_obs) {
          LOG_INFO(
              "\t check obs: id:{}, type:{}, speed:{:.4f}, time_stamp:{}, "
              "frame_cnt:{}, center_sl:({:.4f},{:.4f})",
              obs_ptr->id(), static_cast<int>(obs_ptr->type()),
              obs_ptr->speed(), obs_ptr->get_time_stamp(), obs_ptr->frame_cnt(),
              obs_ptr->center_sl().s(), obs_ptr->center_sl().l());
        }
      }
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsFrontStaticObsClear(task_info, left_bound, right_bound, preview_s,
                            detour_obs_end_s, obs_id, detour_obs);
  frame_check_info.frame_detour_obs_end_s = detour_obs_end_s;
  frame_check_info.frame_obs_id = obs_id;
  frame_check_info.frame_detour_obs = detour_obs;

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_front_static_obs_clear_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  detour_obs_end_s = frame_check_info.frame_detour_obs_end_s;
  obs_id = frame_check_info.frame_obs_id;
  detour_obs = frame_check_info.frame_detour_obs;
  return frame_check_info.frame_final_ans;
}

bool IsLeftFrontStaticObsClear(const TaskInfo &task_info,
                               const double left_bound,
                               const double right_bound, const double preview_s,
                               double &detour_obs_end_s, int &obs_id,
                               std::vector<Obstacle *> &detour_obs,
                               bool use_multi_frame) {
  std::string info_sign = "LEFT front static obs clear";
  LOG_INFO("*** {} check: left_bound:{:.4f}, right_bound:{:.4f}", info_sign,
           left_bound, right_bound);

  // reset output
  detour_obs_end_s = -1.;
  obs_id = -1;
  detour_obs.clear();

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    double frame_detour_obs_end_s{-1.};
    int frame_obs_id{-1};
    FRAME_OBS frame_detour_obs{};

    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO(
          "{} frame_obs_id:{}, frame_detour_obs_end_s:{:.4f}, frame_ans:{}, "
          "frame_time:{}, final_ans:{}",
          sign, frame_obs_id, frame_detour_obs_end_s, frame_ans, frame_time,
          frame_final_ans);
      if (frame_detour_obs.empty()) {
        LOG_DEBUG("\t check obs is empty!");
      } else {
        for (auto obs_ptr : frame_detour_obs) {
          LOG_INFO(
              "\t check obs: id:{}, type:{}, speed:{:.4f}, time_stamp:{}, "
              "frame_cnt:{}, center_sl:({:.4f},{:.4f})",
              obs_ptr->id(), static_cast<int>(obs_ptr->type()),
              obs_ptr->speed(), obs_ptr->get_time_stamp(), obs_ptr->frame_cnt(),
              obs_ptr->center_sl().s(), obs_ptr->center_sl().l());
        }
      }
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsFrontStaticObsClear(task_info, left_bound, right_bound, preview_s,
                            detour_obs_end_s, obs_id, detour_obs);
  frame_check_info.frame_detour_obs_end_s = detour_obs_end_s;
  frame_check_info.frame_obs_id = obs_id;
  frame_check_info.frame_detour_obs = detour_obs;

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_front_static_obs_clear_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  detour_obs_end_s = frame_check_info.frame_detour_obs_end_s;
  obs_id = frame_check_info.frame_obs_id;
  detour_obs = frame_check_info.frame_detour_obs;
  return frame_check_info.frame_final_ans;
}

bool IsRightFrontStaticObsClear(
    const TaskInfo &task_info, const double left_bound,
    const double right_bound, const double preview_s, double &detour_obs_end_s,
    int &obs_id, std::vector<Obstacle *> &detour_obs, bool use_multi_frame) {
  std::string info_sign = "RIGHT front static obs clear";
  LOG_INFO("*** {} check: left_bound:{:.4f}, right_bound:{:.4f}", info_sign,
           left_bound, right_bound);

  // reset output
  detour_obs_end_s = -1.;
  obs_id = -1;
  detour_obs.clear();

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    double frame_detour_obs_end_s{-1.};
    int frame_obs_id{-1};
    FRAME_OBS frame_detour_obs{};

    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO(
          "{} frame_obs_id:{}, frame_detour_obs_end_s:{:.4f}, frame_ans:{}, "
          "frame_time:{}, final_ans:{}",
          sign, frame_obs_id, frame_detour_obs_end_s, frame_ans, frame_time,
          frame_final_ans);
      if (frame_detour_obs.empty()) {
        LOG_DEBUG("\t check obs is empty!");
      } else {
        for (auto obs_ptr : frame_detour_obs) {
          LOG_INFO(
              "\t check obs: id:{}, type:{}, speed:{:.4f}, time_stamp:{}, "
              "frame_cnt:{}, center_sl:({:.4f},{:.4f})",
              obs_ptr->id(), static_cast<int>(obs_ptr->type()),
              obs_ptr->speed(), obs_ptr->get_time_stamp(), obs_ptr->frame_cnt(),
              obs_ptr->center_sl().s(), obs_ptr->center_sl().l());
        }
      }
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans =
      IsFrontStaticObsClear(task_info, left_bound, right_bound, preview_s,
                            detour_obs_end_s, obs_id, detour_obs);
  frame_check_info.frame_detour_obs_end_s = detour_obs_end_s;
  frame_check_info.frame_obs_id = obs_id;
  frame_check_info.frame_detour_obs = detour_obs;

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_front_static_obs_clear_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  detour_obs_end_s = frame_check_info.frame_detour_obs_end_s;
  obs_id = frame_check_info.frame_obs_id;
  detour_obs = frame_check_info.frame_detour_obs;
  return frame_check_info.frame_final_ans;
}

bool IsNonMovableObstacle(const TaskInfo &task_info, const Obstacle *obstacle,
                          double obstacle_distance) {
  auto &decision_data = task_info.decision_data();
  if (decision_data == nullptr) {
    return false;
  }
  if (obstacle->type() != Obstacle::ObstacleType::VEHICLE) {
    return false;
  }
  for (auto &other_obs_ptr : decision_data->all_obstacle()) {
    if (other_obs_ptr == nullptr || other_obs_ptr->is_virtual()) {
      // invalid static obstacle.
      continue;
    }
    if (other_obs_ptr->id() == obstacle->id()) {
      continue;
    }
    if (other_obs_ptr->type() != Obstacle::ObstacleType::VEHICLE) {
      continue;
    }
    if (other_obs_ptr->PolygonBoundary().start_s() -
            obstacle->PolygonBoundary().end_s() >
        obstacle_distance) {
      continue;
    }
    if (other_obs_ptr->PolygonBoundary().start_l() >
            obstacle->PolygonBoundary().end_l() ||
        other_obs_ptr->PolygonBoundary().end_l() <
            obstacle->PolygonBoundary().start_l()) {
      // not blocking the backside vehicle
      continue;
    }
    double delta_s = other_obs_ptr->PolygonBoundary().start_s() -
                     obstacle->PolygonBoundary().end_s();
    if (delta_s < 0.0 || delta_s > obstacle_distance) {
      continue;
    }
    LOG_INFO("obstacle [{}] is blocked by other obstacle [{}].", obstacle->id(),
             other_obs_ptr->id());
    return true;
  }
  return false;
}

bool IsAllowedDetourInReverseLane(const TaskInfo &task_info,
                                  double preview_distance,
                                  double ignore_distance) {
  auto reverse_lane_detour_context =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  reverse_lane_detour_context->Reset();

  bool enable_reverse_lane_detour =
      task_info.curr_referline_pt().lane_type_is_pure_city_driving()
          ? config::PlanningConfig::Instance()
                ->plan_config()
                .reverse_lane_detour.enable_reverse_lane_detour_motorway
          : config::PlanningConfig::Instance()
                ->plan_config()
                .reverse_lane_detour.enable_reverse_lane_detour_non_motorway;
  if (!enable_reverse_lane_detour) return false;

  // check 1
  reverse_lane_detour_context->is_leftmost_lane =
      task_info.curr_referline_pt().is_left_lane();

  auto &ref_points = task_info.reference_line()->ref_points();
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    auto &ref_pt = ref_points[i];
    if (ref_pt.s() < task_info.curr_sl().s() + ignore_distance) {
      continue;
    }

    // prepare
    BoundaryEdgeType left_boundary_edge_type = ref_pt.left_boundary_edge_type();
    LOG_DEBUG("left_boundary_edge_type:{}", left_boundary_edge_type);
    std::vector<DividerFeature> left_divider_feature =
        ref_pt.left_divider_feature();
    double left_road_bound = ref_pt.left_road_bound(),
           left_reverse_road_bound = ref_pt.left_reverse_road_bound();
    // check 2
    reverse_lane_detour_context->is_left_bound_allowed_cross =
        (left_boundary_edge_type == BoundaryEdgeType::MARKING);
    // check 3
    int cross_cnt = 0;
    for (int i = 0; i < left_divider_feature.size(); ++i) {
      DividerFeature divider_feature = left_divider_feature[i];
      if (CanCrossLane(divider_feature.divider_type_,
                       divider_feature.divider_color_))
        cross_cnt++;
    }
    reverse_lane_detour_context->is_left_divider_allowed_cross =
        (cross_cnt == left_divider_feature.size());
    // check 4
    reverse_lane_detour_context->is_reverse_lane_allowed_cross =
        (std::abs(left_road_bound - left_reverse_road_bound) >= 1e-5);

    if (ref_pt.s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }

    if (reverse_lane_detour_context->is_leftmost_lane &&
        reverse_lane_detour_context->is_left_bound_allowed_cross &&
        reverse_lane_detour_context->is_left_divider_allowed_cross &&
        reverse_lane_detour_context->is_reverse_lane_allowed_cross) {
      continue;
    } else {
      LOG_INFO(
          "reverse lane detour is not allowed, ignore_distance:{:.4f}, "
          "preview_distance:{:.4f}, check_fail_distance:{:.4f}, "
          "is_leftmost_lane:{}, is_left_bound_allowed_cross:{}, "
          "is_left_divider_allowed_cross:{}, is_reverse_lane_allowed_cross:{}",
          ignore_distance, preview_distance,
          ref_pt.s() - task_info.curr_sl().s(),
          reverse_lane_detour_context->is_leftmost_lane,
          reverse_lane_detour_context->is_left_bound_allowed_cross,
          reverse_lane_detour_context->is_left_divider_allowed_cross,
          reverse_lane_detour_context->is_reverse_lane_allowed_cross);
      return false;
    }
  }
  LOG_INFO("reverse lane detour is allowed");
  return true;
}

bool IsAllowedDetourInReverseLane(const TaskInfo &task_info,
                                  double preview_distance,
                                  double ignore_distance,
                                  bool use_multi_frame) {
  std::string info_sign = "reverse lane detour";
  LOG_INFO("*** {} check: preview_distance:{:.4f}, ignore_distance:{:.4f}",
           info_sign, preview_distance, ignore_distance);

  // info define
  class FrameCheckInfo : public FrameCheckInfoBase {
   public:
    bool is_leftmost_lane{false};
    bool is_left_bound_allowed_cross{false};
    bool is_left_divider_allowed_cross{false};
    bool is_reverse_lane_allowed_cross{false};
    void PrintFrameCheckInfo(std::string sign) override {
      LOG_INFO(
          "{} reverse_lane_detour_info:[{} {} {} {}], frame_ans:{}, "
          "frame_time:{}, final_ans:{}",
          sign, is_leftmost_lane, is_left_bound_allowed_cross,
          is_left_divider_allowed_cross, is_reverse_lane_allowed_cross,
          frame_ans, frame_time, frame_final_ans);
    }
  };

  //  process
  FrameCheckInfo frame_check_info;
  frame_check_info.frame_info_sign = info_sign;
  frame_check_info.frame_ans = IsAllowedDetourInReverseLane(
      task_info, preview_distance, ignore_distance);
  auto reverse_lane_detour_context =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  frame_check_info.is_leftmost_lane =
      reverse_lane_detour_context->is_leftmost_lane;
  frame_check_info.is_left_bound_allowed_cross =
      reverse_lane_detour_context->is_left_bound_allowed_cross;
  frame_check_info.is_left_divider_allowed_cross =
      reverse_lane_detour_context->is_left_divider_allowed_cross;
  frame_check_info.is_reverse_lane_allowed_cross =
      reverse_lane_detour_context->is_reverse_lane_allowed_cross;

  // multi frame
  auto &conf = config::PlanningConfig::Instance()
                   ->plan_config()
                   .scenario_common.is_allowed_detour_in_reverse_lane_conf;
  MultiFrameCheck(frame_check_info, use_multi_frame, conf.horizon,
                  conf.exp_param);

  return frame_check_info.frame_final_ans;
}

bool IsRoadQueued(const TaskInfo &task_info,
                  std::vector<Obstacle *> &detour_obs) {
  if (detour_obs.empty()) {
    return false;
  }
  auto final_obs = detour_obs.back();
  // single solid line check
  auto is_lane_crossed = [&](const ReferencePoint &ref_pt) {
    std::vector<DividerFeature> left_divider_feature =
        ref_pt.left_divider_feature();
    for (int i = 0; i < left_divider_feature.size(); ++i) {
      DividerFeature divider_feature = left_divider_feature[i];
      if (divider_feature.divider_type_ ==
              DividerFeature::DividerType::SINGLE_SOLID_LINE &&
          divider_feature.divider_color_ ==
              DividerFeature::DividerColor::WHITE) {
        return false;
      }
    }
    return true;
  };
  // obs parking check
  std::vector<Obstacle *> non_parking_obs;
  for (auto &obs : detour_obs) {
    ReferencePoint start_ref_pt, end_ref_pt;
    task_info.reference_line()->GetNearestRefPoint(
        obs->PolygonBoundary().start_s(), &start_ref_pt);
    task_info.reference_line()->GetNearestRefPoint(
        obs->PolygonBoundary().end_s(), &end_ref_pt);
    double right_bound = std::min(start_ref_pt.right_road_bound(),
                                  end_ref_pt.right_road_bound());
    ReferencePoint reference_point;
    if (!task_info.reference_line()->GetNearestRefPoint(obs->center_sl().s(),
                                                        &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double obs_heading =
        obs->is_static() ? obs->heading() : obs->velocity_heading();
    double heading_diff =
        std::abs(normalize_angle(obs_heading - reference_point.heading()));
    if (obs->PolygonBoundary().start_l() > -right_bound + 0.3 ||
        (heading_diff >= M_PI_2 * 0.67 &&
         M_PI - heading_diff >= M_PI_2 * 0.67)) {
      LOG_INFO(
          "obs [{}] away from road bound: {:.3f} - {:.3f}, heading diff {:.3f}",
          obs->id(), obs->PolygonBoundary().start_l(), -right_bound,
          heading_diff);
      non_parking_obs.push_back(obs);
    }
  }
  LOG_INFO("front has {} non-parking car.", non_parking_obs.size());
  if (!non_parking_obs.empty()) {
    if (non_parking_obs.size() > 1) {
      LOG_INFO("front has more than 1 non-parking car.");
      return true;
    }
    LOG_INFO("front has only 1 non-parking car, check blocking.");
    auto &decision_data = task_info.decision_data();
    auto &obstacle = non_parking_obs.back();
    uint64_t matched_id = obstacle->matched_lane_id();
    std::vector<size_t> right_lanes;
    if (PlanningMap::Instance()->GetRightLanes(matched_id, right_lanes) &&
        !right_lanes.empty()) {
      LOG_INFO("obstalce not in the right line.");
      return true;
    }
    if (IsNonMovableObstacle(task_info, obstacle, 35.0)) {
      return true;
    }
    ReferencePoint end_pt;
    task_info.reference_line()->GetNearestRefPoint(
        obstacle->PolygonBoundary().end_s(), &end_pt);
    if (!is_lane_crossed(end_pt)) {
      LOG_INFO("obstacle in the white solid line.");
      return true;
    }
  } else {
    LOG_INFO("front obs is in parking.");
    return false;
  }
  // obs height check
  LOG_INFO("ego height: {:.3f}, obs id : {}, obs height: {:.3f}",
           VehicleParam::Instance()->height(), final_obs->id(),
           final_obs->height());
  double height_threshold = 1.7;
  if (final_obs->height() >
      std::min(VehicleParam::Instance()->height() - 0.2, height_threshold)) {
    LOG_INFO("ego car in queued road.");
    return true;
  }
  return false;
}

void AdcPositionDiscussion(const TaskInfo &task_info) {
  if (task_info.last_frame() == nullptr) return;
  bool is_adc_on_refer_lane =
      task_info.curr_referline_pt().lane_type_is_pure_city_driving()
          ? DataCenter::Instance()
                ->mutable_master_info()
                ->mutable_motorway_lane_borrow_context()
                ->is_adc_on_refer_lane
          : DataCenter::Instance()
                ->mutable_master_info()
                ->mutable_lane_borrow_context()
                ->is_adc_on_refer_lane;
  auto reverse_lane_detour_context =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  if (!reverse_lane_detour_context->is_allowed_detour_in_reverse_lane) {
    reverse_lane_detour_context->adc_position =
        ReverseLaneDetourContext::AdcPosition::NO_NEED_TO_DISSCUSS;
  } else {
    const auto &adc_boundary = task_info.last_frame()
                                   ->outside_planner_data()
                                   .path_obstacle_context.adc_boundary;
    double adc_left_l = adc_boundary.end_l(),
           adc_right_l = adc_boundary.start_l();

    double left_road_bound = task_info.curr_referline_pt().left_road_bound(),
           right_road_bound = -task_info.curr_referline_pt().right_road_bound(),
           left_reverse_road_bound =
               task_info.curr_referline_pt().left_reverse_road_bound();

    LOG_INFO(
        "adc_left_l:{:.3f},adc_right_l:{:.3f},left_road_bound:{:.3f},right_"
        "road_bound:{:.3f},left_reverse_road_bound:{:.3f}",
        adc_left_l, adc_right_l, left_road_bound, right_road_bound,
        left_reverse_road_bound);

    if ((adc_left_l < left_road_bound) && (adc_right_l > right_road_bound) ||
        is_adc_on_refer_lane) {
      reverse_lane_detour_context->adc_position =
          ReverseLaneDetourContext::AdcPosition::LEFTMOST_LANE_COMPLETE;
    } else if ((adc_left_l > left_road_bound) &&
               (adc_right_l < left_road_bound)) {
      reverse_lane_detour_context->adc_position =
          ReverseLaneDetourContext::AdcPosition::DIVIDER_CROSSING;
    } else if ((adc_left_l < left_reverse_road_bound) &&
               (adc_right_l > left_road_bound)) {
      reverse_lane_detour_context->adc_position =
          ReverseLaneDetourContext::AdcPosition::REVERSE_LANE_COMPLETE;
    } else if (adc_left_l > left_reverse_road_bound) {
      reverse_lane_detour_context->adc_position =
          ReverseLaneDetourContext::AdcPosition::OVER_REVERSE_LANE;
    } else if (adc_right_l < right_road_bound) {
      reverse_lane_detour_context->adc_position =
          ReverseLaneDetourContext::AdcPosition::OVER_LEFTMOST_LANE;
    } else {
      reverse_lane_detour_context->adc_position =
          ReverseLaneDetourContext::AdcPosition::NOT_DEFINE;
    }
  }
}

void ComputeCrossRoadIgnoreDistance(const TaskInfo &task_info,
                                    double overlap_start_s,
                                    double overlap_end_s,
                                    double &ignore_distance) {
  if (task_info.curr_sl().s() < overlap_start_s ||
      task_info.curr_sl().s() > overlap_end_s)
    return;
  std::vector<std::pair<double, double>> crosswalk_overlap_s_pair{};
  for (const auto &crosswalk_overlap :
       task_info.reference_line()->crosswalk_overlaps()) {
    if (crosswalk_overlap.object_id != 0 &&
        crosswalk_overlap.end_s < overlap_end_s &&
        crosswalk_overlap.start_s > overlap_start_s) {
      crosswalk_overlap_s_pair.push_back(
          {crosswalk_overlap.start_s, crosswalk_overlap.end_s});
    }
  }
  if (!crosswalk_overlap_s_pair.empty()) {
    std::sort(crosswalk_overlap_s_pair.begin(), crosswalk_overlap_s_pair.end(),
              [](const auto &a, const auto &b) { return a.second < b.second; });
    if (crosswalk_overlap_s_pair.back().first >
        0.5 * (overlap_start_s + overlap_end_s)) {
      ignore_distance =
          std::max(ignore_distance,
                   overlap_end_s - crosswalk_overlap_s_pair.back().first);
    }
  }
  return;
}

bool IsLeftFrontReverseObsExist(
    const TaskInfo &task_info,
    const std::vector<Obstacle *> &left_front_reverse_obs,
    const std::vector<Obstacle *> &front_reverse_obs) {
  if (task_info.last_frame() == nullptr) return false;

  auto reverse_lane_detour_context =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_reverse_lane_detour_context();

  auto check_reverse_obs = [](auto &reverse_lane_detour_context,
                              auto &left_front_reverse_obs, auto &task_info) {
    if (left_front_reverse_obs.empty()) return false;

    const auto &adc_boundary = task_info.last_frame()
                                   ->outside_planner_data()
                                   .path_obstacle_context.adc_boundary;
    const double &adc_speed =
        task_info.last_frame()->inside_planner_data().vel_v;
    reverse_lane_detour_context->relative_time_to_reverse_obs.clear();
    for (int i = 0; i < left_front_reverse_obs.size(); ++i) {
      auto obstacle = left_front_reverse_obs[i];
      double delt_s = obstacle->min_s() - adc_boundary.end_s();
      double delt_v = obstacle->speed() + adc_speed;
      double delt_t = delt_s / delt_v;
      reverse_lane_detour_context->relative_time_to_reverse_obs.emplace_back(
          ReverseLaneDetourContext::ReverseObsInfo(
              obstacle->id(), obstacle->min_s(), adc_boundary.end_s(),
              obstacle->speed(), adc_speed, delt_s, delt_v, delt_t));
    }
    return true;
  };

  return task_info.curr_referline_pt().lane_type_is_pure_city_driving()
             ? check_reverse_obs(reverse_lane_detour_context,
                                 left_front_reverse_obs, task_info)
             : check_reverse_obs(reverse_lane_detour_context, front_reverse_obs,
                                 task_info);
}

bool CheckMotorwayBackOut(const ReferenceLinePtr &reference_line,
                          const double curr_s, const VehicleStateProxy &pose) {
  ReferencePoint pt{};
  auto hdmap = PlanningMap::Instance();
  if (!reference_line->GetNearestRefPoint(curr_s, &pt)) {
    LOG_INFO("Skip BACK_OUT: GetNearestRefPoint fail");
    return false;
  }
  if (pt.lane_type_is_pure_city_driving()) {
    LOG_INFO("Ego car is in Motorway now");
    if (!config::PlanningConfig::Instance()
             ->plan_config()
             .back_out.enable_motor_back_out) {
      LOG_INFO("Skip BACK_OUT: unable to motorway back out");
      return false;
    }
    if (!hdmap->IsRightLane(pose.X(), pose.Y())) {
      LOG_INFO("Skip BACK_OUT: In motor way but not right lane");
      return false;
    }
    const auto &ref_pts = reference_line->ref_points();
    const auto &adc_ref_pt =
        ref_pts[ref_line_util::BinarySearchIndex(ref_pts, curr_s)];
    auto right_road_bound = adc_ref_pt.right_road_bound();
    auto right_lane_bound = adc_ref_pt.right_lane_bound();
    LOG_INFO("right_road_bound: {:.4f}, right_lane_bound: {:.4f}",
             right_road_bound, right_lane_bound);
    if (right_road_bound - right_lane_bound < 1.5) {
      LOG_INFO("Skip BACK_OUT: no enough right pass width");
      return false;
    }
  }
  return true;
}

bool BorrowDirection(const std::shared_ptr<DecisionData> &decision_data,
                     const ReferencePointVec1d &extend_lane_ref_points,
                     const int &obs_id, const double left_min_bound,
                     const double right_min_bound, bool *borrow_left,
                     bool *borrow_right) {
  auto &lane_borrow_context =
      DataCenter::Instance()->mutable_master_info()->lane_borrow_context();
  *borrow_left = false;
  *borrow_right = false;

  Obstacle *obstacle = nullptr;
  auto ret = decision_data->get_obstacle_by_id(obs_id, false, &obstacle);
  if (ret != ErrorCode::PLANNING_OK || obstacle == nullptr) {
    LOG_INFO("get obs {} failed.", obs_id);
    return false;
  }
  auto obs_boundary = obstacle->PolygonBoundary();
  double left_space = left_min_bound - obs_boundary.end_l();
  double right_space = right_min_bound + obs_boundary.start_l();
  const double remain_dis = FLAGS_planning_lane_borrow_lateral_space_threshold;
  if (left_space < remain_dis && right_space < remain_dis) {
    LOG_INFO("left_space: {:.3f}, right_space: {:.3f}, skip lane borrow.",
             left_space, right_space);
    return false;
  }

  LOG_DEBUG("lane borrow line size: {}", extend_lane_ref_points.size());
  *borrow_left = (extend_lane_ref_points.front().left_lane_borrow()) &&
                 (left_space >= remain_dis);
  *borrow_right = (extend_lane_ref_points.front().right_lane_borrow()) &&
                  (right_space >= remain_dis);

  // if both can borrow choose white line
  // color: 0-white, 1-yellow
  if (*borrow_right && *borrow_left) {
    int left_color{0}, right_color{0};
    if (!PlanningMap::Instance()->GetPointLeftRightLaneColor(
            extend_lane_ref_points.front(), left_color, right_color)) {
      LOG_INFO("failed to GetPointLeftRightLaneColor, skip lane borrow.");
      return false;
    }
    if (!left_color && !right_color) {
      if (left_space > right_space + 0.3) {
        *borrow_right = false;
      } else if (right_space > left_space + 0.3) {
        *borrow_left = false;
      } else {
        LOG_INFO("Both side lane color is white, use default");
        if (FLAGS_planning_default_left_right_side) {
          LOG_INFO("default is right");
          *borrow_left = false;
        } else {
          LOG_INFO("default is left");
          *borrow_right = false;
        }
      }
    } else if (left_color && right_color) {
      if (left_space > right_space + 0.3) {
        *borrow_right = false;
      } else if (right_space > left_space + 0.3) {
        *borrow_left = false;
      } else {
        LOG_INFO("Both side lane color is yellow, use default");
        if (FLAGS_planning_default_left_right_side) {
          LOG_INFO("default is right");
          *borrow_left = false;
        } else {
          LOG_INFO("default is left");
          *borrow_right = false;
        }
      }
    } else {
      if (left_color) {
        *borrow_left = false;
      } else {
        *borrow_right = false;
      }
    }
  }

  if (!(borrow_left || borrow_right)) {
    return false;
  }

  return true;
}

void UpdateRecordEvents(ScenarioState::State curr_state) {
  if (curr_state == ScenarioState::DETOUR) {
    DataCenter::Instance()->AddRecordEvent(EventOfInterest::LANE_BORROW);
  }
}

void UpdateLaneType(const TaskInfo &task_info, std::string &lane_type) {
  auto &reference_line = task_info.reference_line();
  ReferencePoint pt{task_info.curr_referline_pt()};
  lane_type = pt.get_lane_type_name();
}

void UpdateDetourInfo(DetourScenarioInfo &detour_conf, bool motorway_drive) {
  DataCenter *data_center{DataCenter::Instance()};
  const config::AutoPlanConfig *plan_config_ptr =
      &config::PlanningConfig::Instance()->plan_config();
  if (motorway_drive) {
    if (data_center->master_info().curr_scenario() ==
        ScenarioState::MOTORWAY_DETOUR) {
      detour_conf.crossroad_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_exit_scenario
              .crossroad_preview_distance;
      detour_conf.road_bound_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_exit_scenario
              .road_bound_preview_distance;
      detour_conf.traffic_light_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_exit_scenario
              .traffic_light_preview_distance;
      detour_conf.lane_turn_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_exit_scenario
              .lane_turn_preview_distance;
      detour_conf.queued_crossroad_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_exit_scenario
              .queued_crossroad_preview_distance;
      detour_conf.queued_traffic_light_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_exit_scenario
              .queued_traffic_light_preview_distance;
    } else {
      detour_conf.crossroad_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_enter_scenario
              .crossroad_preview_distance;
      detour_conf.road_bound_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_enter_scenario
              .road_bound_preview_distance;
      detour_conf.traffic_light_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_enter_scenario
              .traffic_light_preview_distance;
      detour_conf.lane_turn_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_enter_scenario
              .lane_turn_preview_distance;
      detour_conf.queued_crossroad_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_enter_scenario
              .queued_crossroad_preview_distance;
      detour_conf.queued_traffic_light_preview_distance =
          plan_config_ptr->motorway_detour_scenario.detour_enter_scenario
              .queued_traffic_light_preview_distance;
    }
  } else {
    if (data_center->master_info().curr_scenario() == ScenarioState::DETOUR) {
      detour_conf.crossroad_preview_distance =
          plan_config_ptr->detour_scenario.detour_exit_scenario
              .crossroad_preview_distance;
      detour_conf.road_bound_preview_distance =
          plan_config_ptr->detour_scenario.detour_exit_scenario
              .road_bound_preview_distance;
      detour_conf.traffic_light_preview_distance =
          plan_config_ptr->detour_scenario.detour_exit_scenario
              .traffic_light_preview_distance;
      detour_conf.queued_crossroad_preview_distance =
          plan_config_ptr->detour_scenario.detour_exit_scenario
              .queued_crossroad_preview_distance;
      detour_conf.queued_traffic_light_preview_distance =
          plan_config_ptr->detour_scenario.detour_exit_scenario
              .queued_traffic_light_preview_distance;
    } else {
      detour_conf.crossroad_preview_distance =
          plan_config_ptr->detour_scenario.detour_enter_scenario
              .crossroad_preview_distance;
      detour_conf.road_bound_preview_distance =
          plan_config_ptr->detour_scenario.detour_enter_scenario
              .road_bound_preview_distance;
      detour_conf.traffic_light_preview_distance =
          plan_config_ptr->detour_scenario.detour_enter_scenario
              .traffic_light_preview_distance;
      detour_conf.queued_crossroad_preview_distance =
          plan_config_ptr->detour_scenario.detour_enter_scenario
              .queued_crossroad_preview_distance;
      detour_conf.queued_traffic_light_preview_distance =
          plan_config_ptr->detour_scenario.detour_enter_scenario
              .queued_traffic_light_preview_distance;
    }
  }
}

std::string PrintDetourInfo(bool enable, bool check_pass, bool is_motorway) {
  auto motorway_lane_borrow_context =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_motorway_lane_borrow_context();
  auto lane_borrow_context = DataCenter::Instance()
                                 ->mutable_master_info()
                                 ->mutable_lane_borrow_context();
  auto reverse_lane_detour_context =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_reverse_lane_detour_context();

  // save monitor message.
  char str_buffer[256 * 2];
  if (is_motorway) {
    std::string str_dynamic_obs_id{""};
    if (!motorway_lane_borrow_context->dynamic_obs_ids.empty()) {
      int count = 0;
      for (int id : motorway_lane_borrow_context->dynamic_obs_ids) {
        if (count >= 3) {
          break;
        }
        str_dynamic_obs_id += std::to_string(id) + ",";
        count++;
      }
    }
    sprintf(
        str_buffer,
        "[MotorwayDetour][enable: %d %d][refer_lane_clear: %d][queued_road: "
        "%d][traffic_light: "
        "%d][cross_road: %d][road_bound: %d %d][right_first_line: "
        "%d][lane_turn: %d][on_refer_lane: %d][allowed_reverse_lane_detour: %d "
        "%d %d %d %d %d, adc_pos: %d][front_clear: %d %d][dynamic_danger: %d "
        "%d][extend_bound static: %.2f,%.2f, dynamic: %.2f,%.2f, region: "
        "%.2f,%.2f][obs_id static: %d(%.2f),%d,%d, dynamic_obs_id: %s]",
        enable, check_pass,
        motorway_lane_borrow_context->is_refer_lane_static_obs_clear,
        motorway_lane_borrow_context->is_road_queued,
        motorway_lane_borrow_context->is_front_has_traffic_light,
        motorway_lane_borrow_context->is_front_has_cross_road,
        motorway_lane_borrow_context->is_left_front_has_road_boundary,
        motorway_lane_borrow_context->is_right_front_has_road_boundary,
        motorway_lane_borrow_context->is_in_right_first_line,
        motorway_lane_borrow_context->is_front_has_lane_turn,
        motorway_lane_borrow_context->is_adc_on_refer_lane,
        reverse_lane_detour_context->is_allowed_detour_in_reverse_lane,
        reverse_lane_detour_context->is_leftmost_lane,
        reverse_lane_detour_context->is_left_bound_allowed_cross,
        reverse_lane_detour_context->is_left_divider_allowed_cross,
        reverse_lane_detour_context->is_reverse_lane_allowed_cross,
        reverse_lane_detour_context->is_left_front_reverse_obs_exist,
        static_cast<int>(reverse_lane_detour_context->adc_position),
        motorway_lane_borrow_context->is_left_front_static_obs_clear,
        motorway_lane_borrow_context->is_right_front_static_obs_clear,
        motorway_lane_borrow_context->is_left_dynamic_obs_danger,
        motorway_lane_borrow_context->is_right_dynamic_obs_danger,
        motorway_lane_borrow_context->front_left_min_lane_bound,
        motorway_lane_borrow_context->front_right_min_lane_bound,
        motorway_lane_borrow_context->left_min_lane_bound,
        motorway_lane_borrow_context->right_min_lane_bound,
        motorway_lane_borrow_context->region_left_bound,
        motorway_lane_borrow_context->region_right_bound,
        motorway_lane_borrow_context->refer_lane_block_static_obs_id,
        motorway_lane_borrow_context->refer_lane_block_static_obs_height,
        motorway_lane_borrow_context->left_front_block_static_obs_id,
        motorway_lane_borrow_context->right_front_block_static_obs_id,
        str_dynamic_obs_id.c_str());
    DataCenter::Instance()->SetMonitorString(
        str_buffer, MonitorItemSource::MOTOTWAY_DETOUR);
  } else {
    std::string str_dynamic_obs_id{""};
    if (!lane_borrow_context->dynamic_obs_ids.empty()) {
      int count = 0;
      for (int id : lane_borrow_context->dynamic_obs_ids) {
        if (count >= 3) {
          break;
        }
        str_dynamic_obs_id += std::to_string(id) + ",";
        count++;
      }
    }
    sprintf(
        str_buffer,
        "[Detour][enable: %d %d][refer_lane_clear: %d][queued_road: "
        "%d][traffic_light: "
        "%d][cross_road: %d][road_bound: %d][on_refer_lane: "
        "%d][allowed_reverse_lane_detour: %d %d %d %d %d %d, adc_pos: "
        "%d][dynamic_danger: %d][extend_lane static: %.2f,%.2f, dynamic: "
        "%.2f,%.2f, region: %.2f][obs_id static: %d(%.2f), dynamic_obs_id: %s]",
        enable, check_pass, lane_borrow_context->is_refer_lane_static_obs_clear,
        lane_borrow_context->is_road_queued,
        lane_borrow_context->is_front_has_traffic_light,
        lane_borrow_context->is_front_has_cross_road,
        lane_borrow_context->is_front_has_road_boundary,
        lane_borrow_context->is_adc_on_refer_lane,
        reverse_lane_detour_context->is_allowed_detour_in_reverse_lane,
        reverse_lane_detour_context->is_leftmost_lane,
        reverse_lane_detour_context->is_left_bound_allowed_cross,
        reverse_lane_detour_context->is_left_divider_allowed_cross,
        reverse_lane_detour_context->is_reverse_lane_allowed_cross,
        reverse_lane_detour_context->is_left_front_reverse_obs_exist,
        static_cast<int>(reverse_lane_detour_context->adc_position),
        lane_borrow_context->is_left_dynamic_obs_danger,
        lane_borrow_context->front_left_min_lane_bound,
        lane_borrow_context->front_right_min_lane_bound,
        lane_borrow_context->left_min_lane_bound,
        lane_borrow_context->right_min_lane_bound,
        lane_borrow_context->region_left_bound,
        lane_borrow_context->refer_lane_block_static_obs_id,
        lane_borrow_context->refer_lane_block_static_obs_height,
        str_dynamic_obs_id.c_str());
    DataCenter::Instance()->SetMonitorString(str_buffer,
                                             MonitorItemSource::DETOUR);
  }
  return std::string(str_buffer);
}

}  // namespace scenario_common
}  // namespace planning
}  // namespace neodrive
