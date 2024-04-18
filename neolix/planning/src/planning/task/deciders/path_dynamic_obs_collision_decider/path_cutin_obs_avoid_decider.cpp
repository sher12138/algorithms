#include "path_cutin_obs_avoid_decider.h"
#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"

namespace neodrive {
namespace planning {
PathCutinObsAvoidDecider::PathCutinObsAvoidDecider() {
  name_ = "PathCutinObsAvoidDecider";
}

PathCutinObsAvoidDecider::~PathCutinObsAvoidDecider() { Reset(); }

ErrorCode PathCutinObsAvoidDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (task_info.current_frame() == nullptr ||
      task_info.reference_line() == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!InitCollisionCheckRegion(task_info)) {
    LOG_ERROR("Path Dynamic Obs Collision Decider Init failed.");
  }
  if (!DynamicObsProcess(task_info)) {
    LOG_ERROR("Process failed");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool PathCutinObsAvoidDecider::InitCollisionCheckRegion(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  auto adc_boundary = task_info.adc_boundary();

  left_collision_check_region_ = std::move(
      Boundary{adc_boundary.start_s() - check_region_s_s_,
               adc_boundary.end_s() + check_region_e_s_, adc_boundary.end_l(),
               adc_boundary.end_l() + check_region_w_});
  right_collision_check_region_ = std::move(Boundary{
      adc_boundary.start_s() - check_region_s_s_,
      adc_boundary.end_s() + check_region_e_s_,
      adc_boundary.start_l() - check_region_w_, adc_boundary.start_l()});

  occupied_front_s_ =
      std::clamp(inside_data.vel_v * inside_data.vel_v / 2 / 3.0, 0.5, 5.0);

  LOG_INFO("adc boundary, s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, {:.2f}; ",
           adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
           adc_boundary.end_l());
  return true;
}

bool PathCutinObsAvoidDecider::DynamicObsProcess(TaskInfo& task_info) {
  const auto& dynamic_obstacles = task_info.current_frame()
                                      ->planning_data()
                                      .decision_data()
                                      .dynamic_obstacle();
  if (dynamic_obstacles.empty()) {
    LOG_INFO("dynamic obstacles is clear");
    return true;
  }
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();

  double min_dec_t = inside_data.vel_v / 3.0;
  bool is_turning_obs{false};

  for (auto obs : dynamic_obstacles) {
    if (obs == nullptr) {
      LOG_ERROR("dynamic_obstacles is nullptr");
      return true;
    }
    LOG_INFO("start to deal obs[{}]", obs->id());
    double heading_diff =
        normalize_angle(inside_data.vel_heading - obs->velocity_heading());
    if (std::abs(heading_diff) > M_PI_2) {
      LOG_INFO("skip reverse obs");
      continue;
    }
    double adc_heading_diff = normalize_angle(
        inside_data.vel_heading - task_info.curr_referline_pt().heading());
    LOG_INFO("heading:{:2f},adc heading:{:2f}", obs->velocity_heading(),
             inside_data.vel_heading);
    double obs_v_x = std::abs(obs->speed() * std::cos(heading_diff));
    double obs_v_y = std::abs(obs->speed() * std::sin(heading_diff));
    double adc_v_y = inside_data.vel_v * std::sin(adc_heading_diff);
    double pre_t{2.0};
    double pre_l = 1.0 * adc_v_y;
    const auto& obs_boundary = obs->PolygonBoundary();
    LOG_INFO("adc_v_y:{:2f}, min_dec_t:{:2f}, pre_l:{:2f}", adc_v_y, min_dec_t,
             pre_l);
    if (left_collision_check_region_.has_overlap(obs_boundary) &&
        heading_diff > -M_PI_4) {
      UpdateDynmaicObsInfo(obs, left_collision_dynamic_obstacles, heading_diff);
      // is_turning_obs = is_turning_obs =
      //     TurningObstacleNeedAvoidance(obs,
      //     left_collision_dynamic_obstacles);
      double ttc, d_s, d_v;
      if (obs->center_sl().s() > task_info.curr_sl().s()) {
        d_s = obs_boundary.start_s() - task_info.adc_boundary().end_s();
        d_v = inside_data.vel_v - obs_v_x;
        ttc = d_s / d_v;
      } else {
        d_s = task_info.adc_boundary().start_s() - obs_boundary.end_s();
        d_v = obs_v_x - inside_data.vel_v;
        ttc = d_s / d_v;
      }
      LOG_INFO(
          "obs_boundary, s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, "
          "{:.2f}; ",
          obs_boundary.start_s(), obs_boundary.end_s(), obs_boundary.start_l(),
          obs_boundary.end_l());
      double lateral_distance =
          obs_boundary.start_l() - task_info.adc_boundary().end_l() + pre_l;
      l_ttc_ = lateral_distance / obs_v_y;
      LOG_INFO(
          "start to deal left obs[{}], heading "
          "diff:{:2f}, lateral_distance:{:2f}, obs_v_y:{:2f}, "
          "l_ttc:{:2f},ttc:{:2f}",
          obs->id(), heading_diff, lateral_distance, obs_v_y, l_ttc_, ttc);
      if (ttc > 5.0) {
        LOG_INFO("dynamic obs is far from adc");
        continue;
      }
      l_ttc_ = std::min(l_ttc_, max_t_);
      // case no overlap with occupied region
      if (heading_diff > 0 && ttc > l_ttc_) {
        // case overlap with occupied region
        double min_dl = std::max(min_dec_t * adc_v_y, occupied_reigon_l_);
        double k = std::clamp(
            0.1 * left_collision_dynamic_obstacles[obs->id()].count, 0.2, 1.0);
        double fliter_ttc = k * pre_t;
        double min_l;
        if (obs_boundary.start_l() < task_info.adc_boundary().end_l()) {
          min_l = std::max(obs_boundary.start_l(), task_info.curr_sl().l());
        } else {
          min_l = std::max(obs_boundary.start_l() - obs_v_y * fliter_ttc,
                           task_info.adc_boundary().end_l() + min_dl);
        }
        double max_l = obs_boundary.end_l();
        double min_s =
            std::max(obs_boundary.start_s(),
                     task_info.adc_boundary().end_s() + occupied_front_s_);
        double max_s = std::max(min_s + obs_v_x * ttc, min_s + 3.0);
        LOG_INFO(
            "aabox, s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, "
            "{:.2f}; ",
            min_s, max_s, min_l, max_l);
        AABox2d box{Vec2d(min_s, min_l), Vec2d(max_s, max_l)};
        box.set_id(1);
        decision_data->mutable_lateral_virtual_boxes()->emplace_back(box);
        decision_data->create_lateral_virtual_obstacle(
            box, task_info.curr_referline_pt().heading(),
            VirtualObstacle::AVOIDANCE);
        continue;
      }
      continue;
    }

    if (right_collision_check_region_.has_overlap(obs_boundary) &&
        heading_diff < M_PI_4) {
      UpdateDynmaicObsInfo(obs, right_collision_dynamic_obstacles,
                           heading_diff);
      // is_turning_obs =
      //     TurningObstacleNeedAvoidance(obs,
      //     right_collision_dynamic_obstacles);
      double ttc, d_s, d_v;
      if (obs->center_sl().s() > task_info.curr_sl().s()) {
        d_s = obs_boundary.start_s() - task_info.adc_boundary().end_s();
        d_v = inside_data.vel_v - obs_v_x;
        ttc = d_s / d_v;
      } else {
        d_s = task_info.adc_boundary().start_s() - obs_boundary.end_s();
        d_v = obs_v_x - inside_data.vel_v;
        ttc = d_s / d_v;
      }
      LOG_INFO(
          "obs_boundary, s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, "
          "{:.2f}; ",
          obs_boundary.start_s(), obs_boundary.end_s(), obs_boundary.start_l(),
          obs_boundary.end_l());
      double lateral_distance =
          task_info.adc_boundary().start_l() - obs_boundary.end_l() + pre_l;
      l_ttc_ = lateral_distance / obs_v_y;
      LOG_INFO(
          "start to deal right obs[{}], heading "
          "diff:{:2f}, lateral_distance:{:2f}, obs_v_y:{:2f}, "
          "l_ttc:{:2f},ttc:{:2f}",
          obs->id(), heading_diff, lateral_distance, obs_v_y, l_ttc_, ttc);
      if (ttc > 5.0) {
        LOG_INFO("dynamic obs is far from adc");
        continue;
      }
      l_ttc_ = std::min(l_ttc_, max_t_);
      if (heading_diff < 0 && ttc > l_ttc_) {
        // case overlap with occupied region
        double min_dl = std::max(min_dec_t * adc_v_y, occupied_reigon_l_);
        double k = std::clamp(
            0.1 * left_collision_dynamic_obstacles[obs->id()].count, 0.2, 1.0);
        double fliter_ttc = k * pre_t;
        double max_l;
        if (obs_boundary.end_l() > task_info.adc_boundary().start_l()) {
          max_l = std::min(obs_boundary.end_l(), task_info.curr_sl().l());
        } else {
          max_l = std::min(
              obs_boundary.end_l() + obs_v_y * fliter_ttc,
              task_info.adc_boundary().start_l() - min_dec_t * adc_v_y);
        }
        double min_l = obs_boundary.start_l();
        double min_s =
            std::max(obs_boundary.start_s(),
                     task_info.adc_boundary().end_s() + occupied_front_s_);
        double max_s = std::max(min_s + obs_v_x * ttc, min_s + 3.0);
        LOG_INFO(
            "create aabox: s_s, e_s, s_l, e_l: {:.2f}, {:.2f},{:.2f}, "
            "{:.2f}; ",
            min_s, max_s, min_l, max_l);
        AABox2d box{Vec2d(min_s, min_l), Vec2d(max_s, max_l)};
        box.set_id(1);
        decision_data->mutable_lateral_virtual_boxes()->emplace_back(box);
        decision_data->create_lateral_virtual_obstacle(
            box, task_info.curr_referline_pt().heading(),
            VirtualObstacle::AVOIDANCE);
        continue;
      }
      continue;
    }
    LOG_INFO("skip safe obs");
    continue;
  }
  clearObsoleteObstacles(dynamic_obstacles, left_collision_dynamic_obstacles);
  clearObsoleteObstacles(dynamic_obstacles, right_collision_dynamic_obstacles);
  LOG_INFO("finish dynamic obs process");
  return true;
}  // namespace planning

void PathCutinObsAvoidDecider::UpdateDynmaicObsInfo(
    const Obstacle* const obs,
    std::unordered_map<int, PathCollisionDynamicObsData>&
        collision_dynamic_obstacles,
    const double& heading_diff) {
  // judge whether obs is about turning or just stop
  double k;
  if (collision_dynamic_obstacles.find(obs->id()) ==
      collision_dynamic_obstacles.end()) {
    auto& dynamic_obs = collision_dynamic_obstacles[obs->id()];
    dynamic_obs.id = obs->id();
    dynamic_obs.heading_diff =
        dynamic_obs.first_time
            ? heading_diff
            : (dynamic_obs.heading_diff * (1.0 - k) + heading_diff * k);
    dynamic_obs.first_time = false;
    dynamic_obs.last_heading = obs->heading();
    dynamic_obs.lost_cnt = 0;
    LOG_INFO("insert dynamic_obs[{}]", obs->id());
    return;
  } else {
    auto& dynamic_obs = collision_dynamic_obstacles[obs->id()];
    dynamic_obs.heading_diffs.push_back(heading_diff);
    dynamic_obs.accum_heading_diff += dynamic_obs.heading_diffs.back();
    if (dynamic_obs.heading_diffs.size() > 10) {
      dynamic_obs.accum_heading_diff -= dynamic_obs.heading_diffs.front();
      dynamic_obs.heading_diffs.pop_front();
    }
    dynamic_obs.last_heading = normalize_angle(obs->heading());
    dynamic_obs.lost_cnt = 0;
    dynamic_obs.count++;
    LOG_INFO("update dynamic_obs[{}]", obs->id());
    return;
  }
}

bool PathCutinObsAvoidDecider::TurningObstacleNeedAvoidance(
    const Obstacle* const obs,
    std::unordered_map<int, PathCollisionDynamicObsData>&
        collision_dynamic_obstacles) {
  bool is_turning_obs{false};
  double obs_turning_threshold{0.17};
  if (collision_dynamic_obstacles.find(obs->id()) ==
      collision_dynamic_obstacles.end()) {
    return false;
  }
  auto& obs_heading_info = collision_dynamic_obstacles[obs->id()];
  is_turning_obs =
      std::abs(obs_heading_info.accum_heading_diff) > obs_turning_threshold;

  LOG_INFO("obs [{}], accum_heading_diff {:.3f},is turning obs:{}", obs->id(),
           obs_heading_info.accum_heading_diff, is_turning_obs);
  return is_turning_obs;
};

void PathCutinObsAvoidDecider::clearObsoleteObstacles(
    const std::vector<neodrive::planning::Obstacle*>& dyn_obs,
    std::unordered_map<int, PathCollisionDynamicObsData>& obs_data) {
  auto it = obs_data.begin();
  while (it != obs_data.end()) {
    it->second.lost_cnt++;
    bool found = false;
    for (Obstacle* obs : dyn_obs) {
      if (obs->id() == it->first) {
        found = true;
        break;
      }
    }
    // remove disappeared dynamic_obstacle
    if (!found) {
      it = obs_data.erase(it);
    } else {
      ++it;
    }
  }
  auto obs_data_tmp = obs_data;
  for (auto& iter : obs_data) {
    if (iter.second.lost_cnt > 4) {
      obs_data_tmp.erase(iter.first);
    }
  }
  std::swap(obs_data, obs_data_tmp);
  LOG_INFO("obs collsion size: {}", obs_data.size());
}

bool PathCutinObsAvoidDecider::CreateAvoidanceStaticObs(TaskInfo& task_info) {
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data == nullptr || task_info.reference_line() == nullptr) {
    return false;
  }
  const auto& init_sl_point =
      task_info.current_frame()->inside_planner_data().init_sl_point;
  const auto& curr_ref_pt = task_info.curr_referline_pt();
  int* id;
  if (!left_collision_dynamic_obstacles.empty()) {
    // Boundary avoidance_obs_boundary;
    Polygon2d avoidance_obs_polygon;
    double start_l = std::max(task_info.adc_boundary().start_l() + 0.5,
                              min_left_obs_l_ - 0.6);
    AABox2d avoidance_obs_box(Vec2d(init_sl_point.s() - 1.0, start_l),
                              Vec2d(init_sl_point.s() + 10.0, start_l + 3.0));
    task_info.reference_line()->GetPolygonByAABox(avoidance_obs_box,
                                                  &avoidance_obs_polygon);
    if (decision_data->create_lateral_virtual_obstacle(
            avoidance_obs_polygon.aa_bounding_box(),
            task_info.curr_referline_pt().heading(),
            VirtualObstacle::AVOIDANCE) != ErrorCode::PLANNING_OK) {
      LOG_ERROR("Failed to extend avoidance obstacle.");
      return false;
    }
  }
  if (!right_collision_dynamic_obstacles.empty()) {
    Polygon2d avoidance_obs_polygon;
    Boundary avoidance_obs_boundary;
    double end_l = std::min(task_info.adc_boundary().end_l() - 0.5,
                            max_right_obs_l_ - 0.6);
    AABox2d avoidance_obs_box(Vec2d(init_sl_point.s() - 1.0, end_l - 3.0),
                              Vec2d(init_sl_point.s() + 10.0, end_l));
    task_info.reference_line()->GetPolygonByAABox(avoidance_obs_box,
                                                  &avoidance_obs_polygon);
    Box2d box_ = avoidance_obs_polygon.bounding_box_with_heading(
        task_info.curr_referline_pt().heading());
    if (decision_data->create_virtual_obstacle(
            box_, VirtualObstacle::AVOIDANCE) != ErrorCode::PLANNING_OK) {
      LOG_ERROR("Failed to extend avoidance obstacle.");
      return false;
    }
  }
  return true;
}

bool PathCutinObsAvoidDecider::ExtendAvoidanceObs(TaskInfo& task_info,
                                                  const Obstacle* ori_obs) {
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data == nullptr) {
    return false;
  }
  LOG_INFO("start to Extend Avoidance Obs");
  double k{0.1};
  double fliter_ttc =
      std::clamp(fliter_ttc * (1 - k) + l_ttc_ * k, l_ttc_ * 0.2, l_ttc_);
  double pre_length = std::min(15.0, ori_obs->speed() * fliter_ttc);
  double length = ori_obs->speed() * l_ttc_;
  if (decision_data->create_lateral_virtual_obstacle(
          Vec2d{
              ori_obs->center().x() + (pre_length + ori_obs->length()) / 2 *
                                          std::cos(ori_obs->velocity_heading()),
              ori_obs->center().y() +
                  (pre_length + ori_obs->length()) / 2 *
                      std::sin(ori_obs->velocity_heading())},
          length, 0, ori_obs->width(), ori_obs->velocity_heading(),
          VirtualObstacle::AVOIDANCE) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to extend avoidance obstacle.");
    return false;
  }
  LOG_INFO("finished Extending Avoidance Obs");
  return true;
}
}  // namespace planning
}  // namespace neodrive
