#include "motorway_bev_fence_filter_decider.h"
namespace neodrive {
namespace planning {

namespace {
constexpr double kReverseHeadingDiff = 0.75 * M_PI;
constexpr double kAttenTime = 1.0;
constexpr double kGap = 2.5;
constexpr double kProtectedDis = 2.5;
}  // namespace
MotorwayBevFenceFilterDecider::MotorwayBevFenceFilterDecider() {
  name_ = "MotorwayBevFenceFilterDecider";
}

MotorwayBevFenceFilterDecider::~MotorwayBevFenceFilterDecider() {}

bool MotorwayBevFenceFilterDecider::Init(TaskInfo& task_info) {

  return false;
}

bool MotorwayBevFenceFilterDecider::EgoBePortectedByFence(
    double s, double speed, std::vector<MatrixInterval>& combine_data) {
  if (combine_data.empty()) {
    return false;
  }

  if (combine_data.front().Longitudinal().start() > s) {
    return false;
  }

  if (combine_data.back().Longitudinal().end() < s) {
    return false;
  }
  if (combine_data.size() == 1) {
    return true;
  }

  auto iterator = combine_data.begin();
  auto next_iter = iterator;
  LOG_INFO("start  find  int");
  while (iterator != combine_data.end()) {
    next_iter = std::next(iterator);
    if (next_iter->Longitudinal().start() - iterator->Longitudinal().end() >
        kGap) {
      return false;
    }
    if (next_iter->Longitudinal().end() > s + speed * kAttenTime) {
      LOG_INFO("ego be protected!");
      return true;
    }
    iterator = next_iter;
  }
  return false;
}

bool MotorwayBevFenceFilterDecider::Process(TaskInfo& task_info) {
  // Bev 忽略障碍物。

  // 基于两侧栅栏 + 障碍物朝向 --> 忽略障碍物。

  return false;
}

void MotorwayBevFenceFilterDecider::IgnoreOBsByPerceptionRightFence(
    TaskInfo& task_info) {
  auto& motorway_speed_obstacle_context = task_info.current_frame()
                                              ->mutable_outside_planner_data()
                                              ->motorway_speed_obstacle_context;
  const auto& all_obstacles = task_info.decision_data()->all_obstacle();
  const auto& dynamic_obstacles = task_info.decision_data()->dynamic_obstacle();
  const auto& path =
      task_info.current_frame()->outside_planner_data().path_data;
  const double ego_heading =
      task_info.current_frame()->inside_planner_data().vel_heading;
  const double ego_l = task_info.curr_sl().l();
  const double ego_s = task_info.curr_sl().s();
  const double ego_speed =
      task_info.current_frame()->inside_planner_data().vel_v;
  auto& ignore_dynamic_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->motorway_speed_obstacle_context.ignore_dynamic_obs_id;

  const auto& curr_scenario =
      DataCenter::Instance()->master_info().curr_scenario();
  const auto& curr_stage = DataCenter::Instance()
                               ->master_info()
                               .motorway_intersection_context()
                               .stage;
  // need divide with  scenario
  if (curr_scenario == ScenarioState::MOTORWAY_INTERSECTION) {
    LOG_INFO("ego in intersection, skip fence filter");
    return;
  }

  if (path == nullptr) return;
  const auto& ref_line = task_info.reference_line();
  std::vector<const Obstacle*> right_fence_obstacles;
  for (const auto& obstacle : all_obstacles) {
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->sub_type() !=
        PerceptionObstacle_SubType::PerceptionObstacle_SubType_ST_FENCE) {
      continue;
    }

    if (obstacle->center_sl().s() < ego_s) {
      continue;
    }

    if (obstacle->center_sl().l() - ego_l > 0.0) {
      continue;
    } else {
      right_fence_obstacles.push_back(obstacle);
    }
  }

  if (right_fence_obstacles.empty()) {
    return;
  }
  // lat, long
  std::vector<MatrixInterval> data{};
  for (const auto& obs : right_fence_obstacles) {
    data.push_back(
        std::move(MatrixInterval(VecInterval(obs->min_l(), obs->max_l()),
                                 VecInterval(obs->min_s(), obs->max_s()))));
  }
  for (const auto& cc : data) {
    LOG_INFO("Long ss {:.3f},Long ss {:.3f}, Later ss{:.3f}, Later se {:.3f}",
             cc.Longitudinal().start(), cc.Longitudinal().end(),
             cc.Lateral().start(), cc.Lateral().end());
  }
  auto combined_data = std::move(MatrixInternalCombine(data));
  bool ego_be_protected_by_right_fence =
      EgoBePortectedByFence(ego_s, ego_speed, combined_data);
  // 1. construct fence boundary -->
  // 1.0 --> assume the fence is a continue mulit-line
  // 2.0 --> fence is continue internal line --> internal combine and
  // internal find 3.0 --> 区间碰撞结果的区间存在性查询，区间的交集查询。
  // if(!) find adverse obs
  for (const auto& obstacle : dynamic_obstacles) {
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->center_sl().s() < ego_s) {
      continue;
    }
    if (obstacle->center_sl().l() > ego_l) {
      continue;
    }
    double heading_diff = 0.0;
    if (!speed_planner_common::ObsEgoAlongPathHeadingDiffBaseSpeedHeading(
            obstacle, *path, ego_heading, heading_diff)) {
      LOG_INFO("can't get heading diff with obs {}, skip!", obstacle->id());
      continue;
    };
    // reverse get
    if (std::abs(heading_diff) > kReverseHeadingDiff) {
      LOG_INFO("prosss obs {}", obstacle->id());
      // find weather con pass fence,
      if (ObsBePortectedByFence(combined_data, obstacle,
                                ego_be_protected_by_right_fence)) {
        LOG_INFO("obs {} can't pass fence ,ignore", obstacle->id());
        ignore_dynamic_obs_id.insert(obstacle->id());
      }
    }
  }
}

bool MotorwayBevFenceFilterDecider::ObsBePortectedByFence(
    const std::vector<MatrixInterval>& combine_data, Obstacle* const& obstacle,
    const bool ego_be_protected_by_right_fence) {
  if (combine_data.empty()) {
    return false;
  }
  // -> solution 1 ,
  if (ego_be_protected_by_right_fence &&
      combine_data.back().Lateral().start() > obstacle->center_sl().l()) {
    LOG_INFO("ego be protected and obs {} out fence,ignore", obstacle->id());
    return true;
  }

  if (obstacle->min_s() > combine_data.back().Longitudinal().end()) {
    return false;
  }
  if (obstacle->max_s() < combine_data.front().Longitudinal().start()) {
    return false;
  }

  if (combine_data.size() == 1) {
    return true;
  }
  for (const auto& data : combine_data) {
    LOG_INFO("longitudianl start s{:.3f}, end s{:.3f}",
             data.Longitudinal().start(), data.Longitudinal().end());
    LOG_INFO("lateral start {:.3f}, end {:.3f}", data.Lateral().start(),
             data.Lateral().end());
  }
  LOG_INFO("obs s {:.3f},", obstacle->center_sl().s());
  // the first can insert position
  auto iterator = std::lower_bound(combine_data.begin(), combine_data.end(),
                                   obstacle->center_sl().s(),
                                   [](const MatrixInterval& lhs, double rhs) {
                                     return lhs.Longitudinal().end() < rhs;
                                   });

  if (iterator == combine_data.end()) {
    LOG_INFO("obs {} over fence,", obstacle->id());
    return true;
  }
  if (iterator->Lateral().start() * obstacle->center_sl().l() < 0.0) {
    LOG_INFO("obs {} not in same side with fence", obstacle->id());
    return true;
  }

  if ((std::abs(iterator->Lateral().start()) +
       std::abs(iterator->Lateral().end())) /
          2. >
      std::abs(obstacle->center_sl().l())) {
    LOG_INFO("obs {} in fence, can't ignore.", obstacle->id());
    return true;
  }

  double s_start = obstacle->center_sl().s() - obstacle->speed() * kAttenTime;
  if (iterator->Lateral().start() < s_start) {
    LOG_INFO("obs {} can't pass fence, ignore", obstacle->id());
    return true;
  }
  // judge weather exist a hole, what can let  obs across.
  auto prev_iter = iterator;
  LOG_INFO("start  find  int");
  while (iterator != combine_data.begin()) {
    prev_iter = std::prev(iterator);
    if (iterator->Longitudinal().start() - prev_iter->Longitudinal().end() >
        kGap) {
      return false;
    }
    if (prev_iter->Longitudinal().start() < s_start) {
      LOG_INFO("can ignore obs {}", obstacle->id());
      return true;
    }
    iterator = prev_iter;
  }
  return false;
}

void MotorwayBevFenceFilterDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode MotorwayBevFenceFilterDecider::Execute(TaskInfo& task_info) {
  return ErrorCode();
}

}  // namespace planning
}  // namespace neodrive
