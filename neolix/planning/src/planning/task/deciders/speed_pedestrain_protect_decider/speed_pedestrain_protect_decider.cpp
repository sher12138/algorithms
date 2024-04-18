#include "speed_pedestrain_protect_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_limit_trans.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

namespace {
const auto& pedestrain_protect_config_{
    config::PlanningConfig::Instance()
        ->planning_research_config()
        .speed_pedestrain_protect_decider_config};
constexpr double kStaticSpeed = 0.1;
}  // namespace

SpeedPedestrianProtectDecider::SpeedPedestrianProtectDecider() {
  name_ = "SpeedPedestrianProtectDecider";
}

SpeedPedestrianProtectDecider::~SpeedPedestrianProtectDecider() { Reset(); }

ErrorCode SpeedPedestrianProtectDecider::Execute(TaskInfo& task_info) {
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

void SpeedPedestrianProtectDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    double speed_limit =
        std::min(static_pedestrian_limit_, dynamic_pedestrian_limit_);
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::PEDESTRAIN);
    internal_speed_limit.add_upper_bounds(speed_limit);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "PEDESTRAIN {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        speed_limit, 0.0);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

bool SpeedPedestrianProtectDecider::Process(TaskInfo& task_info) {
  /// Init and compute front caution SL bound
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return false;
  }
  /// Extract pedestrian info
  ExtractPedestrianInfo(task_info);
  /// Static pedestrian speed limit
  static_pedestrian_limit_ = ComputeStaticPedestrianSpeedLimit();
  /// Dynamic pedestrian speed limit
  dynamic_pedestrian_limit_ = ComputeDynamicPedestrianSpeedLimit();
  /// Update speed limit
  update_limited_speed_ =
      (static_pedestrian_limit_ + kMathEpsilon < max_speed_) ||
      (dynamic_pedestrian_limit_ + kMathEpsilon < max_speed_);

  return true;
}

bool SpeedPedestrianProtectDecider::Init(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  adc_current_l_ = task_info.curr_sl().l();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = inside_data.vel_v;
  adc_boundary_ = task_info.adc_boundary_origin();

  update_limited_speed_ = false;
  max_speed_ = DataCenter::Instance()->drive_strategy_max_speed();
  init_state_ = {adc_current_s_, adc_current_v_, 0.0};
  ignore_pedestrain_.clear();

  ClearObsHistoryInfo();

  if (task_info.current_frame()->outside_planner_data().path_data == nullptr) {
    LOG_ERROR("path_data is nullptr.");
    return false;
  }

  if (!ComputeFrontCautionBoundAlongPath(
          inside_data,
          *(task_info.current_frame()->outside_planner_data().path_data))) {
    LOG_ERROR("ComputeFrontCautionBoundAlongPath err.");
    return false;
  }

  return true;
}

bool SpeedPedestrianProtectDecider::DataCheck(TaskInfo& task_info) {
  if (task_info.last_frame() == nullptr) {
    LOG_ERROR("last_frame is nullptr.");
    return false;
  }

  if (task_info.current_frame() == nullptr) {
    LOG_ERROR("current_frame is nullptr.");
    return false;
  }
  if (task_info.current_frame()->mutable_outside_planner_data() == nullptr) {
    LOG_ERROR("mutable_outside_planner_data() is nullptr.");
    return false;
  }

  return true;
}

void SpeedPedestrianProtectDecider::ClearObsHistoryInfo() {
  for (auto& iter : pedestrian_info_) {
    iter.second.lost_cnt++;
  }
  auto pedestrian_info_tmp = pedestrian_info_;
  for (auto& iter : pedestrian_info_) {
    if (iter.second.lost_cnt > 4) {
      pedestrian_info_tmp.erase(iter.first);
    }
  }
  std::swap(pedestrian_info_, pedestrian_info_tmp);
  std::string log_str = "id: ";
  for (const auto& iter : pedestrian_info_) {
    log_str += std::to_string(iter.first) + ", ";
  }
  LOG_INFO("pedestrian_info_ num {}; {}", pedestrian_info_.size(), log_str);
}

bool SpeedPedestrianProtectDecider::ComputeFrontCautionBoundAlongPath(
    const InsidePlannerData& inside_data, const PathData& path_data) {
  if (path_data.frenet_path().num_of_points() < 2) {
    LOG_ERROR("path_data.frenet_path.num_of_points[{}] < 2.",
              path_data.frenet_path().num_of_points());
    return false;
  }
  const auto& veh_frenet_path = path_data.frenet_path().points();
  const auto& half_width = VehicleParam::Instance()->width() * 0.5;
  static_lateral_buffer_ = std::max(0.6, max_speed_ * 0.2);
  dynamic_lateral_buffer_ = std::max(1.0, max_speed_ * 0.5);
  const auto max_s =
      std::max(static_cast<double>(pedestrain_protect_config_.long_check_dis),
               adc_current_v_ * pedestrain_protect_config_.long_check_time);
  LOG_INFO(
      "static_lateral_buffer_, dynamic_lateral_buffer_, max_s,: {:.3f}, "
      "{:.3f}, {:.3f}",
      static_lateral_buffer_, dynamic_lateral_buffer_, max_s);

  static_attention_bound_.clear();
  dynamic_attention_bound_.clear();
  for (std::size_t i = 0; i < veh_frenet_path.size(); i++) {
    if (veh_frenet_path[i].s() > max_s + adc_current_s_) {
      break;
    }
    PedestrianAttentionBound static_bound{
        .s = veh_frenet_path[i].s(),
        .l = veh_frenet_path[i].l(),
        .l_s = veh_frenet_path[i].l() - half_width - static_lateral_buffer_,
        .l_e = veh_frenet_path[i].l() + half_width + static_lateral_buffer_};
    static_attention_bound_.emplace_back(std::move(static_bound));
    PedestrianAttentionBound dynamic_bound{
        .s = veh_frenet_path[i].s(),
        .l = veh_frenet_path[i].l(),
        .l_s = veh_frenet_path[i].l() - half_width - dynamic_lateral_buffer_,
        .l_e = veh_frenet_path[i].l() + half_width + dynamic_lateral_buffer_};
    dynamic_attention_bound_.emplace_back(std::move(dynamic_bound));
  }

  return !static_attention_bound_.empty();
}

void SpeedPedestrianProtectDecider::ExtractPedestrianInfo(TaskInfo& task_info) {
  for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
    if (obs == nullptr) {
      continue;
    }
    if (obs->type() != Obstacle::ObstacleType::PEDESTRIAN) {
      continue;
    }
    if (obs->PolygonBoundary().end_s() < adc_current_s_) {
      continue;
    }

    UpdataDynPedestrianInfo(obs);
  }

  for (const auto& obs : task_info.decision_data()->static_obstacle()) {
    if (obs == nullptr) {
      continue;
    }
    if (obs->type() != Obstacle::ObstacleType::PEDESTRIAN) {
      continue;
    }
    if (obs->PolygonBoundary().end_s() < adc_current_s_) {
      continue;
    }

    UpdataDynPedestrianInfo(obs);
  }
}

void SpeedPedestrianProtectDecider::UpdataDynPedestrianInfo(
    const Obstacle* const obs) {
  auto& pedestrian_info = pedestrian_info_[obs->id()];

  pedestrian_info.boundary_history.push_back(obs->PolygonBoundary());
  pedestrian_info.theta.push_back(obs->velocity_heading());
  pedestrian_info.velocity.push_back(obs->speed());
  pedestrian_info.lost_cnt = 0;

  if (pedestrian_info.boundary_history.size() >
      pedestrain_protect_config_.history_data_size) {
    pedestrian_info.boundary_history.pop_front();
    pedestrian_info.theta.pop_front();
    pedestrian_info.velocity.pop_front();
  }
}

double SpeedPedestrianProtectDecider::ComputeStaticPedestrianSpeedLimit() {
  double ans = max_speed_;

  /// extract current is on attention bounds and is static
  std::vector<std::pair<int, int>> static_infos{};
  for (const auto& [id, info] : pedestrian_info_) {
    if (info.boundary_history.empty() || info.theta.empty() ||
        info.velocity.empty()) {
      continue;
    }
    if (info.velocity.back() > kStaticSpeed) {
      continue;
    }
    int index{0};
    if (!HasOverlapWithAttentionBound(static_attention_bound_,
                                      info.boundary_history.back(), &index)) {
      continue;
    }
    static_infos.push_back({id, index});
  }
  if (static_infos.empty()) return ans;

  /// compute static pedestrian speed limit
  double forward_time = 3.0;
  for (const auto& static_info : static_infos) {
    std::array<double, 3> end_state = ComputeStaticEndState(static_info);
    double speed_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
        "static_pedestrian_limit", init_state_, end_state, max_speed_,
        forward_time);
    ans = std::min(ans, speed_limit);
  }

  return ans;
}

double SpeedPedestrianProtectDecider::ComputeDynamicPedestrianSpeedLimit() {
  double ans = DataCenter::Instance()->drive_strategy_max_speed();

  /// extract current is on attention bounds and is dynamic
  std::vector<std::pair<int, int>> dynamic_infos{};
  for (const auto& [id, info] : pedestrian_info_) {
    if (info.boundary_history.empty() || info.theta.empty() ||
        info.velocity.empty()) {
      continue;
    }
    if (info.velocity.back() <= kStaticSpeed) {
      continue;
    }
    int index{0};
    if (!HasOverlapWithAttentionBound(dynamic_attention_bound_,
                                      info.boundary_history.back(), &index)) {
      continue;
    }
    dynamic_infos.push_back({id, index});
  }
  if (dynamic_infos.empty()) return ans;

  /// compute dynamic pedestrian speed limit
  double forward_time = 3.0;
  for (const auto& dynamic_info : dynamic_infos) {
    std::array<double, 3> end_state = ComputeDynamicEndState(dynamic_info);
    double speed_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
        "dynamic_pedestrian_limit", init_state_, end_state, max_speed_,
        forward_time);
    ans = std::min(ans, speed_limit);
  }

  return ans;
}

bool SpeedPedestrianProtectDecider::HasOverlapWithAttentionBound(
    const std::vector<PedestrianAttentionBound>& attention_bound,
    const Boundary& boundary, int* index) const {
  if (boundary.end_s() < attention_bound.front().s ||
      boundary.start_s() > attention_bound.back().s) {
    return false;
  }
  auto start_it = std::lower_bound(
      attention_bound.begin(), attention_bound.end(), boundary.start_s() - 0.1,
      [](const PedestrianAttentionBound& bound, double value) {
        return bound.s < value;
      });
  auto end_it = std::lower_bound(attention_bound.begin(), attention_bound.end(),
                                 boundary.end_s() + 0.1,
                                 [](const PedestrianAttentionBound& bound,
                                    double value) { return bound.s < value; });
  if (start_it == attention_bound.end()) {
    return false;
  }
  for (auto it = start_it; it <= end_it && it != attention_bound.end(); ++it) {
    Segment2d line_1({it->s, it->l_s}, {it->s, it->l_e});
    Segment2d line_2({it->s, boundary.start_l()}, {it->s, boundary.end_l()});
    if (line_1.has_intersect(line_2)) {
      *index = std::distance(attention_bound.begin(), it);
      return true;
    }
  }
  return false;
}

std::array<double, 3> SpeedPedestrianProtectDecider::ComputeStaticEndState(
    const std::pair<int, int>& static_info) {
  std::array<double, 3> end_state{0.0, 0.0, 0.0};
  end_state[0] = static_attention_bound_[static_info.second].s;

  const auto& boundary =
      pedestrian_info_[static_info.first].boundary_history.back();
  double lateral_dis =
      boundary.distance_to({static_attention_bound_[static_info.second].s,
                            static_attention_bound_[static_info.second].l}) -
      VehicleParam::Instance()->width() * 0.5;

  if (lateral_dis < static_lateral_buffer_) {
    double lat_limit_speed = std::max(
        0.0, lateral_dis * pedestrain_protect_config_.lat_dis_speed_ratio);
    lat_limit_speed = (boundary.distance_to(adc_boundary_) > 2.0)
                          ? std::max(lat_limit_speed, 1.0)
                          : lat_limit_speed;
    end_state[1] = lat_limit_speed;
  } else {
    end_state[1] = max_speed_;
  }

  LOG_INFO("obs [{}] lateral_dis {:.3f}, end_state:{:.3f} ,{:.3f}, {:.3f}",
           static_info.first, lateral_dis, end_state[0], end_state[1],
           end_state[2]);
  return end_state;
}

std::array<double, 3> SpeedPedestrianProtectDecider::ComputeDynamicEndState(
    const std::pair<int, int>& dynamic_info) {
  const auto& boundary_history =
      pedestrian_info_[dynamic_info.first].boundary_history;

  std::array<double, 3> end_state{0., 0., 0.};
  end_state[0] = dynamic_attention_bound_[dynamic_info.second].s;

  // step1:
  const auto& boundary = boundary_history.back();
  double lateral_dis =
      boundary.distance_to({dynamic_attention_bound_[dynamic_info.second].s,
                            dynamic_attention_bound_[dynamic_info.second].l}) -
      VehicleParam::Instance()->width() * 0.5;

  double lat_limit_speed = max_speed_;
  if (lateral_dis > dynamic_lateral_buffer_) {
    end_state[1] = lat_limit_speed;
    LOG_INFO("obs [{}] end_state:{:.3f} ,{:.3f}, {:.3f}", dynamic_info.first,
             end_state[0], end_state[1], end_state[2]);
    return end_state;
  }

  lat_limit_speed = std::max(
      lateral_dis * pedestrain_protect_config_.lat_dis_speed_ratio, 0.0);

  // step2:
  std::deque<double> history_lat_dis_vec{};
  double path_match_pt_l = dynamic_attention_bound_[dynamic_info.second].l;
  for (const auto& iter : boundary_history) {
    double history_lat_dis =
        std::min(std::abs(iter.start_l() - path_match_pt_l),
                 std::abs(iter.end_l() - path_match_pt_l)) -
        VehicleParam::Instance()->width() * 0.5;
    if (history_lat_dis_vec.empty()) {
      history_lat_dis_vec.push_back(history_lat_dis);
    } else {
      history_lat_dis_vec.push_back(history_lat_dis_vec.back() * 0.6 +
                                    history_lat_dis * 0.4);
    }
  }
  double history_l_delta =
      history_lat_dis_vec.back() - history_lat_dis_vec.front();
  double history_s_delta = std::abs(boundary_history.back().end_s() -
                                    boundary_history.front().end_s());
  double tan_l_s = history_l_delta / std::max(1e-6, history_s_delta);
  LOG_INFO(
      "lateral_dis and origin lat_limit_speed :{:.3f}, {:.3f}; "
      "history_l_delta, history_s_delta, tan_l_s:{:.3f}, {:.3f}, {:.3f}",
      lateral_dis, lat_limit_speed, history_l_delta, history_s_delta, tan_l_s);

  // step3:
  if ((std::abs(tan_l_s) < pedestrain_protect_config_.tan_threshold) &&
      (adc_current_v_ < lat_limit_speed) && lateral_dis > 0.3) {
    // tan10 = 0.1763
    // ignore_pedestrain_.insert(dynamic_info.first);
    // LOG_INFO(
    //     "pedestrain [{}] is straight, slow and keep safe distance, ignore.",
    //     dynamic_info.first);
  } else if (tan_l_s < -pedestrain_protect_config_.tan_threshold) {
    lat_limit_speed /= std::min(2.0, (1.0 + 2.0 * std::abs(tan_l_s)));
    LOG_INFO("pedestrain [{}] cutin,lat_limit_speed {:.3f}.",
             dynamic_info.first, lat_limit_speed);
  }

  if (boundary.distance_to(adc_boundary_) > 2.0) {
    lat_limit_speed = std::max(lat_limit_speed, 1.0);
  }
  end_state[1] = lat_limit_speed;
  LOG_INFO("obs [{}] end_state:{:.3f} ,{:.3f}, {:.3f}", dynamic_info.first,
           end_state[0], end_state[1], end_state[2]);
  return end_state;
}

}  // namespace planning
}  // namespace neodrive
