#pragma once

#include <deque>
#include <string>
#include <vector>

#include "src/planning/common/speed/speed_data.h"
#include "src/planning/common/speed/speed_point.h"
#include "src/planning/common/st_graph_data/st_graph_data.h"
#include "src/planning/math/level_k/context.h"

namespace neodrive {
namespace planning {

struct STBoundInfo {
  STBoundInfo();
  STBoundInfo(const STPoint& point, const STGraphBoundary::BoundaryType type,
              const int32_t id, const double v)
      : st_point(point), boundary_type(type), obstacle_id(id), obstacle_v(v) {}
  STPoint st_point{};
  STGraphBoundary::BoundaryType boundary_type{
      STGraphBoundary::BoundaryType::UNKNOWN};
  int32_t obstacle_id{-1};
  double obstacle_v{0.0};
};

struct STGoalSInfo {
  STPoint goal_t_s{};
  STGraphBoundary::BoundaryType boundary_type{
      STGraphBoundary::BoundaryType::UNKNOWN};
};

struct STGoalVInfo {
  STPoint goal_t_v{};
  STGraphBoundary::BoundaryType boundary_type{
      STGraphBoundary::BoundaryType::UNKNOWN};
};

struct STGoalAInfo {
  STPoint goal_t_a{};
  STGraphBoundary::BoundaryType boundary_type{
      STGraphBoundary::BoundaryType::UNKNOWN};
};

struct DPStData {
  STGraphData st_graph_data{};
  std::vector<std::vector<STGraphPoint>> cost_table{};
  std::vector<STPoint> dp_st_profile{};
  std::vector<STBoundInfo> upper_tunnel{};      // upper_s_bounds_
  std::vector<STBoundInfo> lower_tunnel{};      // lower_s_bounds_
  std::vector<STBoundInfo> upper_iter_bound{};  // upper_s_bounds_
  std::vector<STBoundInfo> lower_iter_bound{};
  std::vector<STGoalVInfo> upper_iter_v_bound{};  // upper_v_bounds_
  std::vector<STGoalVInfo> lower_iter_v_bound{};
  std::vector<STGoalSInfo> goal_s_upper{};  // goal_upper_s_
  std::vector<STGoalSInfo> goal_s_lower{};  // goal_lower_s_
  std::vector<STGoalVInfo> goal_v_upper{};
  std::vector<STGoalVInfo> goal_v_lower{};
  std::vector<STPoint> goal_s{};
  std::vector<STPoint> goal_v{};
  std::vector<STPoint> goal_a{};
  std::vector<SpeedPoint> smoothed_speed{};
  std::vector<int> dynamic_obstacle_consider{};
  std::vector<int> static_obstacle_ignore{};

  void Reset() {
    dp_st_profile.clear();
    cost_table.clear();
    upper_tunnel.clear();
    lower_tunnel.clear();
    goal_s_upper.clear();
    goal_s_lower.clear();
    goal_v_upper.clear();
    goal_v_lower.clear();
    goal_s.clear();
    goal_v.clear();
    goal_a.clear();
    smoothed_speed.clear();
    dynamic_obstacle_consider.clear();
    static_obstacle_ignore.clear();
  }
};

struct GeoSTVData {
  std::vector<SpeedPoint> smoothed_speed{};

  void Reset() { smoothed_speed.clear(); }
};

struct VehicleInfo {
  int id{0};
  double current_t;
  Vec2d xy_position;
  Vec2d sl_position;
  Vec2d sl_velocity;
  Vec2d sl_accel;
  double heading{0.0};
  double heading_diff{0.0};
  double front_suspension{3.0};
  double rear_suspension{3.0};
  double left_suspension{1.091 / 2.0};
  double right_suspension{1.091 / 2.0};
  double react_time{0.1};
  double max_longitudinal_acc{2.0};
  double max_lateral_acc{0.5};
  double max_longitudinal_brake{5.0};
  double obs_offset{0.0};
  Obstacle* obs_ptr;

  VehicleInfo() = default;

  VehicleInfo(const int obs_id, const Vec2d& p, const Vec2d& v, const Vec2d& a)
      : id(obs_id), sl_position(p), sl_velocity(v), sl_accel(a){};

  VehicleInfo(const int obs_id, const Vec2d& p, const Vec2d& v, const Vec2d& a,
              const std::vector<double>& suspension, const double time,
              const std::vector<double>& param_limit)
      : id(obs_id),
        sl_position(p),
        sl_velocity(v),
        sl_accel(a),
        front_suspension(suspension[0]),
        rear_suspension(suspension[1]),
        left_suspension(suspension[2]),
        right_suspension(suspension[3]),
        react_time(time),
        max_longitudinal_acc(param_limit[0]),
        max_lateral_acc(param_limit[1]),
        max_longitudinal_brake(param_limit[2]){};

  VehicleInfo(const int obs_id, const Vec2d& xy_p, const Vec2d& p,
              const Vec2d& v, const Vec2d& a, const double obs_heading,
              const std::vector<double>& suspension, const double time,
              const std::vector<double>& param_limit)
      : id(obs_id),
        xy_position(xy_p),
        sl_position(p),
        sl_velocity(v),
        sl_accel(a),
        heading(obs_heading),
        front_suspension(suspension[0]),
        rear_suspension(suspension[1]),
        left_suspension(suspension[2]),
        right_suspension(suspension[3]),
        react_time(time),
        max_longitudinal_acc(param_limit[0]),
        max_lateral_acc(param_limit[1]),
        max_longitudinal_brake(param_limit[2]){};

  void set_param(const std::vector<double>& suspension, const double time,
                 const std::vector<double>& param_limit) {
    front_suspension = suspension[0];
    rear_suspension = suspension[1];
    left_suspension = suspension[2];
    right_suspension = suspension[3];
    react_time = time;
    max_longitudinal_acc = param_limit[0];
    max_lateral_acc = param_limit[1];
    max_longitudinal_brake = param_limit[2];
  }
};

struct GameTheoryParam {
  double param_a = 1.0;
  double param_c = 1.0;
  double param_margin = 0.2;
  double rush_time = 0.0;
  double yield_time = 0.5;
  double yield_advance_response_distance = 200.0;
  double rush_margin_distance = 0.0;
};

enum class MotorwayInteractiveType : int {
  NONE = 0,
  STRAIGHT_MERGE = 1,
  STRAIGHT_MEETING = 2,
  LEFT_MERGE = 3,
  LEFT_MEETING = 4,
  RIGHT_MERGE = 5,
  RIGHT_MEETING = 6,
  U_MERGE = 7,
  U_MEETING = 8,
  MERGE_IN_ROAD = 9,
  CUSTOM = 10,
};

enum class BackInteractiveType : int {
  NONE = 0,
  MERGE = 1,
  MEETING = 2,
  CUSTOM = 3,
};

enum class RightOfWay : int {
  UNKNOWN = 0,
  ADVANTAGE = 1,
  UNADVANTAGE = 2,
  EQUAL = 3,
};

struct ConnectionConflictInfo {
  MotorwayInteractiveType type;
  BackInteractiveType back_type;
  RightOfWay way_right;
  uint64_t lane_id{};
  std::vector<double> conflict_area_bound{};
  VehicleInfo agent;
  GameTheoryParam param;
};

struct PredictionCurves {
  std::vector<Vec2d> left_curves{};
  std::vector<Vec2d> right_curves{};

  void Reset() {
    left_curves.clear();
    right_curves.clear();
  }
};

struct IntegrativeConflictInfo {
  enum class ConflictAreaCalcMethod : int {
    UNKNOWN = 0,
    MAP = 1,
    PATH = 2,
    PREDICT = 3,
  };
  int id;
  double style;
  ConflictAreaCalcMethod conflict_area_calc_method;
};

struct MergeAreaEgoStateSequence {
  bool is_rush_decision = false;
  std::vector<double> deduction_ego_v_sequence{};
  std::vector<double> deduction_ego_p_sequence{};
  std::vector<double> deduction_ego_a_sequence{};
  std::vector<double> deduction_ego_t_sequence{};

  void Reset() {
    is_rush_decision = false;
    deduction_ego_v_sequence.clear();
    deduction_ego_p_sequence.clear();
    deduction_ego_a_sequence.clear();
    deduction_ego_t_sequence.clear();
  }
};

struct SpeedContext {
  bool collision_flag{false};
  DPStData dp_st_data{};
  GeoSTVData geo_stv_data{};
  std::string speed_optimizer{""};
  std::string speed_smoother{""};
  bool optimizer_ok{false};
  double target_speed{0.0};
  double target_s{-1.0};
  bool is_emergency{false};

  void Reset() {
    dp_st_data.Reset();
    geo_stv_data.Reset();
    speed_optimizer = "";
    speed_smoother = "";
    collision_flag = false;
    optimizer_ok = false;
  }
};

struct MotorwaySequenceBound {
  std::vector<STPoint> upper_s_tunnel{};  // upper_s_bounds
  std::vector<STPoint> lower_s_tunnel{};  // lower_s_bounds
  std::vector<STPoint> upper_v_tunnel{};  // upper_v_bounds
  std::vector<STPoint> lower_v_tunnel{};  // lower_v_bounds

  void Reset() {
    upper_s_tunnel.clear();
    lower_s_tunnel.clear();
    upper_v_tunnel.clear();
    lower_v_tunnel.clear();
  }
};
enum class MotorwaySequenceDecisionType {
  NONE = 0,
  COMBINE = 1,  /// final combine sequence decision data
  ONE_CIPV = 2,
  MULTI_CIPV = 3,
  GHOST = 4,
  PEDESTRIAN = 5,
  CONFLICT_MERGE = 6,
  BACK_CIPV = 7,
};
struct MotorwaySequenceDecisionData {
  MotorwaySequenceDecisionType source{MotorwaySequenceDecisionType::NONE};

  std::vector<STPoint> goal_s{};
  std::vector<STPoint> goal_v{};
  std::vector<STPoint> goal_a{};
};

struct MotorwaySpeedContext {
  std::vector<MotorwaySequenceDecisionData> goal_decision{};
  bool trigger_backcipv{false};
  double conflict_brake{0.0};
  MotorwaySequenceBound tunnel_decision{};  // optional
  std::vector<STPoint> final_goal_s{};
  std::vector<STPoint> final_goal_v{};
  std::vector<STPoint> final_goal_a{};

  // for only one cipv
  double target_speed{0.0};
  double target_s{-1.0};
  bool is_emergency{false};

  void Reset() {
    goal_decision.clear();
    trigger_backcipv = false;
    conflict_brake = 0.0;

    tunnel_decision.Reset();
    final_goal_s.clear();
    final_goal_v.clear();
    final_goal_a.clear();

    target_speed = 0.;
    target_s = -1.;
    is_emergency = false;
  }
};

};  // namespace planning
}  // namespace neodrive