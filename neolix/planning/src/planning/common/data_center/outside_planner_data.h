#pragma once

#include "obstacles_intention.pb.h"
#include "src/planning/common/data_center/path_context.h"
#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/common/math/box2d.h"
#include "src/planning/common/obstacle/decision_data.h"
#include "src/planning/common/path/path_data.h"
#include "src/planning/common/speed/speed_data.h"
#include "src/planning/common/trajectory/publishable_trajectory.h"
#include "src/planning/common/trajectory/trajectory_point.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

enum class LaneBorrowScenario { NONE = 0, ENTER, CRUISE, BACK };
enum GoDirectionType : int { TURN_LEFT, GO_STRAIGHT, TURN_RIGHT };

enum class PathPlannerType { MAIN = 0, BACKUP, ALL_FAIL };

struct ReversedObsLateralDis {
  double l_dis{std::numeric_limits<double>::infinity()};
  bool cutin_flag{false};
  std::size_t stop_cutin_count{0};
  double updated{false};
  void Reset() {
    l_dis = std::numeric_limits<double>::infinity();
    cutin_flag = false;
    stop_cutin_count = 0;
    updated = false;
  }
};
struct ReversedObsData {
  std::unordered_map<int, ReversedObsLateralDis> obs_last_l_dis{};
  void Reset() { obs_last_l_dis.clear(); }
};

struct IntervelSpeedLimit {
  double start_s = 0.0;
  double end_s = 0.0;
  double speed_limit = 0.0;  // unit m/s
  IntervelSpeedLimit() = default;
  IntervelSpeedLimit(double _start_s, double _end_s, double _speed_limit)
      : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit){};
};

struct LaneBorrowPrepareCollisionInfo {
  int id{};                 // obstacle id
  Polygon2d obs_polygon{};  // obstacle polygon when relative_time is 0.0
  Polygon2d obs_collide_polygon{};  // obstacle closet collide path_s's polygon
  Polygon2d adc_collide_polygon{};  // first collide,closet adc polygon
  Boundary obs_boundary{};  // obstacle boundary when relative_time is 0.0
  Boundary
      obs_collide_boundary{};  // obstacle closest collide path_s's boundary
  Boundary adc_collide_boundary{};  // first collide,closet adc boundary
  bool reverse{false};              // true: reverse, false: forward
  double project_vel{0.};           // obstacle project velocity
  double s{0.};                     // path_s
  double t{0.};                     // obstacle relative_time
  bool skip{false};                 // obstacle whether skip
};

struct LaneBorrowReturnCollisionInfo {
  int id{};                 // obstacle id
  Polygon2d obs_polygon{};  // obstacle polygon when relative_time is 0.0
  Polygon2d obs_collide_polygon{};  // obstacle closet collide path_s's polygon
  Polygon2d adc_collide_polygon{};  // first collide,closet adc polygon
  Boundary obs_boundary{};  // obstacle boundary when relative_time is 0.0
  Boundary
      obs_collide_boundary{};  // obstacle closest collide path_s's boundary
  Boundary adc_collide_boundary{};  // first collide,closet adc boundary
  bool reverse{false};              // true: reverse, false: forward
  double project_vel{0.};           // obstacle project velocity
  double s{0.};                     // path_s
  double t{0.};                     // obstacle relative_time
  bool skip{false};                 // obstacle whether skip
};

struct PathObstacleBoundary {
  Boundary boundary{};
  Obstacle obstacle{};
  Obstacle *obstacle_ptr{nullptr};
};

struct PathObstacleDecision {
  Decision::DecisionType decision_type{Decision::DecisionType::IGNORE};
  PathObstacleBoundary obstacle_boundary{};
  Box2d obstacle_box{{0, 0}, 0, 0, 0};   // just expand 0.3m in all directions.
  std::vector<Vec2d> obstacle_box_xy{};  // expand 0.3 + left/right 2m
  std::vector<Vec2d> obstacle_box_sl{};  // expand 0.3 + left/right 2m
  Boundary boundary{};                   // expand 0.3 + left/right 2m
  double l_bound{0};
  std::size_t motion_intention{0};
};

struct PathObstacleContext {
  bool has_data{false};
  std::size_t curr_pass_by_mode{0};
  std::pair<double, double> passable_l_range{};
  Boundary adc_boundary{};
  std::vector<PathObstacleDecision> obstacle_decision{};
  // [mins, minl, maxs, maxl]
  std::vector<std::array<double, 4>> attention_boxes{};

  void Reset() {
    has_data = false;
    curr_pass_by_mode = 0;
    passable_l_range = {0, 0};
    adc_boundary.reset();
    obstacle_decision.clear();
  }
};

struct PathObserveRefLInfo {
  struct AttentionInfo {
    std::vector<Boundary> ranges{};
    std::vector<Obstacle> obstacles{};

    void Reset() {
      ranges.clear();
      obstacles.clear();
    }
  };
  enum class RefLType { LANEBORROW = 0, REVERSE, FOLLOW, FORCE, NONE };
  RefLType type{RefLType::NONE};
  double observe_ref_l{0.};
  SLPoint observe_ref_sl{};
  Vec2d observe_ref_pt{0., 0.};
  std::vector<Obstacle> attention_dynamic_obstacles{};
  HornLightsCmd::TurnLevel light_turn{HornLightsCmd::NO_TURN};
  AttentionInfo front_attention{};  // only for reverse obs
  AttentionInfo left_attention{};   // only for forward obs
  AttentionInfo right_attention{};  // only for forward obs

  void Reset() {
    observe_ref_l = 0.;
    observe_ref_pt = {0., 0.};
    light_turn = HornLightsCmd::NO_TURN;
    attention_dynamic_obstacles.clear();
    front_attention.Reset();
    left_attention.Reset();
    right_attention.Reset();
  }
};

struct DynamicObsCollisionCheckData {
  int id{0};
  bool first_time{true};
  double heading_diff{0.0};
  double l_dis{std::numeric_limits<double>::infinity()};
  bool is_ignore_obs_in_dp_st{false};
  bool is_ignore_obs_in_backup{false};

  void Reset() {
    id = 0;
    first_time = true;
    heading_diff = 0.0;
    l_dis = std::numeric_limits<double>::infinity();
    is_ignore_obs_in_dp_st = false;
    is_ignore_obs_in_backup = false;
  }
};

struct SpeedDynamicObsCollisionRiskData {
  std::unordered_map<int, DynamicObsCollisionCheckData>
      left_dynamic_obstacles_data;
  std::unordered_map<int, DynamicObsCollisionCheckData>
      right_dynamic_obstacles_data;
  std::unordered_map<int, DynamicObsCollisionCheckData>
      front_dynamic_obstacles_data;

  void Reset() {
    left_dynamic_obstacles_data.clear();
    right_dynamic_obstacles_data.clear();
    front_dynamic_obstacles_data.clear();
  }
};

enum class LaneChangeAdjType {
  CONST = 0,
  ACCELERATION = 1,
  DECELERATION = 2,
  SLIGHTDECELERATION = 3,
  NONE
};

struct LaneChangeEnable {
  bool left_lane_change_enable{true};
  bool right_lane_change_enable{true};
  void Reset() {
    left_lane_change_enable = true;
    right_lane_change_enable = true;
  }
};
// struct LeftLaneChangeEnable {
//   bool left_lane_ll_change_enable{false};
//   bool left_lane_lr_change_enable{false};
//   void Reset() {
//     left_lane_ll_change_enable = false;
//     left_lane_lr_change_enable = false;
//   }
// };
struct LeftChangeTcInfo {
  // tc , v_lat
  std::pair<double, double> ll_tc_para_{};
  std::pair<double, double> lf_tc_para_{};
};

struct RightChangeTcInfo {
  // tc , v_lat
  std::pair<double, double> rf_tc_para_{};
  std::pair<double, double> rr_tc_para_{};
};

struct LeftChangeMssMapInitPoint {
  // theta v , Sr[0]
  std::pair<double, double> ll_mss_init_point{};
  std::pair<double, double> ll_mss_slope{};
  std::pair<double, double> lr_mss_init_point{};
  std::pair<double, double> lr_mss_slope{};
};

struct RightChangeMssMapInitPoint {
  // theta v , Sr[0]
  std::pair<double, double> rf_mss_init_point{};
  std::pair<double, double> rf_mss_slope{};
  std::pair<double, double> rr_mss_init_point{};
  std::pair<double, double> rr_mss_slope{};
};

struct LaneChangeBoundPara {
  double left_bound_end{0.0};
  double right_bound_end{0.0};
  double left_bound_start{0.0};
  double right_bound_start{0.0};
  double left_lane_change_dis{0.0};
  double right_lane_change_dis{0.0};
  void Reset() {
    left_bound_start = 0.0;
    right_bound_start = 0.0;
    left_lane_change_dis = 0.0;
    right_lane_change_dis = 0.0;
  }
};

struct LaneChangeCheckAreaBoundary {
  std::vector<Boundary> ll_check_boundarys{};
  std::vector<Boundary> lr_check_boundarys{};
  std::vector<Boundary> rl_check_boundarys{};
  std::vector<Boundary> rr_check_boundarys{};
  void Reset() {
    ll_check_boundarys.clear();
    lr_check_boundarys.clear();
    rl_check_boundarys.clear();
    rr_check_boundarys.clear();
  }
};

struct LeftChangeZeroObsInfo {
  bool has_pos_zero_obs{false};
  int pos_zero_obs_id{0};
  double pos_zero_obs_lon_dis{1000.0};
  double pos_zero_obs_lat_dis{3.5};
  double pos_zero_obs_lon_speed{30.0};
  bool pos_zero_obs_mss_judge{false};
  void Reset() {
    has_pos_zero_obs = false;
    pos_zero_obs_id = 0;
    pos_zero_obs_lon_dis = 1000.0;
    pos_zero_obs_lat_dis = 3.5;
    pos_zero_obs_lon_speed = 30.0;
    pos_zero_obs_mss_judge = false;
  }
};
struct LeftChangeFiveObsInfo {
  bool has_pos_five_obs{false};
  int pos_five_obs_id{0};
  double pos_five_obs_lon_dis{-1000.0};
  double pos_five_obs_lat_dis{3.5};
  double pos_five_obs_lon_speed{0.0};
  bool pos_five_obs_mss_judge{false};

  void Reset() {
    has_pos_five_obs = false;
    pos_five_obs_id = 0;
    pos_five_obs_lon_dis = -1000.0;
    pos_five_obs_lat_dis = 3.5;
    pos_five_obs_lon_speed = 0.0;
    pos_five_obs_mss_judge = false;
  }
};
struct LeftChangeMssModelSaftyCheck {
  LeftChangeZeroObsInfo left_change_zero_obs_info{};
  LeftChangeFiveObsInfo left_change_five_obs_info{};

  void Reset() {
    left_change_zero_obs_info.Reset();
    left_change_five_obs_info.Reset();
  }
};

struct RightChangeTwoObsInfo {
  bool has_pos_two_obs{false};
  int pos_two_obs_id{0};
  double pos_two_obs_lon_dis{1000.0};
  double pos_two_obs_lat_dis{0.0};
  double pos_two_obs_lon_speed{30.0};
  bool pos_two_obs_mss_judge{false};
  void Reset() {
    has_pos_two_obs = false;
    pos_two_obs_id = 0;
    pos_two_obs_lon_dis = 1000.0;
    pos_two_obs_lat_dis = 3.5;
    pos_two_obs_lon_speed = 30.0;
    pos_two_obs_mss_judge = false;
  }
};

struct RightChangeSevenObsInfo {
  bool has_pos_seven_obs{false};
  int pos_seven_obs_id{0};
  double pos_seven_obs_lon_dis{-1000.0};
  double pos_seven_obs_lat_dis{3.5};
  double pos_seven_obs_lon_speed{0.0};
  bool pos_seven_obs_mss_judge{false};

  void Reset() {
    has_pos_seven_obs = false;
    pos_seven_obs_id = 0;
    pos_seven_obs_lon_dis = -1000.0;
    pos_seven_obs_lat_dis = 3.5;
    pos_seven_obs_lon_speed = 0.0;
    pos_seven_obs_mss_judge = false;
  }
};
struct RightChangeMssModelSaftyCheck {
  RightChangeTwoObsInfo right_change_two_obs_info{};
  RightChangeSevenObsInfo right_change_seven_obs_info{};

  void Reset() {
    right_change_two_obs_info.Reset();
    right_change_seven_obs_info.Reset();
  }
};
struct TemporalDeduction {
  double goal_v{0};
  double time_stamp{std::numeric_limits<double>::infinity()};
  void Reset() {
    goal_v = 0;
    time_stamp = std::numeric_limits<double>::infinity();
  }
};
struct BackupCipv {
  int cipv_id{0};
  bool has_cipv{false};
  void Reset() {
    cipv_id = 0;
    has_cipv = false;
  }
};

struct ObsDecisionBound {
  double time{0.0};
  double obs_s{0.0};
  double obs_speed{0.0};
  void Reset() {
    time = 0.0;
    obs_s = 0.0;
    obs_speed = 0.0;
  }
};
struct ObsBoundPolySeris {
  std::vector<ObsDecisionBound> upper_obs_decision_poly_seris{};
  std::vector<ObsDecisionBound> lower_obs_decision_poly_seris{};
  void Reset() {
    upper_obs_decision_poly_seris.clear();
    lower_obs_decision_poly_seris.clear();
  }
};

enum class AdcCollideCornerPoint : int {
  LEFT_REAR = 0,
  RIGHT_REAR = 1,
  RIGHT_FRONT = 2,
  LEFT_FRONT = 3,
  NONE = 4
};

enum class OBS_SIDE { UNKNOWN = 0, LEFT, RIGHT };
struct AttentionObsInfo {
  bool has_obs_decision{false};
  const Obstacle *obs_ptr{nullptr};
  AdcCollideCornerPoint adc_first_collide_corner_point{
      AdcCollideCornerPoint::NONE};
  double first_collide_t{std::numeric_limits<double>::infinity()};
  double first_collide_s{std::numeric_limits<double>::infinity()};
  double first_collide_obs_project_v{0.0};
  void Reset() {
    has_obs_decision = false;
    obs_ptr = nullptr;
    adc_first_collide_corner_point = AdcCollideCornerPoint::NONE;
    first_collide_t = std::numeric_limits<double>::infinity();
    first_collide_s = std::numeric_limits<double>::infinity();
    first_collide_obs_project_v = 0.0;
  }
};

struct IterDeductionEgoState {
  double ego_v = std::numeric_limits<double>::infinity();
  double ego_a = std::numeric_limits<double>::infinity();
  double ego_p = std::numeric_limits<double>::infinity();
  double ego_t = std::numeric_limits<double>::infinity();
  void Reset() {
    ego_v = std::numeric_limits<double>::infinity();
    ego_a = std::numeric_limits<double>::infinity();
    ego_p = std::numeric_limits<double>::infinity();
    ego_t = std::numeric_limits<double>::infinity();
  }
};

struct DeductionCollisionResult {
  bool has_collision{false};
  int collision_t_index{0};
  int collision_obs_index{0};
  void Reset() {
    has_collision = false;
    collision_t_index = 0;
    collision_obs_index = 0;
  }
};

struct DeductionEgoStateSequence {
  std::vector<double> deduction_ego_v_sequence{};
  std::vector<double> deduction_ego_p_sequence{};
  std::vector<double> deduction_ego_a_sequence{};
  std::vector<double> deduction_ego_t_sequence{};
  void Reset() {
    deduction_ego_v_sequence.clear();
    deduction_ego_p_sequence.clear();
    deduction_ego_a_sequence.clear();
    deduction_ego_t_sequence.clear();
  }
};
struct DynamicasCipv {
  double cipv_obsulte_distance{10000};
  int cipv_id{0};
  double cipv_speed{10000};
  bool if_has_dynamic_obs{false};
  double heading_diff_dynamic{0};
  void Reset() {
    if_has_dynamic_obs = false;
    cipv_id = 0;
    cipv_obsulte_distance = 10000;
    cipv_speed = 10000;
    heading_diff_dynamic = 0;
  }
};

struct StaticasCipv {
  double cipv_obsulte_distance{10000};
  int cipv_id{0};
  double cipv_speed{10000};
  bool if_has_static_obs{false};
  double heading_diff_static{0};
  bool if_is_virtual{false};
  void Reset() {
    if_has_static_obs = false;
    cipv_id = 0;
    cipv_obsulte_distance = 10000;
    cipv_speed = 10000;
    heading_diff_static = 0;
    if_is_virtual = false;
  }
};

struct ReverseCautionObs {
  bool obs_bias_pos{false};
  double reverse_obs_id{0};
  double obs_speed_parallel_car{0.0};
  double lat_distance_obs2adc{0.0};
  double lon_dis_obs2adc{0.0};
  double heading_diff_obs2adc{0.0};
  double obs_absule_s{0.0};
  double obs_absule_speed{0.0};
  void Reset() {
    obs_bias_pos = false;
    reverse_obs_id = 0;
    obs_speed_parallel_car = 10000;
    lat_distance_obs2adc = 10000;
    lon_dis_obs2adc = 10000;
    heading_diff_obs2adc = 0.0;
    obs_absule_s = 0.0;
    obs_absule_speed = 0.0;
  }
};

struct SpeedObstacleDecision {
  std::vector<std::pair<STPoint, double>> lower_points{};
  std::vector<std::pair<STPoint, double>> upper_points{};
  std::vector<double> lower_points_heading_diff{};  // [-pi, pi]
  std::vector<double> upper_points_heading_diff{};
  std::vector<Polygon2d> obstalce_polygons{};  // obstacle's polygon
  std::vector<Boundary>
      obstacle_boundaries{};       // obstacle sl boundary on refer_line
  std::vector<double> sample_t{};  // obstacle point time
  std::vector<double> obstacle_pre_traj_headings{};
  Obstacle obstacle{};
  bool collide{false};
  bool reverse{false};
  bool is_in_left_turn_area{false};
  bool is_in_right_turn_area{false};
  bool risk_obs{false};
  int lower_adc_first_index{std::numeric_limits<int>::max()};
  AdcCollideCornerPoint adc_first_collide_corner_point{
      AdcCollideCornerPoint::NONE};

  bool follow{false};
  double collide_s{200.0};
  double collide_v{0.0};
  double collide_t{200.0};
  double follow_s{200.0};
  double collide_delta_l{100.0};
  double init_delta_l{100.0};
};

struct CipvDecision {
  bool has_cipv{false};
  double cipv_id{10000};
  void Reset() {
    has_cipv = false;
    cipv_id = 10000;
  }
};

struct TrafficLawSpeedLimit {
  double crosswalk_speed_limit{0.0};
  void Reset() { crosswalk_speed_limit = 0.0; }
};

struct DynamicObsCautionInfo {
  DynamicObsCautionInfo(std::string source_, double speed_, double time_,
                        double deceleration_)
      : source(source_),
        speed(speed_),
        time(time_),
        deceleration(deceleration_) {}
  std::string source{""};
  double speed{std::numeric_limits<double>::infinity()};  // v_e
  double time{std::numeric_limits<double>::infinity()};   // t_e
  double deceleration{0.0};                               // a_e
};

struct SpeedObstacleContext {
  std::vector<double> path_accumulated_s{};
  Boundary uniform_trajectory_attention_boundary{};
  std::vector<Box2d> adc_boundaries{};
  std::vector<Box2d> adc_pre_boundaries{};
  std::vector<Boundary> adc_sl_boundaries{};
  std::vector<Vec2d> adc_corner_pt_coordinate{};
  std::vector<SpeedObstacleDecision> dynamic_obstacles_decision{};
  std::vector<SpeedObstacleDecision> static_obstacles_decision{};
  std::vector<SpeedObstacleDecision> static_pre_obstacles_decision{};
  std::vector<SpeedObstacleDecision> virtual_obstacle_decision{};
  std::vector<PathPoint> pre_path_points{};
  std::vector<LaneBorrowPrepareCollisionInfo>
      borrow_lane_prepare_collision_info{};
  std::vector<LaneBorrowReturnCollisionInfo>
      borrow_lane_return_collision_info{};
  SpeedDynamicObsCollisionRiskData dynamic_obstacles_collision_risk_data{};
  CipvDecision cipv_decision{};
  DeductionEgoStateSequence iter_deduction{};
  std::vector<TemporalDeduction> temporal_deduction{};
  BackupCipv backup_cipv{};
  // true - > take  ; false - > follow;
  std::unordered_map<int, bool> iter_deduction_take_follow_decision_map;
  std::unordered_set<neodrive::global::planning::SpeedLimit::SourceType>
      nonmotorway_hard_speed_limit_acc{
          neodrive::global::planning::SpeedLimit::VEHICLE_MEETING,
          neodrive::global::planning::SpeedLimit::COLLISION_RISK,
          neodrive::global::planning::SpeedLimit::MERGE_IN,
          neodrive::global::planning::SpeedLimit::LANE_CHANGE,
          neodrive::global::planning::SpeedLimit::OBS_TURN_RIGHT_RISK};
  std::vector<IntervelSpeedLimit> combine_speed_limits{};  // s -v
  std::vector<Boundary> adc_front_road_boundaries{};
  std::unordered_set<int> dp_st_map_ignore_dynamic_obs_id{};
  std::unordered_set<int> dp_st_map_ignore_static_obs_id{};
  std::unordered_set<int> speed_backup_ignore_dynamic_obs_id{};
  std::vector<DynamicObsCautionInfo> dynamic_obs_caution_infos{};

  void Reset() {
    path_accumulated_s.clear();
    adc_boundaries.clear();
    adc_pre_boundaries.clear();
    adc_sl_boundaries.clear();
    adc_corner_pt_coordinate.clear();
    dynamic_obstacles_decision.clear();
    static_obstacles_decision.clear();
    static_pre_obstacles_decision.clear();
    virtual_obstacle_decision.clear();
    pre_path_points.clear();
    borrow_lane_prepare_collision_info.clear();
    adc_front_road_boundaries.clear();
    combine_speed_limits.clear();
    temporal_deduction.clear();
    dp_st_map_ignore_dynamic_obs_id.clear();
    dp_st_map_ignore_static_obs_id.clear();
    speed_backup_ignore_dynamic_obs_id.clear();
    dynamic_obs_caution_infos.clear();
  }
};

struct MultiModeData {
  bool is_narrow_corridor_mode{false};
  double valid_width{0.0};
  double valid_length{0.0};
  double valid_degrade_length{0.0};
  PieceBoundary valid_width_boundary{};

  void Reset() {
    is_narrow_corridor_mode = false;
    valid_width = 0.0;
    valid_length = 0.0;
    valid_degrade_length = 0.0;
    valid_width_boundary.Reset();
  }
};

struct ConflictLaneContext {
  enum class mTurnType {
    Right = 0,
    Straight = 1,
    Left = 2,
    Unknown,
    TypeSize,
  };

  uint64_t id{0};
  cyberverse::LaneInfoConstPtr lane_ptr{nullptr};
  uint64_t related_id{0};
  cyberverse::LaneInfoConstPtr related_lane_ptr{nullptr};
  int connection_type{1};
  double steering{0.0};
  bool is_stright{true};
  mutable bool is_extended{true};
  mTurnType orientation{mTurnType::Straight};
  ConflictLaneContext(const uint64_t id_) : id{id_} {}
  ConflictLaneContext(const uint64_t id_,
                      const cyberverse::LaneInfoConstPtr lptr_,
                      const uint64_t rid_,
                      const cyberverse::LaneInfoConstPtr rlptr_,
                      const int ctype_, const double meanc_, const bool sorc_,
                      const mTurnType ori_)
      : id{id_},
        lane_ptr{lptr_},
        related_id{rid_},
        related_lane_ptr{rlptr_},
        connection_type{ctype_},
        steering{meanc_},
        is_stright{sorc_},
        orientation{ori_} {}
  const void Repr(cyberverse::HDMap &hdmap) const {
    char tmp[64] = "";
    std::vector<std::pair<std::string, std::string>> ans = {};
    std::vector<std::string> hash_ = {"Right", "Straight", "Left", "Unknown"};

    ans.push_back(std::make_pair("id", hdmap.GetIdHashString(id)));
    sprintf(tmp, "%p", lane_ptr);
    ans.push_back(std::make_pair("lane_ptr", tmp));

    ans.push_back(
        std::make_pair("related_id", hdmap.GetIdHashString(related_id)));
    sprintf(tmp, "%p", related_lane_ptr);
    ans.push_back(std::make_pair("related_lane_ptr", tmp));
    sprintf(tmp, "%x", connection_type);
    ans.push_back(std::make_pair("connection_type", tmp));
    sprintf(tmp, "%.4f", steering);
    ans.push_back(std::make_pair("steering", tmp));

    ans.push_back(std::make_pair("is_stright", is_stright ? "true" : "false"));

    ans.push_back(
        std::make_pair("orientation", hash_[static_cast<int>(orientation)]));

    for (auto it = ans.begin(); it != ans.end(); it++)
      LOG_INFO("{}: {}", it->first, it->second);
  }
  bool operator<(const ConflictLaneContext &b) const { return id < b.id; }
  bool operator==(const ConflictLaneContext &b) const { return id == b.id; }
};

struct MergingStopLineInfo {
  uint64_t lane_id{0};
  uint64_t root_lane_id{0};
  math::AD2 intersection{0.0, 0.0};
  double ego_s{0.0};
  double merging_s{0.0};
  double sum_length{0.0};
  mutable double ego_rd{0.0};
  MergingStopLineInfo(const uint64_t id) : lane_id{id} {}
  MergingStopLineInfo(const uint64_t id, const double es)
      : lane_id{id}, ego_s{es} {}
  MergingStopLineInfo(const uint64_t id, const uint64_t rid, const math::AD2 &p,
                      const double es, const double ms, const double sumL,
                      const double rd)
      : lane_id{id},
        root_lane_id{rid},
        intersection{p},
        ego_s{es},
        merging_s{ms},
        sum_length{sumL},
        ego_rd{rd} {}
  const void Repr(cyberverse::HDMap &hdmap) const {
    char tmp[64] = "";
    std::vector<std::pair<std::string, std::string>> ans = {};

    ans.push_back(std::make_pair("lane_id", hdmap.GetIdHashString(lane_id)));

    ans.push_back(
        std::make_pair("root_lane_id", hdmap.GetIdHashString(root_lane_id)));
    sprintf(tmp, "(%.4f,%.4f)", intersection[0], intersection[1]);
    ans.push_back(std::make_pair("intersection", tmp));
    sprintf(tmp, "%.4f", ego_s);
    ans.push_back(std::make_pair("ego_s", tmp));
    sprintf(tmp, "%.4f", merging_s);
    ans.push_back(std::make_pair("merging_s", tmp));
    sprintf(tmp, "%.4f", sum_length);
    ans.push_back(std::make_pair("sum_length", tmp));
    sprintf(tmp, "%.4f", ego_rd);
    ans.push_back(std::make_pair("ego_rd", tmp));

    for (auto it = ans.begin(); it != ans.end(); it++)
      LOG_INFO("{}: {}", it->first, it->second);
  }
  bool operator<(const MergingStopLineInfo &a) const {
    if (math::Sign(a.ego_s - ego_s) == 0)
      return lane_id < a.lane_id;
    else
      return ego_s > a.ego_s;
  }
  bool operator==(const MergingStopLineInfo &a) const {
    return lane_id == a.lane_id;
  }
};

struct MeetingLaneContext {
  uint64_t lane_id{0};
  uint64_t root_lane_id{0};
  bool inverse{false};
  math::AD2 inlet_left{0.0, 0.0};
  math::AD2 inlet_right{0.0, 0.0};
  math::AD2 outlet_left{0.0, 0.0};
  math::AD2 outlet_right{0.0, 0.0};
  math::Polygon zone_polygon;
  std::array<double, 2> ego_s{0.0, 0.0};
  std::array<double, 2> merging_s{0.0, 0.0};
  double sum_length{0.0};
  mutable std::array<double, 2> ego_rd{0.0, 0.0};

  MeetingLaneContext(const uint64_t id) : lane_id{id} {}

  MeetingLaneContext(const uint64_t id, const uint64_t rid,
                     std::array<math::AD2, 4> &pts, math::AD2 &es,
                     math::AD2 &ms, double slen, bool is_front)
      : lane_id{id},
        root_lane_id{rid},
        inlet_left{pts[0]},
        inlet_right{pts[2]},
        outlet_left{pts[1]},
        outlet_right{pts[3]},
        zone_polygon{{pts[0], pts[2], pts[3], pts[1]}},
        ego_s{es[0], es[1]},
        merging_s{ms[0], ms[1]},
        sum_length{slen},
        inverse{is_front} {}

  const void Repr(cyberverse::HDMap &hdmap) const {
    char tmp[64] = "";
    std::vector<std::pair<std::string, std::string>> ans = {};

    ans.push_back(std::make_pair("lane_id", hdmap.GetIdHashString(lane_id)));

    ans.push_back(
        std::make_pair("root_lane_id", hdmap.GetIdHashString(root_lane_id)));
    sprintf(tmp, "(%.4f,%.4f), (%.4f,%.4f)", inlet_left[0], inlet_left[1],
            inlet_right[0], inlet_right[1]);
    ans.push_back(std::make_pair("inlet", tmp));
    sprintf(tmp, "(%.4f,%.4f), (%.4f,%.4f)", outlet_left[0], outlet_left[1],
            outlet_right[0], outlet_right[1]);
    ans.push_back(std::make_pair("outlet", tmp));
    sprintf(tmp, "%.4f, %.4f", ego_s[0], ego_s[1]);
    ans.push_back(std::make_pair("ego_s", tmp));
    sprintf(tmp, "%.4f, %.4f", merging_s[0], merging_s[1]);
    ans.push_back(std::make_pair("merging_s", tmp));
    sprintf(tmp, "%.4f", sum_length);
    ans.push_back(std::make_pair("sum_length", tmp));
    sprintf(tmp, "%.4f, %.4f", ego_rd[0], ego_rd[1]);
    ans.push_back(std::make_pair("ego_rd", tmp));
    ans.push_back(std::make_pair("inverse", inverse ? "inverse" : "same"));

    for (auto it = ans.begin(); it != ans.end(); it++)
      LOG_INFO("{}: {}", it->first, it->second);
  }

  bool operator<(const MeetingLaneContext &a) const {
    if (math::Sign(a.ego_s[0] - ego_s[0]) == 0)
      return lane_id < a.lane_id;
    else
      return ego_s > a.ego_s;
  }

  bool operator==(const MeetingLaneContext &a) const {
    return lane_id == a.lane_id;
  }
};
struct TrafficConflictZoneContext {
  enum class connectionType {
    Culdesac = 0,
    Straight,
    Diverging,
    Merging,
    Mixed,
    Unknown,
    NDiverging,
    NMerging,
    NMixed,
    TypeSize,
  };

  Vec2d ego_pos_utm{};
  Vec2d connection_pos_utm{};
  ConflictLaneContext current_lane{0};
  ConflictLaneContext next_lane{0};
  const std::set<MergingStopLineInfo> *merging_geoinfo[2]{nullptr, nullptr};
  const std::map<MeetingLaneContext, ConflictLaneContext> *meeting_geoinfo[2]{
      nullptr, nullptr};
  uint64_t left_neighbor_laneid{0};
  uint64_t right_neighbor_laneid{0};
  double distance_to_zone{0.0};
  std::array<double, 3> route_length_to_zone{0.0, 0.0, 0.0};
  std::set<ConflictLaneContext> successor_ids{};
  std::set<ConflictLaneContext> merging_ids{};
  connectionType near_type{connectionType::Unknown};
  connectionType rear_type{connectionType::Unknown};
  connectionType type{connectionType::Unknown};
  std::vector<std::string> state{"", "", "", ""};

  void Update(const Vec2d &egoPosUTM, Vec2d &connectionPosUTM,
              std::array<ConflictLaneContext, 2> &LaneInfos,
              std::array<double, 3> &restRouteLength,
              uint64_t leftNeighborLaneId, uint64_t rightNeighborLaneId,
              std::set<ConflictLaneContext> &sInfos,
              std::set<ConflictLaneContext> &mInfos,
              std::array<connectionType, 3> &types,
              std::array<std::string, 9> &typeStr,
              std::set<MergingStopLineInfo> *m1,
              std::set<MergingStopLineInfo> *m2,
              std::map<MeetingLaneContext, ConflictLaneContext> *m3,
              std::map<MeetingLaneContext, ConflictLaneContext> *m4) {
    ego_pos_utm = egoPosUTM;
    connection_pos_utm = connectionPosUTM;
    current_lane = LaneInfos[0];
    next_lane = LaneInfos[1];
    left_neighbor_laneid = leftNeighborLaneId;
    right_neighbor_laneid = rightNeighborLaneId;
    successor_ids.swap(sInfos);
    merging_ids.swap(mInfos);
    distance_to_zone = ego_pos_utm.distance_to(connection_pos_utm);
    route_length_to_zone.swap(restRouteLength);
    near_type = types[0];
    rear_type = types[1];
    type = types[2];
    merging_geoinfo[0] = m1;
    merging_geoinfo[1] = m2;
    meeting_geoinfo[0] = m3;
    meeting_geoinfo[1] = m4;

    // Generate monitor log
    static char strbuffer[50];
    state[0] = "";
    for (int j = 0; j < 2; j++) {
      std::string tmp = "";
      for (int i = 0, state = static_cast<int>(types[j]); i < 4; i++)
        tmp += 48 + char(state >> (3 - i) & 1);
      state[0] +=
          typeStr[static_cast<int>(types[j])] + "(" + tmp.insert(2, "|");
      if (j == 0)
        state[0] += ") -> ";
      else
        state[0] += ") : ";
    }
    state[0] += typeStr[static_cast<int>(types[2])];

    sprintf(strbuffer, "Distance: %.2f", route_length_to_zone[2]);
    state[1] = std::string(strbuffer);

    sprintf(strbuffer, "Succesors: %d", successor_ids.size());
    state[2] = std::string(strbuffer);

    sprintf(strbuffer, "Mergings: %d", merging_ids.size());
    state[3] = std::string(strbuffer);
  }
};

struct MotorwayGameTheoryDecision {
  std::vector<level_k_context::Vehicle> agents_record;
  void Reset() { agents_record.clear(); }
};

struct TrafficConfilictDecision {
  bool motorway_merge_safe_decelerate = false;
  bool road_merge_safe_decelerate = false;
  std::vector<ConnectionConflictInfo> motorway_free_conflict_infos;
  std::vector<ConnectionConflictInfo> road_free_conflict_infos;
  void Reset() {
    motorway_merge_safe_decelerate = false;
    road_merge_safe_decelerate = false;
    motorway_free_conflict_infos.clear();
    road_free_conflict_infos.clear();
  }
};

struct MotorwayMultiCipvSpeedObstacleDecision {
  std::vector<std::pair<STPoint, double>> lower_points{};
  std::vector<std::pair<STPoint, double>> upper_points{};
  std::vector<double> lower_points_heading_diff{};
  std::vector<double> upper_points_heading_diff{};
  std::vector<Polygon2d> obstacle_polygons{};
  std::vector<Boundary> obstacle_boundaries{};
  std::vector<double> obstacle_pre_traj_headings{};
  std::vector<double> sample_t{};
  Obstacle obstacle{};
  bool collide{false};
  bool reverse{false};
  int lower_adc_first_index{std::numeric_limits<int>::max()};
  bool risk_obs{false};
  bool is_in_left_turn_area{false};
  bool is_in_right_turn_area{false};
  AdcCollideCornerPoint adc_first_collide_corner_point{
      AdcCollideCornerPoint::NONE};
};
struct MotorwaySingleCipvSpeedObstacleDecision {
  Obstacle obstacle{};
  bool collide{false};
  bool reverse{false};
  int lower_adc_first_index{std::numeric_limits<int>::max()};
  bool follow{false};
  double collide_s{200.0};
  double collide_v{0.0};
  double collide_t{200.0};
  double follow_s{200.0};
  double collide_delta_l{100.0};
  double init_delta_l{100.0};
};

struct MotorwaySpeedObstacleContext {
  std::vector<double> path_accumulated_s{};
  std::vector<Box2d> adc_boundaries{};
  std::vector<Boundary> adc_sl_boundaries{};
  std::vector<Vec2d> adc_corner_pt_coordinate{};
  std::unordered_map<int, bool>
      motorway_iter_deduction_take_follow_decision_map;
  std::vector<MotorwayMultiCipvSpeedObstacleDecision>
      multi_cipv_dynamic_obstacles_decision{};
  std::vector<MotorwayMultiCipvSpeedObstacleDecision>
      multi_cipv_static_obstacles_decision{};
  std::vector<MotorwayMultiCipvSpeedObstacleDecision>
      multi_cipv_virtual_obstacle_decision{};
  std::vector<MotorwaySingleCipvSpeedObstacleDecision>
      single_cipv_ostacles_decision{};

  BackupCipv backup_cipv{};
  std::unordered_set<neodrive::global::planning::SpeedLimit::SourceType>
      motorway_hard_speed_limit_acc{
          neodrive::global::planning::SpeedLimit::VEHICLE_MEETING,
          neodrive::global::planning::SpeedLimit::COLLISION_RISK,
          neodrive::global::planning::SpeedLimit::MERGE_IN,
          neodrive::global::planning::SpeedLimit::OBS_TURN_RIGHT_RISK};
  std::vector<Boundary> adc_front_road_boundaries{};
  std::unordered_set<int> ignore_static_obs_id{};
  std::unordered_set<int> ignore_dynamic_obs_id{};
  std::unordered_set<int> speed_backup_ignore_dynamic_obs_id{};
  std::vector<DynamicObsCautionInfo> dynamic_obs_caution_infos{};
  std::vector<LaneBorrowPrepareCollisionInfo>
      borrow_lane_prepare_collision_info{};
  std::vector<LaneBorrowReturnCollisionInfo>
      borrow_lane_return_collision_info{};
  SpeedDynamicObsCollisionRiskData dynamic_obstacles_collision_risk_data{};

  void Reset() {
    path_accumulated_s.clear();
    adc_boundaries.clear();
    adc_sl_boundaries.clear();
    adc_corner_pt_coordinate.clear();
    multi_cipv_dynamic_obstacles_decision.clear();
    multi_cipv_static_obstacles_decision.clear();
    multi_cipv_virtual_obstacle_decision.clear();
    single_cipv_ostacles_decision.clear();
    ignore_static_obs_id.clear();
    ignore_dynamic_obs_id.clear();
    speed_backup_ignore_dynamic_obs_id.clear();
    borrow_lane_prepare_collision_info.clear();
  }
};

struct ParkingOutsideData {
  double parking_speed_cmd{0.833};
  ParkingSpace *parking_space_ptr{nullptr};
};

struct ObstacleIntentionContext {
  uint64_t lane_id{0};
  std::string hash_lane_id{};
  uint64_t right_lane_id{0};
  double x;
  double y;
  double heading;
  double speed;
  double length{1.0};
  double width{0.5};
  double s_in_lane{0.0};
  double l_in_lane{0.0};
  double min_l{1000.0};
  double max_l{1000.0};
  double min_s{1000.0};
  double max_s{1000.0};
  double l_speed{0.0001};
  double s_speed{0.0001};
  double sl_speed_heading{0.0001};
  double time_to_lane_bound{1000.0};
  bool is_rectified{false};
};

struct OutsidePlannerData {
  int path_succeed_tasks{0};
  int speed_succeed_tasks{0};
  int path_fail_tasks{0};
  int speed_fail_tasks{0};

  // junction type
  Junction::JunctionType junction_type{Junction::JunctionType::UNKNOWN};
  MultiModeData multi_mode_data{};
  TrafficConflictZoneContext traffic_conflict_zone_context{};
  TrafficLawSpeedLimit traffic_law_speed_limit{};

  /// for path planning
  FrenetFramePoint frenet_init_point{};
  ReferencePoint init_point_ref_point{};
  FrenetFramePoint frenet_veh_real_point{};
  ReferencePoint veh_real_reference_point{};
  PathObstacleContext path_obstacle_context{};
  std::vector<PathRegion::Bound> shrink_half_ego_boundaries{};
  std::vector<PieceBoundary> road_obs_path_boundries{};
  std::vector<PieceBoundary> road_obs_path_shrink_boundries{};
  std::vector<PieceBoundary> path_opt_boundaries{};
  LaneBorrowScenario lane_borrow_scenario{LaneBorrowScenario::NONE};
  PathObserveRefLInfo path_observe_ref_l_info{};
  LaneChangeEnable lane_change_enable_info{};
  PathContext path_context{};
  PathData *path_data{nullptr};
  std::unordered_map<
      int, std::vector<neodrive::global::planning::ObstaclesIntention>>
      obs_intention;
  std::unordered_map<int, ObstacleIntentionContext> obs_intention_context;
  std::unordered_map<
      int, std::vector<neodrive::global::planning::ObstaclesIntention>>
      filter_obs_intentions;
  // std::unordered_map<int,
  // std::unordered_map<neodrive::global::planning::ObstacleIntentionType,
  // neodrive::global::planning::ObstaclesIntention>> obs_intention_map;
  std::unordered_map<int, std::string> obs_intention_context_str;
  /// for speed planning
  // for non motor way
  SpeedObstacleContext speed_obstacle_context{};
  SpeedContext speed_context{};
  // for motor way
  bool if_has_bump{false};
  MotorwaySpeedObstacleContext motorway_speed_obstacle_context{};
  MotorwaySpeedContext motorway_speed_context{};
  SpeedData *speed_data{nullptr};
  ParkingOutsideData parking_outside_data;
  MotorwayGameTheoryDecision motorway_game_theory_decision;
  TrafficConfilictDecision traffic_conflict_decision;

  bool is_low_speed_obs_bypass_work{false};
  bool speed_slow_down{false};
  bool path_slow_down_check_failed{false};
  bool lms_sensor_check_failed{false};
  bool road_bound_safe_check_failed{false};
  bool moving_obs_slow_down{false};
  bool ask_take_over_immediately{false};
  double traffic_light_stop_line{1000.0};
  bool hold_on{false};
  bool trajectory_replan{false};
  bool parking_stop{false};
  bool consider_reverse_lane_detour_obs{false};

  PathPlannerType path_planner_type{PathPlannerType::MAIN};

  void Reset() {
    path_succeed_tasks = 0;
    speed_succeed_tasks = 0;
    path_fail_tasks = 0;
    speed_fail_tasks = 0;
    traffic_law_speed_limit.Reset();

    path_obstacle_context.Reset();
    road_obs_path_boundries.clear();
    road_obs_path_shrink_boundries.clear();
    path_opt_boundaries.clear();
    path_observe_ref_l_info.Reset();
    lane_borrow_scenario = LaneBorrowScenario::NONE;
    path_context.Reset();

    speed_obstacle_context.Reset();
    speed_context.Reset();
    motorway_speed_context.Reset();
    motorway_speed_obstacle_context.Reset();
    traffic_conflict_decision.Reset();

    is_low_speed_obs_bypass_work = false;
    speed_slow_down = false;
    path_slow_down_check_failed = false;
    lms_sensor_check_failed = false;
    road_bound_safe_check_failed = false;
    moving_obs_slow_down = false;
    ask_take_over_immediately = false;

    consider_reverse_lane_detour_obs = false;

    traffic_light_stop_line = 1000.0;
  }
};

struct CollisionInfo {
  double lower_first_heading_diff{0.0};
  double lower_first_s{1e10};
  double lower_first_t{1e10};
  double lower_first_project_v{1e10};
  bool risk_obs{false};
  void Reset() {
    lower_first_heading_diff = 0.0;
    lower_first_s = 1e10;
    lower_first_t = 1e10;
    lower_first_project_v = 1e10;
    risk_obs = false;
  }
};

struct ObsSInfo {
  std::deque<double> relative_s{};
  int lost_cnt{0};
  void Reset() {
    relative_s.clear();
    lost_cnt = 0;
  }
};

enum class LatResult { OVERTAKE = 0, CUTIN, ACROSS };

struct ObsCutinInfo {
  std::deque<double> lat_dis{};
  std::deque<double> long_pos{};
  int lost_cnt{0};
  void Reset() {
    lat_dis.clear();
    long_pos.clear();
    lost_cnt = 0;
  }
};

struct ObsHeadingInfo {
  std::deque<double> heading_diff{};
  double last_heading{0.0};
  double accum_heading_diff{0.0};
  int lost_cnt{0};
  void Reset() {
    heading_diff.clear();
    last_heading = 0.0;
    accum_heading_diff = 0.0;
    lost_cnt = 0;
  }
};

struct MapTurnRightRiskAreaInfo {
  double risk_area_predeal_s{-1.e5};
  double risk_area_start_s{-1.e5};
  double risk_area_end_s{-1.e5};
  double junction_end_s{-1.e5};

  void Reset() {
    risk_area_predeal_s = -1.e5;
    risk_area_start_s = -1.e5;
    risk_area_end_s = -1.e5;
    junction_end_s = -1.e5;
  }
};

}  // namespace planning
}  // namespace neodrive
