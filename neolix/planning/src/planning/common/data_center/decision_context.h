#pragma once

#include "planning/common/path/sl_point.h"
#include "planning/common/trajectory/trajectory_point.h"
#include "proto/scenario_manager_msgs.pb.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_point.h"

/**
 * context used in decision.
 * all below are held and maintained in
 */
namespace neodrive {
namespace planning {

class ReverseLaneDetourContext {
 public:
  // about reverse lane
  bool is_allowed_detour_in_reverse_lane{false};
  bool is_leftmost_lane{false};
  bool is_left_bound_allowed_cross{false};
  bool is_left_divider_allowed_cross{false};
  bool is_reverse_lane_allowed_cross{false};
  enum class AdcPosition {
    NO_NEED_TO_DISSCUSS = 0,
    LEFTMOST_LANE_COMPLETE = 1,
    DIVIDER_CROSSING = 2,
    REVERSE_LANE_COMPLETE = 3,
    OVER_REVERSE_LANE = 4,
    OVER_LEFTMOST_LANE = 5,
    NOT_DEFINE = 6
  };
  AdcPosition adc_position{AdcPosition::NO_NEED_TO_DISSCUSS};
  bool is_left_front_reverse_obs_exist{false};
  struct ReverseObsInfo {
    int obs_id;
    double obs_min_s;
    double adc_end_s;
    double obs_v;
    double adc_v;
    double delt_s;
    double delt_v;
    double delt_t;
    ReverseObsInfo(int obs_id_, double obs_min_s_, double adc_end_s_,
                   double obs_v_, double adc_v_, double delt_s_, double delt_v_,
                   double delt_t_)
        : obs_id(obs_id_),
          obs_min_s(obs_min_s_),
          adc_end_s(adc_end_s_),
          obs_v(obs_v_),
          adc_v(adc_v_),
          delt_s(delt_s_),
          delt_v(delt_v_),
          delt_t(delt_t_) {}
    void PrintInfo() {
      LOG_INFO(
          "obs_id:{}, obs_min_s:{:.2f}, adc_end_s:{:.2f}, obs_v:{:.2f}, "
          "adc_v:{:.2f}, delt_s:{:.2f}, delt_v:{:.2f}, delt_t:{:.2f}",
          obs_id, obs_min_s, adc_end_s, obs_v, adc_v, delt_s, delt_v, delt_t);
    }
  };
  std::vector<ReverseObsInfo> relative_time_to_reverse_obs{};

 public:
  inline void Reset() {
    is_allowed_detour_in_reverse_lane = false;
    is_leftmost_lane = false;
    is_left_bound_allowed_cross = false;
    is_left_divider_allowed_cross = false;
    is_reverse_lane_allowed_cross = false;
    adc_position = AdcPosition::NO_NEED_TO_DISSCUSS;
    is_left_front_reverse_obs_exist = false;
    relative_time_to_reverse_obs.clear();
  }
};

class LaneBorrowContext {
 public:
  using DetourStageState = neodrive::global::planning::DetourStageState;
  enum class BorrowSide { Left = 1, Right = 2, None = 3 };

 public:
  // scenario info.
  bool is_front_has_road_boundary{false};
  bool is_front_has_traffic_light{false};
  bool is_front_has_cross_road{false};
  bool is_refer_lane_static_obs_clear{false};
  bool is_adc_on_refer_lane{false};
  bool is_left_dynamic_obs_danger{false};
  // front closest static obstacle that block refer lane.
  int refer_lane_block_static_obs_id{-1};
  bool outside_finish_signal{false};
  bool is_road_queued{false};

  // stage
  DetourStageState::State stage{DetourStageState::INIT};

  // lane borrow context.
  double refer_lane_block_static_obs_height{0.0};
  double front_left_min_lane_bound{0.0};
  double front_right_min_lane_bound{0.0};
  double left_min_lane_bound{0.0};
  double right_min_lane_bound{0.0};
  double region_left_bound{0.0};
  double region_right_bound{0.0};

  std::vector<int> dynamic_obs_ids{};
  // lane borrow trigger.
  BorrowSide borrow_side{BorrowSide::None};

  double lane_borrow_extend_ratio{0.};

 public:
  inline void Reset() {
    stage = DetourStageState::INIT;
    borrow_side = BorrowSide::None;
    lane_borrow_extend_ratio = 0.;
  }
};

class MotorwayLaneBorrowContext {
 public:
  using MotorwayDetourStageState =
      neodrive::global::planning::MotorwayDetourStageState;
  enum class BorrowSide { Left = 1, Right = 2, None = 3 };

 public:
  // scenario info.
  bool is_in_right_first_line{false};
  bool is_left_front_has_road_boundary{false};
  bool is_right_front_has_road_boundary{false};
  bool is_front_has_traffic_light{false};
  bool is_front_has_cross_road{false};
  bool is_front_has_lane_turn{false};
  bool is_refer_lane_static_obs_clear{false};
  bool is_left_front_static_obs_clear{false};
  bool is_right_front_static_obs_clear{false};
  bool is_left_dynamic_obs_danger{false};
  bool is_right_dynamic_obs_danger{false};
  bool is_adc_on_refer_lane{false};
  bool outside_finish_signal{false};
  bool is_road_queued{false};

  // front closest static obstacle that block refer lane.
  int refer_lane_block_static_obs_id{-1};
  int left_front_block_static_obs_id{-1};
  int right_front_block_static_obs_id{-1};
  std::vector<int> dynamic_obs_ids{};
  double refer_lane_block_static_obs_height{0.0};
  // stage
  MotorwayDetourStageState::State stage{MotorwayDetourStageState::INIT};

  // lane borrow context.
  double front_left_min_lane_bound{0.0};
  double front_right_min_lane_bound{0.0};
  double left_min_lane_bound{0.0};
  double right_min_lane_bound{0.0};
  double region_left_bound{0.0};
  double region_right_bound{0.0};

  // lane borrow trigger.
  BorrowSide borrow_side{BorrowSide::None};
  double lane_borrow_extend_ratio{0.};

 public:
  inline void Reset() {
    stage = MotorwayDetourStageState::INIT;
    borrow_side = BorrowSide::None;
  }
};

class CruiseContext {
 public:
  using CruiseStageState = neodrive::global::planning::CruiseStageState;

  // stage
  CruiseStageState::State stage{CruiseStageState::KEEP};
  bool is_finish_detour_and_near_station{false};
};

class IntersectionContext {
 public:
  using IntersectionState = neodrive::global::planning::IntersectionState;

  // stage
  IntersectionState::State stage{IntersectionState::INIT};
};

class BarrierGateContext {
 public:
  using BarrierGateStageState =
      neodrive::global::planning::BarrierGateStageState;

  // stage
  BarrierGateStageState::State stage{BarrierGateStageState::INIT};

  // barrier gate infos
  ReferenceLine::TrafficOverlap barrier_gate{};
  bool is_barrier_gate_finished{false};
  bool consider_barrier_gate{false};
  int scenario_filter_cnt{0};
  bool is_wait_stage{false};
  bool is_move_stage{false};
  bool is_stop_stage{false};
  double barrier_gate_first_check_dis{0.0};

 public:
  inline void Reset() {
    is_barrier_gate_finished = false;
    consider_barrier_gate = false;
    scenario_filter_cnt = 0;
    is_wait_stage = false;
    is_move_stage = false;
    is_stop_stage = false;
    barrier_gate_first_check_dis = 0.0;
  }
};

class MotorwayCruiseContext {
 public:
  using MotorwayCruiseStageState =
      neodrive::global::planning::MotorwayCruiseStageState;

 public:
  // stage
  MotorwayCruiseStageState::State stage{MotorwayCruiseStageState::KEEP};
};

class MotorwayLaneChangeContext {
 public:
  using MotorwayLaneChangeStageState =
      neodrive::global::planning::MotorwayLaneChangeStageState;

  // stage
  MotorwayLaneChangeStageState::State stage{
      MotorwayLaneChangeStageState::PREPARE};
  // scenario info.
  bool is_front_has_road_boundary{false};
  bool is_front_has_divider_restrict{false};
  bool is_front_has_traffic_light{false};
  bool is_front_has_lane_turn{false};
  bool is_target_lane_static_obs_clear{false};
  bool is_target_lane_dynamic_obs_clear{false};
  bool is_adc_on_current_lane{false};
  bool is_adc_on_target_lane{false};
  bool is_adc_over_current_lane{false};

  bool is_target_lane_adjacent{false};

  // front closest static obstacle that block refer lane.
  int target_lane_block_obs_id{-1};

 public:
  inline void Reset() {
    stage = MotorwayLaneChangeStageState::PREPARE;

    is_front_has_road_boundary = false;
    is_front_has_divider_restrict = false;
    is_front_has_traffic_light = false;
    is_front_has_lane_turn = false;
    is_target_lane_static_obs_clear = false;
    is_target_lane_dynamic_obs_clear = false;
    is_adc_on_current_lane = false;
    is_adc_on_target_lane = false;

    is_target_lane_adjacent = false;
  }
};

class MotorwayIntersectionContext {
 public:
  using MotorwayIntersectionStageState =
      neodrive::global::planning::MotorwayIntersectionStageState;

  // stage
  MotorwayIntersectionStageState::State stage{
      MotorwayIntersectionStageState::EXIT};
};

}  // namespace planning
}  // namespace neodrive
