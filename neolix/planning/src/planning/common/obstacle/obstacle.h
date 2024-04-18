#pragma once

#include <vector>

#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "decision.pb.h"
#include "perception_obstacle.pb.h"
#include "planning_object.h"
#include "prediction_obstacles.pb.h"
#include "src/planning/common/boundary.h"
#include "src/planning/common/planning_types.h"
#include "src/planning/common/trajectory/prediction_trajectory.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/kinematics.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {
using neodrive::global::planning::VirtualObstacle;
struct ObstacleState {
  Vec2d position{0, 0};
  double velocity{0.};
};

class Obstacle : public PlanningObject {
 public:
  enum class ObstacleType {
    UNKNOWN = 0,
    UNKNOWN_MOVABLE = 1,
    UNKNOWN_UNMOVABLE = 2,
    PEDESTRIAN = 3,
    BICYCLE = 4,
    VEHICLE = 5,
  };

  Obstacle() = default;
  /// Constructor with history size
  explicit Obstacle(std::size_t his_size);
  virtual ~Obstacle() override = default;
  virtual Json::Value to_json() const override;

  void copy_from(
      const neodrive::global::perception::PerceptionObstacle &obstacle);

  void ResetVec();
  // get paras
  int id() const;
  size_t seq_num() const;
  double get_time_stamp() const;
  double height() const;
  double width() const;
  double length() const;
  double heading() const;
  double velocity_heading() const;
  const std::array<double, 10> &kinematics_state() const;
  double speed() const;
  double max_history_speed() const;
  double history_speed_heading() const;

  bool is_virtual() const;
  bool is_init() const;
  bool is_static() const;
  bool is_avoidance() const;

  const VirtualObstacle::VirtualType &virtual_type() const;
  const ObstacleType &type() const;
  const PerceptionObstacle_SubType &sub_type() const;

  uint64_t matched_lane_id() const;
  double matched_lane_heading_deviation() const;
  double matched_lane_offset() const;

  double max_s() const;
  double min_s() const;
  double max_l() const;
  double min_l() const;

  const Vec2d &center() const;
  const std::vector<PredictionTrajectory> &prediction_trajectories() const;
  std::vector<PredictionTrajectory> *mutable_prediction_trajectories();
  const PredictionTrajectory &uniform_trajectory() const;
  PredictionTrajectory *mutable_uniform_trajectory();

  const SLPoint &center_sl() const;
  const ReferencePoint &obstacle_reference_point() const;

  const std::vector<Vec2d> &polygon_corners() const;
  const std::vector<SLPoint> &polygon_sl() const;

  const std::vector<Vec2d> &local_polygon() const;

  const Boundary &PolygonBoundary() const;

  const Boundary &boundary_with_shadow() const;

  size_t decision_continuous_frame_cnt() const;

  size_t frame_cnt() const;

  Box2d bounding_box() const;

  // set paras
  void set_id(const int id);
  void set_seq_num(const size_t seq_num);
  void set_time_stamp(const double time_stamp);
  void set_height(const double height);
  void set_width(const double width);
  void set_length(const double length);
  void set_heading(const double heading);
  void set_velocity_heading(const double velocity_heading);
  void set_state_components(const double time, const double velocity_x,
                          const double velocity_y, const double theta,
                          const double utm_x, const double utm_y);
  void set_speed(const double speed);
  void set_max_history_speed(const double max_speed);
  void set_history_speed_heading(const double heading);

  void set_virtual(const bool is_virtual);
  void set_is_static(const bool is_static);
  void set_avoidance(const bool avoidance);

  void set_virtual_type(const VirtualObstacle::VirtualType &virtual_type);
  void set_type(const ObstacleType &type);
  void set_sub_type(const PerceptionObstacle_SubType &sub_type);

  void set_matched_lane_id(const uint64_t lane_id);
  void set_matched_lane_heading_deviation(const double heading_deviation);
  void set_matched_lane_offset(const double offset);

  void set_max_s(const double max_s);
  void set_min_s(const double min_s);
  void set_max_l(const double max_l);
  void set_min_l(const double min_l);
  void set_center(const Vec2d &center);

  void ClearPredictionTrajectory();
  void AddPredictionTrajectory(
      const PredictionTrajectory &prediction_trajectory);

  void set_local_polygon(const std::vector<Vec2d> &local_polygon);

  void set_decision_continuous_frame_cnt(const size_t continuous_frames_cnt);

  void set_frame_cnt(const size_t frames_cnt);

  //---------------functions--------------------------
  // first step init
  void init_with_reference_line(const ReferenceLinePtr &reference_line);

  // enlarge boundary, use it unless you know how to use
  // normaly in speed planning
  void InitPolygonShadow(const ReferenceLinePtr &reference_line,
                           const Vec2d &l_point, const double l_height);

  void SmoothBoundaryWithHistory(
      const Obstacle &last_frame_obstacle_instance);

  void adjust_timestamp_with(const double diff);

  bool get_decision_by_traj_index(const int index,
                                  std::vector<Decision> *const decision) const;

  bool NeedToConsider(const double obstacle_s, const double adc_stop_s,
                        const double heading_diff) const;

  bool LikeToCollisionAdc(const double obstacle_s,
                             const double obs_time_to_center,
                             const double adc_time_to_collision) const;

  bool AwayFromReferenceLine(const ReferenceLinePtr &reference_line,
                                const double start_l, const double end_l,
                                const double lateral_speed) const;

  // -----------------functions to manipulate inner paras----------------
  // !!! Warning: be careful to use them, unless you know what they are doing.
  // expand origin obs info.
  bool expand_bounding_box_corners(const double dis);

  bool right_expand_bounding_box_corners(const double veh_x, const double veh_y,
                                         const double veh_theta,
                                         const double delta_theta,
                                         const double dis);

  bool left_expand_bounding_box_corners(const double veh_x, const double veh_y,
                                        const double veh_theta,
                                        const double delta_theta,
                                        const double dis);

  bool front_expand_bounding_box_corners(const double veh_x, const double veh_y,
                                         const double veh_theta,
                                         const double delta_theta,
                                         const double dis);

  bool back_expand_bounding_box_corners(const double veh_x, const double veh_y,
                                        const double veh_theta,
                                        const double delta_theta,
                                        const double dis);

  bool expand_bounding_box_corners_sl(const double dis);

  bool right_expand_bounding_box_corners_sl(const double dis);

  bool left_expand_bounding_box_corners_sl(const double dis);

  bool front_expand_bounding_box_corners_sl(const double dis);

  bool back_expand_bounding_box_corners_sl(const double dis);

  // do not expand origin obs info.
  bool expand_bounding_box_corners(const double dis,
                                   std::vector<Vec2d> &box_corners);

  bool expand_bounding_box_corners_xy_from_origin_info(
      const double dis, std::vector<Vec2d> &box_corners);

  bool expand_bounding_box_corners_xy(const double veh_x, const double veh_y,
                                      const double veh_theta,
                                      const double delta_theta,
                                      const double dis,
                                      std::vector<Vec2d> &box_corners);

  bool right_expand_bounding_box_corners_xy(const double veh_x,
                                            const double veh_y,
                                            const double veh_theta,
                                            const double delta_theta,
                                            const double dis,
                                            std::vector<Vec2d> &box_corners);

  bool left_expand_bounding_box_corners_xy(const double veh_x,
                                           const double veh_y,
                                           const double veh_theta,
                                           const double delta_theta,
                                           const double dis,
                                           std::vector<Vec2d> &box_corners);

  bool front_expand_bounding_box_corners_xy(const double veh_x,
                                            const double veh_y,
                                            const double veh_theta,
                                            const double delta_theta,
                                            const double dis,
                                            std::vector<Vec2d> &box_corners);

  bool back_expand_bounding_box_corners_xy(const double veh_x,
                                           const double veh_y,
                                           const double veh_theta,
                                           const double delta_theta,
                                           const double dis,
                                           std::vector<Vec2d> &box_corners);

  bool expand_bounding_box_corners_sl_from_origin_info(
      const double dis, std::vector<Vec2d> &box_corners_sl);

  bool expand_bounding_box_corners_sl(const double dis,
                                      std::vector<Vec2d> &box_corners_sl);

  bool right_expand_bounding_box_corners_sl(const double dis,
                                            std::vector<Vec2d> &box_corners_sl);

  bool left_expand_bounding_box_corners_sl(const double dis,
                                           std::vector<Vec2d> &box_corners_sl);

  bool front_expand_bounding_box_corners_sl(const double dis,
                                            std::vector<Vec2d> &box_corners_sl);

  bool back_expand_bounding_box_corners_sl(const double dis,
                                           std::vector<Vec2d> &box_corners_sl);
  bool get_max_min_s_l_index_point_from_polygon(int &min_s_index,
                                                int &max_s_index,
                                                int &min_l_index,
                                                int &max_l_index);

  /// Get the history states of obstacle, the latest state is in front of queue
  /// @return The deque of history
  const std::deque<ObstacleState> &get_history_states() const;

 private:
  /// Add obstacle state to history
  /// @param state Current(latest) state
  void add_state(ObstacleState &&state);

 private:
  int id_ = 0;
  size_t seq_num_ = 0;
  double time_stamp_ = 0.0;
  double height_ = 0.0;
  double width_ = 0.0;
  double length_ = 0.0;
  double heading_ = 0.0;
  double velocity_heading_ = 0.0;
  std::array<double, 10> kinematics_{0.0};
  double speed_ = 0.0;
  double max_history_speed_ = 0.0;
  double history_speed_heading_ = 0.0;
  double boundary_std_deviation_ = 0.0;
  bool virtual_ = false;
  bool init_ = false;
  bool is_static_ = true;
  bool avoidance_ = false;
  int min_s_index_ = -1;
  int max_s_index_ = -1;
  int min_l_index_ = -1;
  int max_l_index_ = -1;

  VirtualObstacle::VirtualType virtual_type_ = VirtualObstacle::UNKNOWN;
  ObstacleType type_ = ObstacleType::VEHICLE;
  PerceptionObstacle_SubType sub_type_ =
      PerceptionObstacle_SubType::PerceptionObstacle_SubType_ST_CAR;

  double max_s_ = std::numeric_limits<double>::lowest();
  double min_s_ = std::numeric_limits<double>::max();
  double max_l_ = std::numeric_limits<double>::lowest();
  double min_l_ = std::numeric_limits<double>::max();

  Vec2d center_{};
  std::vector<PredictionTrajectory> prediction_trajectories_{};
  PredictionTrajectory uniform_trajectory_{};

  /// variables about history
  std::size_t history_size_{10};
  std::deque<ObstacleState> history_{};

  // for init
  SLPoint center_sl_{};
  ReferencePoint ref_point_{};
  std::vector<Vec2d> bounding_box_corners_{};
  std::vector<SLPoint> bounding_box_corners_sl_{};
  std::vector<Vec2d> polygon_corners_{};
  std::vector<SLPoint> polygon_sl_{};
  std::vector<Vec2d> local_polygon_{};
  Boundary boundary_{};
  Boundary polygon_boundary_{};
  Boundary boundary_with_shadow_{};
  std::size_t decision_continuous_frame_cnt_ = 0;
  std::size_t frame_cnt_ = 1;
  uint64_t matched_lane_id_{0};
  double matched_lane_heading_deviation_{0.0};
  double matched_lane_offset_{0.0};
  math::TrackingDiff td_1D_1e1_{0.1, 100, 0.1};
  std::array<math::AD2, 3> motion;
};

using ObstacleBoundary = std::pair<Boundary, Obstacle *>;

}  // namespace planning
}  // namespace neodrive
