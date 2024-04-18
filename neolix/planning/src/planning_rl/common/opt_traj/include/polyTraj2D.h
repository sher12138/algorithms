#ifndef POLYTRAJ2D_H_
#define POLYTRAJ2D_H_

#include <iostream>
#include <vector>

#include "polyTraj.h"
#include "polyTraj2D.h"
#include "rootTraj.h"

#include "poly_traj_structs.h"

class PolyTraj2D {
  GenerationMode generation_mode_;
  double total_cost_;
  double max_curvature_;
  double min_velocity_;
  double max_distance_;  // to center line
  double max_angle_;
  double max_a_lat_;
  double max_a_lon_;
  double min_a_lon_;

  // Debug info:
  int index_lat_;
  int index_lon_;

  TrajectoryPoint2D latLongTrajectories2GlobalCordinates(
      const trajectory_point_1D& lat_traj, const trajectory_point_1D& long_traj,
      const CurvePoint& foot_curve_point) const;
  static void projectionCondition(const std::vector<CurvePoint>& center_line,
                                  const double& s_foot, const double& x1,
                                  const double& x2, double& f);

 public:
  const PolyTraj* pt_traj_lat_;
  const RootTraj* pt_traj_root_;
  std::vector<TrajectoryPoint2D> trajectory2D_;
  PolyTraj2D(const PolyTraj& traj_lat, const RootTraj& root_traj);
  PolyTraj2D(const std::vector<CurvePoint>& center_line,
             const PolyTraj& traj_lat, const RootTraj& root_traj,
             const GenerationMode mode,
             std::vector<std::vector<trajectory_point_1D> >::const_iterator
                 it_lat_sampled_trajectory,
             int& index_lat, int& index_lon);
  static inline double normalize_angle_positive(const double& angle) {
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  }
  static inline double normalize_angle(const double& angle) {
    double a = normalize_angle_positive(angle);
    if (a > M_PI) a -= 2.0 * M_PI;
    return a;
  }
  double getInitialJerk() const;
  void calc_extreme_values(const double t_delay);
  // void calc_max_curvature();
  // void calc_min_velocity();
  // void calc_max_dist_to_centerline();
  // void calc_max_angle_to_centerline();
  double get_max_curvature() const { return max_curvature_; }
  double get_min_velocity() const { return min_velocity_; }
  double get_max_dist_to_centerline() const { return max_distance_; }
  double get_max_angle_to_centerline() const { return max_angle_; }
  double get_max_lon_acceleration() const { return max_a_lon_; }
  double get_min_lon_acceleration() const { return min_a_lon_; }
  double get_max_lat_acceleration() const { return max_a_lat_; }
  double get_total_cost() const { return total_cost_; }
  int get_index_lat() const { return index_lat_; }
  int get_index_lon() const { return index_lon_; }
  void set_total_cost(double cost) { total_cost_ = cost; }  // for debug
  void calculateNextInitialStates(const double& t_to_next_step,
                                  movement_state& next_lat_state,
                                  movement_state& next_long_state) const;
  TrajectoryPoint2D evaluateTrajectoryAtT(const double& t) const;
  TrajectoryPoint2D calculateNextStartTrajectoryPoint(
      const double& t_next) const;
  void evalLatLongTrajectoriesTimeBased(const double& t_act,
                                        const double& delta_t,
                                        trajectory_point_1D& lat_state,
                                        trajectory_point_1D& long_state) const;
  void evalLatLongTrajectoriesArclengthBased(
      const double& t_act, const double& delta_t, const double& delta_s,
      trajectory_point_1D& lat_state, trajectory_point_1D& long_state) const;

  static void globalCordinates2LatLong(
      const TrajectoryPoint2D& tp_2d,
      const std::vector<CurvePoint>& center_line,
      const GenerationMode& generation_mode, movement_state& lat_state,
      movement_state& lon_state);

  void generateNewControlTrajectory(
      const double& t_current, const double& t_control_horizon,
      const double& control_t_res,
      std::vector<TrajectoryPoint2D>& trajectory) const;

  class CompPolyTraj2D {  // in order to sort trajectories
   public:
    bool operator()(const PolyTraj2D& pt1, const PolyTraj2D& pt2) {
      if (pt1.total_cost_ < pt2.total_cost_)
        return true;
      else
        return false;
    }
  };
  friend std::ostream& operator<<(std::ostream& ostr, const PolyTraj2D& traj2d);

 private:
};

#endif  // POLYTRAJ2D_H_
