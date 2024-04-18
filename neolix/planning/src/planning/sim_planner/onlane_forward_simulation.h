#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <ctime>

#include "calculations.h"
#include "ctx_idm_velocity_controller.h"
#include "ideal_steer_model.h"
#include "idm_velocity_controller.h"
#include "pure_pursuit_controller.h"
#include "sim_map.h"
#include "state.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

constexpr double kPi = acos(-1.0);
constexpr double kIntervalL = 0.4;

class OnLaneForwardSimulation {
 public:
  using State = neodrive::planning::sim_planner::State;
  using Vehicle = neodrive::planning::sim_planner::Vehicle;
  using CtxParam = ContextIntelligentDriverModel::CtxParam;

  struct Param {
    IntelligentDriverModel::Param idm_param;
    double steer_control_gain = 1.5;
    double steer_control_max_lookahead_dist = 50.0;
    // double steer_control_max_lookahead_dist = 5.0;
    double steer_control_min_lookahead_dist = 0.5;
    double max_lat_acceleration_abs = 1.5;
    double max_lat_jerk_abs = 3.0;
    double max_curvature_abs = 0.33;
    double max_lon_acc_jerk = 5.0;
    double max_lon_brake_jerk = 5.0;
    double max_steer_angle_abs = 45.0 / 180.0 * kPi;
    // double max_steer_angle_abs = 100.0 / 180.0 * kPi;
    double max_steer_rate = 0.39;
    // double max_steer_rate = 0.99;
    bool auto_decelerate_if_lat_failed = true;
  };

  static bool getIdmEquivalentVehicleLength(const Vehicle& ego_vehicle,
                                            double* eqv_vehicle_len) {
    // Different from original IDM, we still use the center of rear axle as the
    // vehicle position, since we need to calculate the 'net' distance between
    // ego vehicle and leading vehicle, here we can get a equivalent leading
    // vehicle's length for IDM

    // In case the leading vehicle has an opposite angle due to MOT error
    double len_rb2r = 0.0;
    // ego rear-axle to front bumper + leading rear bumper to rear-axle
    *eqv_vehicle_len = ego_vehicle.param().length() / 2.0 +
                       ego_vehicle.param().d_cr() + len_rb2r;

    return true;
  }

  static bool calculateDesiredState(const State& current_state,
                                    const double steer, const double velocity,
                                    const double wheelbase_len, const double dt,
                                    const Param& param, State* state) {
    neodrive::planning::sim_planner::IdealSteerModel model(
        wheelbase_len, param.idm_param.kAcceleration,
        param.idm_param.kHardBrakingDeceleration, param.max_lon_acc_jerk,
        param.max_lon_brake_jerk, param.max_lat_acceleration_abs,
        param.max_lat_jerk_abs, param.max_steer_angle_abs, param.max_steer_rate,
        param.max_curvature_abs);
    model.setState(current_state);
    model.setControl(neodrive::planning::sim_planner::IdealSteerModel::Control(
        steer, velocity));
    model.Step(dt);
    *state = model.state();
    state->time_stamp = current_state.time_stamp + dt;

    return true;
  };

  static bool calcualateVelocityUsingCtxIdm(
      const double current_pos, const double current_vel,
      const double leading_pos, const double leading_vel,
      const double target_pos, const double target_vel, const double dt,
      const Param& param, const CtxParam& ctx_param, double* velocity) {
    double leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      leading_vel_fin = 0;
    }
    // ~ note that we cannot use frenet state velocity for idm model, since the
    // ~ velocity in the frenet state may be larger than body velocity if the
    // ~ vehicle is in a highly curvy road (ref to the state transformer) which
    // ~ cannot be directly fed back to body velocity.
    return neodrive::planning::sim_planner::ContextIntelligentVelocityControl::
        calculateDesiredVelocity(param.idm_param, ctx_param, current_pos,
                                 leading_pos, target_pos, current_vel,
                                 leading_vel_fin, target_vel, dt, velocity);
  };

  static bool calcualateVelocityUsingCtxIdm(
      const double current_pos, const double current_vel,
      const double target_pos, const double target_vel, const double dt,
      const Param& param, const CtxParam& ctx_param, double* velocity) {
    const double virtual_leading_pos =
        current_pos + 100.0 + 100.0 * current_vel;
    return neodrive::planning::sim_planner::ContextIntelligentVelocityControl::
        calculateDesiredVelocity(param.idm_param, ctx_param, current_pos,
                                 virtual_leading_pos, target_pos, current_vel,
                                 current_vel, target_vel, dt, velocity);
  };

  static bool propagateOnceAdvancedLK(
      const int current_lane_id, const Vehicle& ego_vehicle,
      const neodrive::planning::Boundary& leading_boundary,
      const double lat_track_offset, const double dt, const Param& param,
      State* desired_state) {
    State current_state = ego_vehicle.state();
    double wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;
    // * Step I: CalculateSteer
    bool steer_calculation_failed = false;
    SimMapPoint current_fs;
    if (!SimMap::Instance()->getNearestPoint(current_state.vec_position,
                                             &current_fs)) {
      LOG_ERROR("[LK]steer calculation failed!");
      steer_calculation_failed = true;
    }
    current_fs.l = current_lane_id * kIntervalL;

    double steer, velocity;
    if (!steer_calculation_failed) {
      double approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (!calcualateSteer(current_state, current_fs, wheelbase_len,
                           Vec2d(approx_lookahead_dist, lat_track_offset), "LK",
                           &steer)) {
        LOG_ERROR("[LK]steer calculation failed!");
        steer_calculation_failed = true;
      }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    double sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);
    // * Step II: calculate velocity
    bool has_leading_obs = (leading_boundary.start_s() > 1e+8) ? false : true;

    if (!has_leading_obs) {
      // ~ Without leading vehicle
      if (current_state.velocity < 0.05 &&
          abs(sim_param.idm_param.kDesiredVelocity) < 1e-3) {
        velocity = 0.0;
      } else {
        calcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                   &velocity);
      }
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      if (current_state.velocity < 0.05 &&
          abs(sim_param.idm_param.kDesiredVelocity) < 1e-3) {
        velocity = 0.0;
      } else {
        double eqv_vehicle_len;
        getIdmEquivalentVehicleLength(ego_vehicle, &eqv_vehicle_len);
        sim_param.idm_param.kVehicleLength = eqv_vehicle_len;
        calcualateVelocityUsingIdm(current_fs.s, current_state.velocity,
                                   leading_boundary.start_s(), 0.0, dt,
                                   sim_param, &velocity);
      }
    }
    // * Step III: Use the vehicle Kinematics model to obtain the desired state
    calculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return true;
  }

  static bool propagateOnceAdvancedLK(
      const double l, const Vehicle& ego_vehicle,
      const neodrive::planning::Boundary& leading_boundary,
      const double lat_track_offset, const double dt, const Param& param,
      State* desired_state) {
    State current_state = ego_vehicle.state();
    double wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;
    // * Step I: CalculateSteer
    bool steer_calculation_failed = false;
    SimMapPoint current_fs;
    if (!SimMap::Instance()->getNearestPoint(current_state.vec_position,
                                             &current_fs)) {
      LOG_ERROR("[LK]steer calculation failed!");
      steer_calculation_failed = true;
    }
    current_fs.l = l;

    double steer, velocity;
    if (!steer_calculation_failed) {
      double approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (!calcualateSteer(current_state, current_fs, wheelbase_len,
                           Vec2d(approx_lookahead_dist, lat_track_offset), "LK",
                           &steer)) {
        LOG_ERROR("[LK]steer calculation failed!");
        steer_calculation_failed = true;
      }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    double sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);
    // * Step II: calculate velocity
    bool has_leading_obs = (leading_boundary.start_s() > 1e+8) ? false : true;

    if (!has_leading_obs) {
      // ~ Without leading vehicle
      if (current_state.velocity < 0.05 &&
          abs(sim_param.idm_param.kDesiredVelocity) < 1e-3) {
        velocity = 0.0;
      } else {
        calcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                   &velocity);
      }
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      if (current_state.velocity < 0.05 &&
          abs(sim_param.idm_param.kDesiredVelocity) < 1e-3) {
        velocity = 0.0;
      } else {
        double eqv_vehicle_len;
        getIdmEquivalentVehicleLength(ego_vehicle, &eqv_vehicle_len);
        sim_param.idm_param.kVehicleLength = eqv_vehicle_len;
        calcualateVelocityUsingIdm(current_fs.s, current_state.velocity,
                                   leading_boundary.start_s(), 0.0, dt,
                                   sim_param, &velocity);
      }
    }
    // * Step III: Use the vehicle Kinematics model to obtain the desired state
    calculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return true;
  }

  static bool propagateOnceAdvancedLC(
      const int current_lane_id, const int target_lane_id,
      const Vehicle& ego_vehicle,
      std::unordered_map<std::string, neodrive::planning::Boundary>
          obs_boundary,
      const neodrive::planning::Boundary& current_leading_obs,
      const double lat_track_offset, const double look_ahead_dist,
      const double dt, const Param& param, State* desired_state) {
    State current_state = ego_vehicle.state();
    double wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    double steer, velocity;  // values to be calculated

    // * Step I: calculate steer
    bool steer_calculation_failed = false;
    SimMapPoint ego_on_tarlane_fs;
    if (!SimMap::Instance()->getNearestPoint(current_state.vec_position,
                                             &ego_on_tarlane_fs)) {
      LOG_ERROR("[LC]steer calculation failed!");
      steer_calculation_failed = true;
    }
    ego_on_tarlane_fs.l = target_lane_id * kIntervalL;

    if (!steer_calculation_failed) {
      double approx_lookahead_dist = std::min(
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist),
          std::min(
              std::max(param.steer_control_min_lookahead_dist, look_ahead_dist),
              param.steer_control_max_lookahead_dist));
      if (!calcualateSteer(current_state, ego_on_tarlane_fs, wheelbase_len,
                           Vec2d(approx_lookahead_dist, lat_track_offset), "LC",
                           &steer)) {
        LOG_ERROR("[LC]steer calculation failed!");
        steer_calculation_failed = true;
      }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    double sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);
    // * Step II: calculate velocity
    // target longitudinal state on target lane
    double velocity_tar, s_tar;
    if (!getTargetStateOnTargetLane(target_lane_id, ego_vehicle,
                                    obs_boundary["front"], obs_boundary["rear"],
                                    sim_param, &velocity_tar, &s_tar)) {
      LOG_WARN("Get Target State On Target Lane failed!");
      velocity_tar = current_state.velocity;
    }

    neodrive::planning::sim_planner::ContextIntelligentDriverModel::CtxParam
        ctx_param(0.4, 0.8);

    bool has_leading_obs =
        (current_leading_obs.start_s() > 1e+8) ? false : true;
    if (!has_leading_obs) {
      // ~ Without leading vehicle
      calcualateVelocityUsingCtxIdm(ego_on_tarlane_fs.s, current_state.velocity,
                                    s_tar, velocity_tar, dt, sim_param,
                                    ctx_param, &velocity);
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      double eqv_vehicle_len;
      getIdmEquivalentVehicleLength(ego_vehicle, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;
      calcualateVelocityUsingCtxIdm(ego_on_tarlane_fs.s, current_state.velocity,
                                    current_leading_obs.start_s(), 0.0, s_tar,
                                    velocity_tar, dt, sim_param, ctx_param,
                                    &velocity);
    }
    // * Step3: Use the vehicle Kinematics model to obtain the desired state
    calculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return true;
  }

  static bool propagateOnceAdvancedLC(
      const int current_lane_id, const SimMapPoint& goal_sl_pt,
      const Vehicle& ego_vehicle,
      std::unordered_map<std::string, neodrive::planning::Boundary>
          obs_boundary,
      const neodrive::planning::Boundary& current_leading_obs,
      const double lat_track_offset, const double look_ahead_dist,
      const double dt, const Param& param, State* desired_state) {
    State current_state = ego_vehicle.state();
    double wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    double steer, velocity;

    // * Step I: calculate steer
    bool steer_calculation_failed = false;
    SimMapPoint ego_on_tarlane_fs;
    if (!SimMap::Instance()->getNearestPoint(current_state.vec_position,
                                             &ego_on_tarlane_fs)) {
      LOG_ERROR("[LC]steer calculation failed!");
      steer_calculation_failed = true;
    }
    ego_on_tarlane_fs.l = goal_sl_pt.l;
    // double s_dist = (goal_sl_pt.s - ego_on_tarlane_fs.s > 0)
    //                     ? (goal_sl_pt.s - ego_on_tarlane_fs.s)
    //                     : 1e+3;

    if (!steer_calculation_failed) {
      double approx_lookahead_dist = std::min(
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist),
          std::min(
              std::max(param.steer_control_min_lookahead_dist, look_ahead_dist),
              param.steer_control_max_lookahead_dist));

      if (!calcualateSteer(current_state, ego_on_tarlane_fs, wheelbase_len,
                           Vec2d(approx_lookahead_dist, lat_track_offset), "LC",
                           &steer)) {
        LOG_ERROR("[LC]steer calculation failed!");
        steer_calculation_failed = true;
      }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    double sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);
    // * Step II: calculate velocity
    // target longitudinal state on target lane
    double velocity_tar, s_tar;
    if (!getTargetStateOnTargetLane(goal_sl_pt.l, ego_vehicle,
                                    obs_boundary["front"], obs_boundary["rear"],
                                    sim_param, &velocity_tar, &s_tar)) {
      LOG_WARN("Get Target State On Target Lane failed!");
      velocity_tar = current_state.velocity;
    }

    neodrive::planning::sim_planner::ContextIntelligentDriverModel::CtxParam
        ctx_param(0.4, 0.8);

    bool has_leading_obs =
        (current_leading_obs.start_s() > 1e+8) ? false : true;
    if (!has_leading_obs) {
      // ~ Without leading vehicle
      calcualateVelocityUsingCtxIdm(ego_on_tarlane_fs.s, current_state.velocity,
                                    s_tar, velocity_tar, dt, sim_param,
                                    ctx_param, &velocity);
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      double eqv_vehicle_len;
      getIdmEquivalentVehicleLength(ego_vehicle, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;
      calcualateVelocityUsingCtxIdm(ego_on_tarlane_fs.s, current_state.velocity,
                                    current_leading_obs.start_s(), 0.0, s_tar,
                                    velocity_tar, dt, sim_param, ctx_param,
                                    &velocity);
    }
    // * Step3: Use the vehicle Kinematics model to obtain the desired state
    calculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return true;
  }

  static bool propagateOnceAdvancedLC(
      const SimMapPoint& goal_sl_pt, const Vehicle& ego_vehicle,
      std::unordered_map<std::string, neodrive::planning::Boundary>
          obs_boundary,
      const neodrive::planning::Boundary& current_leading_obs,
      const double lat_track_offset, const double look_ahead_dist,
      const double dt, const Param& param, State* desired_state) {
    State current_state = ego_vehicle.state();
    double wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    double steer, velocity;

    // * Step I: calculate steer
    bool steer_calculation_failed = false;
    SimMapPoint ego_on_tarlane_fs;
    if (!SimMap::Instance()->getNearestPoint(current_state.vec_position,
                                             &ego_on_tarlane_fs)) {
      LOG_ERROR("[LC]steer calculation failed!");
      steer_calculation_failed = true;
    }

    ego_on_tarlane_fs.l = goal_sl_pt.l;
    // double s_dist = (goal_sl_pt.s - ego_on_tarlane_fs.s > 0)
    //                     ? (goal_sl_pt.s - ego_on_tarlane_fs.s)
    //                     : 1e+3;

    if (!steer_calculation_failed) {
      double approx_lookahead_dist = std::min(
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist),
          std::min(
              std::max(param.steer_control_min_lookahead_dist, look_ahead_dist),
              param.steer_control_max_lookahead_dist));

      if (!calcualateSteer(current_state, ego_on_tarlane_fs, wheelbase_len,
                           Vec2d(approx_lookahead_dist, lat_track_offset), "LC",
                           &steer)) {
        LOG_ERROR("[LC]steer calculation failed!");
        steer_calculation_failed = true;
      }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    double sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);
    // * Step II: calculate velocity
    // target longitudinal state on target lane
    double velocity_tar, s_tar;
    if (!getTargetStateOnTargetLane(goal_sl_pt.l, ego_vehicle,
                                    obs_boundary["front"], obs_boundary["rear"],
                                    sim_param, &velocity_tar, &s_tar)) {
      LOG_WARN("Get Target State On Target Lane failed!");
      velocity_tar = current_state.velocity;
    }

    neodrive::planning::sim_planner::ContextIntelligentDriverModel::CtxParam
        ctx_param(0.4, 0.8);

    bool has_leading_obs =
        (current_leading_obs.start_s() > 1e+8) ? false : true;
    if (!has_leading_obs) {
      // ~ Without leading vehicle
      calcualateVelocityUsingCtxIdm(ego_on_tarlane_fs.s, current_state.velocity,
                                    s_tar, velocity_tar, dt, sim_param,
                                    ctx_param, &velocity);
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      double eqv_vehicle_len;
      getIdmEquivalentVehicleLength(ego_vehicle, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;
      calcualateVelocityUsingCtxIdm(ego_on_tarlane_fs.s, current_state.velocity,
                                    current_leading_obs.start_s(), 0.0, s_tar,
                                    velocity_tar, dt, sim_param, ctx_param,
                                    &velocity);
    }
    // * Step3: Use the vehicle Kinematics model to obtain the desired state
    calculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return true;
  }

  static bool getTargetStateOnTargetLane(
      const int lane_id, const Vehicle& ego_vehicle,
      const neodrive::planning::Boundary& front_obs,
      const neodrive::planning::Boundary& rear_obs, const Param& param,
      double* velocity, double* s) {
    SimMapPoint ego_fs;
    if (!SimMap::Instance()->getNearestPoint(ego_vehicle.state().vec_position,
                                             &ego_fs)) {
      LOG_ERROR("[OnLaneForwardSimulation]cor trans failed!");
      return false;
    }
    ego_fs.l = lane_id * kIntervalL;

    double time_headaway = param.idm_param.kDesiredHeadwayTime;
    double min_spacing = param.idm_param.kMinimumSpacing;

    bool has_front = (front_obs.start_s() > 1e+8) ? false : true;
    bool has_rear = (rear_obs.start_s() > 1e+8) ? false : true;

    double s_ref_front = -1;  // tail of front vehicle
    double s_thres_front = -1;
    if (has_front) {
      s_ref_front = front_obs.start_s();
      s_thres_front = s_ref_front - min_spacing -
                      time_headaway * ego_vehicle.state().velocity;
    }

    double s_ref_rear = -1;  // head of rear vehicle
    double s_thres_rear = -1;
    if (has_rear) {
      s_ref_rear = rear_obs.end_s();
      s_thres_rear = s_ref_rear + min_spacing;
    }

    double desired_s = ego_fs.s;
    double desired_v = ego_vehicle.state().velocity;

    // ~ params
    double k_v = 0.1;      // coeff for dv. k_v * s_err
    double p_v_ego = 0.1;  // coeff for user preferred vel
    double dv_lb = -3.0;   // dv lower bound
    double dv_ub = 5.0;    // dv upper bound

    double ego_desired_vel =
        ego_vehicle.state().velocity +
        (param.idm_param.kDesiredVelocity - ego_vehicle.state().velocity) *
            p_v_ego;
    if (has_front && has_rear) {
      // * has both front and rear vehicle
      if (s_ref_front < s_ref_rear) {
        LOG_ERROR("[OnLaneForwardSimulation]front & rear obs over!");
        return false;
      }

      double ds = fabs(s_ref_front - s_ref_rear);
      double s_star = s_ref_rear + ds / 2.0 - ego_vehicle.param().d_cr();

      s_thres_front = std::max(s_star, s_thres_front);
      s_thres_rear = std::min(s_star, s_thres_rear);

      desired_s = std::min(std::max(s_thres_rear, ego_fs.s), s_thres_front);

      double s_err_front = s_thres_front - ego_fs.s;
      double v_ref_front =
          std::max(0.0, truncate(s_err_front * k_v, dv_lb, dv_ub));

      double s_err_rear = s_thres_rear - ego_fs.s;
      double v_ref_rear =
          std::max(0.0, truncate(s_err_rear * k_v, dv_lb, dv_ub));

      desired_v = std::min(std::max(v_ref_rear, ego_desired_vel), v_ref_front);

    } else if (has_front) {
      // * only has front vehicle
      desired_s = std::min(ego_fs.s, s_thres_front);

      double s_err_front = s_thres_front - ego_fs.s;
      double v_ref_front =
          std::max(0.0, truncate(s_err_front * k_v, dv_lb, dv_ub));
      desired_v = std::min(ego_desired_vel, v_ref_front);

    } else if (has_rear) {
      // * only has rear vehicle
      desired_s = std::max(ego_fs.s, s_thres_rear);

      double s_err_rear = s_thres_rear - ego_fs.s;
      double v_ref_rear =
          std::max(0.0, truncate(s_err_rear * k_v, dv_lb, dv_ub));
      desired_v = std::max(v_ref_rear, ego_desired_vel);
    }

    *velocity = desired_v;
    *s = desired_s;

    return true;
  }

  static bool getTargetStateOnTargetLane(
      const double goal_pt_l, const Vehicle& ego_vehicle,
      const neodrive::planning::Boundary& front_obs,
      const neodrive::planning::Boundary& rear_obs, const Param& param,
      double* velocity, double* s) {
    SimMapPoint ego_fs;
    if (!SimMap::Instance()->getNearestPoint(ego_vehicle.state().vec_position,
                                             &ego_fs)) {
      LOG_ERROR("[OnLaneForwardSimulation]cor trans failed!");
      return false;
    }
    ego_fs.l = goal_pt_l;

    double time_headaway = param.idm_param.kDesiredHeadwayTime;
    double min_spacing = param.idm_param.kMinimumSpacing;

    bool has_front = (front_obs.start_s() > 1e+8) ? false : true;
    bool has_rear = (rear_obs.start_s() > 1e+8) ? false : true;

    double s_ref_front = -1;  // tail of front vehicle
    double s_thres_front = -1;
    if (has_front) {
      s_ref_front = front_obs.start_s();
      s_thres_front = s_ref_front - min_spacing -
                      time_headaway * ego_vehicle.state().velocity;
    }

    double s_ref_rear = -1;  // head of rear vehicle
    double s_thres_rear = -1;
    if (has_rear) {
      s_ref_rear = rear_obs.end_s();
      s_thres_rear = s_ref_rear + min_spacing;
    }

    double desired_s = ego_fs.s;
    double desired_v = ego_vehicle.state().velocity;

    // ~ params
    double k_v = 0.1;      // coeff for dv. k_v * s_err
    double p_v_ego = 0.1;  // coeff for user preferred vel
    double dv_lb = -3.0;   // dv lower bound
    double dv_ub = 5.0;    // dv upper bound

    double ego_desired_vel =
        ego_vehicle.state().velocity +
        (param.idm_param.kDesiredVelocity - ego_vehicle.state().velocity) *
            p_v_ego;
    if (has_front && has_rear) {
      // * has both front and rear vehicle
      if (s_ref_front < s_ref_rear) {
        LOG_ERROR("[OnLaneForwardSimulation]front & rear obs over!");
        return false;
      }

      double ds = fabs(s_ref_front - s_ref_rear);
      double s_star = s_ref_rear + ds / 2.0 - ego_vehicle.param().d_cr();

      s_thres_front = std::max(s_star, s_thres_front);
      s_thres_rear = std::min(s_star, s_thres_rear);

      desired_s = std::min(std::max(s_thres_rear, ego_fs.s), s_thres_front);

      double s_err_front = s_thres_front - ego_fs.s;
      double v_ref_front =
          std::max(0.0, truncate(s_err_front * k_v, dv_lb, dv_ub));

      double s_err_rear = s_thres_rear - ego_fs.s;
      double v_ref_rear =
          std::max(0.0, truncate(s_err_rear * k_v, dv_lb, dv_ub));

      desired_v = std::min(std::max(v_ref_rear, ego_desired_vel), v_ref_front);

    } else if (has_front) {
      // * only has front vehicle
      desired_s = std::min(ego_fs.s, s_thres_front);

      double s_err_front = s_thres_front - ego_fs.s;
      double v_ref_front =
          std::max(0.0, truncate(s_err_front * k_v, dv_lb, dv_ub));
      desired_v = std::min(ego_desired_vel, v_ref_front);

    } else if (has_rear) {
      // * only has rear vehicle
      desired_s = std::max(ego_fs.s, s_thres_rear);

      double s_err_rear = s_thres_rear - ego_fs.s;
      double v_ref_rear =
          std::max(0.0, truncate(s_err_rear * k_v, dv_lb, dv_ub));
      desired_v = std::max(v_ref_rear, ego_desired_vel);
    }

    *velocity = desired_v;
    *s = desired_s;

    return true;
  }

 private:
  static bool calcualateSteer(const State& current_state,
                              const SimMapPoint& current_fs,
                              const double wheelbase_len,
                              const Vec2d lookahead_offset,
                              const std::string recall_func, double* steer) {
    SLPoint dest_fs(lookahead_offset.x() + current_fs.s,
                    lookahead_offset.y() + current_fs.l);
    SimMapPoint dest_pt;
    if (!SimMap::Instance()->getNearestPoint(dest_fs, &dest_pt)) {
      LOG_WARN("[OnLaneForwardSimulation]cor trans failed!");
      return false;
    }

    double look_ahead_dist =
        sqrt(pow(dest_pt.x - current_state.vec_position.x(), 2) +
             pow(dest_pt.y - current_state.vec_position.y(),
                 2));  // Go to body coordinates
    double cur_to_dest_angle = convertVec2dToAngle(Eigen::Vector2d(
        dest_pt.x - current_state.vec_position.x(),
        dest_pt.y - current_state.vec_position
                        .y()));  // Convert angle based on position
    double angle_diff = normalizeAngle(cur_to_dest_angle -
                                       current_state.angle);  // Angle change

    neodrive::planning::sim_planner::PurePursuitControl::calculateDesiredSteer(
        wheelbase_len, angle_diff, look_ahead_dist, steer);

    return true;
  }

  // ~ Using virtual leading vehicle
  static bool calcualateVelocityUsingIdm(const double current_vel,
                                         const double dt, const Param& param,
                                         double* velocity) {
    const double virtual_leading_dist = 100.0 + 100.0 * current_vel;
    return neodrive::planning::sim_planner::IntelligentVelocityControl::
        calculateDesiredVelocity(param.idm_param, 0.0,
                                 0.0 + virtual_leading_dist, current_vel,
                                 current_vel, dt, velocity);
  }

  // ~ Using leading vehicle
  static bool calcualateVelocityUsingIdm(const double current_pos,
                                         const double current_vel,
                                         const double leading_pos,
                                         const double leading_vel,
                                         const double dt, const Param& param,
                                         double* velocity) {
    double leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      leading_vel_fin = 0;
    }

    // ~ note that we cannot use frenet state velocity for idm model, since the
    // ~ velocity in the frenet state may be larger than body velocity if the
    // ~ vehicle is in a highly curvy road (ref to the state transformer) which
    // ~ cannot be directly fed back to body velocity.
    return neodrive::planning::sim_planner::IntelligentVelocityControl::
        calculateDesiredVelocity(param.idm_param, current_pos, leading_pos,
                                 current_vel, leading_vel_fin, dt, velocity);
  }
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
