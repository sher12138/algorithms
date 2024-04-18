#include "parking_trajectory_optimizer.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

ParkingTrajectoryOptimizer::ParkingTrajectoryOptimizer(
    const ParkingConfig& config)
    : config_(config) {
  tra_opt_config_ = config_.park_trajectory_optimizer_config();

  // TODO(wyc): smart pointer
  hybrid_astar_ = std::make_unique<HybridAStar>(config_.hybrid_a_star_config(),
                                                config_.common_config());
  //  dual_variable_warm_start_ =
  //  std::make_unique<DualVariableWarmStartProblem>(
  //      config_.dual_variable_config());
  //  distance_approach_ = std::make_unique<DistanceApproachProblem>(
  //      config_.distacne_approach_config(), config_.common_config());
  iterative_anchoring_smoother_ = std::make_unique<IterativeAnchoringSmoother>(
      config_.iterative_anchor_config(), config_.common_config());
}

std::vector<TrajectoryPoint>
ParkingTrajectoryOptimizer::GetOptimizedTrajectory() {
  return optimized_pts_;
}

bool ParkingTrajectoryOptimizer::Generate(
    const TrajectoryPoint& trajectory_init_point,
    const std::vector<double>& end_pose, const std::vector<double>& XYbounds,
    double rotate_angle, const Vec2d& translate_origin,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    const ReferencePoint& anchor_point) {
  if (XYbounds.empty() || end_pose.empty() /*|| obstacles_edges_num.cols() == 0 ||
      obstacles_A.cols() == 0 || obstacles_b.cols() == 0*/) {
    LOG_ERROR("ParkingTrajectoryOptimizer input data not ready");
    return false;
  }

  // Generate Stop trajectory if init point close to destination
  if (IsInitPointNearDestination(trajectory_init_point, end_pose, rotate_angle,
                                 translate_origin)) {
    LOG_INFO(
        "Planning init point is close to destination, skip new "
        "trajectory generation");
    return true;
  }

  // init x, y, z would be rotated.
  double init_x = trajectory_init_point.x();
  double init_y = trajectory_init_point.y();
  double init_heading = trajectory_init_point.theta();
  LOG_INFO(
      "origin x: {:.4f}, y: {:.4f}, heading: {:.4f}; init x: {:.4f}, y: {:.4f}, heading: "
      "{:.4f}",
      translate_origin.x(), translate_origin.y(), rotate_angle, init_x, init_y,
      init_heading);

  // Rotate and scale the state
  PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
                       &init_heading);

  // first path plan [init_v = 0]
  HybridAStartResult result;
  if (hybrid_astar_->Plan(init_x, init_y, init_heading, end_pose[0],
                          end_pose[1], end_pose[2], XYbounds,
                          obstacles_vertices_vec, &result)) {
    LOG_INFO("Hybrid A* successfully solved!");
  } else {
    LOG_ERROR("Hybrid A* failed to solve");
    return false;
  }

  LogHybriad(result, anchor_point, rotate_angle, translate_origin);

  std::vector<HybridAStartResult> partition_trajectories;
  if (!hybrid_astar_->TrajectoryPartition(result, &partition_trajectories)) {
    LOG_ERROR("TrajectoryPartition hybrid_astar results failed");
    return false;
  }
  LOG_INFO("hybrid astar partition trajectory num : {}",
           partition_trajectories.size());
  std::vector<TrajectoryPoint> input;
  std::vector<TrajectoryPoint> ouput;
  // TODO(wyc): no need to be global
  bool ret = false;
  if (tra_opt_config_.use_iterative_anchoring_smoother_) {
    ret =
        IterativeSmoothing(trajectory_init_point, input, partition_trajectories,
                           obstacles_vertices_vec, ouput);
    if (!ret) {
      LOG_ERROR("IterativeSmoothing failed");
      return false;
    }
  } else {
    //    ret = distance_approach_smoothing(trajectory_init_point, input,
    //    XYbounds,
    //                                      obstacles_edges_num, obstacles_A,
    //                                      obstacles_b, partition_trajectories,
    //                                      obstacles_vertices_vec, ouput);
    if (!ret) {
      LOG_ERROR("distance_approach_smoothing failed");
      return false;
    }
  }

  // TEST ouput
  for (auto pt : ouput) {
    LOG_INFO("smooth pt : x {:.4f} y {:.4f} theta {:.4f}", pt.x(), pt.y(),
             pt.theta());
  }

  // rescale the states to the world frame
  PathPointDeNormalizing(rotate_angle, translate_origin, ouput);

  ReTransformOnAnchor(anchor_point, ouput);

  // add direction to output pts
  GetSmoothTrajectoryDirection(ouput);

  optimized_pts_ = ouput;

  LogOutput(ouput);

  return true;
}

bool ParkingTrajectoryOptimizer::IsInitPointNearDestination(
    const TrajectoryPoint& init_pt, const std::vector<double>& end_pose,
    double rotate_angle, const Vec2d& translate_origin) {
  if (end_pose.size() != 4) {
    LOG_ERROR("input end_pose size [{}] != 4", end_pose.size());
    return false;
  }
  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.self_rotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  double distance_to_init_point =
      std::sqrt(pow(init_pt.x() - end_pose_to_world_frame.x(), 2) +
                +pow(init_pt.y() - end_pose_to_world_frame.y(), 2));

  return distance_to_init_point <
         tra_opt_config_.is_near_destination_threshold_;
}

void ParkingTrajectoryOptimizer::PathPointNormalizing(
    double rotate_angle, const Vec2d& translate_origin, double* x, double* y,
    double* theta) {
  if (x == nullptr || y == nullptr || theta == nullptr) {
    LOG_ERROR("input invalid");
    return;
  }
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
  *theta = normalize_angle(*theta - rotate_angle);
  return;
}

void ParkingTrajectoryOptimizer::PathPointDeNormalizing(
    double rotate_angle, const Vec2d& translate_origin, double* x, double* y,
    double* theta) {
  if (x == nullptr || y == nullptr || theta == nullptr) {
    LOG_ERROR("input invalid");
    return;
  }
  double tmp_x = *x;
  *x = (*x) * std::cos(rotate_angle) - (*y) * std::sin(rotate_angle);
  *y = tmp_x * std::sin(rotate_angle) + (*y) * std::cos(rotate_angle);
  *x += translate_origin.x();
  *y += translate_origin.y();
  *theta = normalize_angle(*theta + rotate_angle);
  return;
}

void ParkingTrajectoryOptimizer::PathPointDeNormalizing(
    double rotate_angle, const Vec2d& translate_origin,
    std::vector<TrajectoryPoint>& traj) {
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;

  for (size_t i = 0; i < traj.size(); ++i) {
    auto& pt = traj[i];
    tmp_x = pt.x() * std::cos(rotate_angle) - pt.y() * std::sin(rotate_angle);
    tmp_y = pt.x() * std::sin(rotate_angle) + pt.y() * std::cos(rotate_angle);
    tmp_x += translate_origin.x();
    tmp_y += translate_origin.y();
    tmp_theta = normalize_angle(pt.theta() + rotate_angle);
    pt.set_x(tmp_x);
    pt.set_y(tmp_y);
    pt.set_theta(tmp_theta);
  }
  return;
}

// TODO: deprecate the use of Eigen in trajectory smoothing
void ParkingTrajectoryOptimizer::LoadHybridAstarResultInEigen(
    HybridAStartResult* result, Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS) {
  // load Warm Start result(horizon is timestep number minus one)
  long horizon = result->x.size() - 1;
  if (horizon < 0) {
    LOG_ERROR("xWS.cols() size [{}] error", horizon);
    return;
  }

  xWS->resize(4, horizon + 1);
  uWS->resize(2, horizon);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->y.data(), horizon + 1);
  Eigen::VectorXd theta = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result->a.data(), horizon);
  xWS->row(0) = std::move(x);
  xWS->row(1) = std::move(y);
  xWS->row(2) = std::move(theta);
  xWS->row(3) = std::move(v);
  uWS->row(0) = std::move(steer);
  uWS->row(1) = std::move(a);
  return;
}

bool ParkingTrajectoryOptimizer::GenerateDecoupledTraj(
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc) {
  DiscretizedTrajectory smoothed_trajectory;
  std::vector<TrajectoryPoint> raw_tra;
  for (long i = 0; i < xWS.cols(); ++i) {
    TrajectoryPoint tmp_pt;
    tmp_pt.set_x(xWS(0, i));
    tmp_pt.set_y(xWS(1, i));
    tmp_pt.set_theta(xWS(2, i));
    raw_tra.push_back(tmp_pt);
  }
  if (!iterative_anchoring_smoother_->Smooth(raw_tra, obstacles_vertices_vec,
                                             &smoothed_trajectory)) {
    LOG_ERROR("iterative_anchoring_smoother failed");
    return false;
  }

  LoadResult(smoothed_trajectory, state_result_dc, control_result_dc,
             time_result_dc);
  return true;
}

bool ParkingTrajectoryOptimizer::GenerateDistanceApproachTraj(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const std::vector<double>& XYbounds,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    const Eigen::MatrixXd& last_time_u, const double init_v,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds) {
  // long horizon = xWS.cols() - 1;
  // if (horizon < 0) {
  //   LOG_ERROR("xWS.cols() size [{}] error", horizon);
  //   return false;
  // }

  // Eigen::MatrixXd x0(4, 1);
  // x0 << xWS(0, 0), xWS(1, 0), xWS(2, 0), init_v;

  // Eigen::MatrixXd xF(4, 1);
  // xF << xWS(0, horizon), xWS(1, horizon), xWS(2, horizon), xWS(3, horizon);

  // // load vehicle configuration
  // double front_to_center = VehicleParam::Instance()->front_edge_to_center();
  // double back_to_center = VehicleParam::Instance()->back_edge_to_center();
  // double left_to_center = VehicleParam::Instance()->left_edge_to_center();
  // double right_to_center = VehicleParam::Instance()->right_edge_to_center();
  // Eigen::MatrixXd ego(4, 1);
  // ego << front_to_center, right_to_center, back_to_center, left_to_center;

  // // Get obstacle num
  // std::size_t obstacles_num = obstacles_vertices_vec.size();

  // // Get timestep delta t
  // double ts = config_.common_config().delta_t_;

  // // slack_warm_up, temp usage
  // Eigen::MatrixXd s_warm_up = Eigen::MatrixXd::Zero(obstacles_num, horizon +
  // 1);

  // // Dual variable warm start for distance approach problem
  // if (tra_opt_config_.use_dual_variable_warm_start_) {
  //   if (dual_variable_warm_start_->Solve(
  //           horizon, ts, ego, obstacles_num, obstacles_edges_num,
  //           obstacles_A, obstacles_b, xWS, l_warm_up, n_warm_up, &s_warm_up))
  //           {
  //     LOG_INFO("Dual variable problem successfully solved!");
  //   } else {
  //     LOG_ERROR("Dual variable problem failed to solve");
  //     return false;
  //   }
  // } else {
  //   *l_warm_up =
  //       0.5 * Eigen::MatrixXd::Ones(obstacles_edges_num.sum(), horizon + 1);
  //   *n_warm_up = 0.5 * Eigen::MatrixXd::Ones(4 * obstacles_num, horizon + 1);
  // }

  // // Distance approach trajectory smoothing
  // if (distance_approach_->Solve(
  //         x0, xF, last_time_u, horizon, ts, ego, xWS, uWS, *l_warm_up,
  //         *n_warm_up, s_warm_up, XYbounds, obstacles_num,
  //         obstacles_edges_num, obstacles_A, obstacles_b, state_result_ds,
  //         control_result_ds, time_result_ds, dual_l_result_ds,
  //         dual_n_result_ds)) {
  //   LOG_INFO("Distance approach problem successfully solved!");
  // } else {
  //   LOG_ERROR("Distance approach problem failed to solve");
  //   if (tra_opt_config_.enable_smoother_failsafe_) {
  //     LOG_WARN("Distance approach problem enable_smoother_failsafe");
  //     UseWarmStartAsResult(xWS, uWS, *l_warm_up, *n_warm_up, state_result_ds,
  //                          control_result_ds, time_result_ds,
  //                          dual_l_result_ds, dual_n_result_ds);
  //   } else {
  //     return false;
  //   }
  // }
  return true;
}

void ParkingTrajectoryOptimizer::UseWarmStartAsResult(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds) {
  LOG_ERROR("Use warm start as trajectory output");

  *state_result_ds = xWS;
  *control_result_ds = uWS;
  *dual_l_result_ds = l_warm_up;
  *dual_n_result_ds = n_warm_up;

  long time_result_horizon = xWS.cols() - 1;
  *time_result_ds = Eigen::MatrixXd::Constant(1, time_result_horizon,
                                              config_.common_config().delta_t_);
  return;
}

// TODO: tmp interface, will refactor
void ParkingTrajectoryOptimizer::LoadResult(
    const DiscretizedTrajectory& discretized_trajectory,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc) {
  const std::size_t points_size = discretized_trajectory.num_of_points();
  if (points_size <= 1) {
    LOG_ERROR("points_size error {}", points_size);
    return;
  }
  *state_result_dc = Eigen::MatrixXd::Zero(4, points_size);
  *control_result_dc = Eigen::MatrixXd::Zero(2, points_size - 1);
  *time_result_dc = Eigen::MatrixXd::Zero(1, points_size - 1);

  auto& state_result = *state_result_dc;

  for (std::size_t i = 0; i < points_size; ++i) {
    TrajectoryPoint tra_pt;
    if (!discretized_trajectory.trajectory_point_at(i, tra_pt)) {
      LOG_ERROR("trajectory_point_at {} failed", i);
      continue;
    }
    state_result(0, i) = tra_pt.x();
    state_result(1, i) = tra_pt.y();
    state_result(2, i) = tra_pt.theta();
    state_result(3, i) = tra_pt.velocity();
  }

  auto& control_result = *control_result_dc;
  auto& time_result = *time_result_dc;
  const double wheel_base = VehicleParam::Instance()->wheel_base();
  for (std::size_t i = 0; i + 1 < points_size; ++i) {
    TrajectoryPoint tra_pt;
    if (!discretized_trajectory.trajectory_point_at(i, tra_pt)) {
      LOG_ERROR("trajectory_point_at {} failed", i);
      continue;
    }
    TrajectoryPoint tra_next_pt;
    if (!discretized_trajectory.trajectory_point_at(i + 1, tra_next_pt)) {
      LOG_ERROR("trajectory_point_at {} failed", i + 1);
      continue;
    }
    control_result(0, i) = std::atan(tra_pt.kappa() * wheel_base);
    control_result(1, i) = tra_pt.acceleration();
    time_result(0, i) = tra_next_pt.relative_time() - tra_pt.relative_time();
  }
  return;
}

// TODO(wyc): split
void ParkingTrajectoryOptimizer::CombineTrajectories(
    const std::vector<Eigen::MatrixXd>& xWS_vec,
    const std::vector<Eigen::MatrixXd>& uWS_vec,
    const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& l_warm_up_vec,
    const std::vector<Eigen::MatrixXd>& n_warm_up_vec,
    const std::vector<Eigen::MatrixXd>& dual_l_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& dual_n_result_ds_vec,
    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds) {
  // Repeated midway state point are not added
  std::size_t warm_start_state_size = 0;
  for (const auto& warm_start_state : xWS_vec) {
    warm_start_state_size += warm_start_state.cols();
  }
  warm_start_state_size -= xWS_vec.size() - 1;

  std::size_t warm_start_control_size = 0;
  for (const auto& warm_start_control : uWS_vec) {
    warm_start_control_size += warm_start_control.cols();
  }

  // Repeated midway state point are not added
  std::size_t smoothed_state_size = 0;
  for (const auto& smoothed_state : state_result_ds_vec) {
    smoothed_state_size += smoothed_state.cols();
  }
  smoothed_state_size -= state_result_ds_vec.size() - 1;

  std::size_t smoothed_control_size = 0;
  for (const auto& smoothed_control : control_result_ds_vec) {
    smoothed_control_size += smoothed_control.cols();
  }

  std::size_t time_size = 0;
  for (const auto& smoothed_time : time_result_ds_vec) {
    time_size += smoothed_time.cols();
  }

  std::size_t l_warm_start_size = 0;
  for (const auto& l_warm_start : l_warm_up_vec) {
    l_warm_start_size += l_warm_start.cols();
  }

  std::size_t n_warm_start_size = 0;
  for (const auto& n_warm_start : n_warm_up_vec) {
    n_warm_start_size += n_warm_start.cols();
  }

  std::size_t l_smoothed_size = 0;
  for (const auto& l_smoothed : dual_l_result_ds_vec) {
    l_smoothed_size += l_smoothed.cols();
  }

  std::size_t n_smoothed_size = 0;
  for (const auto& n_smoothed : dual_n_result_ds_vec) {
    n_smoothed_size += n_smoothed.cols();
  }

  Eigen::MatrixXd xWS_ =
      Eigen::MatrixXd::Zero(xWS_vec[0].rows(), warm_start_state_size);
  Eigen::MatrixXd uWS_ =
      Eigen::MatrixXd::Zero(uWS_vec[0].rows(), warm_start_control_size);
  Eigen::MatrixXd state_result_ds_ =
      Eigen::MatrixXd::Zero(state_result_ds_vec[0].rows(), smoothed_state_size);
  Eigen::MatrixXd control_result_ds_ = Eigen::MatrixXd::Zero(
      control_result_ds_vec[0].rows(), smoothed_control_size);
  Eigen::MatrixXd time_result_ds_ =
      Eigen::MatrixXd::Zero(time_result_ds_vec[0].rows(), time_size);
  Eigen::MatrixXd l_warm_up_ =
      Eigen::MatrixXd::Zero(l_warm_up_vec[0].rows(), l_warm_start_size);
  Eigen::MatrixXd n_warm_up_ =
      Eigen::MatrixXd::Zero(n_warm_up_vec[0].rows(), n_warm_start_size);
  Eigen::MatrixXd dual_l_result_ds_ =
      Eigen::MatrixXd::Zero(dual_l_result_ds_vec[0].rows(), l_smoothed_size);
  Eigen::MatrixXd dual_n_result_ds_ =
      Eigen::MatrixXd::Zero(dual_n_result_ds_vec[0].rows(), n_smoothed_size);

  std::size_t traj_size = xWS_vec.size();

  uint64_t counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_state_cols = xWS_vec[i].cols() - 1;
    for (std::size_t j = 0; j < warm_start_state_cols; ++j) {
      xWS_.col(counter) = xWS_vec[i].col(j);
      ++counter;
    }
  }
  xWS_.col(counter) = xWS_vec.back().col(xWS_vec.back().cols() - 1);
  ++counter;
  if (counter != warm_start_state_size) {
    LOG_ERROR("counter != warm_start_state_size");
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_control_cols = uWS_vec[i].cols();
    for (std::size_t j = 0; j < warm_start_control_cols; ++j) {
      uWS_.col(counter) = uWS_vec[i].col(j);
      ++counter;
    }
  }
  if (counter != warm_start_control_size) {
    LOG_ERROR("counter [{}] != warm_start_control_size [{}]", counter,
              warm_start_control_size);
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_state_cols = state_result_ds_vec[i].cols() - 1;
    for (std::size_t j = 0; j < smoothed_state_cols; ++j) {
      state_result_ds_.col(counter) = state_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  state_result_ds_.col(counter) =
      state_result_ds_vec.back().col(state_result_ds_vec.back().cols() - 1);
  ++counter;
  if (counter != smoothed_state_size) {
    LOG_ERROR("counter != smoothed_state_size");
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_control_cols = control_result_ds_vec[i].cols();
    for (std::size_t j = 0; j < smoothed_control_cols; ++j) {
      control_result_ds_.col(counter) = control_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  if (counter != smoothed_control_size) {
    LOG_ERROR("counter [{}] != smoothed_control_size [{}]", counter,
              smoothed_control_size);
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t time_cols = time_result_ds_vec[i].cols();
    for (std::size_t j = 0; j < time_cols; ++j) {
      time_result_ds_.col(counter) = time_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  if (counter != time_size) {
    LOG_ERROR("counter [{}] != time_size [{}]", counter, time_size);
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t l_warm_up_cols = l_warm_up_vec[i].cols();
    for (std::size_t j = 0; j < l_warm_up_cols; ++j) {
      l_warm_up_.col(counter) = l_warm_up_vec[i].col(j);
      ++counter;
    }
  }
  if (counter != l_warm_start_size) {
    LOG_ERROR("counter [{}] != l_warm_start_size [{}]", counter,
              l_warm_start_size);
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t n_warm_up_cols = n_warm_up_vec[i].cols();
    for (std::size_t j = 0; j < n_warm_up_cols; ++j) {
      n_warm_up_.col(counter) = n_warm_up_vec[i].col(j);
      ++counter;
    }
  }
  if (counter != n_warm_start_size) {
    LOG_ERROR("counter [{}] != n_warm_start_size [{}]", counter,
              n_warm_start_size);
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t dual_l_result_ds_cols = dual_l_result_ds_vec[i].cols();
    for (std::size_t j = 0; j < dual_l_result_ds_cols; ++j) {
      dual_l_result_ds_.col(counter) = dual_l_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  if (counter != l_smoothed_size) {
    LOG_ERROR("counter [{}] != l_smoothed_size [{}]", counter,
              l_smoothed_size);
    return;
  }

  counter = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t dual_n_result_ds_cols = dual_n_result_ds_vec[i].cols();
    for (std::size_t j = 0; j < dual_n_result_ds_cols; ++j) {
      dual_n_result_ds_.col(counter) = dual_n_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  if (counter != n_smoothed_size) {
    LOG_ERROR("counter [{}] != n_smoothed_size [{}]", counter,
              n_smoothed_size);
    return;
  }

  *xWS = std::move(xWS_);
  *uWS = std::move(uWS_);
  *state_result_ds = std::move(state_result_ds_);
  *control_result_ds = std::move(control_result_ds_);
  *time_result_ds = std::move(time_result_ds_);
  *l_warm_up = std::move(l_warm_up_);
  *n_warm_up = std::move(n_warm_up_);
  *dual_l_result_ds = std::move(dual_l_result_ds_);
  *dual_n_result_ds = std::move(dual_n_result_ds_);
  return;
}

void ParkingTrajectoryOptimizer::ReTransformOnAnchor(
    const ReferencePoint& anchor_point, std::vector<TrajectoryPoint>& output) {
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;

  double end_x = anchor_point.x(), end_y = anchor_point.y();
  double end_theta = anchor_point.heading();
  for (auto iter = output.begin(); iter != output.end(); iter++) {
    vehicle2earth(end_x, end_y, end_theta, iter->x(), iter->y(), iter->theta(),
                  tmp_x, tmp_y, tmp_theta);
    iter->set_x(tmp_x);
    iter->set_y(tmp_y);
    iter->set_theta(tmp_theta);
  }
}

bool ParkingTrajectoryOptimizer::IterativeSmoothing(
    const TrajectoryPoint& trajectory_init_point,
    std::vector<TrajectoryPoint>& input,
    std::vector<HybridAStartResult>& partition_trajectories,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    std::vector<TrajectoryPoint>& output) {
  if (partition_trajectories.empty() && input.empty()) {
    LOG_ERROR("input is empty");
    return false;
  }
  if (partition_trajectories.empty()) {
    HybridAStartResult tmp_result;
    int direction = input.front().direction();
    for (size_t i = 0; i < input.size(); ++i) {
      if (direction != input[i].direction()) {
        partition_trajectories.push_back(tmp_result);
        direction = input[i].direction();
        tmp_result.x.clear();
        tmp_result.y.clear();
        tmp_result.phi.clear();
        tmp_result.steer.clear();
        tmp_result.accumulated_s.clear();
        tmp_result.direction.clear();
        tmp_result.segment_index.clear();
        tmp_result.v.clear();
        tmp_result.a.clear();
      }
      tmp_result.x.push_back(input[i].x());
      tmp_result.y.push_back(input[i].y());
      tmp_result.phi.push_back(input[i].theta());
      tmp_result.steer.push_back(input[i].steer());
      tmp_result.accumulated_s.push_back(input[i].s());
      tmp_result.direction.push_back(input[i].direction());
      tmp_result.segment_index.push_back(input[i].segment_index());
      tmp_result.v.push_back(1.0);
      tmp_result.a.push_back(0.0);
    }
  }

  Eigen::MatrixXd xWS;
  Eigen::MatrixXd uWS;
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;
  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;
  Eigen::MatrixXd dual_l_result_ds;
  Eigen::MatrixXd dual_n_result_ds;

  std::size_t size = partition_trajectories.size();
  std::vector<Eigen::MatrixXd> xWS_vec;
  std::vector<Eigen::MatrixXd> uWS_vec;
  std::vector<Eigen::MatrixXd> state_result_ds_vec;
  std::vector<Eigen::MatrixXd> control_result_ds_vec;
  std::vector<Eigen::MatrixXd> time_result_ds_vec;
  std::vector<Eigen::MatrixXd> l_warm_up_vec;
  std::vector<Eigen::MatrixXd> n_warm_up_vec;
  std::vector<Eigen::MatrixXd> dual_l_result_ds_vec;
  std::vector<Eigen::MatrixXd> dual_n_result_ds_vec;
  xWS_vec.resize(size);
  uWS_vec.resize(size);
  state_result_ds_vec.resize(size);
  control_result_ds_vec.resize(size);
  time_result_ds_vec.resize(size);
  l_warm_up_vec.resize(size);
  n_warm_up_vec.resize(size);
  dual_l_result_ds_vec.resize(size);
  dual_n_result_ds_vec.resize(size);
  // In for loop
  LOG_INFO("Trajectories size in smoother is {}", size);
  for (std::size_t i = 0; i < size; ++i) {
    LoadHybridAstarResultInEigen(&partition_trajectories[i], &xWS_vec[i],
                                 &uWS_vec[i]);

    Eigen::MatrixXd last_time_u(2, 1);
    double init_v = 0.0;
    // Stitching point control and velocity is set for first piece of
    // trajectories. In the next ones, control and velocity are assumed to be
    // zero as the next trajectories always start from vehicle static state
    if (i == 0) {
      const double init_steer = trajectory_init_point.steer();
      const double init_a = trajectory_init_point.acceleration();
      last_time_u << init_steer, init_a;
      init_v = trajectory_init_point.velocity();
    } else {
      last_time_u << 0.0, 0.0;
      init_v = 0.0;
    }
    if (!GenerateDecoupledTraj(xWS_vec[i], last_time_u(1, 0), init_v,
                               obstacles_vertices_vec, &state_result_ds_vec[i],
                               &control_result_ds_vec[i],
                               &time_result_ds_vec[i])) {
      LOG_INFO("Smoother failed at {} th trajectory, trajectory size: {}", i,
               xWS_vec[i].cols());
      return false;
    }
  }

  // Retrive the trajectory in one piece
  CombineTrajectories(xWS_vec, uWS_vec, state_result_ds_vec,
                      control_result_ds_vec, time_result_ds_vec, l_warm_up_vec,
                      n_warm_up_vec, dual_l_result_ds_vec, dual_n_result_ds_vec,
                      &xWS, &uWS, &state_result_ds, &control_result_ds,
                      &time_result_ds, &l_warm_up, &n_warm_up,
                      &dual_l_result_ds, &dual_n_result_ds);

  long states_size = state_result_ds.cols();
  TrajectoryPoint point;
  double relative_s = 0.0;
  Vec2d last_path_point(state_result_ds(0, 0), state_result_ds(1, 0));
  for (long i = 0; i < states_size; ++i) {
    point.mutable_path_point()->set_x(state_result_ds(0, i));
    point.mutable_path_point()->set_y(state_result_ds(1, i));
    point.mutable_path_point()->set_theta(state_result_ds(2, i));

    Vec2d cur_path_point(state_result_ds(0, i), state_result_ds(1, i));
    if (!output.empty()) {
      relative_s = output.back().s();
    } else {
      relative_s = 0.0;
    }
    relative_s += cur_path_point.distance_to(last_path_point);
    point.mutable_path_point()->set_s(relative_s);
    output.push_back(point);
    last_path_point = cur_path_point;
  }
  // TEST ouput
  for (auto pt : output) {
    LOG_INFO("before interpolate pt : x {:.4f} y {:.4f} theta {:.4f}", pt.x(), pt.y(),
             pt.theta());
  }
  InterpolatePoints(output);
  return true;
}

// bool ParkingTrajectoryOptimizer::distance_approach_smoothing(
//    const TrajectoryPoint& trajectory_init_point,
//    std::vector<TrajectoryPoint>& input, const std::vector<double>& XYbounds,
//    const Eigen::MatrixXi& obstacles_edges_num,
//    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
//    std::vector<HybridAStartResult>& partition_trajectories,
//    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
//    std::vector<TrajectoryPoint>& output) {
// if (partition_trajectories.empty() && input.empty()) {
//   LOG_ERROR("input is empty");
//   return false;
// }
// if (partition_trajectories.empty()) {
//   HybridAStartResult tmp_result;
//   int direction = input.front().direction();
//   for (size_t i = 0; i < input.size(); ++i) {
//     if (direction != input[i].direction()) {
//       partition_trajectories.push_back(tmp_result);
//       direction = input[i].direction();
//       tmp_result.x.clear();
//       tmp_result.y.clear();
//       tmp_result.phi.clear();
//       tmp_result.steer.clear();
//       tmp_result.accumulated_s.clear();
//       tmp_result.direction.clear();
//       tmp_result.segment_index.clear();
//       tmp_result.v.clear();
//       tmp_result.a.clear();
//     }
//     tmp_result.x.push_back(input[i].x());
//     tmp_result.y.push_back(input[i].y());
//     tmp_result.phi.push_back(input[i].theta());
//     tmp_result.steer.push_back(input[i].steer());
//     tmp_result.accumulated_s.push_back(input[i].s());
//     tmp_result.direction.push_back(input[i].direction());
//     tmp_result.segment_index.push_back(input[i].segment_index());
//     tmp_result.v.push_back(1.0);
//     tmp_result.a.push_back(0.0);
//   }
// }

// Eigen::MatrixXd xWS;
// Eigen::MatrixXd uWS;
// Eigen::MatrixXd state_result_ds;
// Eigen::MatrixXd control_result_ds;
// Eigen::MatrixXd time_result_ds;
// Eigen::MatrixXd l_warm_up;
// Eigen::MatrixXd n_warm_up;
// Eigen::MatrixXd dual_l_result_ds;
// Eigen::MatrixXd dual_n_result_ds;

// std::size_t size = partition_trajectories.size();
// std::vector<Eigen::MatrixXd> xWS_vec;
// std::vector<Eigen::MatrixXd> uWS_vec;
// std::vector<Eigen::MatrixXd> state_result_ds_vec;
// std::vector<Eigen::MatrixXd> control_result_ds_vec;
// std::vector<Eigen::MatrixXd> time_result_ds_vec;
// std::vector<Eigen::MatrixXd> l_warm_up_vec;
// std::vector<Eigen::MatrixXd> n_warm_up_vec;
// std::vector<Eigen::MatrixXd> dual_l_result_ds_vec;
// std::vector<Eigen::MatrixXd> dual_n_result_ds_vec;
// xWS_vec.resize(size);
// uWS_vec.resize(size);
// state_result_ds_vec.resize(size);
// control_result_ds_vec.resize(size);
// time_result_ds_vec.resize(size);
// l_warm_up_vec.resize(size);
// n_warm_up_vec.resize(size);
// dual_l_result_ds_vec.resize(size);
// dual_n_result_ds_vec.resize(size);
// // In for loop
// LOG_INFO("Trajectories size in smoother is {}", size);
// for (std::size_t i = 0; i < size; ++i) {
//   LoadHybridAstarResultInEigen(&partition_trajectories[i], &xWS_vec[i],
//                                &uWS_vec[i]);

//   Eigen::MatrixXd last_time_u(2, 1);
//   double init_v = 0.0;
//   // Stitching point control and velocity is set for first piece of
//   // trajectories. In the next ones, control and velocity are assumed to be
//   // zero as the next trajectories always start from vehicle static state
//   if (i == 0) {
//     const double init_steer = trajectory_init_point.steer();
//     const double init_a = trajectory_init_point.acceleration();
//     last_time_u << init_steer, init_a;
//     init_v = trajectory_init_point.velocity();
//   } else {
//     last_time_u << 0.0, 0.0;
//     init_v = 0.0;
//   }
//   if (!GenerateDistanceApproachTraj(
//           xWS_vec[i], uWS_vec[i], XYbounds, obstacles_edges_num,
//           obstacles_A, obstacles_b, obstacles_vertices_vec, last_time_u,
//           init_v, &state_result_ds_vec[i], &control_result_ds_vec[i],
//           &time_result_ds_vec[i], &l_warm_up_vec[i], &n_warm_up_vec[i],
//           &dual_l_result_ds_vec[i], &dual_n_result_ds_vec[i])) {
//     LOG_INFO(
//         "Smoother fail at {} th trajectory with index starts from 0, "
//         "trajectory size: {}",
//         i, xWS_vec[i].cols());
//     return false;
//   }
// }

// // Retrive the trajectory in one piece
// CombineTrajectories(xWS_vec, uWS_vec, state_result_ds_vec,
//                     control_result_ds_vec, time_result_ds_vec,
//                     l_warm_up_vec, n_warm_up_vec, dual_l_result_ds_vec,
//                     dual_n_result_ds_vec, &xWS, &uWS, &state_result_ds,
//                     &control_result_ds, &time_result_ds, &l_warm_up,
//                     &n_warm_up, &dual_l_result_ds, &dual_n_result_ds);

// long states_size = state_result_ds.cols();
// TrajectoryPoint point;
// double relative_s = 0.0;
// Vec2d last_path_point(state_result_ds(0, 0), state_result_ds(1, 0));
// for (long i = 0; i < states_size; ++i) {
//   point.mutable_path_point()->set_x(state_result_ds(0, i));
//   point.mutable_path_point()->set_y(state_result_ds(1, i));
//   point.mutable_path_point()->set_theta(state_result_ds(2, i));

//   Vec2d cur_path_point(state_result_ds(0, i), state_result_ds(1, i));
//   if (!output.empty()) {
//     relative_s = output.back().s();
//   } else {
//     relative_s = 0.0;
//   }
//   relative_s += cur_path_point.distance_to(last_path_point);
//   point.mutable_path_point()->set_s(relative_s);
//   output.push_back(point);
//   last_path_point = cur_path_point;
// }
// InterpolatePoints(output);
//  return true;
//}

bool ParkingTrajectoryOptimizer::InterpolatePoints(
    std::vector<TrajectoryPoint>& output) {
  if (output.empty()) {
    LOG_ERROR("input data is empty");
    return false;
  }
  const double precision = config_.common_config().path_resolution_;
  if (output.size() <= 3) {
    return true;
  }
  // TODO(wyc): this is a N^3 complexity code, consider more optimal way
  for (int i = 0; i < output.size() - 1; ++i) {
    auto iter = output.begin() + i;
    auto iter_next = iter + 1;
    while (fabs(iter->s() - iter_next->s()) > 1.5 * precision) {
      TrajectoryPoint tmp_pt = *iter;
      tmp_pt.set_x(Interpolation::lerp(iter->x(), iter->s(), iter_next->x(),
                                       iter_next->s(), iter->s() + precision));
      tmp_pt.set_y(Interpolation::lerp(iter->y(), iter->s(), iter_next->y(),
                                       iter_next->s(), iter->s() + precision));
      tmp_pt.set_theta(Interpolation::lerp(iter->theta(), iter->s(),
                                           iter_next->theta(), iter_next->s(),
                                           iter->s() + precision));
      tmp_pt.set_kappa(Interpolation::lerp(iter->kappa(), iter->s(),
                                           iter_next->kappa(), iter_next->s(),
                                           iter->s() + precision));
      tmp_pt.set_dkappa(Interpolation::lerp(iter->dkappa(), iter->s(),
                                            iter_next->dkappa(), iter_next->s(),
                                            iter->s() + precision));
      tmp_pt.set_s(iter->s() + precision);
      output.insert(iter + 1, tmp_pt);
      iter++;
    }
  }

  return true;
}

void ParkingTrajectoryOptimizer::GetSmoothTrajectoryDirection(
    std::vector<TrajectoryPoint>& output) {
  if (output.empty()) {
    LOG_INFO("output is empty");
    return;
  }

  if (output.size() < 2) {
    LOG_INFO("output size < 2");
    return;
  }

  for (std::size_t i = 0; i < output.size() - 1; ++i) {
    double heading_angle = output[i].theta();
    const Vec2d tracking_vector(output[i + 1].x() - output[i].x(),
                                output[i + 1].y() - output[i].y());
    double tracking_angle = tracking_vector.angle();
    bool gear =
        std::fabs(normalize_angle(tracking_angle - heading_angle)) < M_PI_2;
    LOG_INFO(
        "output {} : x {:.4f}, y {:.4f}, heading angle {:.4f}, tracking_angle {:.4f}",
        i, output[i].x(), output[i].y(), heading_angle, tracking_angle);
    int gear_int = gear ? 0 : 1;
    output[i].set_direction(gear_int);
  }

  output.back().set_direction(output[output.size() - 2].direction());
}

void ParkingTrajectoryOptimizer::LogHybriad(HybridAStartResult& result,
                                            const ReferencePoint& anchor_point,
                                            double rotate_angle,
                                            const Vec2d& translate_origin) {
  FILE* save;
  if (save = fopen("../test_data/output/parking/hybridastar.csv", "w")) {
    HybridAStartResult new_result(result);
    // x,y,theta,kappa,s, direction, index
    for (size_t i = 0; i < new_result.x.size(); ++i) {
      PathPointDeNormalizing(rotate_angle, translate_origin, &new_result.x[i],
                             &new_result.y[i], &new_result.phi[i]);
      double tmp_x = new_result.x[i], tmp_y = new_result.y[i],
             tmp_heading = new_result.phi[i];
      double x = tmp_x, y = tmp_y, theta = tmp_heading;
      vehicle2earth(anchor_point.x(), anchor_point.y(), anchor_point.heading(),
                    x, y, theta, tmp_x, tmp_y, tmp_heading);
      fprintf(save, "%.4f,%.4f,%.4f, %.4f, %.4f, %d,%d,", tmp_x, tmp_y,
              tmp_heading, new_result.steer[i], new_result.accumulated_s[i],
              new_result.direction[i], new_result.segment_index[i]);
      Box2d bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
          {tmp_x, tmp_y}, tmp_heading);
      std::vector<Vec2d> corners;
      bounding_box.get_all_corners(&corners);
      for (size_t j = 0; j < corners.size(); ++j) {
        fprintf(save, "%.4f,%.4f,", corners[j].x(), corners[j].y());
      }
      fprintf(save, "\n");
    }
    fclose(save);
  }
}

void ParkingTrajectoryOptimizer::LogOutput(
    std::vector<TrajectoryPoint>& output) {
  FILE* save;
  if (save = fopen("../test_data/output/parking/ParkingFile.csv", "w")) {
    // x,y,theta,kappa,s, direction, index
    for (size_t i = 0; i < output.size(); ++i) {
      fprintf(save, "%.4f,%.4f,%.4f,%.4f, %.4f, %d,%d,", output[i].x(),
              output[i].y(), output[i].theta(), output[i].kappa(),
              output[i].s(), output[i].direction(), output[i].segment_index());
      Box2d bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
          {output[i].x(), output[i].y()}, output[i].theta());
      std::vector<Vec2d> corners;
      bounding_box.get_all_corners(&corners);
      for (size_t j = 0; j < corners.size(); ++j) {
        fprintf(save, "%.4f,%.4f,", corners[j].x(), corners[j].y());
      }
      fprintf(save, "\n");
    }
    fclose(save);
  }
}

void ParkingTrajectoryOptimizer::LogTraj(DiscretizedTrajectory& traj) {
  FILE* save;
  if (save = fopen("../test_data/output/parking/ParkingFile.csv", "w")) {
    // x,y,theta,kappa,s, direction, index
    auto& output = traj.trajectory_points();
    for (size_t i = 0; i < output.size(); ++i) {
      fprintf(save, "%.4f,%.4f,%.4f,%.4f, %.4f, %d,%d,", output[i].x(),
              output[i].y(), output[i].theta(), output[i].kappa(),
              output[i].s(), output[i].direction(), output[i].segment_index());
      Box2d bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
          {output[i].x(), output[i].y()}, output[i].theta());
      std::vector<Vec2d> corners;
      bounding_box.get_all_corners(&corners);
      for (size_t j = 0; j < corners.size(); ++j) {
        fprintf(save, "%.4f,%.4f,", corners[j].x(), corners[j].y());
      }
      fprintf(save, "\n");
    }
    fclose(save);
  }
}

//******************discard
// ErrorCode ParkingTrajectoryOptimizer::Plan(
//    const std::vector<TrajectoryPoint>& stitching_trajectory,
//    const std::vector<double>& end_pose, const std::vector<double>& XYbounds,
//    double rotate_angle, const Vec2d& translate_origin,
//    const Eigen::MatrixXi& obstacles_edges_num,
//    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
//    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
//    const ReferencePoint& anchor_point) {
//  if (XYbounds.empty() || end_pose.empty() || obstacles_edges_num.cols() == 0
//  ||
//      obstacles_A.cols() == 0 || obstacles_b.cols() == 0) {
//    LOG_ERROR("ParkingTrajectoryOptimizer input data not ready");
//    return ErrorCode::PLANNING_ERROR_FAILED;
//  }
//  if (stitching_trajectory.empty()) {
//    LOG_ERROR("stitching_trajectory is empty");
//    return ErrorCode::PLANNING_ERROR_FAILED;
//  }

//  // Generate Stop trajectory if init point close to destination
//  if (IsInitPointNearDestination(stitching_trajectory.back(), end_pose,
//                                 rotate_angle, translate_origin)) {
//    LOG_INFO(
//        "Planning init point is close to destination, skip new "
//        "trajectory generation");
//    return ErrorCode::PLANNING_OK;
//  }

//  stitching_trajectory_ = stitching_trajectory;

//  const TrajectoryPoint trajectory_init_point = stitching_trajectory.back();

//  // init x, y, z would be rotated.
//  double init_x = trajectory_init_point.x();
//  double init_y = trajectory_init_point.y();
//  double init_heading = trajectory_init_point.theta();
//  LOG_INFO(
//      "origin x: {:.4f}, y: {:.4f}, heading: {:.4f}; init x: {:.4f}, y: {:.4f}, heading:
//      "
//      "{:.4f}",
//      translate_origin.x(), translate_origin.y(), rotate_angle, init_x,
//      init_y, init_heading);

//  // Rotate and scale the state
//  PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
//                       &init_heading);

//  // first path plan [init_v = 0]
//  HybridAStartResult result;
//  if (hybrid_astar_->Plan(init_x, init_y, init_heading, end_pose[0],
//                          end_pose[1], end_pose[2], XYbounds,
//                          obstacles_vertices_vec, &result)) {
//    LOG_INFO("Hybrid A* successfully solved!");
//  } else {
//    LOG_ERROR("Hybrid A* failed to solve");
//    return ErrorCode::PLANNING_ERROR_FAILED;
//  }
//  //**********************************************
//  FILE* save;
//  if (save = fopen("../test_data/output/hybridastar.csv", "w")) {
//    HybridAStartResult new_result(result);

//    for (int i = 0; i < new_result.x.size() - 2; ++i) {
//      PathPointDeNormalizing(rotate_angle, translate_origin, &new_result.x[i],
//                             &new_result.y[i], &new_result.phi[i]);
//      double tmp_x = new_result.x[i], tmp_y = new_result.y[i],
//             tmp_heading = new_result.phi[i];
//      double x = tmp_x, y = tmp_y, theta = tmp_heading;
//      vehicle2earth(anchor_point.x(), anchor_point.y(),
//      anchor_point.heading(),
//                    x, y, theta, tmp_x, tmp_y, tmp_heading);
//      fprintf(save, "%f,%f,%f,0.0, %f, %f,%f,", tmp_x, tmp_y, tmp_heading,
//              new_result.steer[i], 0.0, 0.0);
//      Box2d bounding_box =
//          VehicleParam::Instance()->get_adc_bounding_box({tmp_x, tmp_y},
//                                                         tmp_heading);
//      std::vector<Vec2d> corners;
//      bounding_box.get_all_corners(&corners);
//      for (int j = 0; j < corners.size(); ++j) {
//        fprintf(save, "%f,%f,", corners[j].x(), corners[j].y());
//      }
//      fprintf(save, "\n");
//    }
//    fclose(save);
//  }
//  //********************************************************

//  // Containers for distance approach trajectory smoothing problem
//  Eigen::MatrixXd xWS;
//  Eigen::MatrixXd uWS;
//  Eigen::MatrixXd state_result_ds;
//  Eigen::MatrixXd control_result_ds;
//  Eigen::MatrixXd time_result_ds;
//  Eigen::MatrixXd l_warm_up;
//  Eigen::MatrixXd n_warm_up;
//  Eigen::MatrixXd dual_l_result_ds;
//  Eigen::MatrixXd dual_n_result_ds;

//  if (tra_opt_config_.enable_parallel_trajectory_smoothing_) {
//    std::vector<HybridAStartResult> partition_trajectories;
//    if (!hybrid_astar_->TrajectoryPartition(result, &partition_trajectories))
//    {
//      LOG_ERROR("TrajectoryPartition hybrid_astar results failed");
//      return ErrorCode::PLANNING_ERROR_FAILED;
//    }
//    std::size_t size = partition_trajectories.size();
//    std::vector<Eigen::MatrixXd> xWS_vec;
//    std::vector<Eigen::MatrixXd> uWS_vec;
//    std::vector<Eigen::MatrixXd> state_result_ds_vec;
//    std::vector<Eigen::MatrixXd> control_result_ds_vec;
//    std::vector<Eigen::MatrixXd> time_result_ds_vec;
//    std::vector<Eigen::MatrixXd> l_warm_up_vec;
//    std::vector<Eigen::MatrixXd> n_warm_up_vec;
//    std::vector<Eigen::MatrixXd> dual_l_result_ds_vec;
//    std::vector<Eigen::MatrixXd> dual_n_result_ds_vec;
//    xWS_vec.resize(size);
//    uWS_vec.resize(size);
//    state_result_ds_vec.resize(size);
//    control_result_ds_vec.resize(size);
//    time_result_ds_vec.resize(size);
//    l_warm_up_vec.resize(size);
//    n_warm_up_vec.resize(size);
//    dual_l_result_ds_vec.resize(size);
//    dual_n_result_ds_vec.resize(size);

//    // In for loop
//    LOG_INFO("Trajectories size in smoother is {}", size);
//    for (std::size_t i = 0; i < size; ++i) {
//      LoadHybridAstarResultInEigen(&partition_trajectories[i], &xWS_vec[i],
//                                   &uWS_vec[i]);

//      Eigen::MatrixXd last_time_u(2, 1);
//      double init_v = 0.0;
//      // Stitching point control and velocity is set for first piece of
//      // trajectories. In the next ones, control and velocity are assumed to
//      be
//      // zero as the next trajectories always start from vehicle static state
//      if (i == 0) {
//        const double init_steer = trajectory_init_point.steer();
//        const double init_a = trajectory_init_point.acceleration();
//        last_time_u << init_steer, init_a;
//        init_v = trajectory_init_point.velocity();
//      } else {
//        last_time_u << 0.0, 0.0;
//        init_v = 0.0;
//      }
//      // TODO: Further testing
//      if (tra_opt_config_.use_iterative_anchoring_smoother_) {
//        if (!GenerateDecoupledTraj(
//                xWS_vec[i], last_time_u(1, 0), init_v, obstacles_vertices_vec,
//                &state_result_ds_vec[i], &control_result_ds_vec[i],
//                &time_result_ds_vec[i])) {
//          LOG_INFO("Smoother failed at {} th trajectory, trajectory size: {}",
//                   i, xWS_vec[i].cols());
//          return ErrorCode::PLANNING_ERROR_FAILED;
//        }
//      } else {
//        if (!GenerateDistanceApproachTraj(
//                xWS_vec[i], uWS_vec[i], XYbounds, obstacles_edges_num,
//                obstacles_A, obstacles_b, obstacles_vertices_vec, last_time_u,
//                init_v, &state_result_ds_vec[i], &control_result_ds_vec[i],
//                &time_result_ds_vec[i], &l_warm_up_vec[i], &n_warm_up_vec[i],
//                &dual_l_result_ds_vec[i], &dual_n_result_ds_vec[i])) {
//          LOG_INFO(
//              "Smoother fail at {} th trajectory with index starts from 0, "
//              "trajectory size: {}",
//              i, xWS_vec[i].cols());
//          return ErrorCode::PLANNING_ERROR_FAILED;
//        }
//      }
//    }

//    // Retrive the trajectory in one piece
//    CombineTrajectories(xWS_vec, uWS_vec, state_result_ds_vec,
//                        control_result_ds_vec, time_result_ds_vec,
//                        l_warm_up_vec, n_warm_up_vec, dual_l_result_ds_vec,
//                        dual_n_result_ds_vec, &xWS, &uWS, &state_result_ds,
//                        &control_result_ds, &time_result_ds, &l_warm_up,
//                        &n_warm_up, &dual_l_result_ds, &dual_n_result_ds);

//  } else {
//    LoadHybridAstarResultInEigen(&result, &xWS, &uWS);

//    const double init_steer = trajectory_init_point.steer();
//    const double init_a = trajectory_init_point.acceleration();
//    Eigen::MatrixXd last_time_u(2, 1);
//    last_time_u << init_steer, init_a;

//    const double init_v = trajectory_init_point.velocity();

//    if (!GenerateDistanceApproachTraj(
//            xWS, uWS, XYbounds, obstacles_edges_num, obstacles_A, obstacles_b,
//            obstacles_vertices_vec, last_time_u, init_v, &state_result_ds,
//            &control_result_ds, &time_result_ds, &l_warm_up, &n_warm_up,
//            &dual_l_result_ds, &dual_n_result_ds)) {
//      LOG_ERROR("GenerateDistanceApproachTraj failed");
//      return ErrorCode::PLANNING_ERROR_FAILED;
//    }
//  }

//  // rescale the states to the world frame
//  long state_size = state_result_ds.cols();
//  for (long i = 0; i < state_size; ++i) {
//    PathPointDeNormalizing(rotate_angle, translate_origin,
//                           &(state_result_ds(0, i)), &(state_result_ds(1, i)),
//                           &(state_result_ds(2, i)));
//  }
//  LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);
//  ReTransformOnAnchor(anchor_point);

//  return ErrorCode::PLANNING_OK;
//}

// void ParkingTrajectoryOptimizer::GetStitchingTrajectory(
//    std::vector<TrajectoryPoint>* stitching_trajectory) {
//  if (stitching_trajectory == nullptr) return;
//  stitching_trajectory->clear();
//  *stitching_trajectory = stitching_trajectory_;
//}

// void ParkingTrajectoryOptimizer::GetOptimizedTrajectory(
//    DiscretizedTrajectory* optimized_trajectory) {
//  if (optimized_trajectory == nullptr) return;
//  optimized_trajectory->mutable_trajectory_points()->clear();
//  *optimized_trajectory = optimized_trajectory_;
//}

// void ParkingTrajectoryOptimizer::LoadTrajectory(
//    const Eigen::MatrixXd& state_result, const Eigen::MatrixXd&
//    control_result, const Eigen::MatrixXd& time_result) {
//  optimized_trajectory_.mutable_trajectory_points()->clear();

//  // Optimizer doesn't take end condition control state into consideration for
//  // now
//  long states_size = state_result.cols();
//  long times_size = time_result.cols();
//  long controls_size = control_result.cols();
//  if (states_size != times_size + 1) {
//    LOG_ERROR("states_size [{}] != times_size [{}] + 1 ", states_size,
//              times_size);
//    return;
//  }
//  if (states_size != controls_size + 1) {
//    LOG_ERROR("states_size [{}] != controls_size [{}] + 1 ", states_size,
//              controls_size);
//    return;
//  }
//  double relative_time = 0.0;
//  double relative_s = 0.0;
//  Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
//  for (long i = 0; i < states_size; ++i) {
//    TrajectoryPoint point;
//    point.mutable_path_point()->set_x(state_result(0, i));
//    point.mutable_path_point()->set_y(state_result(1, i));
//    point.mutable_path_point()->set_theta(state_result(2, i));
//    point.set_velocity(state_result(3, i));
//    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
//    relative_s += cur_path_point.distance_to(last_path_point);
//    point.mutable_path_point()->set_s(relative_s);
//    // TODO: Evaluate how to set end states control input
//    if (i == controls_size) {
//      point.set_steer(0.0);
//      point.set_acceleration(0.0);
//    } else {
//      point.set_steer(control_result(0, i));
//      point.set_acceleration(control_result(1, i));
//    }

//    if (i == 0) {
//      point.set_relative_time(relative_time);
//    } else {
//      relative_time += time_result(0, i - 1);
//      point.set_relative_time(relative_time);
//    }

//    optimized_trajectory_.mutable_trajectory_points()->push_back(point);
//    last_path_point = cur_path_point;
//  }
//  return;
//}

// void ParkingTrajectoryOptimizer::ReTransformOnAnchor(
//    const ReferencePoint& anchor_point) {
//  auto pts = optimized_trajectory_.mutable_trajectory_points();
//  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;

//  double end_x = anchor_point.x(), end_y = anchor_point.y();
//  double end_theta = anchor_point.heading();
//  for (auto iter = pts->begin(); iter != pts->end(); iter++) {
//    vehicle2earth(end_x, end_y, end_theta, iter->x(), iter->y(),
//    iter->theta(),
//                  tmp_x, tmp_y, tmp_theta);
//    iter->set_x(tmp_x);
//    iter->set_y(tmp_y);
//    iter->set_theta(tmp_theta);
//  }
//}

}  // namespace planning
}  // namespace neodrive
