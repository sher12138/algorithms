#include "task/optimizers/backup_path_optimizer/backup_path_optimizer.h"

namespace neodrive {
namespace planning {
namespace {

std::map<PoseType, std::string> posetype2string{
    {PoseType::REASONABLE, "REASONABLE"},
    {PoseType::UNREASONABLE_LOCATION, "UNREASONABLE_LOCATION"},
    {PoseType::UNREASONABLE_ORIENTATION, "UNREASONABLE_ORIENTATION"},
    {PoseType::UNREASONABLE_BOTH, "UNREASONABLE_BOTH"}};
std::map<CollisionType, std::string> collisiontype2string{
    {CollisionType::NO_COLLISION, "NO_COLLISION"},
    {CollisionType::VIS_CURB_COLLISION, "VIS_CURB_COLLISION"},
    {CollisionType::MAP_ROAD_COLIISION, "MAP_ROAD_COLIISION"},
    {CollisionType::OBS_COLIISION, "OBS_COLIISION"},
    {CollisionType::PATH_ERROR, "PATH_ERROR"}};

void VisPathBoxes(const std::vector<Box2d>& boxes) {
  if (!FLAGS_planning_enable_vis_event || boxes.empty()) return;
  LOG_INFO("path collision boxes size:{}", boxes.size());
  auto event = vis::EventSender::Instance()->GetEvent("path collison boxes");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);
  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  for (auto& box : boxes) {
    std::vector<Vec2d> corners;
    box.get_all_corners(&corners);
    auto polygon = event->mutable_polygon()->Add();
    for (auto& xy_pt : corners) {
      set_pt(polygon->add_point(), xy_pt);
    }
  }
}

bool IsAllowedExtendToReverseLane(const ReferencePoint& ref_pt) {
  /// 1.Read the information of the reference point
  BoundaryEdgeType left_boundary_edge_type = ref_pt.left_boundary_edge_type();
  std::vector<DividerFeature> left_divider_feature =
      ref_pt.left_divider_feature();
  double left_road_bound = ref_pt.left_road_bound(),
         left_reverse_road_bound = ref_pt.left_reverse_road_bound();

  /// 2.Information updates
  bool is_left_bound_allowed_cross =
      (left_boundary_edge_type == BoundaryEdgeType::MARKING);
  int cross_cnt = 0;
  for (int i = 0; i < left_divider_feature.size(); ++i) {
    DividerFeature divider_feature = left_divider_feature[i];
    if (scenario_common::CanCrossLane(divider_feature.divider_type_,
                                      divider_feature.divider_color_))
      cross_cnt++;
  }
  bool is_left_divider_allowed_cross =
      (cross_cnt == left_divider_feature.size());
  bool is_reverse_lane_allowed_cross =
      (std::abs(left_road_bound - left_reverse_road_bound) >= 1e-5);

  LOG_INFO(
      "ref_pt infos x:{:.4f}, y:{:.4f}, left_boundary_edge_type:{}, "
      "left_road_bound:{:.4f}, left_reverse_road_bound:{:.4f}, check_res: "
      "{} {} {}",
      ref_pt.x(), ref_pt.y(), ref_pt.left_boundary_edge_type(),
      ref_pt.left_road_bound(), ref_pt.left_reverse_road_bound(),
      is_left_bound_allowed_cross, is_left_divider_allowed_cross,
      is_reverse_lane_allowed_cross);

  /// 3.Excluded logic
  if (!is_left_bound_allowed_cross) return false;
  if (!is_left_divider_allowed_cross) return false;
  if (!is_reverse_lane_allowed_cross) return false;

  return true;
}

void VisFinalPath(const std::vector<PathPoint>& points,
                  const std::string name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pts) {
    for (auto& pt : pts) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->mutable_center()->set_x(pt.coordinate().x());
      sphere->mutable_center()->set_y(pt.coordinate().y());
      sphere->mutable_center()->set_z(0);
      sphere->set_radius(0.1);
    }
  };

  set_pts(event, points);
}

bool IsPathValid(const std::vector<BackupPathPlanner::WayPoint>& pts) {
  return !pts.empty();
}

bool UnconstraintOptimization(TaskInfo& task_info, const TrajectoryPoint& veh,
                              const double delta_s, const double dense_s,
                              std::vector<BackupPathPlanner::WayPoint>* pts,
                              std::string name, double gain) {
  if (pts->size() < 4) return false;

  auto& ref_line = task_info.reference_line();
  auto const outside_data =
      task_info.current_frame()->mutable_outside_planner_data();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();

  /// compute init_state
  double s0{0.}, l0{0.};
  if (SLPoint p; ref_line->GetPointInFrenetFrame({veh.x(), veh.y()}, &p)) {
    std::tie(s0, l0) = std::pair{p.s(), p.l()};
  } else {
    return false;
  }
  LOG_INFO("{} plan from xy({:.3f}, {:.3f}), sl({:.3f}, {:.3f})", name, veh.x(),
           veh.y(), s0, l0);
  ReferencePoint ref_p0{};
  if (!ref_line->GetNearestRefPoint(s0, &ref_p0)) return false;
  double dl0 = SLAnalyticTransformation::calculate_lateral_derivative(
      ref_p0.heading(), veh.theta(), l0, ref_p0.kappa());
  double ddl0 =
      DataCenter::Instance()->master_info().is_use_position_stitch()
          ? 0.0
          : SLAnalyticTransformation::calculate_second_order_lateral_derivative(
                ref_p0.heading(), veh.theta(), ref_p0.kappa(), veh.kappa(),
                ref_p0.dkappa(), l0);

  // adjust init state when encountering high curvature lanes
  path_planner_common::AdjustInitState(ref_line, inside_data, outside_data, dl0,
                                       ddl0);

  /// model
  BackupPath::State init_state{
      .state_l = l0, .state_dl = dl0, .state_ddl = ddl0};
  BackupPath::Control init_control{.control_dddl = 0.};
  BackupPathModel model("BackupPathModel", init_state, init_control, delta_s);

  /// create optimization
  tk::spline goal_sl_spline{};
  std::vector<double> waypoints_s_vec(pts->size(), 0.);
  std::vector<double> waypoints_l_vec(pts->size(), 0.);
  for (std::size_t i = 0; i < pts->size(); ++i) {
    waypoints_l_vec[i] = pts->at(i).l;
    waypoints_s_vec[i] = pts->at(i).s;
  }
  goal_sl_spline.set_points(waypoints_s_vec, waypoints_l_vec,
                            tk::spline::spline_type::linear);
  double total_length = pts->back().s - pts->front().s;
  std::size_t n = static_cast<std::size_t>(total_length / delta_s) + 1;
  const auto& config = config::PlanningConfig::Instance()
                           ->planning_research_config()
                           .path_backup_optimizer_config;
  std::vector<BackupPath::Stage> stages{};
  stages.resize(n + 1);
  for (std::size_t i = 0; i <= n; ++i) {
    stages[i].nx = BackupPath::nx;
    stages[i].nu = BackupPath::nu;
    stages[i].ng = (i == 0) ? 0 : BackupPath::npc;
    stages[i].ns = (i == 0) ? 0 : BackupPath::ns;
    stages[i].line_model = model.line_model_matrix();
    stages[i].cost.Q(0, 0) = 0.5 * config.weight_l * gain;
    stages[i].cost.Q(1, 1) = 0.5 * config.weight_dl;
    stages[i].cost.Q(2, 2) = 0.5 * config.weight_ddl;
    stages[i].cost.q = BackupPath::q::Zero();
    stages[i].cost.q(0, 0) = -0.5 *
                             goal_sl_spline(pts->front().s + i * delta_s) *
                             config.weight_l * gain;
    stages[i].cost.R(0, 0) = 0.5 * config.weight_dddl;
    stages[i].cost.r(0, 0) = 0.;
    stages[i].cost.S = BackupPath::S::Zero();
    stages[i].cost.Z = BackupPath::Z::Zero();
    stages[i].cost.z = BackupPath::z::Zero();

    stages[i].constraints.C = model.polytopic_constraint().C;
    stages[i].constraints.D = model.polytopic_constraint().D;
    stages[i].constraints.dl = model.polytopic_constraint().dl;
    stages[i].constraints.du = model.polytopic_constraint().du;

    stages[i].upper_bounds_x << 2 * BackupPath::INF, 2 * BackupPath::INF,
        2 * BackupPath::INF;
    stages[i].lower_bounds_x << -2 * BackupPath::INF, -2 * BackupPath::INF,
        -2 * BackupPath::INF;

    stages[i].upper_bounds_u << 2.0;
    stages[i].lower_bounds_u << -2.0;

    stages[i].upper_bounds_s = BackupPath::Bounds_s::Zero();
    stages[i].lower_bounds_s = BackupPath::Bounds_s::Zero();
  }

  /// solve
  std::shared_ptr<BackupPathHpipmSolver> solver =
      std::make_shared<BackupPathHpipmSolver>("backup_path_solver", n,
                                              BackupPath::nx, BackupPath::nu);
  int solver_status{-1};
  const auto opt_variables =
      solver->SolveMPC(init_state, stages, &solver_status);
  LOG_INFO("solver_status: {}", solver_status);
  if (solver_status != 0) return false;

  /// dense
  BackupPath::A A_dense_s{};
  BackupPath::B B_dense_s{};
  BackupPath::StateVector state_vector{};
  BackupPath::ControlVector control_vector{};
  Vec2d tmp_pt{};
  double s{}, l{};
  double accumulated_s = pts->front().s;
  double max_accumulated_s = pts->back().s;
  double path_output_s =
      std::min(ref_line->ref_points().back().s() - 1,
               accumulated_s +
                   std::max(DataCenter::Instance()->drive_strategy_max_speed() *
                                FLAGS_planning_trajectory_time_length,
                            FLAGS_planning_trajectory_min_length));
  pts->clear();
  for (std::size_t i = 0; i < opt_variables.size(); ++i) {
    if (accumulated_s + kMathEpsilon > max_accumulated_s ||
        accumulated_s + kMathEpsilon > path_output_s) {
      break;
    }
    s = accumulated_s;
    l = opt_variables[i].xk.state_l;
    ref_line->GetPointInCartesianFrame({s, l}, &tmp_pt);
    pts->emplace_back(
        BackupPathPlanner::WayPoint{.x = tmp_pt.x(),
                                    .y = tmp_pt.y(),
                                    .z = 0.,
                                    .s = s,
                                    .l = l,
                                    .dl = opt_variables[i].xk.state_dl,
                                    .ddl = opt_variables[i].xk.state_ddl});
    LOG_DEBUG(
        "s, le, l, dl, ddl, dddl: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}",
        s, goal_sl_spline(s), l, pts->back().dl, pts->back().ddl,
        opt_variables[i].uk.control_dddl);
    state_vector = BackupPath::StateToVector(opt_variables[i].xk);
    control_vector = BackupPath::ControlToVector(opt_variables[i].uk);
    for (std::size_t j = 1; j < static_cast<std::size_t>(delta_s / dense_s);
         ++j) {
      if (accumulated_s + j * dense_s + kMathEpsilon > max_accumulated_s) {
        break;
      }
      if (accumulated_s + j * dense_s + kMathEpsilon > path_output_s) {
        break;
      }
      A_dense_s.row(0) << 1, j * dense_s, 0.5 * std::pow(j * dense_s, 2);
      A_dense_s.row(1) << 0, 1, j * dense_s;
      A_dense_s.row(2) << 0, 0, 1;
      B_dense_s << 1. / 6 * std::pow(j * dense_s, 3),
          0.5 * std::pow(j * dense_s, 2), j * dense_s;
      BackupPath::State stage_dense_s = BackupPath::VectorToState(
          A_dense_s * state_vector + B_dense_s * control_vector);
      s = accumulated_s + j * dense_s;
      l = stage_dense_s.state_l;
      ref_line->GetPointInCartesianFrame({s, l}, &tmp_pt);
      pts->emplace_back(
          BackupPathPlanner::WayPoint{.x = tmp_pt.x(),
                                      .y = tmp_pt.y(),
                                      .z = 0.,
                                      .s = s,
                                      .l = l,
                                      .dl = stage_dense_s.state_dl,
                                      .ddl = stage_dense_s.state_ddl});
    }
    accumulated_s += delta_s;
  }

  /// extend
  if (!pts->empty()) {
    double half_car_width = VehicleParam::Instance()->width() / 2.;
    accumulated_s = pts->back().s;
    if (accumulated_s < path_output_s) {
      outside_data->path_context.valid_backup_end_s = path_output_s;
      for (double s = accumulated_s + dense_s; s + kMathEpsilon < path_output_s;
           s += dense_s) {
        ReferencePoint ref_pt{};
        ref_line->GetNearestRefPoint(s, &ref_pt);
        double extend_l = pts->back().l;
        LOG_DEBUG("extend_l:{:.4f}, left_bound:{:.4f}, right_bound:{:.4f}",
                  extend_l, ref_pt.left_bound(), -ref_pt.right_bound());
        if ((extend_l < -ref_pt.right_bound() ||
             extend_l > ref_pt.left_bound()) &&
            outside_data->path_context.valid_backup_end_s > s) {
          LOG_INFO("extend_l is out of ref bound, fix valid_backup_end_s!");
          outside_data->path_context.valid_backup_end_s = s;
        }
        Vec2d tmp_pt{};
        ref_line->GetPointInCartesianFrame({s, extend_l}, &tmp_pt);
        pts->emplace_back(BackupPathPlanner::WayPoint{.x = tmp_pt.x(),
                                                      .y = tmp_pt.y(),
                                                      .z = 0.,
                                                      .s = s,
                                                      .l = extend_l,
                                                      .dl = 0.,
                                                      .ddl = 0.});
      }
    } else {
      outside_data->path_context.valid_backup_end_s = accumulated_s;
    }
  }
  return !pts->empty();
}

bool BackupProcess(TaskInfo& task_info,
                   std::vector<BackupPathPlanner::WayPoint>& path_points,
                   std::vector<PathPoint>& out_path_pts,
                   std::vector<FrenetFramePoint>& out_sl_pts,
                   std::vector<ReferencePoint>& out_ref_pts, std::string name,
                   double gain) {
  auto& frame = task_info.current_frame();
  auto& ref_line = task_info.reference_line();
  auto& veh = frame->inside_planner_data().init_point;
  auto& decision_data = frame->planning_data().decision_data();
  /// Unconstraint optimization and save path
  if (!UnconstraintOptimization(task_info, veh, 0.6, 0.2, &path_points, name,
                                gain)) {
    LOG_ERROR("UnconstraintOptimization failed.");
    frame->mutable_outside_planner_data()->path_fail_tasks += 1;
    return false;
  }
  const auto s0 = path_points[0].s;
  LOG_DEBUG("backup final path [x, y, z, s, l, dl, ddl, theta, kappa]");
  for (auto& [x, y, z, s, l, dl, ddl, theta, kappa] : path_points) {
    FrenetFramePoint sl_pt{};
    ReferencePoint ref_pt{};
    ref_line->GetPointInFrenetFrame({x, y}, &sl_pt);
    ref_line->GetNearestRefPoint({x, y}, &ref_pt);
    theta = SLAnalyticTransformation::calculate_theta(ref_pt.heading(),
                                                      ref_pt.kappa(), l, dl);
    kappa = SLAnalyticTransformation::calculate_kappa(
        ref_pt.kappa(), ref_pt.dkappa(), l, dl, ddl);

    LOG_DEBUG(
        "[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}]",
        x, y, z, s, l, dl, ddl, theta, kappa);
    out_path_pts.emplace_back(Vec2d{x, y}, theta, kappa, 0, 0, s - s0);
    out_sl_pts.push_back(std::move(sl_pt));
    out_ref_pts.push_back(std::move(ref_pt));
  }
  VisFinalPath(out_path_pts, name);
  return true;
};

void SetSpeedLimitForSpeedDown(
    const neodrive::global::planning::SpeedLimit_SourceType& type,
    const neodrive::global::planning::SpeedLimit_ConstraintType&
        constraint_type,
    double upper_bound, double acc) {
  neodrive::global::planning::SpeedLimit new_limit{};
  new_limit.set_source_type(type);
  new_limit.add_upper_bounds(upper_bound);
  new_limit.set_constraint_type(constraint_type);
  new_limit.set_acceleration(acc);
  LOG_INFO("{} {} limit speed: speed = {:.2f}, acc = {:.2f}",
           SpeedLimit_SourceType_Name(new_limit.source_type()),
           SpeedLimit_ConstraintType_Name(new_limit.constraint_type()),
           new_limit.upper_bounds().at(0) * 3.6, new_limit.acceleration());

  DataCenter::Instance()->mutable_behavior_speed_limits()->SetSpeedLimit(
      new_limit);
}

bool CreateVirObsForSpeedDown(TaskInfo& task_info, Vec3d& adc_odom,
                              double dis2obs, double obs_length = 500.0,
                              double obs_width = 500.0) {
  // vis func
  auto vis_obs = [](const DecisionData* decision_data) {
    if (!FLAGS_planning_enable_vis_event) return;
    auto event =
        vis::EventSender::Instance()->GetEvent("virtual_obstacles_path_backup");
    event->set_type(visualizer::Event::k3D);
    event->add_attribute(visualizer::Event::kOdom);
    auto set_pt = [](auto ans, auto& p) {
      ans->set_x(p.x()), ans->set_y(p.y()), ans->set_z(0);
    };

    std::vector<Obstacle*> virtual_obstacle_vector{};
    auto ret = decision_data->get_virtual_obstacle_by_type(
        VirtualObstacle::PATH_BACKUP, virtual_obstacle_vector);

    if (ret != ErrorCode::PLANNING_OK || virtual_obstacle_vector.empty()) {
      LOG_INFO("get virtual obstacle failed.");
      return;
    }
    for (auto obstacle : virtual_obstacle_vector) {
      std::vector<Vec2d> pts{};
      auto polygon = event->add_polygon();
      obstacle->bounding_box().get_all_corners(&pts);
      for (auto& p : pts) set_pt(polygon->add_point(), p);
    }
  };

  // prepare
  Vec2d obs_odom_xy{adc_odom.x() + std::cos(adc_odom.z()) * dis2obs,
                    adc_odom.y() + std::sin(adc_odom.z()) * dis2obs};
  double obs_heading =
      normalize_angle(DataCenter::Instance()->vehicle_state_proxy().Heading());
  LOG_INFO(
      "dis2obs:{:.4f}, adc_odom:{:.4f} {:.4f} {:.4f}, obs_odom:{:.4f} "
      "{:.4f} {:.4f}",
      dis2obs, adc_odom.x(), adc_odom.y(), Red2Deg(adc_odom.z()),
      obs_odom_xy.x(), obs_odom_xy.y(), Red2Deg(obs_heading));

  // process
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data->create_virtual_obstacle(
          obs_odom_xy, obs_length, FLAGS_planning_virtual_obstacle_height,
          obs_width, obs_heading,
          VirtualObstacle::PATH_BACKUP) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create path backup virtual_obstacle.");
    return false;
  }

  // vis
  vis_obs(decision_data);
  return true;
}

bool CheckAdcPose(TaskInfo& task_info, Vec3d& adc_odom, PoseType& pose_type,
                  char* backup_monitor_str) {
  const ReferencePoint& ref_p0 = task_info.curr_referline_pt();
  const auto& init_point =
      task_info.current_frame()->inside_planner_data().init_point;
  const auto& frenet_init_point =
      task_info.current_frame()->outside_planner_data().frenet_init_point;
  const auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  double s0 = frenet_init_point.s(), l0 = frenet_init_point.l();
  double adc_velocity =
      DataCenter::Instance()->vehicle_state_proxy().LinearVelocity();

  // edge sl
  auto find_edge_sl = [&](bool is_front, SLPoint& edge_sl) {
    double edge_to_planning_center = ego_car_config.front_edge_to_center;
    if (is_front) {
      edge_to_planning_center = ego_car_config.front_edge_to_center;
    } else {
      edge_to_planning_center = ego_car_config.back_edge_to_center;
    }
    double delta_x = edge_to_planning_center * std::cos(init_point.theta()),
           delta_y = edge_to_planning_center * std::sin(init_point.theta());
    Vec2d edge_xy1{init_point.x() + delta_x, init_point.y() + delta_y};
    Vec2d edge_xy2{init_point.x() - delta_x, init_point.y() - delta_y};
    SLPoint edge_sl1, edge_sl2;
    task_info.reference_line()->GetPointInFrenetFrameWithLastS(
        edge_xy1, frenet_init_point.s(), &edge_sl1);
    task_info.reference_line()->GetPointInFrenetFrameWithLastS(
        edge_xy2, frenet_init_point.s(), &edge_sl2);
    if (is_front) {
      edge_sl = (edge_sl1.s() >= edge_sl2.s()) ? edge_sl1 : edge_sl2;
    } else {
      edge_sl = (edge_sl1.s() >= edge_sl2.s()) ? edge_sl2 : edge_sl1;
    }
    LOG_DEBUG("edge_s1:{:.4f}, edge_s2:{:.4f}, edge_s:{:.4f}", edge_sl1.s(),
              edge_sl2.s(), edge_sl.s());
  };
  // front & back edge
  SLPoint front_edge_sl, back_edge_sl;
  find_edge_sl(true, front_edge_sl);
  find_edge_sl(false, back_edge_sl);
  bool adc_head_out = std::abs(front_edge_sl.l()) >= std::abs(back_edge_sl.l());
  LOG_INFO(
      "front_edge_sl:({:.4f},{:.4f}), back_edge_sl:({:.4f},{:.4f}), "
      "init_point_s:({:.4f},{:.4f}), adc_head_out:{}",
      front_edge_sl.s(), front_edge_sl.l(), back_edge_sl.s(), back_edge_sl.l(),
      frenet_init_point.s(), frenet_init_point.l(), adc_head_out);

  pose_type = PoseType::REASONABLE;
  // 1.location check
  bool is_inroad =
      IsAllowedExtendToReverseLane(ref_p0)
          ? l0 > -ref_p0.right_road_bound() &&
                l0 < ref_p0.left_reverse_road_bound()
          : l0 > -ref_p0.right_road_bound() && l0 < ref_p0.left_road_bound();
  bool is_inlane =
      l0 > -ref_p0.right_lane_bound() && l0 < ref_p0.left_lane_bound();
  if (!is_inroad) pose_type = PoseType::UNREASONABLE_LOCATION;
  LOG_INFO(
      "location check: left_reverse_road/road/lane_bound:{:.4f}/{:.4f}/{:.4f}, "
      "right_road/lane_bound:{:.4f}/{:.4f}, adc_l:{:.4f}, is_inroad:{}, "
      "is_inlane:{}",
      ref_p0.left_reverse_road_bound(), ref_p0.left_road_bound(),
      ref_p0.left_lane_bound(), -ref_p0.right_road_bound(),
      -ref_p0.right_lane_bound(), l0, is_inroad, is_inlane);

  // 2.orientation check
  double heading_error = ref_p0.heading() - init_point.theta(),
         normalized_heading_error = normalize_angle(heading_error),
         abs_normalized_heading_error = std::abs(normalized_heading_error);
  bool is_indoor = task_info.current_frame()->inside_planner_data().is_indoor;
  double orient_thresh;
  if (is_inlane && !adc_head_out) {
    orient_thresh = is_indoor ? Deg2Red(kOrientThreshInLaneIndoor)
                              : Deg2Red(kOrientThreshInLane);
    if (abs_normalized_heading_error > orient_thresh) {
      pose_type = PoseType::UNREASONABLE_ORIENTATION;
    }
  } else {
    orient_thresh = is_indoor ? Deg2Red(kOrientThreshOutLaneIndoor)
                              : Deg2Red(kOrientThreshOutLane);
    if (abs_normalized_heading_error > orient_thresh) {
      pose_type = (pose_type == PoseType::UNREASONABLE_LOCATION)
                      ? PoseType::UNREASONABLE_BOTH
                      : PoseType::UNREASONABLE_ORIENTATION;
    }
  }
  LOG_INFO(
      "orientation check: ref_heading:{:.4f}, adc_heading:{:.4f}, "
      "heading_error:{:.4f} {:.4f} {:.4f}, is_indoor:{}, orient_thresh:{:.4f}",
      Red2Deg(ref_p0.heading()), Red2Deg(init_point.theta()),
      Red2Deg(heading_error), Red2Deg(normalized_heading_error),
      Red2Deg(abs_normalized_heading_error), is_indoor, Red2Deg(orient_thresh));

  // discussion
  char adc_pose_check_str[256];
  sprintf(adc_pose_check_str, "[PoseType: %s]",
          posetype2string[pose_type].c_str());
  std::strcat(backup_monitor_str, adc_pose_check_str);
  LOG_INFO("pose type: {}, adc_velocity:{:.4f}, velocity_threshold:{:.4f}",
           posetype2string[pose_type], adc_velocity, kLowSpeedThresh);
  switch (pose_type) {
    case PoseType::REASONABLE: {
      return true;
    } break;
    case PoseType::UNREASONABLE_BOTH: {
      CreateVirObsForSpeedDown(task_info, adc_odom, 0.0);
      SetSpeedLimitForSpeedDown(SpeedLimitType::PATH_BACKUP,
                                SpeedLimitType::HARD, kLowSpeedThresh, 0.0);
      return adc_velocity >= kLowSpeedThresh;
    } break;
    case PoseType::UNREASONABLE_ORIENTATION: {
      CreateVirObsForSpeedDown(task_info, adc_odom, 0.0);
      SetSpeedLimitForSpeedDown(SpeedLimitType::PATH_BACKUP,
                                SpeedLimitType::HARD, kLowSpeedThresh, 0.0);
      return adc_velocity >= kLowSpeedThresh;
    } break;
    case PoseType::UNREASONABLE_LOCATION: {
      double dis2obs = 3.5 + VehicleParam::Instance()->front_edge_to_center();
      CreateVirObsForSpeedDown(task_info, adc_odom, dis2obs, 1.5, 100.0);
      SetSpeedLimitForSpeedDown(SpeedLimitType::PATH_BACKUP,
                                SpeedLimitType::SOFT, kLowSpeedThresh, 100.0);
      return adc_velocity >= kLowSpeedThresh;
    } break;
    default:
      return true;
      break;
  }
}

CollisionType FinalCollisionCheck(TaskInfo& task_info,
                                  PathData* const path_data,
                                  std::vector<Box2d>& boxes,
                                  double& forward_roi_check_dis) {
  if (path_data == nullptr) {
    LOG_ERROR("input data invalid");
    return CollisionType::PATH_ERROR;
  }
  if (path_data->path().path_points().size() < 2) {
    LOG_ERROR("input path points is empty");
    return CollisionType::PATH_ERROR;
  }

  // prepare
  const auto& reference_line = task_info.reference_line();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  const auto& outside_data = task_info.current_frame()->outside_planner_data();
  const auto& path_pts = path_data->path().path_points();
  const auto& curb_lines = DataCenter::Instance()
                               ->environment()
                               .perception_lanes_proxy()
                               .camera_curb_lines();
  const auto& road_obs_path_boundries = outside_data.road_obs_path_boundries;
  const auto& obs_decision =
      outside_data.path_obstacle_context.obstacle_decision;
  bool is_indoor = task_info.current_frame()->inside_planner_data().is_indoor;
  bool is_motorway =
      (common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .enable_motorway &&
       (task_info.curr_referline_pt().lane_type_is_city_driving() &&
        !task_info.curr_referline_pt().lane_type_is_biking()));

  // process
  LOG_INFO("forward_roi_check_dis = {:.4f}", forward_roi_check_dis);
  for (const auto& pt : path_data->path().path_points()) {
    // ignore forward_roi_check_dis
    if (pt.s() - path_data->path().path_points().front().s() >
        forward_roi_check_dis) {
      break;
    }
    // generate box
    Box2d box = VehicleParam::Instance()->get_adc_bounding_box(
        {pt.x(), pt.y()}, pt.theta(), 0., 0., 0.);
    boxes.emplace_back(box);

    // 1. vis check
    if (is_indoor) {
      for (const auto& curb_line : curb_lines) {
        for (const auto& curb_segment : curb_line) {
          if (box.has_overlap(curb_segment)) {
            LOG_INFO("path box has overlap with curb at s:{:.4f}", pt.s());
            forward_roi_check_dis = pt.s();
            return CollisionType::VIS_CURB_COLLISION;
          }
        }
      }
    }

    // 2. road check
    if (is_indoor) {
      for (std::size_t i = 0; i < road_obs_path_boundries.size(); ++i) {
        double s = road_obs_path_boundries.at(i).s,
               lr = road_obs_path_boundries.at(i).left_road_bound,
               rr = road_obs_path_boundries.at(i).right_road_bound;
        Vec2d l_xy, r_xy;
        if (!reference_line->GetPointInCartesianFrame(SLPoint{s, lr}, &l_xy) ||
            !reference_line->GetPointInCartesianFrame(SLPoint{s, rr}, &r_xy)) {
          LOG_INFO("fail to get cartesian point!");
          forward_roi_check_dis = pt.s();
          return CollisionType::MAP_ROAD_COLIISION;
        }
        if (box.is_point_in(l_xy) || box.is_point_in(r_xy)) {
          LOG_INFO("path box has overlap with road at s:{:.4f}", pt.s());
          forward_roi_check_dis = pt.s();
          return CollisionType::MAP_ROAD_COLIISION;
        }
      }
    }

    // 3.obs check
    if (is_indoor) {
      for (std::size_t i = 0; i < obs_decision.size(); ++i) {
        if (!obs_decision.at(i).obstacle_boundary.obstacle.is_static())
          continue;
        if (box.has_overlap(obs_decision.at(i).obstacle_box)) {
          LOG_INFO("path box has overlap with obs at s:{:.4f}", pt.s());
          forward_roi_check_dis = pt.s();
          return CollisionType::OBS_COLIISION;
        }
      }
    }
  }

  LOG_INFO("path box has not overlap");
  return CollisionType::NO_COLLISION;
};

}  // namespace

BackupPathOptimizer::BackupPathOptimizer() { name_ = "BackupPathOptimizer"; }

BackupPathOptimizer::~BackupPathOptimizer() { Reset(); }

ErrorCode BackupPathOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  std::memset(backup_monitor_str_, 0, sizeof(backup_monitor_str_));
  std::strcpy(backup_monitor_str_, "[PATH_BACKUP]");
  if (frame->outside_planner_data().path_succeed_tasks > 0) {
    LOG_INFO(
        "main path planner succeed, should not execute path backup planner.");
    frame->mutable_outside_planner_data()->path_planner_type =
        PathPlannerType::MAIN;
    adc_odom_start_backup_ = {0., 0., 0.};
    return ErrorCode::PLANNING_OK;
  }

  // process
  PlannerPrepare(task_info);
  for (const auto& pair : backup_funcs_) {
    const auto& funcs_name = pair.first;
    const auto& funcs = pair.second;
    bool backup_planner_succeed = true;
    for (int i = 0; i < funcs.size(); ++i) {
      const auto& func = funcs[i];
      if (!func()) {
        backup_planner_succeed = false;
        LOG_INFO("{} backup planner failed in func {}!", funcs_name, i);
        break;
      } else {
        LOG_INFO("{} backup planner succeed in func {}!", funcs_name, i);
      }
    }
    if (backup_planner_succeed) {
      LOG_INFO("{} backup planner succeed!", funcs_name);
      break;
    }
  }

  // backup output empty check
  if (way_pts_.empty() || out_path_pts_.empty() || out_sl_pts_.empty() ||
      out_ref_pts_.empty()) {
    frame->mutable_outside_planner_data()->path_planner_type =
        PathPlannerType::ALL_FAIL;
    frame->mutable_outside_planner_data()->path_fail_tasks += 1;
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // path empty check
  auto path_data = frame->mutable_outside_planner_data()->path_data;
  if (!path_data) {
    LOG_INFO("path_data is nullptr");
    frame->mutable_outside_planner_data()->path_planner_type =
        PathPlannerType::ALL_FAIL;
    frame->mutable_outside_planner_data()->path_fail_tasks += 1;
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // set path
  path_data->set_path(out_path_pts_);
  path_data->set_frenet_path(FrenetFramePath(out_sl_pts_));
  *path_data->mutable_reference_points() = out_ref_pts_;
  path_data->set_valid_length(way_pts_.back().s - way_pts_.front().s);

  // reset
  pose_type_ = PoseType::REASONABLE;
  collision_type_ = CollisionType::NO_COLLISION;

  // adc pose check
  if (task_info.last_frame() == nullptr ||
      task_info.last_frame()->outside_planner_data().path_planner_type !=
          PathPlannerType::BACKUP) {
    adc_odom_start_backup_ = {
        DataCenter::Instance()->vehicle_state_proxy().X(),
        DataCenter::Instance()->vehicle_state_proxy().Y(),
        DataCenter::Instance()->vehicle_state_proxy().Heading()};
  }
  if (!CheckAdcPose(task_info, adc_odom_start_backup_, pose_type_,
                    backup_monitor_str_)) {
    LOG_INFO("adc pose is not reasonable, can not use backup path!");
    frame->mutable_outside_planner_data()->path_planner_type =
        PathPlannerType::ALL_FAIL;
    frame->mutable_outside_planner_data()->path_fail_tasks += 1;
    DataCenter::Instance()->SetMonitorString(backup_monitor_str_,
                                             MonitorItemSource::PATH_BACKUP);
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // collision check
  double
      buffer_dis = 2.0,
      front_edge_to_center = VehicleParam::Instance()->front_edge_to_center(),
      max_dec =
          config::PlanningConfig::Instance()->plan_config().common.max_av_minus,
      vel_v = task_info.current_frame()->inside_planner_data().vel_v;
  double forward_roi_check_dis =
      front_edge_to_center + vel_v * vel_v / max_dec / 2.0 + buffer_dis;
  std::vector<Box2d> boxes{};
  collision_type_ =
      FinalCollisionCheck(task_info, path_data, boxes, forward_roi_check_dis);
  VisPathBoxes(boxes);
  char collision_check_str[256];
  sprintf(collision_check_str, "[CollisionType: %s]",
          collisiontype2string[collision_type_].c_str());
  std::strcat(backup_monitor_str_, collision_check_str);
  DataCenter::Instance()->SetMonitorString(backup_monitor_str_,
                                           MonitorItemSource::PATH_BACKUP);

  if (collision_type_ != CollisionType::NO_COLLISION) {
    LOG_INFO("backup bath collision:{}", collisiontype2string[collision_type_]);
    double width = 5.0,
           dis2obs = width / 2 + forward_roi_check_dis + front_edge_to_center;
    CreateVirObsForSpeedDown(task_info, adc_odom_start_backup_, dis2obs, width,
                             50.0);
    SetSpeedLimitForSpeedDown(SpeedLimitType::PATH_BACKUP, SpeedLimitType::HARD,
                              kLowSpeedThresh, 0.0);
    if (DataCenter::Instance()->vehicle_state_proxy().LinearVelocity() <
        kLowSpeedThresh) {
      frame->mutable_outside_planner_data()->path_fail_tasks += 1;
      frame->mutable_outside_planner_data()->path_planner_type =
          PathPlannerType::ALL_FAIL;
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }

  // success
  frame->mutable_outside_planner_data()->path_planner_type =
      PathPlannerType::BACKUP;
  frame->mutable_outside_planner_data()->path_succeed_tasks += 1;
  return ErrorCode::PLANNING_OK;
}

void BackupPathOptimizer::PlannerPrepare(TaskInfo& task_info) {
  // kQuinticBvp
  std::vector<std::function<bool()>> quintic_bvp_funcs;
  quintic_bvp_funcs.emplace_back([&]() {
    return BackupPathPlannerFactory{}[BackupPathPlannerType::kQuinticBvp]
        ->GeneratePath(task_info, &way_pts_, gain_);
  });
  quintic_bvp_funcs.emplace_back([&]() {
    return BackupProcess(task_info, way_pts_, out_path_pts_, out_sl_pts_,
                         out_ref_pts_, "final quintic_bvp path", gain_);
  });
  quintic_bvp_funcs.emplace_back([&]() { return IsPathValid(way_pts_); });

  // kRefLine
  std::vector<std::function<bool()>> ref_line_funcs;
  ref_line_funcs.emplace_back([&]() {
    return BackupPathPlannerFactory{}[BackupPathPlannerType::kRefLine]
        ->GeneratePath(task_info, &way_pts_, gain_);
  });
  ref_line_funcs.emplace_back([&]() {
    return BackupProcess(task_info, way_pts_, out_path_pts_, out_sl_pts_,
                         out_ref_pts_, "final ref_line path", gain_);
  });
  ref_line_funcs.emplace_back([&]() { return IsPathValid(way_pts_); });

  // use rule
  backup_funcs_ = {{"quintic_bvp", quintic_bvp_funcs},
                   {"ref_line", ref_line_funcs}};

  // clear
  PathDataReset();
}

void BackupPathOptimizer::PathDataReset() {
  gain_ = 1.0;
  way_pts_.clear();
  out_path_pts_.clear();
  out_sl_pts_.clear();
  out_ref_pts_.clear();
}

}  // namespace planning
}  // namespace neodrive
