#include "minco_path_generator.h"

#include "common_config/config/common_config.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/visualizer_event/visualizer_event.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/occupy_map.h"
#include "src/planning/math/corridor/corridor_builder_2d.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/math/frame_conversion/sl_analytic_transformation.h"

namespace neodrive {
namespace planning {

namespace {

int kMinPieceNums = 2;
double kMinSpeed = 2.0;
double kSampleS = 0.2;
double kReparamSampleS = 2.0;
int kPieceCutNums = 2;

struct VehicleState {
  Eigen::Matrix<double, 2, 1> vec_position{};
  double theta{0.0};
  double curvature{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

void VisInitTraj(const minco::Trajectory& init_traj, const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt;
  double total_time = init_traj.getTotalDuration();
  for (double t = 0.; t < total_time; t += 0.1) {
    auto pos = init_traj.getPos(t);
    auto sphere = event->mutable_sphere()->Add();
    pt.set_x(pos[0]), pt.set_y(pos[1]);
    set_pt(sphere->mutable_center(), pt);
    sphere->set_radius(0.05);
  }
}

void VisInnerPts(const Eigen::MatrixXd& inner_pts, const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt;
  for (int i = 0; i < inner_pts.cols(); ++i) {
    auto sphere = event->mutable_sphere()->Add();
    pt.set_x(inner_pts(0, i)), pt.set_y(inner_pts(1, i));
    set_pt(sphere->mutable_center(), pt);
    sphere->set_radius(0.05);
  }
}

void VisStateList(const std::vector<Eigen::Vector3d>& state_list,
                  const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt;
  for (auto& state : state_list) {
    auto sphere = event->mutable_sphere()->Add();
    pt.set_x(state(0)), pt.set_y(state(1));
    set_pt(sphere->mutable_center(), pt);
    sphere->set_radius(0.05);
  }
}

void VisCorridors2d(const std::vector<Eigen::MatrixXd>& h_polys,
                    const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  Vec2d pt;
  int j = 0;
  for (const auto& h_poly : h_polys) {
    if (j % kPieceCutNums == 0) {
      auto polygon = event->mutable_polygon()->Add();
      for (int i = 0; i < h_poly.cols(); ++i) {
        pt.set_x(h_poly.col(i).tail<2>()(0));
        pt.set_y(h_poly.col(i).tail<2>()(1));
        set_pt(polygon->add_point(), pt);
      }
    }
    ++j;
  }
}

void VisMincoPath(const minco::Trajectory& minco_path,
                  const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  double total_time = minco_path.getTotalDuration();
  LOG_INFO("minco_path total_time: {:.3f}", minco_path.getTotalDuration());

  Vec2d pt;
  for (double t = 0.0; t < total_time; t += 0.05) {
    auto pose = minco_path.getPos(t);
    auto sphere = event->mutable_sphere()->Add();
    pt.set_x(pose[0]), pt.set_y(pose[1]);
    set_pt(sphere->mutable_center(), pt);
    sphere->set_radius(0.05);
  }
}

Eigen::MatrixXd State2FlatOutput(const VehicleState& state) {
  Eigen::MatrixXd flat_output(2, 3);

  double vel = state.velocity, theta = state.theta;
  double acc = state.acceleration;

  Eigen::Matrix2d init_R;
  init_R << cos(theta), -sin(theta), sin(theta), cos(theta);

  if (vel == 0.) {
    vel = 1e-5;
  }

  flat_output << state.vec_position, init_R * Eigen::Vector2d(vel, 0.0),
      init_R * Eigen::Vector2d(acc, state.curvature * std::pow(vel, 2));

  return flat_output;
}

bool SampleInnerPts(const std::vector<Vec2d>& xy_pts,
                    const ReferenceLinePtr& reference_line, const double init_s,
                    const Eigen::MatrixXd& ini_state, int& piece_nums,
                    double& max_s, Eigen::MatrixXd& inner_pts,
                    Eigen::VectorXd& inner_pts_time) {
  auto pt_size = xy_pts.size();
  std::vector<double> x_vec, y_vec;
  x_vec.reserve(pt_size), y_vec.reserve(pt_size);
  for (std::size_t i = 0; i < pt_size; ++i) {
    SLPoint sl_pt{};
    if (!reference_line->GetPointInFrenetFrame(xy_pts[i], &sl_pt) ||
        sl_pt.s() < init_s) {
      continue;
    }
    x_vec.push_back(xy_pts[i].x()), y_vec.push_back(xy_pts[i].y());
  }
  if (x_vec.size() < 4 || y_vec.size() < 4) return false;

  std::vector<double> accmulated_s_vec(x_vec.size(), 0.);
  for (std::size_t i = 1; i < x_vec.size(); ++i) {
    accmulated_s_vec[i] +=
        accmulated_s_vec[i - 1] +
        std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
  }
  tk::spline x_spline, y_spline;
  x_spline.set_points(accmulated_s_vec, x_vec, tk::spline::spline_type::linear);
  y_spline.set_points(accmulated_s_vec, y_vec, tk::spline::spline_type::linear);

  max_s = accmulated_s_vec.back();
  piece_nums =
      std::max(kMinPieceNums,
               static_cast<int>((max_s - kMinPieceNums * kSampleS) / kSampleS));
  LOG_INFO("max_s, piece_nums: {:.3f}, {}", max_s, piece_nums);
  inner_pts = Eigen::MatrixXd::Zero(2, piece_nums - 1);
  inner_pts_time.resize(piece_nums);
  inner_pts_time.setConstant(max_s / piece_nums /
                             std::hypot(ini_state(0, 1), ini_state(1, 1)));

  for (int i = 0; i + 1 < piece_nums; ++i) {
    inner_pts(0, i) = x_spline((i + 1) * max_s / piece_nums);
    inner_pts(1, i) = y_spline((i + 1) * max_s / piece_nums);
    LOG_DEBUG("s: {:.3f}, origin_inner_pts[{}]: {:3f}, {:.3f}, time: {:.3f}",
              (i + 1) * max_s / piece_nums, i, inner_pts(0, i), inner_pts(1, i),
              inner_pts_time[i + 1]);
  }

  return true;
}

void ReparamalizeTraj(const Eigen::MatrixXd& ini_state,
                      const Eigen::MatrixXd& fini_state, int& piece_nums,
                      double& max_s, Eigen::MatrixXd& inner_pts,
                      Eigen::VectorXd& inner_pts_time, double& total_time,
                      std::vector<Eigen::Vector3d>& state_list,
                      minco::Trajectory& init_traj) {
  minco::MinJerkOpt init_min_jerk_opt;
  init_min_jerk_opt.reset(ini_state, fini_state, piece_nums);
  init_min_jerk_opt.generate(inner_pts, inner_pts_time);
  init_traj = init_min_jerk_opt.getTraj(1);

  /// re-paramalize trajectory
  total_time = init_traj.getTotalDuration();
  piece_nums =
      std::max(kMinPieceNums, static_cast<int>(max_s / kReparamSampleS) + 1);
  LOG_INFO("total_time: {:.3f}, re-paramalize piece_nums: {}", total_time,
           piece_nums);

  double interval_time = total_time / piece_nums;
  inner_pts.resize(2, piece_nums - 1);
  inner_pts_time.resize(piece_nums);
  inner_pts_time.setConstant(interval_time);

  double curr_t{0.};
  for (int i = 0; i < piece_nums; i++) {
    for (int j = 0; j < kPieceCutNums; j++) {
      if (i == 0 && j == 0) continue;
      double t = curr_t + 1.0 * j / kPieceCutNums * interval_time;
      auto pos = init_traj.getPos(t);
      state_list.emplace_back(pos[0], pos[1], init_traj.getAngle(t));
      LOG_DEBUG("state_list[{}]: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
                state_list.size() - 1, state_list.back()(0),
                state_list.back()(1), state_list.back()(2), t);
      if (j + 1 == kPieceCutNums && i + 1 < piece_nums) {
        t = curr_t + 1.0 * (j + 1) / kPieceCutNums * interval_time;
        pos = init_traj.getPos(t);
        inner_pts.col(i) = pos;
        LOG_DEBUG("reparamlize_inner_pts[{}]: {:.3f}, {:.3f}, {:.3f}", i,
                  inner_pts(0, i), inner_pts(1, i), t);
      }
    }
    curr_t += interval_time;
  }

  LOG_INFO("state_list size: {}", state_list.size());
}

void ObsData(const PathBoundary& path_boundaries,
             const PathBoundary& shrink_path_boundaries,
             std::vector<math::Node<math::Point>>& obs_data,
             std::vector<std::array<double, 2>>& origin_freespace_data,
             std::vector<math::Node<math::Point>>& shrink_obs_data,
             std::vector<std::array<double, 2>>& shrink_freespace_data) {
  obs_data.clear(), origin_freespace_data.clear();
  const auto& left_pts = path_boundaries.left_xy_boundary;
  const auto& right_pts = path_boundaries.right_xy_boundary;
  if (left_pts.empty() || right_pts.empty() ||
      (left_pts.size() != right_pts.size())) {
    return;
  }
  int id = 0;
  for (const auto& xy : left_pts) {
    obs_data.emplace_back(math::Node<math::Point>{
        .id = id++, .shape = math::Point(xy.x(), xy.y())});
  }
  for (const auto& xy : right_pts) {
    obs_data.emplace_back(math::Node<math::Point>{
        .id = id++, .shape = math::Point(xy.x(), xy.y())});
  }
  origin_freespace_data.resize(right_pts.size() + left_pts.size());
  std::size_t j = 0;
  for (int i = 0; i < left_pts.size(); ++i) {
    origin_freespace_data[j++] = {left_pts[i].x(), left_pts[i].y()};
  }
  for (int i = right_pts.size() - 1; i >= 0; --i) {
    origin_freespace_data[j++] = {right_pts[i].x(), right_pts[i].y()};
  }

  shrink_obs_data.clear(), shrink_freespace_data.clear();
  const auto& shrink_left_pts = shrink_path_boundaries.left_xy_boundary;
  const auto& shrink_right_pts = shrink_path_boundaries.right_xy_boundary;
  if (shrink_left_pts.empty() || shrink_right_pts.empty() ||
      (shrink_left_pts.size() != shrink_right_pts.size())) {
    return;
  }
  id = 0;
  for (const auto& xy : shrink_left_pts) {
    shrink_obs_data.emplace_back(math::Node<math::Point>{
        .id = id++, .shape = math::Point(xy.x(), xy.y())});
  }
  for (const auto& xy : shrink_right_pts) {
    shrink_obs_data.emplace_back(math::Node<math::Point>{
        .id = id++, .shape = math::Point(xy.x(), xy.y())});
  }
  shrink_freespace_data.resize(shrink_right_pts.size() +
                               shrink_left_pts.size());
  j = 0;
  for (int i = 0; i < shrink_left_pts.size(); ++i) {
    shrink_freespace_data[j++] = {shrink_left_pts[i].x(),
                                  shrink_left_pts[i].y()};
  }
  for (int i = shrink_right_pts.size() - 1; i >= 0; --i) {
    shrink_freespace_data[j++] = {shrink_right_pts[i].x(),
                                  shrink_right_pts[i].y()};
  }
}

void AdjustStateList(const std::vector<PieceBoundary>& solve_boundaries,
                     ReferenceLinePtr ref_line,
                     std::vector<Eigen::Vector3d>& state_list) {
  std::vector<double> s_vec(solve_boundaries.size());
  std::vector<double> l_l_vec(solve_boundaries.size());
  std::vector<double> r_l_vec(solve_boundaries.size());
  for (std::size_t i = 0; i < solve_boundaries.size(); ++i) {
    s_vec[i] = solve_boundaries[i].s;
    l_l_vec[i] = solve_boundaries[i].left_bound;
    r_l_vec[i] = solve_boundaries[i].right_bound;
  }
  tk::spline upper_sl, lower_sl;
  upper_sl.set_points(s_vec, l_l_vec, tk::spline::spline_type::linear);
  lower_sl.set_points(s_vec, r_l_vec, tk::spline::spline_type::linear);

  for (auto& state : state_list) {
    SLPoint sl_pt{};
    if (!ref_line->GetPointInFrenetFrame({state[0], state[1]}, &sl_pt)) {
      continue;
    }
    double upper_l = upper_sl(sl_pt.s()), lower_l = lower_sl(sl_pt.s());
    if (sl_pt.l() + 0.1 >= upper_l)
      sl_pt.set_l(std::max(upper_l - 0.2, (upper_l + lower_l) / 2.0));
    else if (sl_pt.l() <= lower_l + 0.1)
      sl_pt.set_l(std::min(lower_l + 0.2, (upper_l + lower_l) / 2.0));

    Vec2d xy_pt{};
    if (!ref_line->GetPointInCartesianFrame(sl_pt, &xy_pt)) {
      continue;
    }
    state[0] = xy_pt.x(), state[1] = xy_pt.y();
  }
}

}  // namespace

ErrorCode MincoPathGenerator::Optimize(const ReferenceLinePtr& reference_line,
                                       const InsidePlannerData& inside_data,
                                       OutsidePlannerData* const outside_data) {
  if (config::PlanningConfig::Instance()
          ->planning_research_config()
          .only_for_backup_path_planner) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Init(reference_line, inside_data, outside_data)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!RunMinco(reference_line, inside_data, outside_data)) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

bool MincoPathGenerator::Init(const ReferenceLinePtr& reference_line,
                              const InsidePlannerData& inside_data,
                              OutsidePlannerData* const outside_data) {
  if (outside_data == nullptr || outside_data->path_data == nullptr ||
      outside_data->speed_data == nullptr ||
      outside_data->road_obs_path_shrink_boundries.size() < 4 ||
      outside_data->path_context.path_goal_points.goal_xy_points.size() < 4 ||
      outside_data->road_obs_path_shrink_boundries.size() < 4) {
    LOG_ERROR("input data invalid nullptr");
    return false;
  }

  /// path output length
  const auto& frenet_init_point = outside_data->frenet_init_point;
  const auto& path_boundary =
      outside_data->path_context.original_path_boundary.path_boundary;
  double path_output_length =
      fmin(reference_line->ref_points().back().s() - 1,
           frenet_init_point.s() +
               fmax(DataCenter::Instance()->drive_strategy_max_speed() *
                        FLAGS_planning_trajectory_time_length,
                    FLAGS_planning_trajectory_min_length));
  if (frenet_init_point.s() > path_output_length) {
    LOG_ERROR("frenet_init_point s > referline_end_s: {:.3f}, {:.3f}",
              frenet_init_point.s(), reference_line->ref_points().back().s());
    return false;
  }

  /// ini_state and fini_state
  const auto& back_sl =
      outside_data->path_context.path_goal_points.goal_sl_points.back();
  ReferencePoint back_ref_pt;
  if (!reference_line->GetNearestRefPoint(back_sl.s(), &back_ref_pt)) {
    return false;
  }
  const auto& back_xy =
      outside_data->path_context.path_goal_points.goal_xy_points.back();

  VehicleState ini_veh_state{
      .vec_position = Eigen::Matrix<double, 2, 1>(inside_data.init_point.x(),
                                                  inside_data.init_point.y()),
      .theta = inside_data.init_point.theta(),
      .curvature = inside_data.init_point.kappa(),
      .velocity = std::max(kMinSpeed, inside_data.init_point.velocity()),
      .acceleration = std::max(0.0, inside_data.init_point.acceleration())};
  ini_state_ = State2FlatOutput(ini_veh_state);
  VehicleState fini_veh_state{
      .vec_position = Eigen::Matrix<double, 2, 1>(back_xy.x(), back_xy.y()),
      .theta = back_ref_pt.heading(),
      .curvature = back_ref_pt.kappa(),
      .velocity = std::max(kMinSpeed, inside_data.init_point.velocity()),
      .acceleration = 0.0};
  fini_state_ = State2FlatOutput(fini_veh_state);

  LOG_DEBUG("init_pt: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
            inside_data.init_point.x(), inside_data.init_point.y(),
            inside_data.init_point.theta(), inside_data.init_point.kappa(),
            std::max(inside_data.init_point.velocity(), kMinSpeed),
            inside_data.init_point.acceleration());
  LOG_DEBUG("fini_pt: {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}",
            back_xy.x(), back_xy.y(), back_ref_pt.heading(),
            back_ref_pt.kappa(),
            std::max(inside_data.init_point.velocity(), kMinSpeed),
            inside_data.init_point.acceleration());
  LOG_INFO("ini_state_x: {:.3f}, {:.3f}, {:.3f}", ini_state_(0, 0),
           ini_state_(0, 1), ini_state_(0, 2));
  LOG_INFO("ini_state_y: {:.3f}, {:.3f}, {:.3f}", ini_state_(1, 0),
           ini_state_(1, 1), ini_state_(1, 2));
  LOG_INFO("fin_state_x: {:.3f}, {:.3f}, {:.3f}", fini_state_(0, 0),
           fini_state_(0, 1), fini_state_(0, 2));
  LOG_INFO("fin_state_y: {:.3f}, {:.3f}, {:.3f}", fini_state_(1, 0),
           fini_state_(1, 1), fini_state_(1, 2));

  const auto& config = config::PlanningConfig::Instance()
                           ->planning_research_config()
                           .minco_path_optimizer_config;
  kMinPieceNums = std::fmax(2, config.min_piece_nums);
  kMinSpeed = std::fmax(0.0, config.min_speed);
  kSampleS = std::fmax(0.1, config.sample_s);
  kReparamSampleS = std::fmax(0.1, config.reparam_sample_s);
  kPieceCutNums = std::fmax(2, config.piece_cut_nums);

  return true;
}

bool MincoPathGenerator::RunMinco(const ReferenceLinePtr& reference_line,
                                  const InsidePlannerData& inside_data,
                                  OutsidePlannerData* const outside_data) {
  LOG_INFO(">>> prepare");
  /// inner_pts
  int piece_nums{0};
  double max_s{0.};
  Eigen::MatrixXd inner_pts;
  Eigen::VectorXd inner_pts_time;
  if (!SampleInnerPts(
          outside_data->path_context.path_goal_points.goal_xy_points,
          reference_line, inside_data.init_sl_point.s(), ini_state_, piece_nums,
          max_s, inner_pts, inner_pts_time)) {
    LOG_ERROR("SampleInnerPts failed.");
    return false;
  }
  VisInnerPts(inner_pts, "origin_inner_pts");

  /// re-paramalize trajectory
  double total_time{0.};
  std::vector<Eigen::Vector3d> state_list;
  minco::Trajectory init_traj;
  ReparamalizeTraj(ini_state_, fini_state_, piece_nums, max_s, inner_pts,
                   inner_pts_time, total_time, state_list, init_traj);
  Eigen::VectorXd duration_time;
  duration_time.resize(1);
  duration_time(0) = total_time;

  VisInitTraj(init_traj, "init_traj");
  VisInnerPts(inner_pts, "reparamlize_inner_pts");
  VisStateList(state_list, "state_list");

  /// obs data
  std::vector<math::Node<math::Point>> obs_data{};
  std::vector<std::array<double, 2>> origin_freespace_data{};
  std::vector<math::Node<math::Point>> shrink_obs_data{};
  std::vector<std::array<double, 2>> shrink_freespace_data{};
  ObsData(outside_data->path_context.original_path_boundary,
          outside_data->path_context.shrink_path_boundary, obs_data,
          origin_freespace_data, shrink_obs_data, shrink_freespace_data);
  if (obs_data.empty() || origin_freespace_data.empty() ||
      shrink_obs_data.empty() || shrink_freespace_data.empty()) {
    LOG_ERROR("obs_data for corridors invalid.");
    return false;
  }

  double grid_step = config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .minco_path_optimizer_config.grid_step;
  OccupyMap origin_om{{.grid_step = grid_step}, origin_freespace_data, {}};
  OccupyMap shrink_om{{.grid_step = grid_step}, shrink_freespace_data, {}};

  // adjust state_list with road_obs_boundaries
  std::vector<Eigen::Vector3d> adjust_state_list = state_list;
  AdjustStateList(outside_data->road_obs_path_shrink_boundries, reference_line,
                  adjust_state_list);

  VisStateList(adjust_state_list, "adjust_state_list");

  /// generate corridors 2d
  LOG_INFO(">>> corridors 2d");
  std::vector<Eigen::MatrixXd> origin_h_polys{};
  corridor::CorridorBuilder2d(adjust_state_list, obs_data, 5.0, 5.0,
                              origin_h_polys);
  VisCorridors2d(origin_h_polys, "origin_corridors_2d");
  std::vector<Eigen::MatrixXd> shrink_h_polys{};
  corridor::CorridorBuilder2d(adjust_state_list, shrink_obs_data, 5.0, 5.0,
                              shrink_h_polys);
  VisCorridors2d(shrink_h_polys, "shrink_corridors_2d");

  /// solve
  LOG_INFO(">>> optimize");
  MincoGenerator generator(ini_state_, fini_state_, inner_pts, duration_time,
                           state_list, origin_h_polys, shrink_h_polys,
                           origin_freespace_data, shrink_freespace_data,
                           kPieceCutNums, kPieceCutNums, std::move(origin_om),
                           std::move(shrink_om));
  if (!generator.OptimizeTrajectory()) {
    LOG_ERROR("MincoGenertor failed.");
    return false;
  }

  /// minco path
  auto minco_path = generator.MincoPath();
  VisMincoPath(minco_path, "minco_path");

  /// fill path data
  if (!FillPathData(reference_line, minco_path, outside_data->path_data)) {
    LOG_ERROR("FillPathData failed.");
    return false;
  }

  return true;
}

bool MincoPathGenerator::FillPathData(const ReferenceLinePtr ref_ptr,
                                      const minco::Trajectory& minco_path,
                                      PathData* const path_data) {
  if (path_data == nullptr) {
    LOG_ERROR("path_data is nullptr");
    return false;
  }

  double total_time = minco_path.getTotalDuration();
  double dense_t = 0.02;
  std::size_t pts_num = static_cast<std::size_t>(total_time / dense_t);

  std::vector<PathPoint> discrete_path_pts(pts_num);
  std::vector<FrenetFramePoint> frenet_path_pts(pts_num);
  std::vector<ReferencePoint> ref_pts(pts_num);
  for (std::size_t i = 0; i < pts_num; ++i) {
    auto pos = minco_path.getPos(i * dense_t);
    double x = pos[0], y = pos[1];
    double angle = minco_path.getAngle(i * dense_t);
    double kappa = minco_path.getCurv(i * dense_t);
    PathPoint path_point(Vec2d(x, y), angle, kappa, 0., 0., 0.);
    if (i != 0) {
      double dis =
          (path_point.coordinate() - discrete_path_pts[i - 1].coordinate())
              .length();
      path_point.set_s(discrete_path_pts[i - 1].s() + dis);
    }
    discrete_path_pts[i] = std::move(path_point);

    SLPoint sl_pt{};
    ref_ptr->GetPointInFrenetFrame(Vec2d(x, y), &sl_pt);
    ReferencePoint ref_pt;
    ref_ptr->GetNearestRefPoint(sl_pt.s(), &ref_pt);
    FrenetFramePoint frenet_pt;
    double dl = SLAnalyticTransformation::calculate_lateral_derivative(
        ref_pt.heading(), angle, sl_pt.l(), ref_pt.kappa());
    double ddl =
        SLAnalyticTransformation::calculate_second_order_lateral_derivative(
            ref_pt.heading(), angle, ref_pt.kappa(), kappa, ref_pt.dkappa(),
            sl_pt.l());
    ref_pts[i] = std::move(ref_pt);
    frenet_path_pts[i] = FrenetFramePoint(sl_pt.s(), sl_pt.l(), dl, ddl);
  }

  path_data->set_frenet_path(FrenetFramePath(frenet_path_pts));
  path_data->set_path(DiscretizedPath(discrete_path_pts));
  path_data->set_reference_points(ref_pts);

  return true;
}

}  // namespace planning
}  // namespace neodrive
