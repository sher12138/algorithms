#include "third_order_spline_path_generator.h"

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/util/speed_planner_common.h"

namespace {
constexpr double k_obs_delta_s = 0.5;
constexpr double k_road_delta_s = 1.0;
constexpr double k_dist_buff = 0.1;
}  // namespace

/// TODO:(zhp) 设计问题: 障碍物采样，应该是根据 id
/// 进行来采样，而不是标志来进行采样

namespace neodrive {
namespace planning {
void VisBoundaries(const std::vector<PieceBoundary>& sample_boundaries,
                   const ReferenceLinePtr& reference_line) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("lat boundaries");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto event, auto& pt) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pt.x());
    sphere->mutable_center()->set_y(pt.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.1);
  };

  for (const auto& node : sample_boundaries) {
    Vec2d left_xy_point{}, right_xy_point{};
    if (!reference_line->GetPointInCartesianFrame({node.s, node.left_bound},
                                                  &left_xy_point) ||
        !reference_line->GetPointInCartesianFrame({node.s, node.right_bound},
                                                  &right_xy_point)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    set_pt(event, left_xy_point);
    set_pt(event, right_xy_point);
  }
}

void VisFinalPath(const std::vector<PathPoint>& points) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("final path");
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

void VisOptPath(
    double lon_sample_s,
    const std::vector<ThirdOrderSplinePath::OptVariables>& opt_path_variables,
    const std::vector<double>& knots_delta_s,
    const ReferenceLinePtr& reference_line) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("opt path");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  for (std::size_t i = 0; i < opt_path_variables.size(); ++i) {
    SLPoint frenet_point(lon_sample_s, opt_path_variables[i].xk.state_l0);
    Vec2d cartesian_point;
    if (!reference_line->GetPointInCartesianFrame(frenet_point,
                                                  &cartesian_point)) {
      LOG_ERROR("Fail to convert sl point to cartesian point");
      return;
    }
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(cartesian_point.x());
    sphere->mutable_center()->set_y(cartesian_point.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.1);
    lon_sample_s += knots_delta_s[i];
  }
}

ThirdOrderSplinePathGenerator::~ThirdOrderSplinePathGenerator() {
  lateral_boundaries_with_obs_.clear();
  lateral_boundaries_without_obs_.clear();
  sample_boundaries_.clear();
  knots_delta_s_.clear();
  upper_l_0_bounds_.clear();
  lower_l_0_bounds_.clear();
  upper_l_1_bounds_.clear();
  lower_l_1_bounds_.clear();
  upper_l_2_bounds_.clear();
  lower_l_2_bounds_.clear();
  goal_l0_.clear();
  goal_l1_.clear();
  goal_l2_.clear();
  opt_frenet_frame_points_.clear();
}

ErrorCode ThirdOrderSplinePathGenerator::Optimize(
    const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data) {
  LOG_INFO(
      "****************third order spline path planning*******************");
  if (config::PlanningConfig::Instance()
          ->planning_research_config()
          .only_for_backup_path_planner) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // 1. check and init
  if (Init(reference_line, inside_data, outside_data) !=
      ErrorCode::PLANNING_OK) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // 2. lateral_boundaries with forward and reverse driving
  if (!LateralBoundariesPreProcess(inside_data, outside_data)) {
    LOG_ERROR("lateral_boundaries_pre_process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // 3. forward process
  bool ret_process{false};
  if (!inside_data.is_reverse_driving) {
    LOG_INFO("is forward driving:");
    ret_process = Process(reference_line, inside_data,
                          lateral_boundaries_with_obs_, outside_data);
  } else {
    LOG_INFO("is reverse driving:");
    ret_process = Process(reference_line, inside_data,
                          lateral_boundaries_without_obs_, outside_data);
  }
  return ret_process ? ErrorCode::PLANNING_OK
                     : ErrorCode::PLANNING_ERROR_FAILED;
}

ErrorCode ThirdOrderSplinePathGenerator::Init(
    const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data) {
  // 1. check
  if (outside_data == nullptr || outside_data->path_data == nullptr ||
      outside_data->speed_data == nullptr ||
      outside_data->road_obs_path_shrink_boundries.size() < 4) {
    LOG_ERROR("input data invalid nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  const auto& frenet_init_point = outside_data->frenet_init_point;
  const auto& reference_point = outside_data->init_point_ref_point;
  const auto& init_point = inside_data.init_point;

  // 2. init
  // 2.1 path_output_length cal
  path_output_length_ =
      fmin(reference_line->ref_points().back().s() - 1,
           frenet_init_point.s() +
               fmax(DataCenter::Instance()->drive_strategy_max_speed() *
                        FLAGS_planning_trajectory_time_length,
                    FLAGS_planning_trajectory_min_length));
  LOG_INFO("path_out_max_s: {:.3f}", path_output_length_);
  if (frenet_init_point.s() > path_output_length_) {
    LOG_ERROR("frenet_init_point s > referline_end_s: {:.3f}, {:.3f}",
              frenet_init_point.s(), reference_line->ref_points().back().s());
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // 2.2 forward driving cal
  double init_dl = SLAnalyticTransformation::calculate_lateral_derivative(
      reference_point.heading(), init_point.theta(), frenet_init_point.l(),
      reference_point.kappa());
  double init_ddl =
      DataCenter::Instance()->master_info().is_use_position_stitch()
          ? 0.0
          : SLAnalyticTransformation::calculate_second_order_lateral_derivative(
                reference_point.heading(), init_point.theta(),
                reference_point.kappa(), init_point.kappa(),
                reference_point.dkappa(), frenet_init_point.l());

  // adjust init state when encountering high curvature lanes
  path_planner_common::AdjustInitState(reference_line, inside_data,
                                       outside_data, init_dl, init_ddl);

  if (!FLAGS_use_decouple) {
    double init_ddl_abs = std::fabs(init_ddl);
    double init_ddl_sign = init_ddl / init_ddl_abs;
    if (inside_data.vel_v > FLAGS_planning_piecewise_middle_high_speed &&
        init_ddl_abs > 0.2) {
      LOG_INFO("velo {:.3f} > {:.3f}, init_ddl {:.3f} change into 0.0",
               inside_data.vel_v, FLAGS_planning_piecewise_middle_high_speed);
      init_ddl = 0.0;
    } else if (inside_data.vel_v > FLAGS_planning_piecewise_high_speed &&
               init_ddl_abs > 0.1) {
      LOG_INFO("velo {:.3f} > {:.3f}, init_ddl {:.3f} change into 0.0",
               inside_data.vel_v, FLAGS_planning_piecewise_high_speed);
      init_ddl = 0.0;
    }
    init_ddl = init_ddl_abs > 0.27 ? (init_ddl_sign * 0.27) : init_ddl;
    init_ddl = 0.0;

    // decoupled planning and control
    double init_l_0{frenet_init_point.l()};
    double init_dl_0{init_dl};
    LOG_INFO("init_l_0 abs: {:.4f}, heading_diff: {:.4f}, 2_degree: {:.4f}",
             std::fabs(init_l_0),
             std::fabs(normalize_angle(init_point.theta() -
                                       reference_point.heading())),
             10.0 * M_PI / 180.);
    if (std::fabs(init_l_0) <= 0.20 &&
        std::fabs(
            normalize_angle(init_point.theta() - reference_point.heading())) <=
            2.0 * M_PI / 180.) {
      const auto& lateral_boundaries =
          outside_data->road_obs_path_shrink_boundries;
      const auto start_s = lateral_boundaries.front().s;
      for (std::size_t i = 0; i < lateral_boundaries.size(); ++i) {
        if (lateral_boundaries[i].left_is_obs ||
            lateral_boundaries[i].right_is_obs) {
          break;
        }
        if (lateral_boundaries[i].s - start_s > 5.0) {
          init_l_0 = 0.0;
          init_dl_0 = 0.0;
          break;
        }
      }
    }

    init_l_state_[0] = init_l_0;
    init_l_state_[1] = init_dl_0;
    init_l_state_[2] = init_ddl;
  } else {
    outside_data->frenet_init_point.set_dl(init_dl);
    outside_data->frenet_init_point.set_ddl(init_ddl);
    init_l_state_[0] = frenet_init_point.l();
    init_l_state_[1] = init_dl;
    init_l_state_[2] = init_ddl;
  }

  init_s_state_[0] = frenet_init_point.s();
  init_s_state_[1] = 0.0;
  init_s_state_[2] = 0.0;
  LOG_INFO("init s, l, dl, ddl: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
           init_s_state_[0], init_l_state_[0], init_l_state_[1],
           init_l_state_[2]);

  // 2.3 paras and data member init
  lateral_boundaries_with_obs_.clear();
  lateral_boundaries_without_obs_.clear();

  // speed_enlarge_buffer
  double en_large_buffer{FLAGS_planning_speed_plan_enlarge_self_buffer};
  double front_en_large_buffer{en_large_buffer};
  double back_en_large_buffer{en_large_buffer};
  if (speed_planner_common::GetAdcEnlargeBuffer(
          inside_data, inside_data.curr_multi_level, &en_large_buffer,
          &front_en_large_buffer, &back_en_large_buffer)) {
    speed_enlarge_buffer_ =
        std::max(speed_enlarge_buffer_,
                 std::max(en_large_buffer, std::max(front_en_large_buffer,
                                                    back_en_large_buffer))) +
        0.2;
  }
  LOG_INFO("speed_enlarge_buffer: {:.3f}", speed_enlarge_buffer_);

  return ErrorCode::PLANNING_OK;
}

bool ThirdOrderSplinePathGenerator::LateralBoundariesPreProcess(
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data) {
  double half_width = VehicleParam::Instance()->width() * 0.5;  // 0.56
  const auto& lat_boundaries = outside_data->road_obs_path_shrink_boundries;
  const double circle_distance = FLAGS_planning_piecewise_circle_distance;
  const double indoor_radius =
      config::PlanningConfig::Instance()->plan_config().indoor.radius;
  const double radius = inside_data.is_indoor
                            ? indoor_radius
                            : std::sqrt(half_width * half_width +
                                        circle_distance * circle_distance);

  lateral_boundaries_without_obs_.resize(lat_boundaries.size());
  bool with_obs_end_flag{false};
  for (std::size_t i = 0; i < lat_boundaries.size(); ++i) {
    auto bound_info = lat_boundaries[i];
    if (!with_obs_end_flag) {
      if (bound_info.left_bound - bound_info.right_bound < kMathEpsilon) {
        with_obs_end_flag = true;
      } else {
        lateral_boundaries_with_obs_.emplace_back(bound_info);
      }
    }
    bound_info.left_bound = bound_info.left_lane_bound - radius;
    bound_info.right_bound = bound_info.right_lane_bound + radius;
    bound_info.left_is_obs = false;
    bound_info.right_is_obs = false;
    lateral_boundaries_without_obs_[i] = bound_info;
  }

  return true;
}

bool ThirdOrderSplinePathGenerator::Process(
    const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data,
    const std::vector<PieceBoundary>& lat_boundaries,
    OutsidePlannerData* const outside_data) {
  // 0. clear private member
  ClearPrivateMember();

  // 1. for control test mode
  if (FLAGS_planning_only_provide_reference_line) {
    LOG_INFO(
        "path planning only provide reference line for control test mode.");
    return path_planner_common::OnlyProvideOriginReferenceLine(
        reference_line, inside_data, outside_data->frenet_init_point,
        outside_data->path_data);
  }

  // 2. bounds for third order spline problem
  // 2.1 sample with obs and road bound
  if (!SampleBoundaries(lat_boundaries)) {
    LOG_ERROR("sample boundaries failed.");
    return false;
  }
  outside_data->path_opt_boundaries = sample_boundaries_;
  VisBoundaries(sample_boundaries_, reference_line);

  // 2.2 circles for lateral bound

  // 3. generate third order spline path problem
  if (!GenerateOptProblem(reference_line, inside_data, outside_data)) {
    LOG_ERROR("generate third order spline path problem failed.");
    return false;
  }

  // 4. optimize
  if (!OptimizePath(reference_line)) {
    LOG_ERROR("OptimizePath failed.");
    return false;
  }

  // 5. fill path data
  if (!FillPathData(reference_line, outside_data->path_obstacle_context,
                    inside_data, outside_data, opt_frenet_frame_points_,
                    outside_data->path_data)) {
    LOG_ERROR("FillPathData err");
    return false;
  }

  return true;
}

void ThirdOrderSplinePathGenerator::ClearPrivateMember() {
  sample_boundaries_.clear();
  knots_delta_s_.clear();
  upper_l_0_bounds_.clear();
  lower_l_0_bounds_.clear();
  upper_l_1_bounds_.clear();
  lower_l_1_bounds_.clear();
  upper_l_2_bounds_.clear();
  lower_l_2_bounds_.clear();
  goal_l0_.clear();
  goal_l1_.clear();
  goal_l2_.clear();
  opt_frenet_frame_points_.clear();
}

bool ThirdOrderSplinePathGenerator::SampleBoundaries(
    const std::vector<PieceBoundary>& lat_boundaries) {
  // 1. obs need sample pairs
  const auto& obs_need_sample_pairs = ObsNeedSamplePairs(lat_boundaries);

  // 2. road need sample pairs
  const auto& road_need_sample_pairs =
      RoadNeedSamplePairs(obs_need_sample_pairs, lat_boundaries);

  // 3. sample
  return SampleBoundaries(lat_boundaries, obs_need_sample_pairs,
                          road_need_sample_pairs);
}

std::vector<std::pair<std::size_t, std::size_t>>
ThirdOrderSplinePathGenerator::ObsNeedSamplePairs(
    const std::vector<PieceBoundary>& lat_boundaries) const {
  std::vector<std::size_t> obs_index{};
  std::vector<std::pair<std::size_t, std::size_t>> obs_need_sample_pairs{};
  for (std::size_t i = 0; i < lat_boundaries.size(); ++i) {
    if (lat_boundaries[i].left_is_obs || lat_boundaries[i].right_is_obs) {
      obs_index.emplace_back(i);
    }
  }
  if (!obs_index.empty()) {
    obs_need_sample_pairs.emplace_back(
        std::make_pair(obs_index.front(), obs_index.front()));
    for (std::size_t index = 1; index < obs_index.size(); ++index) {
      double width = std::abs(lat_boundaries[obs_index[index]].left_bound -
                              lat_boundaries[obs_index[index]].right_bound);
      if (width > VehicleParam::Instance()->width() / 2) {
        if (obs_index[index] == obs_need_sample_pairs.back().second + 1) {
          obs_need_sample_pairs.back().second = obs_index[index];
        } else {
          obs_need_sample_pairs.emplace_back(
              std::make_pair(obs_index[index], obs_index[index]));
        }
      } else {
        if (index == obs_index.size() - 1) break;
        double width_last =
                   std::abs(lat_boundaries[obs_index[index - 1]].left_bound -
                            lat_boundaries[obs_index[index - 1]].right_bound),
               width_next =
                   std::abs(lat_boundaries[obs_index[index + 1]].left_bound -
                            lat_boundaries[obs_index[index + 1]].right_bound);
        LOG_DEBUG("path generator ind:{} width_last:{} width:{} width_next:{}",
                  obs_index[index], width_last, width, width_next);
        if (width < width_last && width <= width_next) {
          obs_need_sample_pairs.emplace_back(
              std::make_pair(obs_index[index], obs_index[index]));
        } else {
          obs_need_sample_pairs.back().second = obs_index[index];
        }
      }
    }
  }
  std::sort(obs_need_sample_pairs.begin(), obs_need_sample_pairs.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  // TEST
  LOG_DEBUG("obs_need_sample_pairs: ");
  for (const auto& pair : obs_need_sample_pairs) {
    LOG_DEBUG("{}, {}", pair.first, pair.second);
  }

  return obs_need_sample_pairs;
}

std::vector<std::pair<std::size_t, std::size_t>>
ThirdOrderSplinePathGenerator::RoadNeedSamplePairs(
    const std::vector<std::pair<std::size_t, std::size_t>>&
        obs_need_sample_pairs,
    const std::vector<PieceBoundary>& lat_boundaries) const {
  std::vector<std::pair<std::size_t, std::size_t>> res{};
  std::size_t start_index{0};
  std::size_t end_index{lat_boundaries.size() - 1};

  if (obs_need_sample_pairs.empty()) {
    res.emplace_back(std::make_pair(start_index, end_index));
  } else {
    auto add_cut =
        [](const std::vector<PieceBoundary>& bounds_info,
           std::size_t start_index, std::size_t end_index,
           std::vector<std::pair<std::size_t, std::size_t>>& road_s_pair,
           std::string type, double threshold) {
          LOG_DEBUG("road type:{} start:{} end:{}", type, start_index,
                    end_index);
          std::vector<std::size_t> cut_indexs;
          for (std::size_t i = start_index; i <= end_index; ++i) {
            if (end_index - start_index < 3) {
              // too few points, all indexs taken
              cut_indexs.emplace_back(i);
              LOG_DEBUG("cut_index:{}", i);
            } else {
              if (i == start_index || i == end_index) {
                cut_indexs.emplace_back(i);
                LOG_DEBUG("cut_index:{}", i);
              } else {
                double ul = bounds_info[i].left_bound,
                       ll = bounds_info[i].right_bound,
                       ul_next = bounds_info[i + 1].left_bound,
                       ll_next = bounds_info[i + 1].right_bound;
                // if boundary mutation，record index
                if (std::abs(ul_next - ul) > threshold ||
                    std::abs(ll_next - ll) > threshold) {
                  cut_indexs.emplace_back(i);
                  cut_indexs.emplace_back(i + 1);
                  LOG_DEBUG("cut_index:{}-{}", i, i + 1);
                }
              }
            }
          }
          for (std::size_t i = 0; i + 1 < cut_indexs.size(); ++i) {
            road_s_pair.emplace_back(
                std::make_pair(cut_indexs[i], cut_indexs[i + 1]));
          }
        };

    // road discuss
    double kDisThresh{0.5};
    if (start_index + 1 < obs_need_sample_pairs.front().first &&
        obs_need_sample_pairs.front().first >= 1) {
      res.emplace_back(std::make_pair(obs_need_sample_pairs.front().first - 1,
                                      obs_need_sample_pairs.front().first));
      add_cut(lat_boundaries, start_index,
              obs_need_sample_pairs.front().first - 1, res, "ONLY_BEFORE_OBS",
              kDisThresh);
    }
    if (obs_need_sample_pairs.back().second + 1 < end_index) {
      res.emplace_back(std::make_pair(obs_need_sample_pairs.back().second,
                                      obs_need_sample_pairs.back().second + 1));
      add_cut(lat_boundaries, obs_need_sample_pairs.back().second + 1,
              end_index, res, "ONLY_AFTER_OBS", kDisThresh);
    }
    for (std::size_t index = 1; index < obs_need_sample_pairs.size(); ++index) {
      if (obs_need_sample_pairs[index - 1].second + 1 <
          obs_need_sample_pairs[index].first - 1) {
        res.emplace_back(
            std::make_pair(obs_need_sample_pairs[index - 1].second,
                           obs_need_sample_pairs[index - 1].second + 1));
        res.emplace_back(std::make_pair(obs_need_sample_pairs[index].first - 1,
                                        obs_need_sample_pairs[index].first));
        add_cut(lat_boundaries, obs_need_sample_pairs[index - 1].second + 1,
                obs_need_sample_pairs[index].first - 1, res, "ELSE",
                kDisThresh);
      }
    }
  }
  std::sort(res.begin(), res.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  // TEST
  LOG_DEBUG("road_need_sample_pairs: ");
  for (const auto& pair : res) {
    LOG_DEBUG("{}, {}", pair.first, pair.second);
  }

  return res;
}

bool ThirdOrderSplinePathGenerator::SampleBoundaries(
    const std::vector<PieceBoundary>& lat_boundaries,
    const std::vector<std::pair<std::size_t, std::size_t>>&
        obs_need_sample_pairs,
    const std::vector<std::pair<std::size_t, std::size_t>>&
        road_need_sample_pairs) {
  if (lat_boundaries.size() <= 4) {
    LOG_ERROR("lat_boundaries size <= 4, could not create tk::spine");
    return false;
  }

  // 1. create bound spine
  std::vector<double> s_vec(lat_boundaries.size(), 0.0);
  std::vector<double> left_bound_vec(lat_boundaries.size(), 0.0);
  std::vector<double> right_bound_vec(lat_boundaries.size(), 0.0);
  for (std::size_t i = 0; i < lat_boundaries.size(); ++i) {
    s_vec[i] = lat_boundaries[i].s;
    left_bound_vec[i] = lat_boundaries[i].left_bound;
    right_bound_vec[i] = lat_boundaries[i].right_bound;
  }
  tk::spline left_bound_spine{}, right_bound_spine{};
  left_bound_spine.set_points(s_vec, left_bound_vec,
                              tk::spline::spline_type::linear);
  right_bound_spine.set_points(s_vec, right_bound_vec,
                               tk ::spline::spline_type::linear);

  // 1. obs
  const auto& obs_boundaries =
      ObsSampleBoundaries(left_bound_spine, right_bound_spine,
                          obs_need_sample_pairs, lat_boundaries);

  // 2. road
  const auto& road_boundaries =
      RoadSampleBoundaries(left_bound_spine, right_bound_spine,
                           road_need_sample_pairs, lat_boundaries);

  // 3. combine
  for (const auto& bound : obs_boundaries)
    sample_boundaries_.emplace_back(bound);
  for (const auto& bound : road_boundaries)
    sample_boundaries_.emplace_back(bound);
  std::sort(sample_boundaries_.begin(), sample_boundaries_.end(),
            [](const auto& a, const auto& b) { return a.s < b.s; });
  sample_boundaries_.erase(
      std::unique(sample_boundaries_.begin(), sample_boundaries_.end(),
                  [](const auto& a, const auto& b) { return a.s == b.s; }),
      sample_boundaries_.end());
  return true;
}

std::vector<PieceBoundary> ThirdOrderSplinePathGenerator::ObsSampleBoundaries(
    const tk::spline& left_bound_spine, const tk::spline& right_bound_spine,
    const std::vector<std::pair<std::size_t, std::size_t>>& obs_sample_pairs,
    const std::vector<PieceBoundary>& lat_boundaries) const {
  std::vector<PieceBoundary> res{};
  for (const auto& obs_pair : obs_sample_pairs) {
    double start_s = lat_boundaries[obs_pair.first].s;
    double end_s = lat_boundaries[obs_pair.second].s;
    if (end_s - start_s < k_obs_delta_s + kMathEpsilon ||
        obs_pair.first == obs_pair.second) {
      if (obs_pair.first < obs_pair.second) {
        PieceBoundary front_bound{.left_is_obs = true,
                                  .right_is_obs = true,
                                  .s = start_s,
                                  .left_bound = left_bound_spine(start_s),
                                  .right_bound = right_bound_spine(start_s)};
        PieceBoundary back_bound{.left_is_obs = true,
                                 .right_is_obs = true,
                                 .s = end_s,
                                 .left_bound = left_bound_spine(end_s),
                                 .right_bound = right_bound_spine(end_s)};
        res.emplace_back(front_bound);
        res.emplace_back(back_bound);
      } else {
        PieceBoundary center_bound{
            .left_is_obs = true,
            .right_is_obs = true,
            .s = (start_s + end_s) / 2.,
            .left_bound = left_bound_spine((start_s + end_s) / 2.),
            .right_bound = right_bound_spine((start_s + end_s) / 2.)};
        res.emplace_back(center_bound);
      }
    } else {
      for (double sample_s = start_s; sample_s <= end_s;
           sample_s += k_obs_delta_s) {
        PieceBoundary bound{.left_is_obs = true,
                            .right_is_obs = true,
                            .s = sample_s,
                            .left_bound = left_bound_spine(sample_s),
                            .right_bound = right_bound_spine(sample_s)};
        res.emplace_back(bound);
      }
      if (!res.empty() && end_s - res.back().s > k_obs_delta_s * 0.5) {
        PieceBoundary bound{.left_is_obs = true,
                            .right_is_obs = true,
                            .s = end_s,
                            .left_bound = left_bound_spine(end_s),
                            .right_bound = right_bound_spine(end_s)};
        res.emplace_back(bound);
      }
    }
  }
  // TEST
  LOG_DEBUG("obs_sample s: ");
  for (const auto& s : res) {
    LOG_DEBUG("s: {:.3f}", s.s);
  }

  return res;
}

std::vector<PieceBoundary> ThirdOrderSplinePathGenerator::RoadSampleBoundaries(
    const tk::spline& left_bound_spine, const tk::spline& right_bound_spine,
    const std::vector<std::pair<std::size_t, std::size_t>>& road_sample_pairs,
    const std::vector<PieceBoundary>& lat_boundaries) const {
  std::vector<PieceBoundary> res{};
  for (const auto& road_pair : road_sample_pairs) {
    double start_s = lat_boundaries[road_pair.first].s;
    double end_s = lat_boundaries[road_pair.second].s;
    if (end_s - start_s < k_obs_delta_s + kMathEpsilon ||
        road_pair.first == road_pair.second) {
      if (road_pair.first < road_pair.second) {
        PieceBoundary front_bound{.left_is_obs = false,
                                  .right_is_obs = false,
                                  .s = start_s,
                                  .left_bound = left_bound_spine(start_s),
                                  .right_bound = right_bound_spine(start_s)};
        PieceBoundary back_bound{.left_is_obs = false,
                                 .right_is_obs = false,
                                 .s = end_s,
                                 .left_bound = left_bound_spine(end_s),
                                 .right_bound = right_bound_spine(end_s)};
        res.emplace_back(front_bound);
        res.emplace_back(back_bound);
      } else {
        PieceBoundary center_bound{
            .left_is_obs = false,
            .right_is_obs = false,
            .s = (start_s + end_s) / 2.,
            .left_bound = left_bound_spine((start_s + end_s) / 2.),
            .right_bound = right_bound_spine((start_s + end_s) / 2.)};
        res.emplace_back(center_bound);
      }
    } else {
      for (double sample_s = start_s; sample_s <= end_s;
           sample_s += k_road_delta_s) {
        PieceBoundary bound{.left_is_obs = false,
                            .right_is_obs = false,
                            .s = sample_s,
                            .left_bound = left_bound_spine(sample_s),
                            .right_bound = right_bound_spine(sample_s)};
        res.emplace_back(bound);
      }
      if (!res.empty() && end_s - res.back().s > k_road_delta_s * 0.5) {
        PieceBoundary bound{.left_is_obs = false,
                            .right_is_obs = false,
                            .s = end_s,
                            .left_bound = left_bound_spine(end_s),
                            .right_bound = right_bound_spine(end_s)};
        res.emplace_back(bound);
      }
    }
  }
  // TEST
  LOG_DEBUG("road_sample s: ");
  for (const auto& s : res) {
    LOG_DEBUG("s: {:.3f}", s.s);
  }

  return res;
}

bool ThirdOrderSplinePathGenerator::GenerateOptProblem(
    const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data) {
  // 1. delta_s for knots
  knots_delta_s_ = std::move(DeltaSForKnots());

  // 2. l0, l1, l2, dl, ddl, dddl bounds
  if (!PathBounds(reference_line, inside_data, outside_data)) {
    LOG_ERROR("PiecewiseJerkPathBounds failed.");
    return false;
  }

  // 3. path goal l
  PathGoalL(inside_data, outside_data);

  // 4. create third order spline path mpc solver
  init_state_.state_l0 = init_l_state_[0];
  init_state_.state_dl = init_l_state_[1];
  init_state_.state_ddl = init_l_state_[2];
  init_control_.control_dddl = 0.0;
  third_order_spline_path_solver_ = std::make_shared<ThirdOrderSplinePathMPC>(
      VehicleParam::Instance()->length(), inside_data.vel_v, knots_delta_s_,
      init_state_, init_control_, upper_l_0_bounds_, lower_l_0_bounds_,
      upper_l_1_bounds_, lower_l_1_bounds_, upper_l_2_bounds_,
      lower_l_2_bounds_, upper_dl_0_bounds_, lower_dl_0_bounds_,
      upper_ddl_0_bounds_, lower_ddl_0_bounds_, upper_dddl_0_bounds_,
      lower_dddl_0_bounds_, goal_l0_, goal_l1_, goal_l2_);

  return true;
}

std::vector<double> ThirdOrderSplinePathGenerator::DeltaSForKnots() const {
  std::vector<double> res(sample_boundaries_.size() - 1, 0.0);
  if (res.empty()) return res;
  for (std::size_t index = 0; index < sample_boundaries_.size() - 1; ++index) {
    res[index] = sample_boundaries_[index + 1].s - sample_boundaries_[index].s;
  }
  // TEST
  LOG_DEBUG("delta_s for knot:");
  for (std::size_t i = 0; i < res.size(); ++i) {
    LOG_DEBUG("[{}], delta_s: {:.3f}: ", i, res[i]);
  }

  return res;
}

void ThirdOrderSplinePathGenerator::PathGoalL(
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data) {
  goal_l0_ = std::vector<double>(sample_boundaries_.size(), 0.0);
  goal_l1_ = std::vector<double>(sample_boundaries_.size(), 0.0);
  goal_l2_ = std::vector<double>(sample_boundaries_.size(), 0.0);

  const auto& goal_sl_points =
      outside_data->path_context.path_goal_points.goal_sl_points;
  if (!goal_sl_points.empty() && goal_sl_points.size() >= 4) {
    std::vector<double> s_vec{};
    std::vector<double> l_vec{};
    for (const auto& sl : goal_sl_points) {
      s_vec.emplace_back(sl.s());
      l_vec.emplace_back(sl.l());
    }
    tk::spline goal_sl_spline(s_vec, l_vec, tk::spline::spline_type::linear);

    for (std::size_t i = 0; i < sample_boundaries_.size(); ++i) {
      goal_l0_[i] = goal_sl_spline(sample_boundaries_[i].s);
      goal_l1_[i] = goal_sl_spline(sample_boundaries_[i].s +
                                   0.5 * VehicleParam::Instance()->length());
      goal_l2_[i] = goal_sl_spline(sample_boundaries_[i].s +
                                   VehicleParam::Instance()->length());
    }
    LOG_INFO("goal_l size:{}", goal_l0_.size());
  }
}

bool ThirdOrderSplinePathGenerator::PathBounds(
    const ReferenceLinePtr& reference_line,
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data) {
  // 1. l_bounds_
  upper_l_0_bounds_.resize(sample_boundaries_.size());
  lower_l_0_bounds_.resize(sample_boundaries_.size());
  upper_l_1_bounds_.resize(sample_boundaries_.size());
  lower_l_1_bounds_.resize(sample_boundaries_.size());
  upper_l_2_bounds_.resize(sample_boundaries_.size());
  lower_l_2_bounds_.resize(sample_boundaries_.size());
  std::vector<double> sample_s_vec(sample_boundaries_.size(), 0.0);
  for (std::size_t i = 0; i < sample_boundaries_.size(); ++i) {
    lower_l_0_bounds_[i] = sample_boundaries_[i].right_bound;
    upper_l_0_bounds_[i] = sample_boundaries_[i].left_bound;
    lower_l_1_bounds_[i] = sample_boundaries_[i].right_bound;
    upper_l_1_bounds_[i] = sample_boundaries_[i].left_bound;
    lower_l_2_bounds_[i] = sample_boundaries_[i].right_bound;
    upper_l_2_bounds_[i] = sample_boundaries_[i].left_bound;
    sample_s_vec[i] = sample_boundaries_[i].s;
  }
  if (sample_s_vec.size() > 4) {
    upper_l_spline_.set_points(sample_s_vec, upper_l_0_bounds_,
                               tk::spline::linear);
    lower_l_spline_.set_points(sample_s_vec, lower_l_0_bounds_,
                               tk::spline::linear);
    auto length = VehicleParam::Instance()->length();
    for (std::size_t i = 0; i < sample_s_vec.size(); ++i) {
      double upper_l_1 = upper_l_spline_(sample_s_vec[i] + 0.5 * length);
      double lower_l_1 = lower_l_spline_(sample_s_vec[i] + 0.5 * length);
      double upper_l_2 = upper_l_spline_(sample_s_vec[i] + length);
      double lower_l_2 = lower_l_spline_(sample_s_vec[i] + length);
      if (upper_l_1 - lower_l_1 > 2 * speed_enlarge_buffer_) {
        upper_l_1 -= speed_enlarge_buffer_;
        lower_l_1 += speed_enlarge_buffer_;
      } else {
        double dis = upper_l_1 - lower_l_1;
        upper_l_1 -= 0.4 * dis;
        lower_l_1 += 0.4 * dis;
      }
      if (upper_l_2 - lower_l_2 > 2 * speed_enlarge_buffer_) {
        upper_l_2 -= speed_enlarge_buffer_;
        lower_l_2 += speed_enlarge_buffer_;
      } else {
        double dis = upper_l_2 - lower_l_2;
        upper_l_2 -= 0.4 * dis;
        lower_l_2 += 0.4 * dis;
      }

      upper_l_1_bounds_[i] = upper_l_1;
      lower_l_1_bounds_[i] = lower_l_1;
      upper_l_2_bounds_[i] = upper_l_2;
      lower_l_2_bounds_[i] = lower_l_2;
    }
  }
  // Debug
  for (std::size_t i = 0; i < sample_boundaries_.size(); ++i) {
    LOG_DEBUG(
        "s, u_l0, l_l0, u_l1, l_l1, u_l2, l_l2: {:.2f}, {:.2f}, {:.2f}, "
        "{:.2f}, {:.2f}, {:.2f}, {:.2f}",
        sample_boundaries_[i].s, upper_l_0_bounds_[i], lower_l_0_bounds_[i],
        upper_l_1_bounds_[i], lower_l_1_bounds_[i], upper_l_2_bounds_[i],
        lower_l_2_bounds_[i]);
  }

  // 2. dl, ddl, dddl bound
  if (!UseAdaptiveBounds(inside_data, init_l_state_, &dl_bound_, &ddl_bound_,
                         &dddl_bound_)) {
    LOG_ERROR("UseAdaptiveBounds failed.");
    return false;
  }
  LOG_INFO("dl_bound, ddl_bound, dddl_bound: {:.3f}, {:.3f}, {:.3f}", dl_bound_,
           ddl_bound_, dddl_bound_);
  upper_dl_0_bounds_.resize(sample_boundaries_.size());
  lower_dl_0_bounds_.resize(sample_boundaries_.size());
  upper_ddl_0_bounds_.resize(sample_boundaries_.size());
  lower_ddl_0_bounds_.resize(sample_boundaries_.size());
  upper_dddl_0_bounds_.resize(sample_boundaries_.size());
  lower_dddl_0_bounds_.resize(sample_boundaries_.size());
  for (std::size_t i = 0; i < sample_s_vec.size(); ++i) {
    upper_dl_0_bounds_[i] = dl_bound_;
    lower_dl_0_bounds_[i] = -dl_bound_;
    upper_dddl_0_bounds_[i] = dddl_bound_;
    lower_dddl_0_bounds_[i] = -dddl_bound_;

    ReferencePoint ref_point;
    double kappa_ref =
        reference_line->GetNearestRefPoint(sample_s_vec[i], &ref_point)
            ? ref_point.kappa()
            : 0.0;
    upper_ddl_0_bounds_[i] = ddl_bound_ - kappa_ref;
    lower_ddl_0_bounds_[i] = -ddl_bound_ - kappa_ref;
    // Debug
    LOG_DEBUG("kappa_ref:{:.2f}", kappa_ref);
    LOG_DEBUG(
        "s, u_dl0, l_dl0, u_ddl0, l_ddl0, u_dddl0, l_dddl0: {:.2f}, {:.2f}, "
        "{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}",
        sample_boundaries_[i].s, upper_dl_0_bounds_[i], lower_dl_0_bounds_[i],
        upper_ddl_0_bounds_[i], lower_ddl_0_bounds_[i], upper_dddl_0_bounds_[i],
        lower_dddl_0_bounds_[i]);
  }

  return true;
}

bool ThirdOrderSplinePathGenerator::UseAdaptiveBounds(
    const InsidePlannerData& inside_data,
    const std::array<double, 3>& init_state, double* const used_dl,
    double* const used_ddl, double* const used_dddl) {
  if (used_dl == nullptr || used_ddl == nullptr || used_dddl == nullptr) {
    LOG_ERROR("input is nullptr");
    return false;
  }
  if (!UseDynamicDddxBounds(inside_data, used_dl, used_ddl, used_dddl)) {
    LOG_ERROR("UseDynamicDddxBounds failed");
    return false;
  }
  double abs_dl = std::fabs(init_state.at(1));
  double abs_ddl = std::fabs(init_state.at(2));
  if (abs_dl > *used_dl) {
    LOG_INFO("abs_dl {} over *used_dl {}", abs_dl, *used_dl);
    *used_dl = abs_dl + 0.1;
  }
  if (abs_ddl > *used_ddl) {
    LOG_INFO("abs_ddl {} over *used_ddl {}", abs_ddl, *used_ddl);
    *used_ddl = abs_ddl + 0.1;
  }
  return true;
}

bool ThirdOrderSplinePathGenerator::UseDynamicDddxBounds(
    const InsidePlannerData& inside_data, double* const dl_ptr,
    double* const ddl_ptr, double* const dddl_ptr) {
  if (dl_ptr == nullptr || ddl_ptr == nullptr || dddl_ptr == nullptr) {
    LOG_ERROR("input is nullptr");
    return false;
  }

  // dynamic speed threshold, m/s
  const double low_velo = FLAGS_planning_piecewise_low_speed;
  const double middle_velo = FLAGS_planning_piecewise_middle_speed;
  const double middle_high_velo = FLAGS_planning_piecewise_middle_high_speed;
  const double high_velo = FLAGS_planning_piecewise_high_speed;
  const double ulti_low_velo = 0.7;  // m/s
  if (inside_data.vel_v <= ulti_low_velo) {
    *dl_ptr = 1.0;
    *ddl_ptr = 0.5;
    *dddl_ptr = 0.5;
  } else if (inside_data.vel_v > ulti_low_velo &&
             inside_data.vel_v <= low_velo) {
    *dl_ptr = 1.0;
    *ddl_ptr = 0.5;
    *dddl_ptr = 0.5;
  } else if (inside_data.vel_v > low_velo && inside_data.vel_v <= middle_velo) {
    *dl_ptr = 0.8;
    *ddl_ptr = 0.3;
    *dddl_ptr = 0.3;
  } else if (inside_data.vel_v > middle_velo &&
             inside_data.vel_v <= middle_high_velo) {
    *dl_ptr = 0.8;
    *ddl_ptr = 0.3;
    *dddl_ptr = 0.3;
  } else if (inside_data.vel_v > middle_high_velo &&
             inside_data.vel_v <= high_velo) {
    *dl_ptr = 0.6;
    *ddl_ptr = 0.3;
    *dddl_ptr = 0.3;
  } else if (inside_data.vel_v > high_velo) {
    *dl_ptr = 0.5;
    *ddl_ptr = 0.3;
    *dddl_ptr = 0.3;
  } else {
    LOG_ERROR("out of design, err");
  }
  if (inside_data.is_changing_lane || inside_data.is_prepare_borrowing) {
    *dl_ptr = 1.0;
    *ddl_ptr = 1.0;
    *dddl_ptr = 1.0;
  }
  LOG_INFO(
      "curr velo {}, low_velo {}, middle_velo {}, high_velo {}, dl/ddl/dddl "
      "({}, {}, {})",
      inside_data.vel_v, low_velo, middle_velo, high_velo, *dl_ptr, *ddl_ptr,
      *dddl_ptr);
  return true;
}

bool ThirdOrderSplinePathGenerator::OptimizePath(
    const ReferenceLinePtr& reference_line) {
  if (!third_order_spline_path_solver_->Process()) {
    LOG_ERROR("third_order_spline_path_mpc failed");
    return false;
  }
  opt_path_variables_ = third_order_spline_path_solver_->optimal_solution();

  if (!DenseThirdOrderSplinePath(reference_line, path_output_length_,
                                 opt_path_variables_,
                                 opt_frenet_frame_points_)) {
    LOG_ERROR("ToThirdOrderSplinePath failed");
    return false;
  }

  return true;
}

bool ThirdOrderSplinePathGenerator::DenseThirdOrderSplinePath(
    const ReferenceLinePtr& reference_line, const double& path_output_length,
    const std::vector<ThirdOrderSplinePath::OptVariables>& opt_path_variables,
    std::vector<FrenetFramePoint>& opt_frenet_frame_points) {
  // 1. dense
  ThirdOrderSplinePath::A A_dense_s{};
  ThirdOrderSplinePath::B B_dense_s{};
  ThirdOrderSplinePath::StateVector state_vector{};
  ThirdOrderSplinePath::ControlVector control_vector{};
  double accumulated_s = init_s_state_[0];
  double max_accumulated_s = sample_boundaries_.back().s;
  for (std::size_t i = 0; i < opt_path_variables_.size(); ++i) {
    if (accumulated_s + kMathEpsilon > max_accumulated_s ||
        accumulated_s + kMathEpsilon > path_output_length_) {
      break;
    }
    opt_frenet_frame_points.emplace_back(
        FrenetFramePoint(accumulated_s, opt_path_variables_[i].xk.state_l0,
                         opt_path_variables_[i].xk.state_dl,
                         opt_path_variables_[i].xk.state_ddl));
    state_vector =
        ThirdOrderSplinePath::StateToVector(opt_path_variables_[i].xk);
    control_vector =
        ThirdOrderSplinePath::ControlToVector(opt_path_variables_[i].uk);
    for (std::size_t j = 1;
         j < static_cast<std::size_t>(knots_delta_s_[i] / dense_delta_s_);
         ++j) {
      if (accumulated_s + j * dense_delta_s_ + kMathEpsilon >
          max_accumulated_s) {
        break;
      }
      if (accumulated_s + j * dense_delta_s_ + kMathEpsilon >
          path_output_length_) {
        break;
      }
      A_dense_s.row(0) << 1, j * dense_delta_s_,
          0.5 * std::pow(j * dense_delta_s_, 2);
      A_dense_s.row(1) << 0, 1, j * dense_delta_s_;
      A_dense_s.row(2) << 0, 0, 1;
      B_dense_s << 1. / 6 * std::pow(j * dense_delta_s_, 3),
          0.5 * std::pow(j * dense_delta_s_, 2), j * dense_delta_s_;
      ThirdOrderSplinePath::State stage_dense_s =
          ThirdOrderSplinePath::VectorToState(A_dense_s * state_vector +
                                              B_dense_s * control_vector);
      opt_frenet_frame_points.emplace_back(FrenetFramePoint(
          accumulated_s + j * dense_delta_s_, stage_dense_s.state_l0,
          stage_dense_s.state_dl, stage_dense_s.state_ddl));
    }

    accumulated_s += knots_delta_s_[i];
  }

  // 2. extend to path_output_length
  if (!opt_frenet_frame_points.empty()) {
    accumulated_s = opt_frenet_frame_points.back().s();
    if (accumulated_s < path_output_length_) {
      for (double s = accumulated_s + dense_delta_s_;
           s + kMathEpsilon < path_output_length_; s += dense_delta_s_) {
        double extend_l = opt_frenet_frame_points.back().l();
        opt_frenet_frame_points.emplace_back(
            FrenetFramePoint(s, extend_l, 0.0, 0.0));
      }
    }
  }

  LOG_DEBUG("dense frenet points info: ");
  for (const auto& p : opt_frenet_frame_points) {
    LOG_DEBUG("s, l, dl, ddl: {:.3f}, {:.3f}, {:.3f}, {:.3f}", p.s(), p.l(),
              p.dl(), p.ddl());
  }

  return !opt_frenet_frame_points.empty();
}

bool ThirdOrderSplinePathGenerator::FillPathData(
    const ReferenceLinePtr& reference_line,
    const PathObstacleContext& path_obstacle_context,
    const InsidePlannerData& inside_data,
    OutsidePlannerData* const outside_data,
    const std::vector<FrenetFramePoint>& frenet_path,
    PathData* const path_data) {
  if (path_data == nullptr || frenet_path.empty()) {
    LOG_ERROR("path_data is nullptr, frenet_path is empty");
    return false;
  }
  path_data->set_frenet_path(FrenetFramePath(frenet_path));
  std::vector<PathPoint> path_points{};
  path_points.reserve(frenet_path.size());
  path_data->mutable_reference_points()->resize(frenet_path.size());
  LOG_INFO("Front s {:.3f}", frenet_path.front().s());
  for (std::size_t i = 0; i < frenet_path.size(); ++i) {
    const auto& frenet_point = frenet_path[i];
    Vec2d cartesian_point;
    if (!reference_line->GetPointInCartesianFrame(frenet_point,
                                                  &cartesian_point)) {
      LOG_ERROR("Fail to convert sl point to cartesian point");
      return false;
    }
    ReferencePoint ref_point;
    if (!reference_line->GetNearestRefPoint(frenet_point.s(), &ref_point)) {
      LOG_ERROR("failed to GetNearestRefPoint s: {:.4f} ", frenet_point.s());
      return false;
    }
    double theta = SLAnalyticTransformation::calculate_theta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    double kappa = SLAnalyticTransformation::calculate_kappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    PathPoint path_point(cartesian_point, theta, kappa, 0.0, 0.0, 0.0);
    if (i != 0) {
      double distance =
          (path_points.back().coordinate() - path_point.coordinate()).length();
      path_point.set_s(path_points.back().s() + distance);
    }
    path_points.push_back(std::move(path_point));
    // 0.2 * 75 = 15.0m, just print front 15.0m info.
    if (i % 2 == 0) {
      LOG_DEBUG(
          "frenet point info i {}, s {:.3f}, l {:.3f}, "
          "dl {:.3f}, ddl {:.3f}, theta {:.3f}, kappa {:.3f}",
          i, frenet_point.s(), frenet_point.l(), frenet_point.dl(),
          frenet_point.ddl(), theta, kappa);
    }
    (*path_data->mutable_reference_points())[i] = std::move(ref_point);
  }

  VisOptPath(init_s_state_[0], opt_path_variables_, knots_delta_s_,
             reference_line);

  VisFinalPath(path_points);

  path_data->set_path(DiscretizedPath(path_points));
  double collision_free_length =
      frenet_path.back().s() - frenet_path.front().s();
  if (!path_planner_common::CalcFinalTrajValidLength(
          FLAGS_planning_piecewise_planner, path_obstacle_context,
          reference_line, frenet_path, &collision_free_length)) {
    LOG_INFO("CalcFinalTrajValidLength err");
  }

  double kappa_cut_length = collision_free_length;
  if (!path_planner_common::CutOffFinalTrajValidLength(path_data->path(),
                                                       &kappa_cut_length)) {
    LOG_INFO("CutOffFinalTrajValidLength err");
    return false;
  }
  double final_valid_length =
      std::fmin(collision_free_length, kappa_cut_length);
  LOG_INFO(
      "collision_free_length {:.3f}, kappa_cut_length {:.3f}, "
      "final_valid_length {:.3f}",
      collision_free_length, kappa_cut_length, final_valid_length);
  path_data->set_valid_length(final_valid_length);
  LOG_INFO("FillPathData final_valid_length {:.3f}", final_valid_length);

  std::vector<double> history_piecewise_valid_len =
      outside_data->path_context.piecewise_path_context
          .history_piecewise_valid_len;
  if (!path_planner_common::UpdateHistoryValidLength(
          final_valid_length, history_piecewise_valid_len)) {
    LOG_INFO("UpdateHistoryValidLength err");
    return false;
  }
  outside_data->path_context.piecewise_path_context
      .history_piecewise_valid_len = history_piecewise_valid_len;

  return true;
}

}  // namespace planning
}  // namespace neodrive
