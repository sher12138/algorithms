#include "iterative_anchoring_smoother.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

IterativeAnchoringSmoother::IterativeAnchoringSmoother(
    const ParkingIterativeAnchoringConfig& iterative_config,
    const ParkingCommonConfig common_config) {
  iterative_config_ = iterative_config;
  common_config_ = common_config;
  ego_length_ = VehicleParam::Instance()->length();
  ego_width_ = VehicleParam::Instance()->width();
  center_shift_distance_ =
      ego_length_ / 2.0 - VehicleParam::Instance()->back_edge_to_center();
}

bool IterativeAnchoringSmoother::Smooth(
    const std::vector<TrajectoryPoint>& raw_tra,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    DiscretizedTrajectory* discretized_trajectory) {
  if (raw_tra.size() < 2) {
    LOG_ERROR("raw points size smaller than two, smoother quit");
    return false;
  }

  // Set gear of the trajectory
  gear_ = CheckGear(raw_tra);

  double curr_heading = raw_tra.front().theta();
  curr_heading = normalize_angle(curr_heading);

  bool reverse_path = false;
  // // setting reverse flag here, chi
  // if ((gear_ && fabs(curr_heading) <= M_PI_2) ||
  //     (!gear_ && fabs(curr_heading) > M_PI_2)) {
  //   reverse_path = false;
  // } else {
  //   reverse_path = true;
  // }

  // mchan fix : setting reverse flag here, chi
  if (gear_) {
    LOG_INFO("****************forward path*********************");
    reverse_path = false;
  } else {
    LOG_INFO("****************reverse path*********************");
    reverse_path = true;
  }

  // TEST
  LOG_INFO("current traj size : {}", raw_tra.size());
  for (auto pt : raw_tra) {
    LOG_INFO("***********ready to smooth : x {:.4f} y {:.4f} theta {:.4f}",
             pt.x(), pt.y(), pt.theta());
  }
  // Set obstacle in form of linesegments
  std::vector<std::vector<Segment2d>> obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<Segment2d> obstacle_linesegments;
    for (size_t i = 0; i + 1 < vertices_num; ++i) {
      Segment2d line_segment =
          Segment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

  // Interpolate the traj
  DiscretizedPath warm_start_path;
  std::vector<PathPoint> tmp_path_pts;
  size_t tra_size = raw_tra.size();
  double accumulated_s = 0.0;
  if (reverse_path == false) {
    Vec2d last_path_point(raw_tra.front().x(), raw_tra.front().y());
    for (size_t i = 0; i < tra_size; ++i) {
      Vec2d cur_path_point(raw_tra[i].x(), raw_tra[i].y());
      accumulated_s += cur_path_point.distance_to(last_path_point);
      PathPoint path_point;
      path_point.set_x(raw_tra[i].x());
      path_point.set_y(raw_tra[i].y());
      path_point.set_theta(raw_tra[i].theta());
      path_point.set_s(accumulated_s);
      tmp_path_pts.push_back(path_point);
      last_path_point = cur_path_point;
    }
  } else {
    // reverse, s is not corret, doesnot matter
    Vec2d last_path_point(raw_tra.back().x(), raw_tra.back().y());
    for (int i = tra_size - 1; i >= 0; i--) {
      Vec2d cur_path_point(raw_tra[i].x(), raw_tra[i].y());
      accumulated_s += cur_path_point.distance_to(last_path_point);
      PathPoint path_point;
      path_point.set_x(raw_tra[i].x());
      path_point.set_y(raw_tra[i].y());
      path_point.set_theta(raw_tra[i].theta());
      path_point.set_s(accumulated_s);
      tmp_path_pts.push_back(path_point);
      last_path_point = cur_path_point;
    }
  }
  warm_start_path.set_path_points(tmp_path_pts);

  const double interpolated_delta_s = iterative_config_.interpolated_delta_s_;
  std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
  double path_length = warm_start_path.param_length();
  double delta_s = path_length / std::ceil(path_length / interpolated_delta_s);
  path_length += delta_s * 1.0e-6;
  for (double s = 0; s < path_length; s += delta_s) {
    PathPoint point2d;
    if (!warm_start_path.evaluate_linear_approximation(s, point2d)) {
      LOG_ERROR("evaluate_linear_approximation failed");
      return false;
    }
    interpolated_warm_start_point2ds.emplace_back(point2d.x(), point2d.y());
  }

  const size_t interpolated_size = interpolated_warm_start_point2ds.size();
  if (interpolated_size < 4) {
    LOG_ERROR(
        "interpolated_warm_start_path smaller than 4, can't enforce "
        "heading continuity");
    // return false; // mchan add for test
  } else if (interpolated_size < 6) {
    LOG_INFO(
        "interpolated_warm_start_path smaller than 6, can't enforce "
        "initial zero kappa");
    enforce_initial_kappa_ = false;
  } else {
    enforce_initial_kappa_ = true;
  }
  // reverse
  if (reverse_path) {
    reverse(interpolated_warm_start_point2ds.begin(),
            interpolated_warm_start_point2ds.end());
  }

  // Adjust heading to ensure heading continuity
  AdjustStartEndHeading(raw_tra, &interpolated_warm_start_point2ds);

  // Reset path profile by discrete point heading and curvature estimation
  DiscretizedPath interpolated_warm_start_path;
  if (!SetPathProfile(interpolated_warm_start_point2ds,
                      &interpolated_warm_start_path)) {
    LOG_ERROR("Set path profile fails");
    return false;
  }

  // Generate feasible bounds for each path point
  std::vector<double> bounds;
  if (!GenerateInitialBounds(interpolated_warm_start_path, &bounds)) {
    LOG_ERROR(
        "Generate initial bounds failed, path point to close to obstacle");
    return false;
  }

  // Check initial path collision avoidance, if it fails, smoother assumption
  // fails. Try reanchoring
  input_colliding_point_index_.clear();
  if (!CheckCollisionAvoidance(interpolated_warm_start_path,
                               &input_colliding_point_index_)) {
    LOG_WARN("Interpolated input path points colliding with obstacle");
    LOG_WARN("still try to smooth path");
  }

  // Smooth path to have smoothed x, y, phi, kappa and s
  DiscretizedPath smoothed_path_points;
  // mchan add to fix smooth core dump
  if (interpolated_warm_start_path.path_points().size() < 4) {
    smoothed_path_points = interpolated_warm_start_path;
  } else if (!SmoothPath(interpolated_warm_start_path, bounds,
                         &smoothed_path_points)) {
    return false;
  }

  input_colliding_point_index_.clear();
  if (!CheckCollisionAvoidance(smoothed_path_points,
                               &input_colliding_point_index_)) {
    LOG_ERROR("smoothed path points colliding with obstacle");
    return false;
  }

  if (!FakeCombinePathAndSpeed(smoothed_path_points, discretized_trajectory)) {
    LOG_ERROR("FakeCombinePathAndSpeed failed");
    return false;
  }

  LOG_INFO("discretized_trajectory size {}",
           discretized_trajectory->num_of_points());
  return true;
}

bool IterativeAnchoringSmoother::CheckGear(
    const std::vector<TrajectoryPoint>& raw_tra) {
  if (raw_tra.size() < 2) {
    LOG_ERROR("raw_tra.size()[{}] < 2", raw_tra.size());
    return false;
  }
  double init_heading_angle = raw_tra.front().theta();
  const Vec2d init_tracking_vector(raw_tra[1].x() - raw_tra[0].x(),
                                   raw_tra[1].y() - raw_tra[0].y());
  double init_tracking_angle = init_tracking_vector.angle();
  bool ret =
      std::fabs(normalize_angle(init_tracking_angle - init_heading_angle)) <
      M_PI_2;
  LOG_INFO("init heading angle : {:.4f}", init_heading_angle);
  LOG_INFO("init tracking angel : {:.4f}", init_tracking_angle);
  return ret;
}

void IterativeAnchoringSmoother::AdjustStartEndHeading(
    const std::vector<TrajectoryPoint>& raw_tra,
    std::vector<std::pair<double, double>>* const point2d) {
  // Sanity check
  if (point2d == nullptr) {
    LOG_ERROR("point2d is null");
    return;
  }
  if (raw_tra.size() <= 1) {
    LOG_ERROR("raw_tra.size() [{}] <= 1", raw_tra.size());
    return;
  }
  if (point2d->size() <= 3) {
    LOG_ERROR("point2d->size() [{}] <= 3", point2d->size());
    return;
  }

  // Set initial heading and bounds
  const double initial_heading = raw_tra.front().theta();
  const double end_heading = raw_tra.back().theta();

  // Adjust the point position to have heading by finite element difference of
  // the point and the other point equal to the given warm start initial or end
  // heading
  const double first_to_second_dx = point2d->at(1).first - point2d->at(0).first;
  const double first_to_second_dy =
      point2d->at(1).second - point2d->at(0).second;
  const double first_to_second_s =
      std::sqrt(first_to_second_dx * first_to_second_dx +
                first_to_second_dy * first_to_second_dy);
  Vec2d first_point(point2d->at(0).first, point2d->at(0).second);
  Vec2d initial_vec(first_to_second_s, 0);
  initial_vec.self_rotate(gear_ ? initial_heading
                                : normalize_angle(initial_heading + M_PI));
  initial_vec += first_point;
  point2d->at(1) = std::make_pair(initial_vec.x(), initial_vec.y());

  const size_t path_size = point2d->size();
  const double second_last_to_last_dx =
      point2d->at(path_size - 1).first - point2d->at(path_size - 2).first;
  const double second_last_to_last_dy =
      point2d->at(path_size - 1).second - point2d->at(path_size - 2).second;
  const double second_last_to_last_s =
      std::sqrt(second_last_to_last_dx * second_last_to_last_dx +
                second_last_to_last_dy * second_last_to_last_dy);
  Vec2d last_point(point2d->at(path_size - 1).first,
                   point2d->at(path_size - 1).second);
  Vec2d end_vec(second_last_to_last_s, 0);
  end_vec.self_rotate(gear_ ? normalize_angle(end_heading + M_PI)
                            : end_heading);
  end_vec += last_point;
  point2d->at(path_size - 2) = std::make_pair(end_vec.x(), end_vec.y());
  return;
}

bool IterativeAnchoringSmoother::SetPathProfile(
    const std::vector<std::pair<double, double>>& point2d,
    DiscretizedPath* raw_path_points) {
  if (raw_path_points == nullptr) {
    LOG_ERROR("raw_path_points is null");
    return false;
  }
  raw_path_points->clear();
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }
  if (point2d.size() != headings.size() || point2d.size() != kappas.size() ||
      point2d.size() != dkappas.size() ||
      point2d.size() != accumulated_s.size()) {
    LOG_ERROR(
        "point2d.size() [{}] != headings.size() [{}] "
        "!= kappas.size() [{}] != dkappas.size() [{}]"
        "!= accumulated_s.size() [{}]",
        point2d.size(), headings.size(), kappas.size(), dkappas.size(),
        accumulated_s.size());
    return false;
  }

  // Load into path point
  size_t points_size = point2d.size();
  for (size_t i = 0; i < points_size; ++i) {
    PathPoint path_point;
    path_point.set_x(point2d[i].first);
    path_point.set_y(point2d[i].second);
    path_point.set_theta(headings[i]);
    path_point.set_s(accumulated_s[i]);
    path_point.set_kappa(kappas[i]);
    path_point.set_dkappa(dkappas[i]);
    raw_path_points->mutable_path_points()->push_back(path_point);
  }
  return true;
}

bool IterativeAnchoringSmoother::GenerateInitialBounds(
    const DiscretizedPath& path_points, std::vector<double>* initial_bounds) {
  if (initial_bounds == nullptr) {
    LOG_ERROR("initial_bounds is null");
    return false;
  }
  initial_bounds->clear();

  const bool estimate_bound = iterative_config_.estimate_bound_;
  const double default_bound = iterative_config_.default_bound_;
  const double vehicle_shortest_dimension =
      iterative_config_.vehicle_shortest_dimension_;
  const double kEpislon = 1e-8;

  if (!estimate_bound) {
    std::vector<double> default_bounds(path_points.num_of_points(),
                                       default_bound);
    *initial_bounds = std::move(default_bounds);
    return true;
  }

  // refine obstacle formulation and speed it up
  for (const auto& path_point : path_points.path_points()) {
    double min_bound = std::numeric_limits<double>::infinity();
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const Segment2d& linesegment : obstacle_linesegments) {
        min_bound =
            std::min(min_bound,
                     linesegment.distance_to({path_point.x(), path_point.y()}));
      }
    }
    min_bound -= vehicle_shortest_dimension;
    min_bound = min_bound < kEpislon ? 0.0 : min_bound;
    initial_bounds->push_back(min_bound);
  }
  return true;
}

bool IterativeAnchoringSmoother::CheckCollisionAvoidance(
    const DiscretizedPath& path_points,
    std::vector<size_t>* colliding_point_index) {
  if (colliding_point_index == nullptr) {
    LOG_ERROR("colliding_point_index is null");
    return false;
  }

  colliding_point_index->clear();
  size_t path_points_size = path_points.num_of_points();
  for (size_t i = 0; i < path_points_size; ++i) {
    // Skip checking collision for thoese points colliding originally
    bool skip_checking = false;
    for (size_t index = 0; index < colliding_point_index->size(); index++) {
      if (i == colliding_point_index->at(index)) {
        skip_checking = true;
        break;
      }
    }
    if (skip_checking) {
      continue;
    }

    PathPoint pt1;
    if (!path_points.path_point_at(i, pt1)) {
      LOG_ERROR("path_point_at {} failed", i);
      return false;
    }

    const double heading =
        gear_ ? pt1.theta() : normalize_angle(pt1.theta() + M_PI);
    Box2d ego_box({pt1.x() + center_shift_distance_ * std::cos(heading),
                   pt1.y() + center_shift_distance_ * std::sin(heading)},
                  heading, ego_length_, ego_width_);

    bool is_colliding = false;
    // chi, need to change here
    // for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
    //   for (const Segment2d& linesegment : obstacle_linesegments) {
    //     if (ego_box.has_overlap(linesegment)) {
    //       colliding_point_index->push_back(i);
    //       LOG_INFO("point at {} collied with LineSegment {}", i,
    //                linesegment.debug_string());
    //       is_colliding = true;
    //       break;
    //     }
    //   }
    //   if (is_colliding) {
    //     break;
    //   }
    // }
  }

  if (!colliding_point_index->empty()) {
    return false;
  }
  return true;
}

bool IterativeAnchoringSmoother::SmoothPath(
    const DiscretizedPath& raw_path_points, const std::vector<double>& bounds,
    DiscretizedPath* smoothed_path_points) {
  std::vector<std::pair<double, double>> raw_point2d;
  for (const auto& path_point : raw_path_points.path_points()) {
    raw_point2d.emplace_back(path_point.x(), path_point.y());
  }

  const size_t max_iteration_num = iterative_config_.path_max_iteration_num_;

  bool is_collision_free = false;
  std::vector<size_t> colliding_point_index;
  std::vector<std::pair<double, double>> smoothed_point2d;
  size_t counter = 0;

  std::vector<double> flexible_bounds;
  flexible_bounds = bounds;

  while (!is_collision_free) {
    if (counter > max_iteration_num) {
      LOG_ERROR("Maximum iteration reached, path smoother early stops");
      return true;
    }

    AdjustPathBounds(colliding_point_index, &flexible_bounds);

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    if (!FemPosSmoother::Instance()->Solve(raw_point2d, flexible_bounds, &opt_x,
                                           &opt_y)) {
      LOG_ERROR("Smoothing path fails");
      return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2) {
      LOG_ERROR("Return by fem_pos_smoother is wrong. Size smaller than 2 ");
      return false;
    }

    if (opt_x.size() != opt_y.size()) {
      LOG_ERROR("opt_x.size() [{}] != opt_y.size() [{}]", opt_x.size(),
                opt_y.size());
      return false;
    }

    size_t point_size = opt_x.size();
    smoothed_point2d.clear();
    for (size_t i = 0; i < point_size; ++i) {
      smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
    }

    if (!SetPathProfile(smoothed_point2d, smoothed_path_points)) {
      LOG_ERROR("Set path profile fails");
      return false;
    }

    is_collision_free =
        CheckCollisionAvoidance(*smoothed_path_points, &colliding_point_index);

    LOG_INFO("loop iteration number is {}", counter);
    ++counter;
  }
  return true;
}

void IterativeAnchoringSmoother::AdjustPathBounds(
    const std::vector<size_t>& colliding_point_index,
    std::vector<double>* bounds) {
  if (bounds == nullptr) {
    LOG_ERROR("bounds is null");
    return;
  }

  const double collision_decrease_ratio =
      iterative_config_.collision_decrease_ratio_;

  for (const auto index : colliding_point_index) {
    bounds->at(index) *= collision_decrease_ratio;
  }

  // Anchor the end points to enforce the initial end end heading continuity and
  // zero kappa
  bounds->at(0) = 0.0;
  bounds->at(1) = 0.0;
  bounds->at(bounds->size() - 1) = 0.0;
  bounds->at(bounds->size() - 2) = 0.0;
  if (enforce_initial_kappa_) {
    bounds->at(2) = 0.0;
  }
  return;
}

bool IterativeAnchoringSmoother::FakeCombinePathAndSpeed(
    const DiscretizedPath& path_points,
    DiscretizedTrajectory* discretized_trajectory) {
  if (discretized_trajectory == nullptr) {
    LOG_ERROR("discretized_trajectory is null");
    return false;
  }
  discretized_trajectory->mutable_trajectory_points()->clear();
  if (path_points.path_points().empty()) {
    LOG_ERROR("path data is empty");
    return false;
  }

  PathPoint path_point;
  const double time_res = common_config_.path_time_resolution_;
  double dt = 0.0;
  for (size_t i = 0; i < path_points.path_points().size();
       ++i, dt += time_res) {
    path_point = path_points.path_points().at(i);
    TrajectoryPoint trajectory_point(path_point, 1.0, 0.0, 0.0, dt);
    discretized_trajectory->append_trajectory_point(trajectory_point);
  }
  AdjustPathAndSpeedByGear(discretized_trajectory);

  return true;
}

void IterativeAnchoringSmoother::AdjustPathAndSpeedByGear(
    DiscretizedTrajectory* discretized_trajectory) {
  if (gear_) {
    return;
  }
  std::for_each(discretized_trajectory->mutable_trajectory_points()->begin(),
                discretized_trajectory->mutable_trajectory_points()->end(),
                [](TrajectoryPoint& trajectory_point) {
                  trajectory_point.mutable_path_point()->set_theta(
                      normalize_angle(trajectory_point.theta() + M_PI));
                  //        trajectory_point.mutable_path_point()->set_s(-1.0 *
                  //                                                     trajectory_point.s());
                  trajectory_point.mutable_path_point()->set_kappa(
                      -1.0 * trajectory_point.kappa());
                  // dkappa stays the same as direction of both kappa and s are
                  // reversed
                  //        trajectory_point.set_velocity(-1.0 *
                  //        trajectory_point.velocity());
                  //        trajectory_point.set_acceleration(-1.0 *
                  //                                          trajectory_point.acceleration());
                  trajectory_point.set_direction(1);
                });
  return;
}

bool IterativeAnchoringSmoother::ReAnchoring(
    const std::vector<size_t>& colliding_point_index,
    DiscretizedPath* path_points) {
  if (path_points == nullptr) {
    LOG_ERROR("path_points is null");
    return false;
  }
  if (colliding_point_index.empty()) {
    LOG_INFO("no point needs to be re-anchored");
    return true;
  }
  size_t max_colliding_point_index = *(std::max_element(
      colliding_point_index.begin(), colliding_point_index.end()));
  if (path_points->num_of_points() <= max_colliding_point_index) {
    LOG_ERROR(
        "path_points->num_of_points() [{}] <=max_colliding_point_index [{}]",
        path_points->num_of_points(), max_colliding_point_index);
    return false;
  }

  for (const auto index : colliding_point_index) {
    if (index == 0 || index == path_points->num_of_points() - 1) {
      LOG_ERROR("Initial and end points collision avoid condition failed.");
      return false;
    }
    if (index == 1 || index == path_points->num_of_points() - 2) {
      LOG_ERROR(
          "second to last point or second point pos reanchored. Heading "
          "discontinuity might "
          "happen");
    }
  }

  const size_t reanchoring_trails_num =
      static_cast<size_t>(iterative_config_.reanchoring_trails_num_);
  const double reanchoring_pos_stddev =
      iterative_config_.reanchoring_pos_stddev_;
  const double reanchoring_length_stddev =
      iterative_config_.reanchoring_length_stddev_;
  std::random_device rd;
  std::default_random_engine gen = std::default_random_engine(rd());
  std::normal_distribution<> pos_dis{0, reanchoring_pos_stddev};
  std::normal_distribution<> length_dis{0, reanchoring_length_stddev};

  for (const auto index : colliding_point_index) {
    bool reanchoring_success = false;
    for (size_t i = 0; i < reanchoring_trails_num; ++i) {
      // Get ego box for collision check on collision point index
      bool is_colliding = false;
      for (size_t j = index - 1; j < index + 2; ++j) {
        PathPoint pt1;
        if (!path_points->path_point_at(j, pt1)) {
          LOG_ERROR("path_point_at {} failed", j);
          return false;
        }

        const double heading =
            gear_ ? pt1.theta() : normalize_angle(pt1.theta() + M_PI);
        Box2d ego_box({pt1.x() + center_shift_distance_ * std::cos(heading),
                       pt1.y() + center_shift_distance_ * std::sin(heading)},
                      heading, ego_length_, ego_width_);
        for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
          for (const Segment2d& linesegment : obstacle_linesegments) {
            if (ego_box.has_overlap(linesegment)) {
              is_colliding = true;
              break;
            }
          }
          if (is_colliding) {
            break;
          }
        }
        if (is_colliding) {
          break;
        }
      }

      if (is_colliding) {
        // Adjust the point by randomly move around the original points
        PathPoint pt1;
        PathPoint pt2;
        if (index == 1) {
          if (!path_points->path_point_at(index - 1, pt1)) {
            LOG_ERROR("path_point_at {} failed", index - 1);
            return false;
          }
          if (!path_points->path_point_at(index, pt2)) {
            LOG_ERROR("path_point_at {} failed", index);
            return false;
          }
          const double adjust_theta = pt1.theta();
          const double delta_s = std::abs(pt2.s() - pt1.s());
          double rand_dev = clamp(length_dis(gen), 0.8, -0.8);
          double adjusted_delta_s = delta_s * (1.0 + rand_dev);
          pt2.set_x(pt1.x() + adjusted_delta_s * std::cos(adjust_theta));
          pt2.set_y(pt1.y() + adjusted_delta_s * std::sin(adjust_theta));
        } else if (index == path_points->num_of_points() - 2) {
          if (!path_points->path_point_at(index + 1, pt1)) {
            LOG_ERROR("path_point_at {} failed", index - 1);
            return false;
          }
          if (!path_points->path_point_at(index, pt2)) {
            LOG_ERROR("path_point_at {} failed", index);
            return false;
          }
          const double adjust_theta = normalize_angle(pt1.theta() + M_PI);
          const double delta_s = std::abs(pt1.s() - pt2.s());
          double rand_dev = clamp(length_dis(gen), 0.8, -0.8);
          double adjusted_delta_s = delta_s * (1.0 + rand_dev);
          pt2.set_x(pt1.x() + adjusted_delta_s * std::cos(adjust_theta));
          pt2.set_y(pt1.y() + adjusted_delta_s * std::sin(adjust_theta));
        } else {
          if (!path_points->path_point_at(index, pt2)) {
            LOG_ERROR("path_point_at {} failed", index);
            return false;
          }
          double rand_dev_x = clamp(pos_dis(gen), 2.0 * reanchoring_pos_stddev,
                                    -2.0 * reanchoring_pos_stddev);
          double rand_dev_y = clamp(pos_dis(gen), 2.0 * reanchoring_pos_stddev,
                                    -2.0 * reanchoring_pos_stddev);
          pt2.set_x(pt2.x() + rand_dev_x);
          pt2.set_y(pt2.y() + rand_dev_y);
        }

        // Adjust heading accordingly
        // TODO: refactor into math module
        // current point heading adjustment
        for (size_t i = index - 1; i < index + 2; ++i) {
          path_points->mutable_path_points()->at(i).set_theta(
              CalcHeadings(*path_points, i));
        }
      } else {
        reanchoring_success = true;
        break;
      }
    }

    if (!reanchoring_success) {
      LOG_ERROR(
          "interpolated points at index {}, can't be successfully reanchored",
          index);
      return false;
    }
  }
  return true;
}

double IterativeAnchoringSmoother::CalcHeadings(
    const DiscretizedPath& path_points, const size_t index) {
  if (path_points.num_of_points() <= 2) {
    LOG_ERROR(" path_points.num_of_points() [{}] <= 2",
              path_points.num_of_points());
    return 0.0;
  }
  double dx = 0.0;
  double dy = 0.0;
  auto& tmp_path_points = path_points.path_points();
  if (index == 0) {
    dx = tmp_path_points[index + 1].x() - tmp_path_points[index].x();
    dy = tmp_path_points[index + 1].y() - tmp_path_points[index].y();
  } else if (index == path_points.num_of_points() - 1) {
    dx = tmp_path_points[index].x() - tmp_path_points[index - 1].x();
    dy = tmp_path_points[index].y() - tmp_path_points[index - 1].y();
  } else {
    dx =
        0.5 * (tmp_path_points[index + 1].x() - tmp_path_points[index - 1].x());
    dy =
        0.5 * (tmp_path_points[index + 1].y() - tmp_path_points[index - 1].y());
  }
  return std::atan2(dy, dx);
}

// ****discard

bool IterativeAnchoringSmoother::Smooth(
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    DiscretizedTrajectory* discretized_trajectory) {
  if (xWS.cols() < 2) {
    LOG_ERROR(
        "reference points size smaller than two, smoother early "
        "returned");
    return false;
  }

  // Set gear of the trajectory
  gear_ = CheckGear(xWS);
  if (xWS.size() <= 1) {
    LOG_ERROR("xWS.size()[{}] <= 1", xWS.size());
    return false;
  }
  double curr_heading = xWS(2, 0);
  curr_heading = normalize_angle(curr_heading);

  bool reverse_path = false;
  // setting reverse flag here, chi
  if ((gear_ && fabs(curr_heading) <= M_PI_2) ||
      (!gear_ && fabs(curr_heading) > M_PI_2)) {
    reverse_path = false;
  } else {
    reverse_path = true;
  }

  // Set obstacle in form of linesegments
  std::vector<std::vector<Segment2d>> obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<Segment2d> obstacle_linesegments;
    for (size_t i = 0; i + 1 < vertices_num; ++i) {
      Segment2d line_segment =
          Segment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

  // Interpolate the traj
  DiscretizedPath warm_start_path;
  std::vector<PathPoint> tmp_path_pts;
  long xWS_size = xWS.cols();
  double accumulated_s = 0.0;
  if (reverse_path == false) {
    Vec2d last_path_point(xWS(0, 0), xWS(1, 0));
    for (long i = 0; i < xWS_size; ++i) {
      Vec2d cur_path_point(xWS(0, i), xWS(1, i));
      accumulated_s += cur_path_point.distance_to(last_path_point);
      PathPoint path_point;
      path_point.set_x(xWS(0, i));
      path_point.set_y(xWS(1, i));
      path_point.set_theta(xWS(2, i));
      path_point.set_s(accumulated_s);
      tmp_path_pts.push_back(path_point);
      last_path_point = cur_path_point;
    }
  } else {
    // reverse, s is not corret, doesnot matter
    Vec2d last_path_point(xWS(0, xWS_size - 1), xWS(1, xWS_size - 1));
    for (long i = xWS_size - 1; i >= 0; i--) {
      Vec2d cur_path_point(xWS(0, i), xWS(1, i));
      accumulated_s += cur_path_point.distance_to(last_path_point);
      PathPoint path_point;
      path_point.set_x(xWS(0, i));
      path_point.set_y(xWS(1, i));
      path_point.set_theta(xWS(2, i));
      path_point.set_s(accumulated_s);
      tmp_path_pts.push_back(path_point);
      last_path_point = cur_path_point;
    }
  }
  warm_start_path.set_path_points(tmp_path_pts);

  const double interpolated_delta_s = iterative_config_.interpolated_delta_s_;
  std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
  double path_length = warm_start_path.param_length();
  double delta_s = path_length / std::ceil(path_length / interpolated_delta_s);
  path_length += delta_s * 1.0e-6;
  for (double s = 0; s < path_length; s += delta_s) {
    PathPoint point2d;
    if (!warm_start_path.evaluate_linear_approximation(s, point2d)) {
      LOG_ERROR("evaluate_linear_approximation failed");
      return false;
    }
    interpolated_warm_start_point2ds.emplace_back(point2d.x(), point2d.y());
  }

  const size_t interpolated_size = interpolated_warm_start_point2ds.size();
  if (interpolated_size < 4) {
    LOG_ERROR(
        "interpolated_warm_start_path smaller than 4, can't enforce "
        "heading continuity");
    return false;
  } else if (interpolated_size < 6) {
    LOG_INFO(
        "interpolated_warm_start_path smaller than 4, can't enforce "
        "initial zero kappa");
    enforce_initial_kappa_ = false;
  } else {
    enforce_initial_kappa_ = true;
  }
  // reverse
  if (reverse_path) {
    reverse(interpolated_warm_start_point2ds.begin(),
            interpolated_warm_start_point2ds.end());
  }

  // Adjust heading to ensure heading continuity
  AdjustStartEndHeading(xWS, &interpolated_warm_start_point2ds);

  // Reset path profile by discrete point heading and curvature estimation
  DiscretizedPath interpolated_warm_start_path;
  if (!SetPathProfile(interpolated_warm_start_point2ds,
                      &interpolated_warm_start_path)) {
    LOG_ERROR("Set path profile fails");
    return false;
  }

  // Generate feasible bounds for each path point
  std::vector<double> bounds;
  if (!GenerateInitialBounds(interpolated_warm_start_path, &bounds)) {
    LOG_ERROR(
        "Generate initial bounds failed, path point to close to obstacle");
    return false;
  }

  // Check initial path collision avoidance, if it fails, smoother assumption
  // fails. Try reanchoring
  input_colliding_point_index_.clear();
  if (!CheckCollisionAvoidance(interpolated_warm_start_path,
                               &input_colliding_point_index_)) {
    LOG_WARN("Interpolated input path points colliding with obstacle");
    LOG_WARN("still try to smooth path");
  }

  // Smooth path to have smoothed x, y, phi, kappa and s
  DiscretizedPath smoothed_path_points;
  if (!SmoothPath(interpolated_warm_start_path, bounds,
                  &smoothed_path_points)) {
    return false;
  }

  input_colliding_point_index_.clear();
  if (!CheckCollisionAvoidance(smoothed_path_points,
                               &input_colliding_point_index_)) {
    LOG_ERROR("smoothed path points colliding with obstacle");
    return false;
  }

  if (!FakeCombinePathAndSpeed(smoothed_path_points, discretized_trajectory)) {
    LOG_ERROR("FakeCombinePathAndSpeed failed");
    return false;
  }

#if 0
  {
      // Smooth speed to have smoothed v and a
      SpeedData smoothed_speeds;
      if (!SmoothSpeed(init_a, init_v, smoothed_path_points.param_length(),
                       &smoothed_speeds)) {
        return false;
      }

      // TODO: Evaluate performance
      // SpeedData smoothed_speeds;
      // if (!GenerateStopProfileFromPolynomial(
      //         init_a, init_v, smoothed_path_points.Length(), &smoothed_speeds)) {
      //   return false;
      // }

      // Combine path and speed
      if (!CombinePathAndSpeed(smoothed_path_points, smoothed_speeds,
                               discretized_trajectory)) {
        return false;
      }

      AdjustPathAndSpeedByGear(discretized_trajectory);

  }
#endif

  LOG_INFO("discretized_trajectory size {}",
           discretized_trajectory->num_of_points());
  return true;
}

bool IterativeAnchoringSmoother::CheckGear(const Eigen::MatrixXd& xWS) {
  if (xWS.size() <= 1) {
    LOG_ERROR("xWS.size()[{}] <= 1", xWS.size());
    return false;
  }
  double init_heading_angle = xWS(2, 0);
  const Vec2d init_tracking_vector(xWS(0, 1) - xWS(0, 0),
                                   xWS(1, 1) - xWS(1, 0));
  double init_tracking_angle = init_tracking_vector.angle();
  return std::abs(normalize_angle(init_tracking_angle - init_heading_angle)) <
         M_PI_2;
}

void IterativeAnchoringSmoother::AdjustStartEndHeading(
    const Eigen::MatrixXd& xWS,
    std::vector<std::pair<double, double>>* const point2d) {
  // Sanity check
  if (point2d == nullptr) {
    LOG_ERROR("point2d is null");
    return;
  }
  if (xWS.cols() <= 1) {
    LOG_ERROR("xWS.cols() [{}] <= 1", xWS.cols());
    return;
  }
  if (point2d->size() <= 3) {
    LOG_ERROR("point2d->size() [{}] <= 3", point2d->size());
    return;
  }

  // Set initial heading and bounds
  const double initial_heading = xWS(2, 0);
  const double end_heading = xWS(2, xWS.cols() - 1);

  // Adjust the point position to have heading by finite element difference of
  // the point and the other point equal to the given warm start initial or end
  // heading
  const double first_to_second_dx = point2d->at(1).first - point2d->at(0).first;
  const double first_to_second_dy =
      point2d->at(1).second - point2d->at(0).second;
  const double first_to_second_s =
      std::sqrt(first_to_second_dx * first_to_second_dx +
                first_to_second_dy * first_to_second_dy);
  Vec2d first_point(point2d->at(0).first, point2d->at(0).second);
  Vec2d initial_vec(first_to_second_s, 0);
  initial_vec.self_rotate(gear_ ? initial_heading
                                : normalize_angle(initial_heading + M_PI));
  initial_vec += first_point;
  point2d->at(1) = std::make_pair(initial_vec.x(), initial_vec.y());

  const size_t path_size = point2d->size();
  const double second_last_to_last_dx =
      point2d->at(path_size - 1).first - point2d->at(path_size - 2).first;
  const double second_last_to_last_dy =
      point2d->at(path_size - 1).second - point2d->at(path_size - 2).second;
  const double second_last_to_last_s =
      std::sqrt(second_last_to_last_dx * second_last_to_last_dx +
                second_last_to_last_dy * second_last_to_last_dy);
  Vec2d last_point(point2d->at(path_size - 1).first,
                   point2d->at(path_size - 1).second);
  Vec2d end_vec(second_last_to_last_s, 0);
  end_vec.self_rotate(gear_ ? normalize_angle(end_heading + M_PI)
                            : end_heading);
  end_vec += last_point;
  point2d->at(path_size - 2) = std::make_pair(end_vec.x(), end_vec.y());
  return;
}

bool IterativeAnchoringSmoother::SmoothSpeed(const double init_a,
                                             const double init_v,
                                             const double path_length,
                                             SpeedData* smoothed_speeds) {
  const double max_forward_v = common_config_.max_forward_v_;
  const double max_reverse_v = common_config_.max_reverse_v_;
  const double max_forward_acc = common_config_.max_forward_acc_;
  const double max_reverse_acc = common_config_.max_reverse_acc_;
  const double max_acc_jerk = common_config_.max_acc_jerk_;
  const double delta_t = common_config_.delta_t_;

  const double total_t = 2 * path_length / max_reverse_acc * 10;
  LOG_INFO("total_t is : {:.4f}", total_t);
  const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, 4000, delta_t, 0,
      {0.0, std::abs(init_v), std::abs(init_a)});

  // set end constraints
  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear_ ? max_forward_v : max_reverse_v;
  const double max_acc = gear_ ? max_forward_acc : max_reverse_acc;

  const auto upper_dx = std::fmax(max_v, std::abs(init_v));
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-max_acc, max_acc});

  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  std::vector<double> x_ref(num_of_knots, path_length);
  piecewise_jerk_problem.set_x_ref(iterative_config_.ref_s_weight_, x_ref);
  piecewise_jerk_problem.set_weight_ddx(iterative_config_.acc_weight_);
  piecewise_jerk_problem.set_weight_dddx(iterative_config_.jerk_weight_);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    LOG_ERROR("Piecewise jerk speed optimizer failed!");
    return false;
  }

  // Extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  // Assign speed point by gear
  SpeedPoint tmp_speed_pt(s[0], 0.0, ds[0], dds[0], 0.0);
  smoothed_speeds->mutable_speed_vector()->push_back(tmp_speed_pt);
  const double kEpislon = 1.0e-4;
  const double sEpislon = 1.0e-1;
  for (size_t i = 1; i < num_of_knots; ++i) {
    if (s[i - 1] - s[i] > kEpislon) {
      LOG_ERROR(
          "unexpected decreasing s in speed smoothing at time {:.4f} with "
          "total "
          "time {:.4f}",
          static_cast<double>(i) * delta_t, total_t);
      return false;
    }
    SpeedPoint tmp_speed_pt(s[i], delta_t * static_cast<double>(i), ds[i],
                            dds[i], (dds[i] - dds[i - 1]) / delta_t);
    smoothed_speeds->mutable_speed_vector()->push_back(tmp_speed_pt);
    // Cut the speed data when it is about to meet end condition
    if (path_length - s[i] < sEpislon) {
      break;
    }
  }
  return true;
}

bool IterativeAnchoringSmoother::GenerateStopProfileFromPolynomial(
    const double init_acc, const double init_speed, const double stop_distance,
    SpeedData* smoothed_speeds) {
  static constexpr double kMaxT = 8.0;
  static constexpr double kUnitT = 0.2;
  for (double t = 2.0; t <= kMaxT; t += kUnitT) {
    QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, stop_distance,
                                   0.0, 0.0, t);
    if (!IsValidPolynomialProfile(curve)) {
      continue;
    }
    for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
      const double curve_s = curve.evaluate(0, curve_t);
      const double curve_v = curve.evaluate(1, curve_t);
      const double curve_a = curve.evaluate(2, curve_t);
      const double curve_da = curve.evaluate(3, curve_t);
      SpeedPoint tmp_speed_pt(curve_s, curve_t, curve_v, curve_a, curve_da);
      smoothed_speeds->mutable_speed_vector()->push_back(tmp_speed_pt);
    }
    return true;
  }
  LOG_ERROR("GenerateStopProfileFromPolynomial fails.");
  return false;
}

bool IterativeAnchoringSmoother::CombinePathAndSpeed(
    const DiscretizedPath& path_points, const SpeedData& speed_points,
    DiscretizedTrajectory* discretized_trajectory) {
  if (discretized_trajectory == nullptr) {
    LOG_ERROR("discretized_trajectory is null");
    return false;
  }
  discretized_trajectory->mutable_trajectory_points()->clear();
  // TODO: move to confs
  const double kDenseTimeResolution = common_config_.path_time_resolution_;
  const double time_horizon =
      speed_points.total_time() + kDenseTimeResolution * 1.0e-6;
  if (path_points.path_points().empty()) {
    LOG_ERROR("path data is empty");
    return false;
  }
  LOG_INFO("speed_points.TotalTime() {:.4f}", speed_points.total_time());
  for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
       cur_rel_time += kDenseTimeResolution) {
    SpeedPoint speed_point;
    if (!speed_points.get_speed_point_with_time(cur_rel_time, &speed_point)) {
      LOG_ERROR("Fail to get speed point with relative time {:.4f}",
                cur_rel_time);
      return false;
    }

    if (speed_point.s() > path_points.param_length()) {
      break;
    }

    PathPoint path_point;
    if (!path_points.evaluate(speed_point.s(), path_point)) {
      LOG_ERROR("evaluate {:.4f} failed", speed_point.s());
      return false;
    }

    TrajectoryPoint trajectory_point(path_point, speed_point.v(),
                                     speed_point.a(), speed_point.j(),
                                     speed_point.t());
    discretized_trajectory->append_trajectory_point(trajectory_point);
  }
  LOG_INFO("path length before combine {:.4f}", path_points.param_length());
  LOG_INFO("trajectory length after combine {:.4f}",
           discretized_trajectory->spatial_length());
  return true;
}

bool IterativeAnchoringSmoother::IsValidPolynomialProfile(
    const QuinticPolynomialCurve1d& curve) {
  for (double evaluate_t = 0.1; evaluate_t <= curve.param_length();
       evaluate_t += 0.2) {
    const double v = curve.evaluate(1, evaluate_t);
    const double a = curve.evaluate(2, evaluate_t);
    static constexpr double kEpsilon = 1e-3;
    if (v < -kEpsilon || a > 1.0) {
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
