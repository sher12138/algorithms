#include "fem_pos_smoother.h"

#include "src/planning/math/curve1d/spline.h"

namespace neodrive {
namespace planning {

bool FemPosSmoother::Smooth(const std::vector<EvaluatedPoint>& evaluated_points,
                            const double delta_s, const double dense_s) {
  need_opt_pts_.clear(), need_opt_pts_.resize(evaluated_points.size());
  need_opt_bounds_.clear(), need_opt_bounds_.resize(evaluated_points.size());
  seg_idxs_.clear(), seg_idxs_.resize(evaluated_points.size());
  start_s_ = evaluated_points[0].path_point.s();
  for (std::size_t i = 0; i < evaluated_points.size(); ++i) {
    need_opt_pts_[i] = {evaluated_points[i].path_point.x(),
                        evaluated_points[i].path_point.y()};
    need_opt_bounds_[i] = evaluated_points[i].lateral_bound;
    seg_idxs_[i] = evaluated_points[i].path_point.seg_idx();
  }
  if (need_opt_pts_.size() < 4 || need_opt_bounds_.size() < 4 ||
      (need_opt_pts_.size() != need_opt_bounds_.size())) {
    LOG_ERROR("Fem Pos opt_pts/bounds.size < 4");
    return false;
  }
  need_opt_bounds_.front() = 0.0;
  need_opt_bounds_.back() = 0.0;

  NormalizePoints(&need_opt_pts_);

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  if (!Solve(need_opt_pts_, need_opt_bounds_, &opt_x, &opt_y)) {
    LOG_ERROR("Fem Pos reference line smoothing failed");
    return false;
  }
  if (opt_x.size() < 2 || opt_y.size() < 2 || (opt_x.size() != opt_y.size())) {
    LOG_ERROR("opt_x_size, opt_y_size: {}, {}", opt_x.size(), opt_y.size());
    return false;
  }
  if (need_opt_pts_.size() != opt_x.size()) {
    LOG_ERROR("opt_pts_size, opt_x_size: {}, {}", need_opt_pts_.size(),
              opt_x.size());
    return false;
  }
  for (std::size_t i = 0; i < opt_x.size(); ++i) {
    need_opt_pts_[i] = {opt_x[i], opt_y[i]};
  }

  DeNormalizePoints(&need_opt_pts_);

  GenerateRefPoint(evaluated_points, need_opt_pts_, delta_s, dense_s);

  return true;
}

bool FemPosSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    LOG_ERROR("opt_x or opt_y is nullptr");
    return false;
  }

  FemPosOsqpSolver solver;
  solver.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation());
  solver.set_weight_path_length(config_.weight_path_length());
  solver.set_weight_ref_deviation(config_.weight_ref_deviation());

  solver.set_max_iter(config_.max_iter());
  solver.set_time_limit(config_.time_limit());
  solver.set_verbose(config_.verbose());
  solver.set_scaled_termination(config_.scaled_termination());
  solver.set_warm_start(config_.warm_start());

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);
  if (!solver.Solve()) {
    LOG_ERROR("QpWithOsqp failed");
    return false;
  }
  *opt_x = solver.opt_x();
  *opt_y = solver.opt_y();

  return true;
}

void FemPosSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void FemPosSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool FemPosSmoother::GenerateRefPoint(
    const std::vector<EvaluatedPoint>& evaluated_points,
    std::vector<std::pair<double, double>>& xy_points, const double delta_s,
    const double dense_s) {
  if (std::abs(delta_s - dense_s) > kMathEpsilon && dense_s < delta_s) {
    std::vector<double> x_pts(xy_points.size(), 0.0);
    std::vector<double> y_pts(xy_points.size(), 0.0);
    std::vector<double> s_pts(xy_points.size(), 0.0);
    for (std::size_t i = 0; i < xy_points.size(); ++i) {
      x_pts[i] = xy_points[i].first;
      y_pts[i] = xy_points[i].second;
      s_pts[i] =
          (i == 0)
              ? start_s_
              : (s_pts[i - 1] +
                 std::sqrt(
                     std::pow(xy_points[i].first - xy_points[i - 1].first, 2) +
                     std::pow(xy_points[i].second - xy_points[i - 1].second,
                              2)));
      LOG_INFO("x, y, s: {:.3f}, {:.3f}, {:.3f}", x_pts[i], y_pts[i], s_pts[i]);
    }
    if (x_pts.size() < 4) {
      return false;
    }
    tk::spline x_s_spline{}, y_s_spline{};
    x_s_spline.set_points(s_pts, x_pts, tk::spline::spline_type::cspline);
    y_s_spline.set_points(s_pts, y_pts, tk::spline::spline_type::cspline);
    std::size_t xy_points_size =
        static_cast<std::size_t>(s_pts.back() / dense_s + 1);
    xy_points.resize(xy_points_size);
    for (std::size_t i = 0; i < xy_points_size; ++i) {
      xy_points[i] = {x_s_spline(i * dense_s), y_s_spline(i * dense_s)};
    }
  }

  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas,
                          &dkappas)) {
    return false;
  }
  if (xy_points.size() != seg_idxs_.size()){
    LOG_ERROR("xy_points size {} != seg_idxs_ size {}", xy_points.size(),
              seg_idxs_.size());
    return false;
  }  
  std::size_t points_size = xy_points.size();
  generate_points_.clear();
  generate_points_.resize(xy_points.size());
  for (std::size_t i = 0; i < points_size; ++i) {
    ReferencePoint tmp_pt;
    tmp_pt.set_x(xy_points[i].first);
    tmp_pt.set_y(xy_points[i].second);
    tmp_pt.set_heading(headings[i]);
    tmp_pt.set_kappa(kappas[i]);
    tmp_pt.set_dkappa(dkappas[i]);
    tmp_pt.set_s(accumulated_s[i]);
    tmp_pt.set_z(evaluated_points[i].path_point.z());
    tmp_pt.set_pitch(evaluated_points[i].path_point.pitch());
    tmp_pt.set_seg_idx(seg_idxs_[i]);
    generate_points_[i] = std::move(tmp_pt);
  }

  return true;
}

bool FemPosSmoother::ComputePathProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<double>* headings, std::vector<double>* accumulated_s,
    std::vector<double>* kappas, std::vector<double>* dkappas) {
  if (headings == nullptr || accumulated_s == nullptr || kappas == nullptr ||
      dkappas == nullptr) {
    LOG_ERROR("headings, kappas, dkappas = nullptr");
    return false;
  }
  headings->clear();
  kappas->clear();
  dkappas->clear();
  accumulated_s->clear();
  if (xy_points.size() < 2) {
    return false;
  }

  std::size_t points_size = xy_points.size();
  headings->resize(points_size);
  accumulated_s->resize(points_size);
  kappas->resize(points_size);
  dkappas->resize(points_size);
  std::vector<double> dxs(points_size);
  std::vector<double> dys(points_size);
  std::vector<double> y_over_s_first_derivatives(points_size);
  std::vector<double> x_over_s_first_derivatives(points_size);
  std::vector<double> y_over_s_second_derivatives(points_size);
  std::vector<double> x_over_s_second_derivatives(points_size);

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].first - xy_points[i].first);
      y_delta = (xy_points[i + 1].second - xy_points[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].first - xy_points[i - 1].first);
      y_delta = (xy_points[i].second - xy_points[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
    }
    dxs[i] = x_delta;
    dys[i] = y_delta;
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    headings->at(i) = std::atan2(dys[i], dxs[i]);
  }

  // Get linear interpolated s for dkappa calculation
  double distance = start_s_;
  accumulated_s->at(0) = distance;
  double fx = xy_points[0].first;
  double fy = xy_points[0].second;
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].first;
    ny = xy_points[i].second;
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->at(i) = end_segment_s + distance;
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xy_points[i + 1].first - xy_points[i].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points[i + 1].second - xy_points[i].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points[i].first - xy_points[i - 1].first) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points[i].second - xy_points[i - 1].second) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives[i] = xds;
    y_over_s_first_derivatives[i] = yds;
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives[i] = xdds;
    y_over_s_second_derivatives[i] = ydds;
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->at(i) = kappa;
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->at(i) = dkappa;
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive
