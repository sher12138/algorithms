#include "discretized_path.h"

namespace neodrive {
namespace planning {

DiscretizedPath::DiscretizedPath(const std::vector<PathPoint>& path_points) {
  path_points_ = path_points;
}

bool DiscretizedPath::evaluate(const double param, PathPoint& pt) const {
  if (path_points_.empty()) return false;

  auto it_lower = query_lower_bound(param);
  if (it_lower == path_points_.begin()) {
    pt = path_points_.front();
  } else if (it_lower == path_points_.end()) {
    pt = path_points_.back();
  } else {
    return PathPoint::interpolate(*(it_lower - 1), *it_lower, param, pt);
  }
  return true;
}

bool DiscretizedPath::evaluate_linear_approximation(const double param,
                                                    PathPoint& pt) const {
  if (path_points_.empty()) return false;

  auto it_lower = query_lower_bound(param);
  if (it_lower == path_points_.begin()) {
    pt = path_points_.front();
  } else if (it_lower == path_points_.end()) {
    pt = path_points_.back();
  } else {
    return PathPoint::interpolate_linear_approximation(*(it_lower - 1),
                                                       *it_lower, param, pt);
  }
  return true;
}

double DiscretizedPath::param_length() const {
  if (path_points_.empty()) return 0.0;

  return path_points_.back().s() - path_points_.front().s();
}

int DiscretizedPath::query_closest_point(const double param) const {
  if (path_points_.empty()) return -1;

  auto it_lower = query_lower_bound(param);

  if (it_lower == path_points_.begin()) return 0;

  if (it_lower == path_points_.end()) return int(path_points_.size() - 1);

  double d0 = param - (it_lower - 1)->s();
  double d1 = it_lower->s() - param;

  if (d0 < d1) {
    return int((it_lower - path_points_.begin()) - 1);
  } else {
    return int((it_lower - path_points_.begin()));
  }
}

bool DiscretizedPath::query_closest_point(const Vec2d& pos,
                                          PathPoint& pt) const {
  if (path_points_.empty()) {
    return false;
  }

  double min_dis{std::numeric_limits<double>::infinity()};
  int idx_min{0};
  auto GetMinDis = [&](auto cur_idx) {
    double cur_dis = std::pow(pos.x() - path_points_[cur_idx].x(), 2) +
                     std::pow(pos.y() - path_points_[cur_idx].y(), 2);
    if (min_dis > cur_dis) {
      min_dis = cur_dis;
      pt = path_points_[cur_idx];
      idx_min = cur_idx;
    }
  };

  for (int i = 0; i < path_points_.size(); i += 3) {
    GetMinDis(i);
  }

  int idx_begin{std::max(0, idx_min - 2)};
  int idx_end{std::min(idx_min + 2, static_cast<int>(path_points_.size() - 1))};
  for (int i = idx_begin; i <= idx_end; i++) {
    GetMinDis(i);
  }

  return true;
}

bool DiscretizedPath::QueryClosestPointWithIndex(const Vec2d& pos,
                                                     PathPoint& pt,
                                                     int& index) const {
  if (path_points_.empty()) {
    LOG_INFO("path_points_.empty() Skip");
    return false;
  }
  double min_dis{std::numeric_limits<double>::infinity()};
  int idx_min{0};
  auto GetMinDis = [&](auto cur_idx) {
    double cur_dis = std::pow(pos.x() - path_points_[cur_idx].x(), 2) +
                     std::pow(pos.y() - path_points_[cur_idx].y(), 2);
    if (min_dis > cur_dis) {
      min_dis = cur_dis;
      pt = path_points_[cur_idx];
      idx_min = cur_idx;
    }
  };

  for (int i = 0; i < path_points_.size(); i += 3) {
    GetMinDis(i);
  }

  int idx_begin{std::max(0, idx_min - 2)};
  int idx_end{std::min(idx_min + 2, static_cast<int>(path_points_.size() - 1))};
  for (int i = idx_begin; i <= idx_end; i++) {
    GetMinDis(i);
  }
  index = idx_min;
  return true;
}

std::vector<PathPoint>* DiscretizedPath::mutable_path_points() {
  return &path_points_;
}

void DiscretizedPath::set_path_points(
    const std::vector<PathPoint>& path_points) {
  path_points_ = path_points;
}

const std::vector<PathPoint>& DiscretizedPath::path_points() const {
  return path_points_;
}

std::size_t DiscretizedPath::num_of_points() const {
  return path_points_.size();
}

bool DiscretizedPath::path_point_at(const std::size_t index,
                                    PathPoint& pt) const {
  if (path_points_.empty()) return false;

  if (index >= path_points_.size()) {
    pt = path_points_.back();
  } else {
    pt = path_points_[index];
  }
  return true;
}

bool DiscretizedPath::start_point(PathPoint& pt) const {
  if (path_points_.empty()) return false;

  pt = path_points_.front();
  return true;
}

bool DiscretizedPath::end_point(PathPoint& pt) const {
  if (path_points_.empty()) return false;

  pt = path_points_.back();
  return true;
}

std::vector<PathPoint>::const_iterator DiscretizedPath::query_lower_bound(
    const double param) const {
  auto func = [](const PathPoint& tp, const double param) {
    return tp.s() < param;
  };
  return std::lower_bound(path_points_.begin(), path_points_.end(), param,
                          func);
}

void DiscretizedPath::clear() { path_points_.clear(); }

}  // namespace planning
}  // namespace neodrive
