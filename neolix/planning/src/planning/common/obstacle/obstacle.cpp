#include "obstacle.h"

#include "common/math/util.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {
using neodrive::global::perception::PerceptionObstacle;
using neodrive::global::prediction::PredictionObstacle;

Obstacle::Obstacle(std::size_t his_size) : history_size_{his_size} {}

void Obstacle::copy_from(const PerceptionObstacle &obstacle) {
  set_state_components(obstacle.timestamp(), obstacle.velocity().x(),
                     obstacle.velocity().y(), obstacle.theta(),
                     obstacle.position().x(), obstacle.position().y());
  set_time_stamp(obstacle.timestamp());
  set_id(obstacle.id());
  set_height(obstacle.height());
  set_width(obstacle.width());
  set_length(obstacle.length());
  set_heading(obstacle.theta());
  set_virtual(false);
  set_speed(Vec2d(obstacle.velocity().x(), obstacle.velocity().y()).length());
  double vel_heading =
      std::atan2(obstacle.velocity().y(), obstacle.velocity().x());
  set_velocity_heading(vel_heading);
  LOG_DEBUG(
      "perception obs, velocity, heading, velocity_heading: [{}], {:.3f}, "
      "{:.3f}, {:.3f}, {:.3f}, {:.3f}",
      obstacle.id(), speed_, obstacle.theta(), vel_heading,
      obstacle.position().x(), obstacle.position().y());

  center_.set_x(obstacle.position().x());
  center_.set_y(obstacle.position().y());

  add_state({.position = center_, .velocity = speed_});

  init_ = false;
  switch (obstacle.type()) {
    case PerceptionObstacle::UNKNOWN_MOVABLE:
      set_type(ObstacleType::UNKNOWN_MOVABLE);
      break;
    case PerceptionObstacle::UNKNOWN_UNMOVABLE:
      set_type(ObstacleType::UNKNOWN_UNMOVABLE);
      break;
    case PerceptionObstacle::PEDESTRIAN:
      set_type(ObstacleType::PEDESTRIAN);
      break;
    case PerceptionObstacle::BICYCLE:
      set_type(ObstacleType::BICYCLE);
      break;
    case PerceptionObstacle::VEHICLE:
      set_type(ObstacleType::VEHICLE);
      break;
    default:
      set_type(ObstacleType::UNKNOWN);
      break;
  }

  set_sub_type(obstacle.sub_type());

  prediction_trajectories_.clear();

  if (obstacle.polygon_point().empty()) {
    LOG_WARN("obs[{}] polygon is empty, use box corners instead", id_);
    mutable_polygon()->Init(bounding_box());
  } else {
    std::vector<Vec2d> points;
    for (const auto &pt : obstacle.polygon_point()) {
      points.emplace_back(Vec2d{pt.x(), pt.y()});
    }
    mutable_polygon()->Init(points);
  }

  if (obstacle.has_boundary_std_deviation()) {
    boundary_std_deviation_ = obstacle.boundary_std_deviation();
  }
}

void Obstacle::ResetVec() {
  bounding_box_corners_.clear();
  bounding_box_corners_sl_.clear();
  polygon_corners_.clear();
  polygon_sl_.clear();
  local_polygon_.clear();
  boundary_.reset();
  polygon_boundary_.reset();
  boundary_with_shadow_.reset();
}

// get paras
int Obstacle::id() const { return id_; }

size_t Obstacle::seq_num() const { return seq_num_; }

double Obstacle::get_time_stamp() const { return time_stamp_; }

double Obstacle::height() const { return height_; }

double Obstacle::width() const { return width_; }

double Obstacle::length() const { return length_; }

double Obstacle::heading() const { return heading_; }

double Obstacle::velocity_heading() const { return velocity_heading_; }

const std::array<double, 10> &Obstacle::kinematics_state() const {
  return kinematics_;
}

double Obstacle::speed() const { return speed_; }

double Obstacle::max_history_speed() const { return max_history_speed_; }

double Obstacle::history_speed_heading() const {
  return history_speed_heading_;
}

bool Obstacle::is_virtual() const { return virtual_; }

bool Obstacle::is_init() const { return init_; }

bool Obstacle::is_static() const { return is_static_; }

bool Obstacle::is_avoidance() const { return avoidance_; }

const VirtualObstacle::VirtualType &Obstacle::virtual_type() const {
  return virtual_type_;
}

const Obstacle::ObstacleType &Obstacle::type() const { return type_; }

const PerceptionObstacle_SubType &Obstacle::sub_type() const {
  return sub_type_;
}

double Obstacle::max_s() const { return max_s_; }

double Obstacle::min_s() const { return min_s_; }

double Obstacle::max_l() const { return max_l_; }

double Obstacle::min_l() const { return min_l_; }

const Vec2d &Obstacle::center() const { return center_; }

const std::vector<PredictionTrajectory> &Obstacle::prediction_trajectories()
    const {
  return prediction_trajectories_;
}

std::vector<PredictionTrajectory> *Obstacle::mutable_prediction_trajectories() {
  return &prediction_trajectories_;
}

const PredictionTrajectory &Obstacle::uniform_trajectory() const {
  return uniform_trajectory_;
}

PredictionTrajectory *Obstacle::mutable_uniform_trajectory() {
  return &uniform_trajectory_;
}

const SLPoint &Obstacle::center_sl() const { return center_sl_; }

const ReferencePoint &Obstacle::obstacle_reference_point() const {
  return ref_point_;
}

const std::vector<Vec2d> &Obstacle::polygon_corners() const {
  return polygon_corners_;
}

const std::vector<SLPoint> &Obstacle::polygon_sl() const { return polygon_sl_; }

const std::vector<Vec2d> &Obstacle::local_polygon() const {
  return local_polygon_;
}

const Boundary &Obstacle::PolygonBoundary() const { return polygon_boundary_; }

const Boundary &Obstacle::boundary_with_shadow() const {
  return boundary_with_shadow_;
}

size_t Obstacle::decision_continuous_frame_cnt() const {
  return decision_continuous_frame_cnt_;
}

size_t Obstacle::frame_cnt() const { return frame_cnt_; }

Box2d Obstacle::bounding_box() const {
  return Box2d(center_, heading_, length_, width_);
}

// set paras
void Obstacle::set_id(const int id) { id_ = id; }

void Obstacle::set_seq_num(const size_t seq_num) { seq_num_ = seq_num; }

void Obstacle::set_time_stamp(double time_stamp) { time_stamp_ = time_stamp; }

void Obstacle::set_height(const double height) { height_ = height; }

void Obstacle::set_width(const double width) { width_ = width; }

void Obstacle::set_length(const double length) { length_ = length; }

void Obstacle::set_heading(const double heading) { heading_ = heading; }

void Obstacle::set_velocity_heading(const double velocity_heading) {
  velocity_heading_ = velocity_heading;
}

void Obstacle::set_state_components(const double time, const double velocity_x,
                                  const double velocity_y, const double theta,
                                  const double utm_x, const double utm_y) {
  double dt = std::abs(time - time_stamp_);
  LOG_DEBUG("{}: {},{},{}, ({},{})", id_, time_stamp_, time, dt, utm_x, utm_y);
  if (dt < 1e-4 || dt > 0.9999) {
    kinematics_[2] = 0;
    kinematics_[3] = 0;
    kinematics_[5] = 0;
  } else {
    kinematics_[2] = (velocity_x - kinematics_[0]) / dt;
    kinematics_[3] = (velocity_y - kinematics_[1]) / dt;
    kinematics_[5] = (theta - kinematics_[4]) / dt;
  }
  kinematics_[0] = velocity_x;
  kinematics_[1] = velocity_y;
  kinematics_[4] = theta;
  kinematics_[6] = time_stamp_;
  kinematics_[7] = dt;
  kinematics_[8] = utm_x;
  kinematics_[9] = utm_y;
}

void Obstacle::set_speed(const double speed) { speed_ = speed; }

void Obstacle::set_max_history_speed(const double max_speed) {
  max_history_speed_ = max_speed;
}

void Obstacle::set_history_speed_heading(const double heading) {
  history_speed_heading_ = heading;
}

void Obstacle::set_virtual(const bool is_virtual) { virtual_ = is_virtual; }

void Obstacle::set_is_static(const bool is_static) { is_static_ = is_static; }

void Obstacle::set_avoidance(const bool avoidance) { avoidance_ = avoidance; }

void Obstacle::set_virtual_type(
    const VirtualObstacle::VirtualType &virtual_type) {
  virtual_type_ = virtual_type;
}

void Obstacle::set_type(const ObstacleType &type) { type_ = type; }

void Obstacle::set_sub_type(const PerceptionObstacle_SubType &sub_type) {
  sub_type_ = sub_type;
}

void Obstacle::set_matched_lane_id(const uint64_t lane_id) {
  matched_lane_id_ = lane_id;
}

void Obstacle::set_matched_lane_heading_deviation(
    const double heading_deviation) {
  matched_lane_heading_deviation_ = heading_deviation;
}

void Obstacle::set_matched_lane_offset(const double offset) {
  matched_lane_offset_ = offset;
}

uint64_t Obstacle::matched_lane_id() const { return matched_lane_id_; }

double Obstacle::matched_lane_heading_deviation() const {
  return matched_lane_heading_deviation_;
}

double Obstacle::matched_lane_offset() const { return matched_lane_offset_; }

void Obstacle::set_max_s(const double max_s) { max_s_ = max_s; }

void Obstacle::set_min_s(const double min_s) { min_s_ = min_s; }

void Obstacle::set_max_l(const double max_l) { max_l_ = max_l; }

void Obstacle::set_min_l(const double min_l) { min_l_ = min_l; }

void Obstacle::set_center(const Vec2d &center) { center_ = center; }

void Obstacle::ClearPredictionTrajectory() {
  prediction_trajectories_.clear();
}

void Obstacle::AddPredictionTrajectory(
    const PredictionTrajectory &prediction_trajectory) {
  prediction_trajectories_.push_back(prediction_trajectory);
}

void Obstacle::set_local_polygon(const std::vector<Vec2d> &local_polygon) {
  local_polygon_ = local_polygon;
}

void Obstacle::set_decision_continuous_frame_cnt(
    const size_t decision_continuous_frame_cnt) {
  decision_continuous_frame_cnt_ = decision_continuous_frame_cnt;
}

void Obstacle::set_frame_cnt(const size_t frame_cnt) { frame_cnt_ = frame_cnt; }

//---------------functions--------------------------
void Obstacle::init_with_reference_line(
    const ReferenceLinePtr &reference_line) {
  ResetVec();
  if (!reference_line->GetPointInFrenetFrame(center_, &center_sl_)) {
    LOG_INFO(
        "obs center cannot project on refer_line, skip the obs, obs id: {},",
        id_);
    return;
  }
  LOG_INFO("init obs id:{}", id_);
  auto &points = polygon().points();
  for (auto &pt : points) {
    polygon_sl_.emplace_back();
    polygon_corners_.push_back(pt);
    // if (!reference_line->GetPointInFrenetFrame(pt, &polygon_sl_.back())) {
    if (!reference_line->GetObsCornerInFrenetFrame(pt, center_,
                                                   &polygon_sl_.back())) {
      LOG_WARN(
          "obs polygon cannot project on refer_line, skip the obs, obs "
          "id: {}, polygon size: {}",
          id_, points.size());
      return;
    }
    polygon_boundary_.set_start_s(
        std::min(polygon_sl_.back().s(), polygon_boundary_.start_s()));
    polygon_boundary_.set_end_s(
        std::max(polygon_sl_.back().s(), polygon_boundary_.end_s()));
    polygon_boundary_.set_start_l(
        std::min(polygon_sl_.back().l(), polygon_boundary_.start_l()));
    polygon_boundary_.set_end_l(
        std::max(polygon_sl_.back().l(), polygon_boundary_.end_l()));
  }

  boundary_ = polygon_boundary_;
  auto box = bounding_box();
  std::vector<Vec2d> corners;
  box.get_all_corners(&corners);
  for (auto &pt : corners) {
    bounding_box_corners_sl_.emplace_back();
    bounding_box_corners_.push_back(pt);
    if (!reference_line->GetObsCornerInFrenetFrame(
            pt, center_, &bounding_box_corners_sl_.back())) {
      LOG_WARN(
          "obs bound_box_corner cannot project on refer_line, skip the obs, "
          "obs id: {}, bound_box size: {}",
          id_, corners.size());
      return;
    }
    boundary_.set_start_s(
        std::min(bounding_box_corners_sl_.back().s(), boundary_.start_s()));
    boundary_.set_end_s(
        std::max(bounding_box_corners_sl_.back().s(), boundary_.end_s()));
    boundary_.set_start_l(
        std::min(bounding_box_corners_sl_.back().l(), boundary_.start_l()));
    boundary_.set_end_l(
        std::max(bounding_box_corners_sl_.back().l(), boundary_.end_l()));
  }

  max_l_ = boundary_.end_l();
  min_l_ = boundary_.start_l();
  max_s_ = boundary_.end_s();
  min_s_ = boundary_.start_s();

  is_static_ = prediction_trajectories_.empty();

  init_ = true;
}

void Obstacle::InitPolygonShadow(const ReferenceLinePtr &reference_line,
                                   const Vec2d &l_point,
                                   const double l_height) {
  auto s_points = polygon_corners_;
  if (height() < l_height) {
    for (auto &pt : polygon_corners_) {
      double x_s =
          (l_point.x() * height() - pt.x() * l_height) / (height() - l_height);
      double y_s =
          (l_point.y() * height() - pt.y() * l_height) / (height() - l_height);
      s_points.push_back({x_s, y_s});
    }
  } else {
    for (auto &pt : polygon_corners_) {
      double detal_x = pt.x() - l_point.x();
      double detal_y = pt.y() - l_point.y();
      double theta_s = std::atan2(detal_y, detal_x);
      double x_s = FLAGS_planning_blind_area_polygon_shadow_distance *
                       std::cos(theta_s) +
                   pt.x();
      double y_s = FLAGS_planning_blind_area_polygon_shadow_distance *
                       std::sin(theta_s) +
                   pt.y();
      s_points.push_back({x_s, y_s});
    }
  }

  if (!Polygon2d::compute_convex_hull(s_points,
                                      mutable_polygon_with_shadow())) {
    LOG_WARN("polygon is not convex, use default boundary, skip");
    boundary_with_shadow_ = boundary_;
    return;
  }
  auto &points = polygon_with_shadow().points();
  SLPoint sl_point;
  for (auto &pt : points) {
    if (!reference_line->GetPointInFrenetFrame(pt, &sl_point)) continue;
    boundary_with_shadow_.set_start_s(
        std::min(sl_point.s(), boundary_with_shadow_.start_s()));
    boundary_with_shadow_.set_end_s(
        std::max(sl_point.s(), boundary_with_shadow_.end_s()));
    boundary_with_shadow_.set_start_l(
        std::min(sl_point.l(), boundary_with_shadow_.start_l()));
    boundary_with_shadow_.set_end_l(
        std::max(sl_point.l(), boundary_with_shadow_.end_l()));
  }

  boundary_with_shadow_.set_start_s(
      std::min(boundary_.start_s(), boundary_with_shadow_.start_s()));
  boundary_with_shadow_.set_end_s(
      std::max(boundary_.end_s(), boundary_with_shadow_.end_s()));
  boundary_with_shadow_.set_start_l(
      std::min(boundary_.start_l(), boundary_with_shadow_.start_l()));
  boundary_with_shadow_.set_end_l(
      std::max(boundary_.end_l(), boundary_with_shadow_.end_l()));
}

void Obstacle::SmoothBoundaryWithHistory(
    const Obstacle &last_frame_obstacle_instance) {
  double start_l_diff = std::abs(
      last_frame_obstacle_instance.boundary_.start_l() - boundary_.start_l());
  if (start_l_diff < boundary_std_deviation_) {
    boundary_.set_start_l(last_frame_obstacle_instance.boundary_.start_l());
  }
  double end_l_diff = std::abs(last_frame_obstacle_instance.boundary_.end_l() -
                               boundary_.end_l());
  if (end_l_diff < boundary_std_deviation_) {
    boundary_.set_end_l(last_frame_obstacle_instance.boundary_.end_l());
  }
}

void Obstacle::adjust_timestamp_with(const double diff) {
  for (auto &prej : prediction_trajectories_) {
    prej.adjust_timestamp_with(diff);
  }
}

bool Obstacle::get_decision_by_traj_index(
    const int index, std::vector<Decision> *const traj_decision) const {
  auto &decision_vec = decision();
  bool has_decision = false;
  for (auto &decision : decision_vec) {
    if (decision.traj_index() == index) {
      traj_decision->push_back(decision);
      has_decision = true;
    }
  }
  return has_decision;
}

bool Obstacle::NeedToConsider(const double obstacle_s,
                                const double adc_stop_s,
                                const double heading_diff) const {
  constexpr double kObstacleCareDistance = 25.0;
  return !(obstacle_s < adc_stop_s || obstacle_s > kObstacleCareDistance ||
           std::fabs(heading_diff) < M_PI / 3.0 ||
           std::fabs(heading_diff) > M_PI * 2.0 / 3.0);
}

bool Obstacle::LikeToCollisionAdc(const double obstacle_s,
                                     const double obs_time_to_center,
                                     const double adc_time_to_collision) const {
  // not care obstacle if time to center > 10.0
  constexpr double kObstacleCareTimescope = 10.0;
  constexpr double kSplitDistance = 10.0;
  constexpr double kDeltaT1 = 3.0;
  constexpr double kDeltaT2 = 5.0;

  return !(obs_time_to_center > kObstacleCareTimescope ||
           ((obs_time_to_center > adc_time_to_collision + kDeltaT1) &&
            obstacle_s > kSplitDistance) ||
           ((obs_time_to_center < adc_time_to_collision - kDeltaT1) &&
            obstacle_s > kSplitDistance) ||
           ((obs_time_to_center > adc_time_to_collision + kDeltaT2) &&
            obstacle_s <= kSplitDistance) ||
           ((obs_time_to_center < adc_time_to_collision - kDeltaT2) &&
            obstacle_s <= kSplitDistance));
}

bool Obstacle::AwayFromReferenceLine(const ReferenceLinePtr &reference_line,
                                        const double start_l,
                                        const double end_l,
                                        const double lateral_speed) const {
  double center_l = (end_l + start_l) / 2.0;
  if (lateral_speed * center_l > 0.0) {
    LOG_INFO(
        "obstacle:{} away from reference line:"
        "speed {}, center_l {}",
        id_, lateral_speed, center_l);
    return true;
  }
  return false;
}

Json::Value Obstacle::to_json() const {
  Json::Value root;
  root = PlanningObject::to_json();
  root["id"] = id();
  root["height"] = height();
  root["width"] = width();
  root["length"] = length();
  root["heading"] = heading();
  root["speed"] = speed();
  root["virtual"] = is_virtual();
  root["init"] = is_init();
  root["center"]["x"] = center_.x();
  root["center"]["y"] = center_.y();
  root["center_sl"] = center_sl_.to_json();
  root["source_type"] = VirtualObstacle::VirtualType_Name(virtual_type());
  root["boundary"]["start_s"] = boundary_.start_s();
  root["boundary"]["end_s"] = boundary_.end_s();
  root["boundary"]["start_l"] = boundary_.start_l();
  root["boundary"]["end_l"] = boundary_.end_l();
  root["prediction_trajectory_size"] = prediction_trajectories_.size();
  root["polygon_size"] = polygon_sl().size();

  std::string type = "UNKNOWN";
  if (type_ == ObstacleType::VEHICLE) {
    type = "VEHICLE";
  } else if (type_ == ObstacleType::UNKNOWN_MOVABLE) {
    type = "UNKNOWN_MOVABLE";
  } else if (type_ == ObstacleType::UNKNOWN_UNMOVABLE) {
    type = "UNKNOWN_UNMOVABLE";
  } else if (type_ == ObstacleType::PEDESTRIAN) {
    type = "PEDESTRIAN";
  } else if (type_ == ObstacleType::BICYCLE) {
    type = "BICYCLE";
  }
  root["type"] = type;
  return root;
}

// -----------------functions to manipulate inner paras----------------
// !!!be careful to use them, unless you know what they are.

bool Obstacle::expand_bounding_box_corners(const double dis) {
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_.size() <= 3");
    return false;
  }
  bounding_box_corners_.clear();
  Box2d expand_box(center_, heading_, length_ + 2.0 * dis, width_ + 2.0 * dis);
  expand_box.get_all_corners(&bounding_box_corners_);
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("expand_bounding_box_corners fail!");
    return false;
  }
  return true;
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::right_expand_bounding_box_corners(const double veh_x,
                                                 const double veh_y,
                                                 const double veh_theta,
                                                 const double delta_theta,
                                                 const double dis) {
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : bounding_box_corners_) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_y_index = 0;
  size_t second_min_y_index = 0;
  double min_y = std::numeric_limits<double>::max();
  double second_min_y = std::numeric_limits<double>::max();

  for (size_t i = 0; i < local_pts.size(); ++i) {
    double y = local_pts[i].y();
    if (y < min_y) {
      second_min_y = min_y;
      second_min_y_index = min_y_index;
      min_y = y;
      min_y_index = i;
    } else if (y >= min_y && y < second_min_y) {
      second_min_y = y;
      second_min_y_index = i;
    }
  }

  // two min y pts right expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i == min_y_index || i == second_min_y_index) {
      double x = local_pts[i].x() - dis * sin(delta_theta);
      double y = local_pts[i].y() - dis * cos(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }
  // transfer into earth coordinate
  bounding_box_corners_.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    bounding_box_corners_.emplace_back(Vec2d{x_g, y_g});
  }
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("right_expand_bounding_box_corners fail!");
    return false;
  }
  return true;
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::left_expand_bounding_box_corners(const double veh_x,
                                                const double veh_y,
                                                const double veh_theta,
                                                const double delta_theta,
                                                const double dis) {
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : bounding_box_corners_) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_y_index = 0;
  size_t second_min_y_index = 0;
  double min_y = std::numeric_limits<double>::max();
  double second_min_y = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_pts.size(); ++i) {
    double y = local_pts[i].y();
    if (y < min_y) {
      second_min_y = min_y;
      second_min_y_index = min_y_index;
      min_y = y;
      min_y_index = i;
    } else if (y >= min_y && y < second_min_y) {
      second_min_y = y;
      second_min_y_index = i;
    }
  }

  // two max y pts left expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i != min_y_index && i != second_min_y_index) {
      double x = local_pts[i].x() + dis * sin(delta_theta);
      double y = local_pts[i].y() + dis * cos(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }
  // transfer into earth coordinate
  bounding_box_corners_.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    bounding_box_corners_.emplace_back(Vec2d{x_g, y_g});
  }
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("left_expand_bounding_box_corners fail!");
    return false;
  }
  return true;
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::front_expand_bounding_box_corners(const double veh_x,
                                                 const double veh_y,
                                                 const double veh_theta,
                                                 const double delta_theta,
                                                 const double dis) {
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : bounding_box_corners_) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_x_index = 0;
  size_t second_min_x_index = 0;
  double min_x = std::numeric_limits<double>::max();
  double second_min_x = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_pts.size(); ++i) {
    double x = local_pts[i].x();
    if (x < min_x) {
      second_min_x = min_x;
      second_min_x_index = min_x_index;
      min_x = x;
      min_x_index = i;
    } else if (x >= min_x && x < second_min_x) {
      second_min_x = x;
      second_min_x_index = i;
    }
  }
  // two max x pts front expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i != min_x_index && i != second_min_x_index) {
      double x = local_pts[i].x() + dis * cos(delta_theta);
      double y = local_pts[i].y() - dis * sin(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }
  // transfer into earth coordinate
  bounding_box_corners_.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    bounding_box_corners_.emplace_back(Vec2d{x_g, y_g});
  }
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("front_expand_bounding_box_corners fail!");
    return false;
  }
  return true;
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::back_expand_bounding_box_corners(const double veh_x,
                                                const double veh_y,
                                                const double veh_theta,
                                                const double delta_theta,
                                                const double dis) {
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : bounding_box_corners_) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_x_index = 0;
  size_t second_min_x_index = 0;
  double min_x = std::numeric_limits<double>::max();
  double second_min_x = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_pts.size(); ++i) {
    double x = local_pts[i].x();
    if (x < min_x) {
      second_min_x = min_x;
      second_min_x_index = min_x_index;
      min_x = x;
      min_x_index = i;
    } else if (x >= min_x && x < second_min_x) {
      second_min_x = x;
      second_min_x_index = i;
    }
  }
  // two min x pts back expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i == min_x_index || i == second_min_x_index) {
      double x = local_pts[i].x() - dis * cos(delta_theta);
      double y = local_pts[i].y() + dis * sin(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }

  // transfer into earth coordinate
  bounding_box_corners_.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    bounding_box_corners_.emplace_back(Vec2d{x_g, y_g});
  }
  if (bounding_box_corners_.size() <= 3) {
    LOG_ERROR("back_expand_bounding_box_corners fail!");
    return false;
  }
  return true;
}

bool Obstacle::expand_bounding_box_corners_sl(const double dis) {
  if (bounding_box_corners_sl_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  // lateral expand
  size_t max_l_index = 0;
  size_t second_max_l_index = 0;
  double max_l = std::numeric_limits<double>::lowest();
  double second_max_l = std::numeric_limits<double>::lowest();
  // longi expand
  size_t max_s_index = 0;
  size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double l = bounding_box_corners_sl_[i].l();
    if (l > max_l) {
      second_max_l = max_l;
      second_max_l_index = max_l_index;
      max_l = l;
      max_l_index = i;
    } else if (l <= max_l && l > second_max_l) {
      second_max_l = l;
      second_max_l_index = i;
    }

    double s = bounding_box_corners_sl_[i].s();
    if (s > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = s;
      max_s_index = i;
    } else if (s <= max_s && s > second_max_s) {
      second_max_s = s;
      second_max_s_index = i;
    }
  }

  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double l = bounding_box_corners_sl_[i].l();
    if (i == max_l_index || i == second_max_l_index) {
      bounding_box_corners_sl_[i].set_l(l + dis);
    } else {
      bounding_box_corners_sl_[i].set_l(l - dis);
    }

    double s = bounding_box_corners_sl_[i].s();
    if (i == max_s_index || i == second_max_s_index) {
      bounding_box_corners_sl_[i].set_s(s + dis);
    } else {
      bounding_box_corners_sl_[i].set_s(s - dis);
    }
  }
  return true;
}

bool Obstacle::right_expand_bounding_box_corners_sl(const double dis) {
  if (bounding_box_corners_sl_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  // lateral expand
  size_t max_l_index = 0;
  size_t second_max_l_index = 0;
  double max_l = std::numeric_limits<double>::lowest();
  double second_max_l = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double l = bounding_box_corners_sl_[i].l();
    if (l > max_l) {
      second_max_l = max_l;
      second_max_l_index = max_l_index;
      max_l = l;
      max_l_index = i;
    } else if (l <= max_l && l > second_max_l) {
      second_max_l = l;
      second_max_l_index = i;
    }
  }
  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double l = bounding_box_corners_sl_[i].l();
    if (i != max_l_index && i != second_max_l_index) {
      bounding_box_corners_sl_[i].set_l(l - dis);
    }
  }
  return true;
}

bool Obstacle::left_expand_bounding_box_corners_sl(const double dis) {
  if (bounding_box_corners_sl_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  // lateral expand
  size_t max_l_index = 0;
  size_t second_max_l_index = 0;
  double max_l = std::numeric_limits<double>::lowest();
  double second_max_l = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double l = bounding_box_corners_sl_[i].l();
    if (l > max_l) {
      second_max_l = max_l;
      second_max_l_index = max_l_index;
      max_l = l;
      max_l_index = i;
    } else if (l <= max_l && l > second_max_l) {
      second_max_l = l;
      second_max_l_index = i;
    }
  }
  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double l = bounding_box_corners_sl_[i].l();
    if (i == max_l_index || i == second_max_l_index) {
      bounding_box_corners_sl_[i].set_l(l + dis);
    }
  }
  return true;
}

bool Obstacle::front_expand_bounding_box_corners_sl(const double dis) {
  if (bounding_box_corners_sl_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  // longi expand
  size_t max_s_index = 0;
  size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double s = bounding_box_corners_sl_[i].s();
    if (s > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = s;
      max_s_index = i;
    } else if (s <= max_s && s > second_max_s) {
      second_max_s = s;
      second_max_s_index = i;
    }
  }

  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double s = bounding_box_corners_sl_[i].s();
    if (i == max_s_index || i == second_max_s_index) {
      bounding_box_corners_sl_[i].set_s(s + dis);
    }
  }
  return true;
}

bool Obstacle::back_expand_bounding_box_corners_sl(const double dis) {
  if (bounding_box_corners_sl_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  // longi expand
  size_t max_s_index = 0;
  size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double s = bounding_box_corners_sl_[i].s();
    if (s > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = s;
      max_s_index = i;
    } else if (s <= max_s && s > second_max_s) {
      second_max_s = s;
      second_max_s_index = i;
    }
  }

  for (size_t i = 0; i < bounding_box_corners_sl_.size(); ++i) {
    double s = bounding_box_corners_sl_[i].s();
    if (i != max_s_index && i != second_max_s_index) {
      bounding_box_corners_sl_[i].set_s(s - dis);
    }
  }
  return true;
}

bool Obstacle::expand_bounding_box_corners(const double dis,
                                           std::vector<Vec2d> &box_corners) {
  Box2d expand_box(center_, heading_, length_ + 2.0 * dis, width_ + 2.0 * dis);
  expand_box.get_all_corners(&box_corners);
  if (box_corners.size() <= 3) {
    LOG_ERROR("expand_bounding_box_corners fail!");
    return false;
  }
  return true;
}

bool Obstacle::expand_bounding_box_corners_xy_from_origin_info(
    const double dis, std::vector<Vec2d> &box_corners) {
  Box2d expand_box(center_, heading_, length_ + 2.0 * dis, width_ + 2.0 * dis);
  expand_box.get_all_corners(&box_corners);
  if (box_corners.size() <= 3) {
    LOG_ERROR("expand expand_bounding_box_corners_xy fail!");
    return false;
  }
  return true;
}

bool Obstacle::expand_bounding_box_corners_xy(const double veh_x,
                                              const double veh_y,
                                              const double veh_theta,
                                              const double delta_theta,
                                              const double dis,
                                              std::vector<Vec2d> &box_corners) {
  return box_corners.size() > 3 &&
         left_expand_bounding_box_corners_xy(veh_x, veh_y, veh_theta,
                                             delta_theta, dis, box_corners) &&
         right_expand_bounding_box_corners_xy(veh_x, veh_y, veh_theta,
                                              delta_theta, dis, box_corners) &&
         front_expand_bounding_box_corners_xy(veh_x, veh_y, veh_theta,
                                              delta_theta, dis, box_corners) &&
         back_expand_bounding_box_corners_xy(veh_x, veh_y, veh_theta,
                                             delta_theta, dis, box_corners);
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::right_expand_bounding_box_corners_xy(
    const double veh_x, const double veh_y, const double veh_theta,
    const double delta_theta, const double dis,
    std::vector<Vec2d> &box_corners) {
  if (box_corners.size() <= 3) {
    LOG_ERROR("box_corners.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : box_corners) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_y_index = 0;
  size_t second_min_y_index = 0;
  double min_y = std::numeric_limits<double>::max();
  double second_min_y = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_pts.size(); ++i) {
    double y = local_pts[i].y();
    if (y < min_y) {
      second_min_y = min_y;
      second_min_y_index = min_y_index;
      min_y = y;
      min_y_index = i;
    } else if (y >= min_y && y < second_min_y) {
      second_min_y = y;
      second_min_y_index = i;
    }
  }
  // two min y pts right expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i == min_y_index || i == second_min_y_index) {
      double x = local_pts[i].x() - dis * sin(delta_theta);
      double y = local_pts[i].y() - dis * cos(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }
  // transfer into earth coordinate
  box_corners.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    box_corners.emplace_back(Vec2d{x_g, y_g});
  }
  if (box_corners.size() <= 3) {
    LOG_ERROR("right_expand_bounding_box_corners_xy fail!");
    return false;
  }
  return true;
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::left_expand_bounding_box_corners_xy(
    const double veh_x, const double veh_y, const double veh_theta,
    const double delta_theta, const double dis,
    std::vector<Vec2d> &box_corners) {
  if (box_corners.size() <= 3) {
    LOG_ERROR("box_corners.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : box_corners) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_y_index = 0;
  size_t second_min_y_index = 0;
  double min_y = std::numeric_limits<double>::max();
  double second_min_y = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_pts.size(); ++i) {
    double y = local_pts[i].y();
    if (y < min_y) {
      second_min_y = min_y;
      second_min_y_index = min_y_index;
      min_y = y;
      min_y_index = i;
    } else if (y >= min_y && y < second_min_y) {
      second_min_y = y;
      second_min_y_index = i;
    }
  }
  // two max y pts left expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i != min_y_index && i != second_min_y_index) {
      double x = local_pts[i].x() + dis * sin(delta_theta);
      double y = local_pts[i].y() + dis * cos(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }
  // transfer into earth coordinate
  box_corners.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    box_corners.emplace_back(Vec2d{x_g, y_g});
  }
  if (box_corners.size() <= 3) {
    LOG_ERROR("left_expand_bounding_box_corners_xy fail!");
    return false;
  }
  return true;
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::front_expand_bounding_box_corners_xy(
    const double veh_x, const double veh_y, const double veh_theta,
    const double delta_theta, const double dis,
    std::vector<Vec2d> &box_corners) {
  if (box_corners.size() <= 3) {
    LOG_ERROR("box_corners.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : box_corners) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_x_index = 0;
  size_t second_min_x_index = 0;
  double min_x = std::numeric_limits<double>::max();
  double second_min_x = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_pts.size(); ++i) {
    double x = local_pts[i].x();
    if (x < min_x) {
      second_min_x = min_x;
      second_min_x_index = min_x_index;
      min_x = x;
      min_x_index = i;
    } else if (x >= min_x && x < second_min_x) {
      second_min_x = x;
      second_min_x_index = i;
    }
  }
  // two max x pts front expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i != min_x_index && i != second_min_x_index) {
      double x = local_pts[i].x() + dis * cos(delta_theta);
      double y = local_pts[i].y() - dis * sin(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }
  // transfer into earth coordinate
  box_corners.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    box_corners.emplace_back(Vec2d{x_g, y_g});
  }
  if (box_corners.size() <= 3) {
    LOG_ERROR("front_expand_bounding_box_corners_xy fail!");
    return false;
  }
  return true;
}

// delta_theta is vehicle and ref point's heading dif,
// note that veh local coor is front x left y.
bool Obstacle::back_expand_bounding_box_corners_xy(
    const double veh_x, const double veh_y, const double veh_theta,
    const double delta_theta, const double dis,
    std::vector<Vec2d> &box_corners) {
  if (box_corners.size() <= 3) {
    LOG_ERROR("box_corners.size() <= 3");
    return false;
  }
  // transfer into vehicle coorinates
  std::vector<Vec2d> local_pts;
  for (auto &pt : box_corners) {
    double loc_x = 0.0;
    double loc_y = 0.0;
    double loc_theta = 0.0;
    earth2vehicle(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, loc_x, loc_y,
                  loc_theta);
    local_pts.emplace_back(Vec2d{loc_x, loc_y});
  }
  // find two right pts / min y pts
  size_t min_x_index = 0;
  size_t second_min_x_index = 0;
  double min_x = std::numeric_limits<double>::max();
  double second_min_x = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_pts.size(); ++i) {
    double x = local_pts[i].x();
    if (x < min_x) {
      second_min_x = min_x;
      second_min_x_index = min_x_index;
      min_x = x;
      min_x_index = i;
    } else if (x >= min_x && x < second_min_x) {
      second_min_x = x;
      second_min_x_index = i;
    }
  }
  // two min x pts back expand dis in reference line direction
  for (size_t i = 0; i < local_pts.size(); ++i) {
    if (i == min_x_index || i == second_min_x_index) {
      double x = local_pts[i].x() - dis * cos(delta_theta);
      double y = local_pts[i].y() + dis * sin(delta_theta);
      local_pts[i].set_x(x);
      local_pts[i].set_y(y);
    }
  }

  // transfer into earth coordinate
  box_corners.clear();
  for (auto &pt : local_pts) {
    double x_g = 0.0;
    double y_g = 0.0;
    double theta_g = 0.0;
    vehicle2earth(veh_x, veh_y, veh_theta, pt.x(), pt.y(), 0.0, x_g, y_g,
                  theta_g);
    box_corners.emplace_back(Vec2d{x_g, y_g});
  }
  if (box_corners.size() <= 3) {
    LOG_ERROR("back_expand_bounding_box_corners_xy fail!");
    return false;
  }
  return true;
}

bool Obstacle::expand_bounding_box_corners_sl_from_origin_info(
    const double dis, std::vector<Vec2d> &box_corners_sl) {
  if (bounding_box_corners_sl_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  box_corners_sl.clear();
  for (const auto &pt : bounding_box_corners_sl_) {
    box_corners_sl.emplace_back(Vec2d{pt.s(), pt.l()});
  }

  // lateral expand
  size_t max_l_index = 0;
  size_t second_max_l_index = 0;
  double max_l = std::numeric_limits<double>::lowest();
  double second_max_l = std::numeric_limits<double>::lowest();
  // longi expand
  size_t max_s_index = 0;
  size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (l > max_l) {
      second_max_l = max_l;
      second_max_l_index = max_l_index;
      max_l = l;
      max_l_index = i;
    } else if (l <= max_l && l > second_max_l) {
      second_max_l = l;
      second_max_l_index = i;
    }

    double s = box_corners_sl[i].x();
    if (s > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = s;
      max_s_index = i;
    } else if (s <= max_s && s > second_max_s) {
      second_max_s = s;
      second_max_s_index = i;
    }
  }

  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (i == max_l_index || i == second_max_l_index) {
      box_corners_sl[i].set_y(l + dis);
    } else {
      box_corners_sl[i].set_y(l - dis);
    }

    double s = box_corners_sl[i].x();
    if (i == max_s_index || i == second_max_s_index) {
      box_corners_sl[i].set_x(s + dis);
    } else {
      box_corners_sl[i].set_x(s - dis);
    }
  }
  return true;
}

bool Obstacle::expand_bounding_box_corners_sl(
    const double dis, std::vector<Vec2d> &box_corners_sl) {
  if (box_corners_sl.size() <= 3) {
    LOG_ERROR("box_corners_sl.size() <= 3");
    return false;
  }
  // lateral expand
  size_t max_l_index = 0;
  size_t second_max_l_index = 0;
  double max_l = std::numeric_limits<double>::lowest();
  double second_max_l = std::numeric_limits<double>::lowest();
  // longi expand
  size_t max_s_index = 0;
  size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (l > max_l) {
      second_max_l = max_l;
      second_max_l_index = max_l_index;
      max_l = l;
      max_l_index = i;
    } else if (l <= max_l && l > second_max_l) {
      second_max_l = l;
      second_max_l_index = i;
    }

    double s = box_corners_sl[i].x();
    if (s > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = s;
      max_s_index = i;
    } else if (s <= max_s && s > second_max_s) {
      second_max_s = s;
      second_max_s_index = i;
    }
  }

  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (i == max_l_index || i == second_max_l_index) {
      box_corners_sl[i].set_y(l + dis);
    } else {
      box_corners_sl[i].set_y(l - dis);
    }

    double s = box_corners_sl[i].x();
    if (i == max_s_index || i == second_max_s_index) {
      box_corners_sl[i].set_x(s + dis);
    } else {
      box_corners_sl[i].set_x(s - dis);
    }
  }
  return true;
}

bool Obstacle::right_expand_bounding_box_corners_sl(
    const double dis, std::vector<Vec2d> &box_corners_sl) {
  if (box_corners_sl.size() <= 3) {
    LOG_ERROR("box_corners_sl.size() <= 3");
    return false;
  }
  // lateral expand
  size_t max_l_index = 0;
  size_t second_max_l_index = 0;
  double max_l = std::numeric_limits<double>::lowest();
  double second_max_l = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (l > max_l) {
      second_max_l = max_l;
      second_max_l_index = max_l_index;
      max_l = l;
      max_l_index = i;
    } else if (l <= max_l && l > second_max_l) {
      second_max_l = l;
      second_max_l_index = i;
    }
  }
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (i != max_l_index && i != second_max_l_index) {
      box_corners_sl[i].set_y(l - dis);
    }
  }
  return true;
}

bool Obstacle::left_expand_bounding_box_corners_sl(
    const double dis, std::vector<Vec2d> &box_corners_sl) {
  if (box_corners_sl.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  // lateral expand
  size_t max_l_index = 0;
  size_t second_max_l_index = 0;
  double max_l = std::numeric_limits<double>::lowest();
  double second_max_l = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (l > max_l) {
      second_max_l = max_l;
      second_max_l_index = max_l_index;
      max_l = l;
      max_l_index = i;
    } else if (l <= max_l && l > second_max_l) {
      second_max_l = l;
      second_max_l_index = i;
    }
  }
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double l = box_corners_sl[i].y();
    if (i == max_l_index || i == second_max_l_index) {
      box_corners_sl[i].set_y(l + dis);
    }
  }
  return true;
}

bool Obstacle::front_expand_bounding_box_corners_sl(
    const double dis, std::vector<Vec2d> &box_corners_sl) {
  if (box_corners_sl.size() <= 3) {
    LOG_ERROR("box_corners_sl.size() <= 3");
    return false;
  }
  // longi expand
  size_t max_s_index = 0;
  size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double s = box_corners_sl[i].x();
    if (s > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = s;
      max_s_index = i;
    } else if (s <= max_s && s > second_max_s) {
      second_max_s = s;
      second_max_s_index = i;
    }
  }
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double s = box_corners_sl[i].x();
    if (i == max_s_index || i == second_max_s_index) {
      box_corners_sl[i].set_x(s + dis);
    }
  }
  return true;
}

bool Obstacle::back_expand_bounding_box_corners_sl(
    const double dis, std::vector<Vec2d> &box_corners_sl) {
  if (bounding_box_corners_sl_.size() <= 3) {
    LOG_ERROR("bounding_box_corners_sl_.size() <= 3");
    return false;
  }
  // longi expand
  size_t max_s_index = 0;
  size_t second_max_s_index = 0;
  double max_s = std::numeric_limits<double>::lowest();
  double second_max_s = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double s = box_corners_sl[i].x();
    if (s > max_s) {
      second_max_s = max_s;
      second_max_s_index = max_s_index;
      max_s = s;
      max_s_index = i;
    } else if (s <= max_s && s > second_max_s) {
      second_max_s = s;
      second_max_s_index = i;
    }
  }
  for (size_t i = 0; i < box_corners_sl.size(); ++i) {
    double s = box_corners_sl[i].x();
    if (i != max_s_index && i != second_max_s_index) {
      box_corners_sl[i].set_x(s - dis);
    }
  }
  return true;
}

const std::deque<ObstacleState> &Obstacle::get_history_states() const {
  return history_;
}

void Obstacle::add_state(ObstacleState &&state) {
  history_.push_front(std::move(state));
  while (history_.size() > history_size_) {
    history_.pop_back();
  }
}

bool Obstacle::get_max_min_s_l_index_point_from_polygon(int &min_s_index,
                                                        int &max_s_index,
                                                        int &min_l_index,
                                                        int &max_l_index) {
  if (polygon_sl_.empty()) {
    LOG_INFO("polygon_sl_.empty()");
    return false;
  }
  double max_s = std::numeric_limits<double>::lowest();
  double max_l = std::numeric_limits<double>::lowest();
  double min_s = std::numeric_limits<double>::max();
  double min_l = std::numeric_limits<double>::max();
  for (size_t i = 0; i < polygon_sl_.size(); i++) {
    if (max_s < polygon_sl_[i].s()) {
      max_s = polygon_sl_[i].s();
      max_s_index = static_cast<int>(i);
    }
    if (max_l < polygon_sl_[i].l()) {
      max_l = polygon_sl_[i].l();
      max_l_index = static_cast<int>(i);
    }
    if (min_s > polygon_sl_[i].s()) {
      min_s = polygon_sl_[i].s();
      min_s_index = static_cast<int>(i);
    }
    if (min_l > polygon_sl_[i].l()) {
      min_l = polygon_sl_[i].l();
      min_l_index = static_cast<int>(i);
    }
  }
  min_s_index_ = min_s_index;
  max_s_index_ = max_s_index;
  min_l_index_ = min_l_index;
  max_l_index_ = max_l_index;
  return true;
}

}  // namespace planning
}  // namespace neodrive
