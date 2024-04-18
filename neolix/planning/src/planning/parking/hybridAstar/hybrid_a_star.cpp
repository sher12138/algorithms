#include "hybrid_a_star.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

HybridAStar::HybridAStar(const ParkingHybridAStarConfig config,
                         const ParkingCommonConfig common_config) {
  config_ = config;
  common_config_ = common_config;
  reed_shepp_generator_.reset(new ReedShepp(config_));
  grid_a_star_heuristic_generator_.reset(new GridSearch(config_));

  next_node_num_ = config_.next_node_num_;
  //  max_steer_angle_ = VehicleParam::Instance()->max_steer_angle() /
  //                     VehicleParam::Instance()->steer_ratio() * 180.0 / M_PI;
  max_steer_angle_ = common_config_.max_steering_angle_;
  step_size_ = config_.step_size_;
  xy_grid_resolution_ = config_.xy_grid_resolution_;
  delta_t_ = common_config_.delta_t_;
  traj_forward_penalty_ = config_.traj_forward_penalty_;
  traj_back_penalty_ = config_.traj_back_penalty_;
  traj_gear_switch_penalty_ = config_.traj_gear_switch_penalty_;
  traj_steer_penalty_ = config_.traj_steer_penalty_;
  traj_steer_change_penalty_ = config_.traj_steer_change_penalty_;
}

bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result) {
  // clear containers
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;

  std::vector<std::vector<Segment2d>> obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    std::size_t vertices_num = obstacle_vertices.size();
    std::vector<Segment2d> obstacle_linesegments;
    for (std::size_t i = 0; i < vertices_num - 1; ++i) {
      Segment2d line_segment =
          Segment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

  // load XYbounds
  XYbounds_ = XYbounds;
  // load nodes and obstacles
  start_node_ = std::make_shared<Node3d>(sx, sy, sphi, XYbounds_, config_);
  end_node_ = std::make_shared<Node3d>(ex, ey, ephi, XYbounds_, config_);
  if (!ValidityCheck(start_node_)) {
    LOG_INFO("start_node in collision with obstacles");
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    LOG_INFO("end_node in collision with obstacles");
    return false;
  }

  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                  obstacles_linesegments_vec_);

  // load open set, pq
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

  // Hybrid A* begins
  std::size_t explored_node_num = 0;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    if (AnalyticExpansion(current_node)) {
      break;
    }
    close_set_.emplace(current_node->GetIndex(), current_node);
    for (std::size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num += 1;
        CalculateNodeCost(current_node, next_node);
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }
  LOG_INFO("explored node num is {}", explored_node_num);
  if (final_node_ == nullptr) {
    LOG_INFO("Hybrid A searching return null ptr(open_set ran out)");
    return false;
  }

  if (!RetrieveResult(result)) {
    LOG_INFO("GetResult failed");
    return false;
  }

  //  if (!GetResult(result)) {
  //    LOG_INFO("GetResult failed");
  //    return false;
  //  }

  return true;
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_check)) {
    LOG_INFO("ShortestRSP failed");
    return false;
  }

  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  LOG_INFO("Reach the end configuration with Reeds Shepp");
  // load the whole RSP as nodes and add to the close set
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  // vector x,y,phi
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(
      new Node3d(reeds_shepp_to_end->x, reeds_shepp_to_end->y,
                 reeds_shepp_to_end->phi, XYbounds_, config_));
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  if (node == nullptr) return false;
  if (node->GetStepSize() <= 0) return false;

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  std::size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  std::size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (std::size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      return false;
    }
    Box2d bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
        {traversed_x[i], traversed_y[i]}, traversed_phi[i]);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const Segment2d& linesegment : obstacle_linesegments) {
        if (bounding_box.has_overlap(linesegment)) {
          LOG_INFO(
              "collision detected s_x: {:.4f}, e_x: {:.4f}, s_y: {:.4f}, e_y: {:.4f}",
              linesegment.start().x(), linesegment.end().x(),
              linesegment.start().y(), linesegment.end().y());
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(
      new Node3d(reeds_shepp_to_end->x, reeds_shepp_to_end->y,
                 reeds_shepp_to_end->phi, XYbounds_, config_));
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, std::size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    std::size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (std::size_t i = 0; i < arc / step_size_; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = normalize_angle(
        last_phi + traveled_distance / VehicleParam::Instance()->wheel_base() *
                       std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  // check if the vehicle runs outside of XY boundary
  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(new Node3d(
      intermediate_x, intermediate_y, intermediate_phi, XYbounds_, config_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  // in the future, if add more complex heuristic cost,
  // add here, e.g. back penalty...
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::RetrieveResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      LOG_ERROR("result size check failed");
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      LOG_ERROR("states sizes are not equal");
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  // get drive_direction, segment, curv
  if (!GetdirectionSegment(result)) {
    LOG_ERROR("GetSpeedProfile from Hybrid Astar path fails");
    return false;
  }

  size_t x_size = result->x.size();
  if (x_size != result->y.size() || x_size != result->phi.size() ||
      x_size != result->steer.size() ||
      x_size != result->accumulated_s.size() ||
      x_size != result->direction.size() ||
      x_size != result->segment_index.size()) {
    LOG_ERROR(
        "state sizes not equal, x: {}, y: {}, phi: {}"
        "steer: {}, s: {}, direction: {}, index: {}",
        result->x.size(), result->y.size(), result->phi.size(),
        result->steer.size(), result->accumulated_s.size(),
        result->direction.size(), result->segment_index.size());
    return false;
  }
  // fake v and a
  for (size_t i = 0; i < x_size; ++i) {
    result->v.push_back(1.0);
    result->a.push_back(0.0);
  }

  return true;
}

bool HybridAStar::GetdirectionSegment(HybridAStartResult* result) {
  const auto& x = result->x;
  const auto& y = result->y;
  const auto& phi = result->phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    LOG_ERROR(
        "states sizes are not equal when do trajectory partitioning of "
        "Hybrid A Star result");
    return false;
  }
  // divide results to segments
  std::size_t horizon = x.size();
  std::vector<HybridAStartResult> partitioned_results;
  partitioned_results.clear();
  partitioned_results.emplace_back();
  auto* current_traj = &(partitioned_results.back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.angle();
  bool current_gear =
      std::abs(normalize_angle(tracking_angle - heading_angle)) < (M_PI_2);

  for (std::size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.angle();
    bool gear =
        std::abs(normalize_angle(tracking_angle - heading_angle)) < (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_results.emplace_back();
      current_traj = &(partitioned_results.back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  // Retrieve steer, direction, index from each segments
  int index_segment = 0;
  for (auto& tmp_result : partitioned_results) {
    // get gear info
    double init_heading = tmp_result.phi.front();
    const Vec2d init_tracking_vector(tmp_result.x[1] - tmp_result.x[0],
                                     tmp_result.y[1] - tmp_result.y[0]);
    const bool gear =
        std::abs(normalize_angle(init_heading - init_tracking_vector.angle())) <
        M_PI_2;

    // get path lengh
    std::size_t path_points_size = tmp_result.x.size();

    double accumulated_s = 0.0;
    int gear_int = gear ? 0 : 1;
    tmp_result.accumulated_s.clear();
    tmp_result.steer.clear();
    tmp_result.segment_index.clear();
    tmp_result.direction.clear();

    auto last_x = tmp_result.x.front();
    auto last_y = tmp_result.y.front();
    for (std::size_t i = 0; i < path_points_size; ++i) {
      double x_diff = tmp_result.x[i] - last_x;
      double y_diff = tmp_result.y[i] - last_y;
      accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
      tmp_result.accumulated_s.push_back(accumulated_s);
      tmp_result.segment_index.push_back(index_segment);
      tmp_result.direction.push_back(gear_int);
      last_x = tmp_result.x[i];
      last_y = tmp_result.y[i];
    }
    index_segment++;
    double discrete_steer = 0.0;
    for (std::size_t i = 0; i + 1 < path_points_size; ++i) {
      discrete_steer =
          (tmp_result.phi[i + 1] - tmp_result.phi[i]) *
          VehicleParam::Instance()->wheel_base() /
          (tmp_result.accumulated_s[i + 1] - tmp_result.accumulated_s[i]);
      discrete_steer =
          gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
      tmp_result.steer.push_back(discrete_steer);
    }
    if (tmp_result.steer.size() >= 1) {
      tmp_result.steer.push_back(tmp_result.steer.back());
    }
  }

  HybridAStartResult stitched_result;
  for (const auto& tmp_result : partitioned_results) {
    std::copy(tmp_result.x.begin(), tmp_result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(tmp_result.y.begin(), tmp_result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(tmp_result.phi.begin(), tmp_result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(tmp_result.steer.begin(), tmp_result.steer.end() - 1,
              std::back_inserter(stitched_result.steer));
    std::copy(tmp_result.accumulated_s.begin(),
              tmp_result.accumulated_s.end() - 1,
              std::back_inserter(stitched_result.accumulated_s));
    std::copy(tmp_result.direction.begin(), tmp_result.direction.end() - 1,
              std::back_inserter(stitched_result.direction));
    std::copy(tmp_result.segment_index.begin(),
              tmp_result.segment_index.end() - 1,
              std::back_inserter(stitched_result.segment_index));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.steer.push_back(partitioned_results.back().steer.back());
  stitched_result.accumulated_s.push_back(
      partitioned_results.back().accumulated_s.back());
  stitched_result.direction.push_back(
      partitioned_results.back().direction.back());
  stitched_result.segment_index.push_back(
      partitioned_results.back().segment_index.back());

  *result = stitched_result;
  return true;
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  size_t x_size = result.x.size();
  if (x_size != result.y.size() || x_size != result.phi.size() ||
      x_size != result.steer.size() || x_size != result.accumulated_s.size() ||
      x_size != result.direction.size() ||
      x_size != result.segment_index.size()) {
    LOG_ERROR(
        "state sizes not equal, x: {}, y: {}, phi: {}"
        "steer: {}, s: {}, direction: {}, index: {}",
        result.x.size(), result.y.size(), result.phi.size(),
        result.steer.size(), result.accumulated_s.size(),
        result.direction.size(), result.segment_index.size());
    return false;
  }
  if (x_size < 1) {
    LOG_ERROR("x_size < 1");
    return false;
  }
  partitioned_result->clear();
  HybridAStartResult tmp_result;
  int tmp_seg_index = result.segment_index.front();
  size_t i = 0;
  while (i < x_size) {
    tmp_result.x.clear();
    tmp_result.y.clear();
    tmp_result.phi.clear();
    tmp_result.steer.clear();
    tmp_result.accumulated_s.clear();
    tmp_result.direction.clear();
    tmp_result.segment_index.clear();

    tmp_result.v.clear();
    tmp_result.a.clear();
    size_t j = i;
    for (; j < x_size; ++j) {
      tmp_result.x.push_back(result.x[j]);
      tmp_result.y.push_back(result.y[j]);
      tmp_result.phi.push_back(result.phi[j]);
      tmp_result.steer.push_back(result.steer[j]);
      tmp_result.accumulated_s.push_back(result.accumulated_s[j]);
      tmp_result.direction.push_back(result.direction[j]);
      tmp_result.segment_index.push_back(result.segment_index[j]);

      tmp_result.v.push_back(result.v[j]);
      tmp_result.a.push_back(result.a[j]);
      if (tmp_seg_index != result.segment_index[j]) {
        // the final point direction, s, index, steer is not correct
        // but it does not matter
        partitioned_result->push_back(tmp_result);
        tmp_seg_index = result.segment_index[j];
        break;
      }
    }
    i = j;
  }
  tmp_result.segment_index.back() = tmp_seg_index;
  partitioned_result->push_back(tmp_result);

  return true;
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      LOG_ERROR("result size check failed");
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      LOG_ERROR("states sizes are not equal");
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  // generate speed
  if (!GetTemporalSpeedProfile(result)) {
    LOG_ERROR("GetSpeedProfile from Hybrid Astar path fails");
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    LOG_ERROR(
        "state sizes not equal, "
        "result->x.size(): {}, result->y.size(): {}, result->phi.size(): "
        "{},result->v.size(): {}",
        result->x.size(), result->y.size(), result->phi.size(),
        result->v.size());
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    LOG_ERROR(
        "control sizes not equal or not right,"
        "x size:  {}, acceleration size:  {}, steer size:  {}",
        result->x.size(), result->a.size(), result->steer.size());
    return false;
  }
  return true;
}
bool HybridAStar::GetTemporalSpeedProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  if (!TrajectoryPartitionInner(*result, &partitioned_results)) {
    LOG_ERROR("TrajectoryPartitionInner fail");
    return false;
  }
  HybridAStartResult stitched_result;
  for (const auto& result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  *result = stitched_result;
  return true;
}
bool HybridAStar::TrajectoryPartitionInner(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    LOG_ERROR(
        "states sizes are not equal when do trajectory partitioning of "
        "Hybrid A Star result");
    return false;
  }

  std::size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.angle();
  bool current_gear =
      std::abs(normalize_angle(tracking_angle - heading_angle)) < (M_PI_2);
  for (std::size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.angle();
    bool gear =
        std::abs(normalize_angle(tracking_angle - heading_angle)) < (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  // Retrieve v, a and steer from path
  for (auto& result : *partitioned_result) {
    if (config_.use_s_curve_speed_smooth__) {
      if (!GenerateSCurveSpeedAcceleration(&result)) {
        LOG_ERROR("GenerateSCurveSpeedAcceleration fail");
        return false;
      }
    } else {
      if (!GenerateSpeedAcceleration(&result)) {
        LOG_ERROR("GenerateSpeedAcceleration fail");
        return false;
      }
    }
  }
  return true;
}
bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  if (result == nullptr) return false;
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    LOG_ERROR("result size check when generating speed and acceleration fail");
    return false;
  }
  const std::size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  for (std::size_t i = 1; i + 1 < x_size; ++i) {
    //    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
    //                             std::cos(result->phi[i]) +
    //                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
    //                             std::cos(result->phi[i])) /
    //                            2.0 +
    //                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
    //                             std::sin(result->phi[i]) +
    //                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
    //                             std::sin(result->phi[i])) /
    //                            2.0;
    double discrete_v =
        0.5 * (((result->x[i + 1] - result->x[i - 1]) / delta_t_) *
               std::cos(result->phi[i])) +
        0.5 * (((result->y[i + 1] - result->y[i - 1]) / delta_t_) *
               std::sin(result->phi[i]));
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (std::size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  for (std::size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            VehicleParam::Instance()->wheel_base() / step_size_;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
  }
  return true;
}

bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStartResult* result) {
  // sanity check
  if (result == nullptr) return false;
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    LOG_ERROR("result size check when generating speed and acceleration fail");
    return false;
  }
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    LOG_ERROR("result sizes not equal");
    return false;
  }

  // get gear info
  double init_heading = result->phi.front();
  const Vec2d init_tracking_vector(result->x[1] - result->x[0],
                                   result->y[1] - result->y[0]);
  const bool gear =
      std::abs(normalize_angle(init_heading - init_tracking_vector.angle())) <
      M_PI_2;

  // get path lengh
  std::size_t path_points_size = result->x.size();

  double accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  for (std::size_t i = 0; i < path_points_size; ++i) {
    double x_diff = result->x[i] - last_x;
    double y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }
  // assume static initial state
  const double init_v = 0.0;
  const double init_a = 0.0;

  // minimum time speed optimization
  const double max_forward_v = 2.0;
  const double max_reverse_v = 1.0;
  const double max_forward_acc = 2.0;
  const double max_reverse_acc = 1.0;
  const double max_acc_jerk = 0.5;
  const double delta_t = 0.2;

  // TODO: explore better time horizon heuristic
  const double path_length = result->accumulated_s.back();
  const double total_t = std::max(gear ? 1.5 *
                                             (max_forward_v * max_forward_v +
                                              path_length * max_forward_acc) /
                                             (max_forward_acc * max_forward_v)
                                       : 1.5 *
                                             (max_reverse_v * max_reverse_v +
                                              path_length * max_reverse_acc) /
                                             (max_reverse_acc * max_reverse_v),
                                  10.0);

  const std::size_t num_of_knots =
      static_cast<std::size_t>(total_t / delta_t) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, 4000, delta_t, 0,
      {0.0, std::abs(init_v), std::abs(init_a)});

  // set end constraints
  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear ? max_forward_v : max_reverse_v;
  const double max_acc = gear ? max_forward_acc : max_reverse_acc;

  const auto upper_dx = std::fmax(max_v, std::abs(init_v));
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-max_acc, max_acc});

  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  // move to confs
  std::vector<double> x_ref(num_of_knots, path_length);
  piecewise_jerk_problem.set_x_ref(10000.0, x_ref);
  piecewise_jerk_problem.set_weight_ddx(10.0);
  piecewise_jerk_problem.set_weight_dddx(10.0);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  // solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    LOG_ERROR("Piecewise jerk speed optimizer failed!");
    return false;
  }

  // extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  // assign speed point by gear
  SpeedData speed_data;
  SpeedPoint tmp_speed_pt({s[0], 0.0}, ds[0], dds[0], 0.0);
  speed_data.mutable_speed_vector()->push_back(tmp_speed_pt);
  const double kEpislon = 1.0e-6;
  const double sEpislon = 1.0e-6;
  for (std::size_t i = 1; i < num_of_knots; ++i) {
    if (s[i - 1] - s[i] > kEpislon) {
      LOG_INFO(
          "unexpected decreasing s in speed smoothing at time {:.4f} with total "
          "time {:.3f}",
          static_cast<double>(i) * delta_t, total_t);
      break;
    }
    SpeedPoint tmp_speed_pt({s[i], delta_t * static_cast<double>(i)}, ds[i],
                            dds[i], (dds[i] - dds[i - 1]) / delta_t);
    speed_data.mutable_speed_vector()->push_back(tmp_speed_pt);
    // cut the speed data when it is about to meet end condition
    if (path_length - s[i] < sEpislon) {
      break;
    }
  }

  // combine speed and path profile
  DiscretizedPath path_data;
  std::vector<PathPoint> path_pts;
  for (std::size_t i = 0; i < path_points_size; ++i) {
    PathPoint path_point;
    path_point.set_x(result->x[i]);
    path_point.set_y(result->y[i]);
    path_point.set_theta(result->phi[i]);
    path_point.set_s(result->accumulated_s[i]);
    path_pts.push_back(path_point);
  }
  path_data.set_path_points(path_pts);

  HybridAStartResult combined_result;

  // move to confs
  const double kDenseTimeResoltuion = 0.5;
  const double time_horizon =
      speed_data.total_time() + kDenseTimeResoltuion * 1.0e-6;
  if (path_data.path_points().empty()) {
    LOG_ERROR("path data is empty");
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
       cur_rel_time += kDenseTimeResoltuion) {
    SpeedPoint speed_point;
    if (!speed_data.get_speed_point_with_time(cur_rel_time, &speed_point)) {
      LOG_ERROR("Fail to get speed point with relative time {:.4f}",
                cur_rel_time);
      return false;
    }

    if (speed_point.s() > path_data.param_length()) {
      break;
    }

    PathPoint path_point;
    path_data.evaluate(speed_point.s(), path_point);

    combined_result.x.push_back(path_point.x());
    combined_result.y.push_back(path_point.y());
    combined_result.phi.push_back(path_point.theta());
    combined_result.accumulated_s.push_back(path_point.s());
    if (!gear) {
      combined_result.v.push_back(-speed_point.v());
      combined_result.a.push_back(-speed_point.a());
    } else {
      combined_result.v.push_back(speed_point.v());
      combined_result.a.push_back(speed_point.a());
    }
  }

  combined_result.a.pop_back();

  // recalc step size
  path_points_size = combined_result.x.size();

  // load steering from phi
  for (std::size_t i = 0; i + 1 < path_points_size; ++i) {
    double discrete_steer =
        (combined_result.phi[i + 1] - combined_result.phi[i]) *
        VehicleParam::Instance()->wheel_base() /
        (combined_result.accumulated_s[i + 1] -
         combined_result.accumulated_s[i]);
    discrete_steer =
        gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
    combined_result.steer.push_back(discrete_steer);
  }

  *result = combined_result;
  return true;
}

}  // namespace planning
}  // namespace neodrive
