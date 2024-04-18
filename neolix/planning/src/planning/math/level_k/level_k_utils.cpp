#include "level_k_utils.h"
namespace neodrive {
namespace planning {
namespace level_k_utils {
Node* DeepCopy(const Node* node) {
  Node* copied_node = new Node();
  copied_node->data_ = node->data_;
  copied_node->father_ =
      (node->father_ != nullptr) ? DeepCopy(node->father_) : nullptr;
  return copied_node;
}

State StateTransform(const State& current_state, const double a,
                     const double omeca, const double delta_t) {
  State next_state = current_state;
  next_state.x =
      current_state.x + current_state.v * cos(current_state.theta) * delta_t;
  next_state.y =
      current_state.y + current_state.v * sin(current_state.theta) * delta_t;
  double v = std::max(current_state.v + a * delta_t, 0.0);
  next_state.v = std::min(10.0, v);

  next_state.theta = current_state.theta + omeca * delta_t;
  next_state.a = a;
  next_state.omeca = omeca;
  return next_state;
}

double Cal2DEuclideanDist(const std::tuple<double, double>& a,
                          const std::tuple<double, double>& b) {
  return std::hypot(std::get<0>(a) - std::get<0>(b),
                    std::get<1>(a) - std::get<1>(b));
}

double CalDeltaState(State& planning_state, State& cur_state) {
  return planning_state - cur_state;
}

void BfsBuild(Actions& actions, const std::string& vehicle_type, double& ttime,
              const double& delta_t,
              const std::vector<std::string>& action_types,
              std::vector<Node>& pending_current_nodes,
              std::vector<std::vector<Node>>& pending_nodes_records) {
  std::vector<Node> pending_next_nodes;
  for (auto& node : pending_current_nodes) {
    for (size_t i = 0; i < action_types.size(); i++) {
      float last_action_lat = node.data_.state.omeca;
      float last_action_lon = node.data_.state.a;
      std::tuple<double, double> action_value =
          actions.action_set[action_types[i]];
      State state_tmp =
          StateTransform(node.data_.state, std::get<0>(action_value),
                         std::get<1>(action_value), delta_t);
      if (!IsOverlap(vehicle_type, state_tmp)) {
        Node child(node.data_);
        child.data_.state = state_tmp;
        child.father_ = &node;
        node.insert(i, &child);
        pending_next_nodes.emplace_back(child);
      }
    }
  }
  ttime -= delta_t;
  if (!pending_next_nodes.empty() && ttime > 0) {
    BfsBuild(actions, vehicle_type, ttime, delta_t, action_types,
             pending_next_nodes, pending_nodes_records);
  }
  pending_nodes_records.emplace_back(pending_next_nodes);
}

bool IsOverlap(const std::string& vehicle_type, const State& state) {
  return false;
}

double CalOverlap(const std::vector<Vec2d>& vertice1,
                  const std::vector<Vec2d>& vertice2) {
  return 0.0;
}

bool CalCollision(const Vehicle& e, const std::vector<Vehicle>& as) {
  if (as.empty()) {
    return false;
  }
  auto trans_data = [](const Vehicle& v, TrajectoryData& d) {
    d.id = v.id;
    for (size_t i = 0; i < v.traj.size(); ++i) {
      d.timestamp.emplace_back(0.1 * i);
      d.position.emplace_back(math::AD2{v.traj[i].state.x, v.traj[i].state.y});
      d.heading.emplace_back(v.traj[i].state.theta);
      d.polygon.emplace_back(v.traj[i].polygon);
    }
  };
  std::vector<TrajectoryData> datas{};
  TrajectoryData ego_data;
  trans_data(e, ego_data);
  datas.emplace_back(ego_data);
  for (const auto& iter : as) {
    TrajectoryData agent_data;
    trans_data(iter, agent_data);
    datas.emplace_back(agent_data);
  }
  obsmap::TemporalCollision temporal_collision;
  return temporal_collision.Process(datas);
}

bool CalCollision(const std::vector<Vehicle>& traj_a,
                  const std::vector<Vehicle>& traj_b) {
  if (traj_a.empty() || traj_b.empty()) {
    return false;
  }

  auto trans_data = [](const std::vector<Vehicle>& traj, TrajectoryData& d) {
    d.id = traj.front().id;
    for (size_t i = 0; i < traj.size(); ++i) {
      d.timestamp.emplace_back(0.1 * i);
      d.position.emplace_back(math::AD2{traj[i].state.x, traj[i].state.y});
      d.heading.emplace_back(traj[i].state.theta);
      d.polygon.emplace_back(traj[i].polygon);
    }
  };

  TrajectoryData traj_da;
  trans_data(traj_a, traj_da);
  TrajectoryData traj_db;
  trans_data(traj_a, traj_db);
  std::vector<TrajectoryData> datas{traj_da, traj_db};

  obsmap::TemporalCollision temporal_collision;
  return temporal_collision.Process(datas);
}

double SearchNearestPtDist(const Vec2d& current_pt,
                           const std::vector<Vec2d>& all_pts) {
  std::vector<double> dist_list;
  for (const auto& pt : all_pts) {
    double distance = std::sqrt(std::pow(pt.x() - current_pt.x(), 2) +
                                std::pow(pt.y() - current_pt.y(), 2));
    dist_list.emplace_back(distance);
  }

  std::sort(dist_list.begin(), dist_list.end());

  return dist_list[0];
}

bool CompareByValue(const std::pair<int, double>& a,
                    const std::pair<int, double>& b) {
  return a.second > b.second;
}

}  // namespace level_k_utils
}  // namespace planning
}  // namespace neodrive