#include "path_goal_graph_search.h"

#include <algorithm>
#include <queue>
#include <string>

#include "common/data_center/data_center.h"
#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/config/auto_planning_research_config.h"

namespace neodrive {
namespace planning {

std::vector<PSS> PathGoalGraphSearch::ObstacleSamplePair(
    const std::vector<PathRegion::Bound>& bounds_info) {
  std::vector<PSS> obstacle_s_pairs{};
  std::vector<std::size_t> need_sample_index{};
  for (std::size_t index = 0; index < bounds_info.size(); ++index) {
    if (bounds_info[index].lower_id >= 0 || bounds_info[index].upper_id >= 0) {
      need_sample_index.emplace_back(index);
    }
  }
  std::sort(need_sample_index.begin(), need_sample_index.end());
  need_sample_index.erase(
      std::unique(need_sample_index.begin(), need_sample_index.end()),
      need_sample_index.end());
  if (need_sample_index.empty()) return obstacle_s_pairs;

  std::size_t front_index{need_sample_index.front()},
      back_index{need_sample_index.front()};
  obstacle_s_pairs.emplace_back(std::make_pair(front_index, back_index));
  for (std::size_t index = 1; index < need_sample_index.size(); ++index) {
    double width =
        std::abs(bounds_info[need_sample_index[index]].upper_point.l() -
                 bounds_info[need_sample_index[index]].lower_point.l());
    if (width > VehicleParam::Instance()->width() / 2) {
      if (need_sample_index[index] == obstacle_s_pairs.back().second + 1) {
        obstacle_s_pairs.back().second = need_sample_index[index];
      } else {
        obstacle_s_pairs.emplace_back(
            std::make_pair(need_sample_index[index], need_sample_index[index]));
      }
    } else {
      if (index == need_sample_index.size() - 1) break;
      double width_last = std::abs(
                 bounds_info[need_sample_index[index - 1]].upper_point.l() -
                 bounds_info[need_sample_index[index - 1]].lower_point.l()),
             width_next = std::abs(
                 bounds_info[need_sample_index[index + 1]].upper_point.l() -
                 bounds_info[need_sample_index[index + 1]].lower_point.l());
      LOG_DEBUG("graph search ind:{} width_last:{} width:{} width_next:{}",
                need_sample_index[index], width_last, width, width_next);
      if (width < width_last && width <= width_next) {
        obstacle_s_pairs.emplace_back(
            std::make_pair(need_sample_index[index], need_sample_index[index]));
      } else {
        obstacle_s_pairs.back().second = need_sample_index[index];
      }
    }
  }
  LOG_DEBUG("obstacle_s_pairs: ");
  for (auto pair : obstacle_s_pairs) {
    LOG_DEBUG("obs pair:{} - {}", pair.first, pair.second);
  }

  return obstacle_s_pairs;
}

std::vector<PSS> PathGoalGraphSearch::RoadSamplePair(
    const std::vector<PathRegion::Bound>& bounds_info,
    const std::vector<PSS>& obstacle_s_pairs) {
  std::vector<PSS> road_s_pair{};
  std::size_t start_index{0};
  std::size_t end_index{bounds_info.size() - 1};

  if (obstacle_s_pairs.empty()) {
    road_s_pair.emplace_back(std::make_pair(start_index, end_index));
  } else {
    auto add_cut = [](const std::vector<PathRegion::Bound>& bounds_info,
                      std::size_t start_index, std::size_t end_index,
                      std::vector<PSS>& road_s_pair, std::string type,
                      double threshold) {
      LOG_DEBUG("road type:{} start:{} end:{}", type, start_index, end_index);
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
            double ul = bounds_info[i].upper_point.l(),
                   ll = bounds_info[i].lower_point.l(),
                   ul_next = bounds_info[i + 1].upper_point.l(),
                   ll_next = bounds_info[i + 1].lower_point.l();
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
    if (start_index + 1 < obstacle_s_pairs.front().first &&
        obstacle_s_pairs.front().first >= 1) {
      road_s_pair.emplace_back(std::make_pair(
          obstacle_s_pairs.front().first - 1, obstacle_s_pairs.front().first));
      add_cut(bounds_info, start_index, obstacle_s_pairs.front().first - 1,
              road_s_pair, "ONLY_BEFORE_OBS", kDisThresh);
    }
    if (obstacle_s_pairs.back().second + 1 < end_index) {
      road_s_pair.emplace_back(std::make_pair(
          obstacle_s_pairs.back().second, obstacle_s_pairs.back().second + 1));
      add_cut(bounds_info, obstacle_s_pairs.back().second + 1, end_index,
              road_s_pair, "ONLY_AFTER_OBS", kDisThresh);
    }
    for (std::size_t index = 1; index < obstacle_s_pairs.size(); ++index) {
      if (obstacle_s_pairs[index - 1].second + 1 <
          obstacle_s_pairs[index].first - 1) {
        road_s_pair.emplace_back(
            std::make_pair(obstacle_s_pairs[index - 1].second,
                           obstacle_s_pairs[index - 1].second + 1));
        road_s_pair.emplace_back(std::make_pair(
            obstacle_s_pairs[index].first - 1, obstacle_s_pairs[index].first));
        add_cut(bounds_info, obstacle_s_pairs[index - 1].second + 1,
                obstacle_s_pairs[index].first - 1, road_s_pair, "ELSE",
                kDisThresh);
      }
    }
  }
  std::sort(road_s_pair.begin(), road_s_pair.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  LOG_DEBUG("road_s_pairs: ");
  for (auto pair : road_s_pair) {
    LOG_DEBUG("road pair:{} - {}", pair.first, pair.second);
  }

  return road_s_pair;
}

void PathGoalGraphSearch::SampleIntervals(
    const std::vector<PathRegion::Bound>& bounds_info,
    const std::vector<PSS>& sample_s_pair, const std::size_t add_index,
    std::vector<std::size_t>* sample_intervals) {
  for (const auto& [start_index, end_index] : sample_s_pair) {
    if (end_index - start_index < add_index) {
      sample_intervals->emplace_back(start_index);
      sample_intervals->emplace_back(end_index);
    } else {
      for (std::size_t index = start_index; index <= end_index;
           index += add_index) {
        sample_intervals->emplace_back(index);
      }
      if (!sample_intervals->empty() &&
          end_index - sample_intervals->back() > std::floor(add_index * 0.5)) {
        sample_intervals->emplace_back(end_index);
      }
    }
  }
}

std::vector<std::size_t> PathGoalGraphSearch::SampleIntervals(
    const std::vector<PathRegion::Bound>& bounds_info) {
  auto obs_sample_pairs = ObstacleSamplePair(bounds_info);
  auto road_sample_pairs = RoadSamplePair(bounds_info, obs_sample_pairs);

  std::vector<std::size_t> sample_intervals{};
  SampleIntervals(bounds_info, obs_sample_pairs,
                  path_goal_sl_config_.obs_add_index, &sample_intervals);
  SampleIntervals(bounds_info, road_sample_pairs,
                  path_goal_sl_config_.road_add_index, &sample_intervals);
  std::sort(sample_intervals.begin(), sample_intervals.end(),
            [](const auto& a, const auto& b) { return a < b; });
  sample_intervals.erase(
      std::unique(sample_intervals.begin(), sample_intervals.end()),
      sample_intervals.end());

  LOG_INFO("sample_intervals size: {}", sample_intervals.size());

  return sample_intervals;
}

void PathGoalGraphSearch::VisGraph(
    const neodrive::planning::ReferenceLinePtr reference_line,
    const std::vector<Node>& decision_nodes) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("path graph");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [&](const auto& pts, const auto& node) {
    auto sphere = event->mutable_sphere()->Add();
    sphere->mutable_center()->set_x(pts.x());
    sphere->mutable_center()->set_y(pts.y());
    sphere->mutable_center()->set_z(0);
    sphere->set_radius(0.1);
    auto text = event->mutable_text()->Add();
    text->mutable_position()->set_x(pts.x());
    text->mutable_position()->set_y(pts.y());
    text->mutable_position()->set_z(0);
    text->set_text(std::to_string(int(node.cost)));
  };

  for (const auto& node : decision_nodes) {
    Vec2d xy_point{};
    if (!reference_line->GetPointInCartesianFrame({node.s, node.l},
                                                  &xy_point)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    set_pts(xy_point, node);
  }
}

std::vector<PathGoalGraphSearch::Node> PathGoalGraphSearch::GenerateGraph(
    const std::vector<PathRegion::Bound>& bounds_info,
    const std::vector<std::size_t>& sample_intervals, const double init_s,
    const double init_l, const double observe_l) {
  std::vector<Node> ans{};
  if (bounds_info.empty() || sample_intervals.empty()) return ans;

  /// Generate node 0
  ans.push_back(Node{.index = 0, .s = init_s - 1, .l = init_l});

  /// Generate other layer nodes
  for (std::size_t i = 0; i < sample_intervals.size(); ++i) {
    auto& bound_info = bounds_info[sample_intervals[i]];
    auto s = bound_info.lower_point.s();
    auto ul = bound_info.upper_point.l();
    auto ll = bound_info.lower_point.l();
    auto u_type = bound_info.upper_type == PathRegion::Bound::BoundType::OBS
                      ? ObsTypeTrans(bound_info.upper_obs_type)
                      : static_cast<int>(bound_info.upper_type);
    auto l_type = bound_info.lower_type == PathRegion::Bound::BoundType::OBS
                      ? ObsTypeTrans(bound_info.lower_obs_type)
                      : static_cast<int>(bound_info.lower_type);
    double interval_l =
        std::max((ul - ll) / path_goal_sl_config_.max_lateral_sample_size,
                 static_cast<double>(path_goal_sl_config_.interval_l));
    for (double l = ll; l < ul; l += interval_l) {
      ans.push_back(Node{.index = static_cast<int>(ans.size()),
                         .s = s,
                         .l = l,
                         .u_l = ul,
                         .l_l = ll,
                         .u_type = u_type,
                         .l_type = l_type});
    }
    if (observe_l > ll && observe_l < ul) {
      ans.push_back(Node{.index = static_cast<int>(ans.size()),
                         .s = s,
                         .l = observe_l,
                         .u_l = ul,
                         .l_l = ll,
                         .u_type = u_type,
                         .l_type = l_type});
    }
    ans.push_back(Node{.index = static_cast<int>(ans.size()),
                       .s = s,
                       .l = ul,
                       .u_l = ul,
                       .l_l = ll,
                       .u_type = u_type,
                       .l_type = l_type});
  }

  /// Generate node end
  ans.push_back(Node{.index = static_cast<int>(ans.size()),
                     .s = ans.back().s + 1.0,
                     .l = init_l});

  /// Generate edges
  std::vector<int> layer_idxes{0};  // Start index of each layer
  for (std::size_t i = 1; i < ans.size(); ++i) {
    if (std::abs(ans[i].s - ans[i - 1].s) < 1e-4) continue;
    layer_idxes.push_back(i);
  }
  layer_idxes.push_back(ans.size());
  for (std::size_t i = 1; i + 1 < layer_idxes.size(); ++i) {
    int l = layer_idxes[i - 1], r = layer_idxes[i];
    for (int j = layer_idxes[i]; j < layer_idxes[i + 1]; ++j) {
      for (int k = l; k < r; ++k) ans[k].next_costs.insert({j, 0.});
    }
  }

  return ans;
}

double PathGoalGraphSearch::RefCost(const Node& node,
                                    const double observe_ref_l) {
  return path_goal_sl_config_.ref_cost * std::pow(node.l - observe_ref_l, 2);
}
double PathGoalGraphSearch::LateralNudgeCost(const Node& node) {
  bool is_upper_obs =
           IsBoundTypeEqual(node.u_type, PathRegion::Bound::BoundType::OBS),
       is_lower_obs =
           IsBoundTypeEqual(node.l_type, PathRegion::Bound::BoundType::OBS);
  bool is_upper_unmovable_obs =
           node.u_type == ObsTypeTrans(static_cast<int>(
                              Obstacle::ObstacleType::UNKNOWN_UNMOVABLE)),
       is_lower_unmovable_obs =
           node.l_type == ObsTypeTrans(static_cast<int>(
                              Obstacle::ObstacleType::UNKNOWN_UNMOVABLE));

  double obs_dis;
  const double& ratio = path_goal_sl_config_.lateral_dis_ratio;
  double lateral_static_obs_dis = path_goal_sl_config_.lateral_static_obs_dis;
  if (is_upper_obs && is_lower_obs) {
    double upper_obs_dis = std::abs(node.u_l - node.l);
    double lower_obs_dis = std::abs(node.l - node.l_l);
    obs_dis = std::min(upper_obs_dis, lower_obs_dis);
    if (is_upper_unmovable_obs && is_lower_unmovable_obs) {
      lateral_static_obs_dis *= ratio;
    } else if (is_upper_unmovable_obs) {
      lateral_static_obs_dis *= (upper_obs_dis < lower_obs_dis) ? ratio : 1.0;
    } else if (is_lower_unmovable_obs) {
      lateral_static_obs_dis *= (lower_obs_dis < upper_obs_dis) ? ratio : 1.0;
    } else {
      lateral_static_obs_dis *= 1.0;
    }
  } else if (is_upper_obs) {
    obs_dis = node.u_l - node.l;
    if (is_upper_unmovable_obs) {
      lateral_static_obs_dis *= ratio;
    }
  } else if (is_lower_obs) {
    obs_dis = node.l - node.l_l;
    if (is_lower_unmovable_obs) {
      lateral_static_obs_dis *= ratio;
    }
  } else {
    obs_dis = 2 * lateral_static_obs_dis;
  }
  return (obs_dis > lateral_static_obs_dis)
             ? 0.0
             : path_goal_sl_config_.lateral_static_obs_cost /
                   std::max(0.01, obs_dis);
}
double PathGoalGraphSearch::RoadCost(const Node& node) {
  double road_dis = 2 * path_goal_sl_config_.road_dis;
  if (IsBoundTypeEqual(node.u_type, PathRegion::Bound::BoundType::ROAD) &&
      IsBoundTypeEqual(node.l_type, PathRegion::Bound::BoundType::OBS)) {
    road_dis = node.u_l - node.l;
  }
  if (IsBoundTypeEqual(node.u_type, PathRegion::Bound::BoundType::OBS) &&
      IsBoundTypeEqual(node.l_type, PathRegion::Bound::BoundType::ROAD)) {
    road_dis = node.l - node.l_l;
  }
  road_dis = std::abs(road_dis);

  return (road_dis > path_goal_sl_config_.road_dis)
             ? 0.0
             : path_goal_sl_config_.road_cost / std::max(0.01, road_dis);
}
double PathGoalGraphSearch::LaneCost(const Node& node) {
  double lane_dis = 2 * path_goal_sl_config_.lane_dis;
  if (IsBoundTypeEqual(node.u_type, PathRegion::Bound::BoundType::LANE) &&
      !IsBoundTypeEqual(node.l_type, PathRegion::Bound::BoundType::LANE)) {
    lane_dis = node.u_l - node.l;
  }
  if (!IsBoundTypeEqual(node.u_type, PathRegion::Bound::BoundType::LANE) &&
      IsBoundTypeEqual(node.l_type, PathRegion::Bound::BoundType::LANE)) {
    lane_dis = node.l - node.l_l;
  }
  if (IsBoundTypeEqual(node.u_type, PathRegion::Bound::BoundType::LANE) &&
      IsBoundTypeEqual(node.l_type, PathRegion::Bound::BoundType::LANE)) {
    lane_dis = std::min(node.l - node.l_l, node.u_l - node.l);
  }
  lane_dis = std::abs(lane_dis);

  return (lane_dis > path_goal_sl_config_.lane_dis)
             ? 0.0
             : path_goal_sl_config_.lane_cost / std::max(0.01, lane_dis);
}
double PathGoalGraphSearch::LongitudinalNudgeCost(
    const Node& node, const std::vector<AABox2d>& static_obstacles,
    const double max_s) {
  auto longitudinal_dis = [&](const AABox2d& car_box, const AABox2d& obstacle) {
    double ans = 2 * (ego_length_ + path_goal_sl_config_.lon_static_obs_dis);
    if (obstacle.has_overlap(car_box)) {
      ans = 0.0;
    }
    return ans;
  };

  AABox2d car_box({node.s + VehicleParam::Instance()->length() * 0.5, node.l},
                  VehicleParam::Instance()->length() +
                      path_goal_sl_config_.car_box_lon_buffer,
                  VehicleParam::Instance()->width() +
                      path_goal_sl_config_.car_box_lateral_buffer);

  double min_dis = 2 * (ego_length_ + path_goal_sl_config_.lon_static_obs_dis);
  for (const auto& box : static_obstacles) {
    if (box.min_x() >= max_s || box.max_x() <= node.s) {
      continue;
    }
    auto dis = longitudinal_dis(car_box, box);
    min_dis = (min_dis > dis) ? dis : min_dis;
  }

  return min_dis > ego_length_ + path_goal_sl_config_.lon_static_obs_dis
             ? 0.0
             : path_goal_sl_config_.lon_static_obs_cost /
                   std::max(0.01, min_dis);
}

struct ObstacleCompare {
  bool operator()(const Obstacle& lhs, const Obstacle& rhs) const {
    return lhs.id() < rhs.id();
  }
};

void PathGoalGraphSearch::VisObstacles(
    const std::unordered_map<int, std::pair<Obstacle, int>>& map,
    const std::string& name) {
  if (!FLAGS_planning_enable_vis_event || map.empty()) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto& obstacle : map) {
    auto polygon = event->mutable_polygon()->Add();

    for (auto pt : obstacle.second.first.polygon_corners())
      set_pt(polygon->add_point(), pt);

    auto text = event->mutable_text()->Add();
    set_pt(text->mutable_position(), obstacle.second.first.center());
    text->set_text("id: " + std::to_string(obstacle.second.first.id()) + "\n" +
                   "count: " + std::to_string(obstacle.second.second));
  }
}

double PathGoalGraphSearch::DynamicObsCost(
    const Vec2d& pts, const std::unordered_map<int, std::pair<Obstacle, int>>&
                          dynamic_obstacles) {
  if (dynamic_obstacles.empty()) return 0.0;

  double cost = 0.0;
  for (const auto& obs : dynamic_obstacles) {
    const double x0 = pts.x() - obs.second.first.center().x();
    const double y0 = pts.y() - obs.second.first.center().y();
    const double dx = std::abs(x0 * std::cos(obs.second.first.heading()) +
                               y0 * std::sin(obs.second.first.heading()));
    const double dy = std::abs(x0 * std::sin(obs.second.first.heading()) -
                               y0 * std::cos(obs.second.first.heading()));

    if (dx < obs.second.first.length() / 2.0 &&
        dy <= obs.second.first.width() / 2.0) {
      cost += (path_goal_sl_config_.dynamic_obs_cost *
               (1.0 - std::pow(dy / (obs.second.first.width() / 2.0), 2))) *
              obs.second.second / path_goal_sl_config_.dynamic_obs_count;
    }
  }
  return cost;
}

double PathGoalGraphSearch::HistoryPathCost(const Node& node,
                                            const double init_s,
                                            const double max_s,
                                            const PathData* last_path_data) {
  if (last_path_data == nullptr) return 0.0;
  if (max_s <= init_s) return 0.0;
  if (node.s < init_s || node.s > max_s) return 0.0;

  double cost = 0.0;
  FrenetFramePoint proj_point{};
  if (last_path_data->frenet_path().interpolate(node.s, proj_point)) {
    cost = std::abs(path_goal_sl_config_.history_path_cost *
                    (node.l - proj_point.l()) * (max_s - node.s) /
                    (max_s - init_s));
  }
  return cost;
}

double PathGoalGraphSearch::SmoothCost(const Node& node, const Node& child) {
  return std::abs(path_goal_sl_config_.smooth_cost * (node.l - child.l) /
                  std::max(0.01, std::abs(node.s - child.s)));
}

void PathGoalGraphSearch::CalcCost(
    const neodrive::planning::ReferenceLinePtr reference_line,
    const double observe_ref_l, const double init_s, const double pre_s,
    const std::vector<AABox2d>& static_obstacles,
    const std::vector<Obstacle>& dynamic_obstacles,
    const PathData* last_path_data, std::vector<Node>& nodes) {
  double max_s = nodes[nodes.size() - 2].s;
  LOG_INFO("max_s: {:.3f}", max_s);
  auto obstacles = dynamic_obstacles;
  static std::unordered_map<int, std::pair<Obstacle, int>> obs_map;
  for (const auto& obs : obstacles) {
    auto it = obs_map.find(obs.id());
    if (it == obs_map.end()) {
      obs_map.insert(std::pair<int, std::pair<Obstacle, int>>(
          obs.id(), {obs, path_goal_sl_config_.dynamic_obs_init_cnt + 1}));
    } else {
      it->second.second = std::min(
          it->second.second + path_goal_sl_config_.dynamic_obs_reduce_cnt + 1,
          path_goal_sl_config_.dynamic_obs_count +
              path_goal_sl_config_.dynamic_obs_reduce_cnt);
      double k =
          std::max(static_cast<double>(
                       path_goal_sl_config_.dynamic_obs_lowpass_filter_ratio),
                   0.001);
      it->second.first.set_center(Vec2d{
          it->second.first.center().x() * (1 - k) + k * obs.center().x(),
          it->second.first.center().y() * (1 - k) + k * obs.center().y()});
      it->second.first.set_heading(it->second.first.heading() * (1 - k) +
                                   k * obs.heading());
      it->second.first.set_height(it->second.first.height() * (1 - k) +
                                  k * obs.height());
      it->second.first.set_width(it->second.first.width() * (1 - k) +
                                 k * obs.width());
      const auto& points = it->second.first.polygon().points();

      std::vector<Vec2d> points_pre{};
      for (std::size_t i = 0; i < 4; ++i) {
        points_pre.emplace_back(
            Vec2d{points[i].x() * (1 - k) + k * obs.polygon().points()[i].x(),
                  points[i].y() * (1 - k) + k * obs.polygon().points()[i].y()});
      }
      it->second.first.mutable_polygon()->Init(points_pre);
      it->second.first.init_with_reference_line(reference_line);
    }
  }

  for (auto obs = obs_map.begin(); obs != obs_map.end(); ++obs) {
    obs->second.second -= path_goal_sl_config_.dynamic_obs_reduce_cnt;
  }
  for (auto obs = obs_map.begin(); obs != obs_map.end();) {
    if (obs->second.second <= 0) {
      LOG_INFO("obs[{}] erase", obs->first);
      obs_map.erase(obs++);
    } else {
      ++obs;
    }
  }
  for (const auto& obs : obs_map) {
    LOG_INFO("obs_map obs[{}] count: {}", obs.first, obs.second.second);
  }

  VisObstacles(obs_map, "obs_map");

  for (auto& node : nodes) {
    double static_cost{0.};
    Vec2d pts;
    reference_line->GetPointInCartesianFrame({node.s, node.l}, &pts);
    /// refer cost
    static_cost += RefCost(node, observe_ref_l);
    /// lateral nudge cost
    static_cost += LateralNudgeCost(node);
    /// longitudinal nudge cost
    static_cost += LongitudinalNudgeCost(node, static_obstacles, max_s);
    /// road cost
    static_cost += RoadCost(node);
    /// lane cost
    static_cost += LaneCost(node);
    /// dynamic obs cost
    static_cost += DynamicObsCost(pts, obs_map);
    /// history cost
    if (obs_map.empty()) {
      static_cost += HistoryPathCost(node, init_s, pre_s, last_path_data);
    }
    node.cost = static_cost;
    /// smooth cost
    for (auto& [next, cost] : node.next_costs) {
      /// dummy node should be set cost == 0.0
      if (next + 1 == nodes.size()) {
        cost = 0.;
        continue;
      }
      if (node.index == 0) {
        cost = 0.;
        continue;
      }
      /// smooth cost
      auto& child = nodes[next];
      cost = static_cost + SmoothCost(node, child);
    }
  }
}

void PathGoalGraphSearch::ComputePath(const std::vector<Node>& nodes,
                                      const std::vector<int>& path, int x,
                                      std::vector<int>* path_indexes) {
  if (path[x] != -1) {
    ComputePath(nodes, path, path[x], path_indexes);
  }
  path_indexes->emplace_back(x);
}

std::vector<SLPoint> PathGoalGraphSearch::Dijkstra(std::vector<Node>& nodes,
                                                   int start, int end) {
  if (nodes.empty()) return {};

  using PDI = std::pair<double, int>;
  std::vector<double> dis(nodes.size(), 1e20);
  dis[start] = 1e-5;
  std::vector<int> pre(nodes.size(), -1);

  std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq{
      std::greater<PDI>()};
  pq.push({0., start});

  while (!pq.empty()) {
    auto [weight, node] = pq.top();
    pq.pop();

    for (const auto& [next_node, cost] : nodes[node].next_costs) {
      if (weight + cost < dis[next_node]) {
        dis[next_node] = cost + weight;
        pre[next_node] = node;
        pq.push({dis[next_node], next_node});
      }
    }
  }

  /// compute path
  std::vector<int> path_indexes{};
  ComputePath(nodes, pre, end, &path_indexes);
  std::vector<SLPoint> ans{};
  LOG_DEBUG("goal sl is:");
  for (const auto& index : path_indexes) {
    if (index == start || index == end) continue;
    ans.push_back(SLPoint(nodes[index].s, nodes[index].l));
    LOG_DEBUG("sl: {:.3f}, {:.3f}", ans.back().s(), ans.back().l());
  }

  return ans;
}

std::vector<SLPoint> PathGoalGraphSearch::ComputeBestGoalSL(
    const neodrive::planning::ReferenceLinePtr reference_line,
    const InsidePlannerData& inside_data,
    const std::vector<PathRegion::Bound>& bounds_info,
    const double observe_ref_l, const std::vector<AABox2d>& static_obstacles,
    const std::vector<Obstacle>& dynamic_obstacles,
    const PathData* last_path_data) {
  if (bounds_info.empty()) return {};

  /// update params
  ego_length_ = VehicleParam::Instance()->length();
  ego_half_width_ = VehicleParam::Instance()->width() * 0.5;

  /// Sample: road and obstacle
  auto sample_intervals = SampleIntervals(bounds_info);
  if (sample_intervals.empty()) return {};

  double pre_t = 3.0;
  Vec2d pre_point{
      inside_data.init_point.x() + inside_data.init_point.velocity() * pre_t *
                                       std::cos(inside_data.vel_heading),
      inside_data.init_point.y() + inside_data.init_point.velocity() * pre_t *
                                       std::sin(inside_data.vel_heading)};
  SLPoint sl_pt{};
  reference_line->GetPointInFrenetFrame(pre_point, &sl_pt);
  double pre_s = std::min(
      inside_data.init_sl_point.s() + path_goal_sl_config_.history_path_min_dis,
      sl_pt.s());

  /// Generate graph
  auto decision_nodes = GenerateGraph(
      bounds_info, sample_intervals, inside_data.init_sl_point.s(),
      inside_data.init_sl_point.l(), observe_ref_l);
  LOG_INFO("Searching sample nodes size: {}", decision_nodes.size());
  CalcCost(reference_line, observe_ref_l, inside_data.init_sl_point.s(), pre_s,
           static_obstacles, dynamic_obstacles, last_path_data, decision_nodes);

  VisGraph(reference_line, decision_nodes);
  /// Dijkstra search
  LOG_INFO("Starting Dijkstra searching.....");
  auto ans = Dijkstra(decision_nodes, 0, decision_nodes.size() - 1);
  LOG_INFO("End Dijkstra search!!!");
  if (ans.empty()) {
    return {};
  }

  return ans;
}

}  // namespace planning
}  // namespace neodrive