#pragma once

#include "common/math/aabox2d.h"
#include "common/obstacle/obstacle.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/path/sl_point.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {
using PSS = std::pair<std::size_t, std::size_t>;

static constexpr int kObsBoundTypeBiasEnum = -10;

inline int ObsTypeTrans(int obs_type) {
  // obs type in perception transfer to bound type in goal graph search node
  int k = -1, b = kObsBoundTypeBiasEnum;
  return k * obs_type + b;
}
inline bool IsBoundTypeEqual(int bound_type,
                             PathRegion::Bound::BoundType type) {
  if (type == PathRegion::Bound::BoundType::OBS) {
    return bound_type <= kObsBoundTypeBiasEnum;
  } else {
    return bound_type == static_cast<int>(type);
  }
}

class PathGoalGraphSearch {
 public:
  std::vector<SLPoint> ComputeBestGoalSL(
      const neodrive::planning::ReferenceLinePtr reference_line,
      const InsidePlannerData& inside_data,
      const std::vector<PathRegion::Bound>& bounds_info,
      const double observe_ref_l, const std::vector<AABox2d>& static_obstacles,
      const std::vector<Obstacle>& dynamic_obstacles,
      const PathData* last_path_data);

 private:
  struct Node {
    int index{0};

    double s{0.};
    double l{0.};

    double u_l{0.};
    double l_l{0.};

    int u_type{-1};  // obs <= kObsBoundTypeBiasEnum, others >= 0
    int l_type{-1};  // obs <= kObsBoundTypeBiasEnum, others >= 0

    std::unordered_map<int, double>
        next_costs{};  // the index node's edge and cost
    double cost{0.};
  };
  std::vector<PSS> ObstacleSamplePair(
      const std::vector<PathRegion::Bound>& bounds_info);
  std::vector<PSS> RoadSamplePair(
      const std::vector<PathRegion::Bound>& bounds_info,
      const std::vector<PSS>& obstacle_s_pairs);
  void SampleIntervals(const std::vector<PathRegion::Bound>& bounds_info,
                       const std::vector<PSS>& sample_s_pair,
                       const std::size_t add_index,
                       std::vector<std::size_t>* sample_intervals);
  std::vector<std::size_t> SampleIntervals(
      const std::vector<PathRegion::Bound>& bounds_info);
  std::vector<Node> GenerateGraph(
      const std::vector<PathRegion::Bound>& bounds_info,
      const std::vector<std::size_t>& sample_intervals, const double init_s,
      const double init_l, const double observe_l);
  double RefCost(const Node& node, const double observe_ref_l);
  double LateralNudgeCost(const Node& node);
  double RoadCost(const Node& node);
  double LaneCost(const Node& node);
  double LongitudinalNudgeCost(const Node& node,
                               const std::vector<AABox2d>& static_obstacles,
                               const double max_s);
  double DynamicObsCost(const Vec2d& pts,
                        const std::unordered_map<int, std::pair<Obstacle, int>>&
                            dynamic_obstacles);
  double HistoryPathCost(const Node& node, const double init_s,
                         const double max_s, const PathData* last_path_data);
  double SmoothCost(const Node& node, const Node& child);
  void CalcCost(const neodrive::planning::ReferenceLinePtr reference_line,
                const double observe_ref_l, const double init_s,
                const double pre_s,
                const std::vector<AABox2d>& static_obstacles,
                const std::vector<Obstacle>& dynamic_obstacles,
                const PathData* last_path_data, std::vector<Node>& nodes);
  void ComputePath(const std::vector<Node>& nodes, const std::vector<int>& path,
                   int x, std::vector<int>* path_indexes);
  std::vector<SLPoint> Dijkstra(std::vector<Node>& nodes, int start, int end);
  void VisGraph(const neodrive::planning::ReferenceLinePtr reference_line,
                const std::vector<Node>& decision_nodes);
  void VisObstacles(
      const std::unordered_map<int, std::pair<Obstacle, int>>& map,
      const std::string& name);

 private:
  double ego_length_;
  double ego_half_width_;
  const config::AutoPlanningResearchConfig::PathRoadGraphConfig::PathGoalSl&
      path_goal_sl_config_ = config::PlanningConfig::Instance()
                                 ->planning_research_config()
                                 .path_road_graph_config.path_goal_sl;
};

}  // namespace planning
}  // namespace neodrive
