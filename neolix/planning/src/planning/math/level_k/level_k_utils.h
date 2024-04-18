#include <cmath>

#include "context.h"
#include "src/planning/math/common/temporal_solver.h"
namespace neodrive {
namespace planning {
namespace level_k_utils {
using Vehicle = level_k_context::Vehicle;
using State = level_k_context::State;
using Actions = level_k_context::Actions;
using Weights = level_k_context::Weights;
using Believes = level_k_context::Believes;
using Node = level_k_context::Node;

using TrajectoryData = obsmap::TrajectoryData;

Node* DeepCopy(const Node* node);

State StateTransform(const State& current_state, const double a,
                     const double omeca, const double delta_t);

double Cal2DEuclideanDist(const std::tuple<double, double>& a,
                          const std::tuple<double, double>& b);

double CalDeltaState(State& planning_state, State& cur_state);

void BfsBuild(Actions& actions, const std::string& vehicle_type, double& ttime,
              const double& delta_t,
              const std::vector<std::string>& action_types,
              std::vector<Node>& pending_current_nodes,
              std::vector<std::vector<Node>>& pending_nodes_records);

bool IsOverlap(const std::string& vehicle_type, const State& state);

double CalOverlap(const std::vector<Vec2d>& vertice1,
                  const std::vector<Vec2d>& vertice2);

bool CalCollision(const Vehicle& e, const std::vector<Vehicle>& as);

bool CalCollision(const std::vector<Vehicle>& traj_a,
                  const std::vector<Vehicle>& traj_b);

double SearchNearestPtDist(const Vec2d& current_pt,
                           const std::vector<Vec2d>& all_pts);

bool CompareByValue(const std::pair<int, double>& a,
                    const std::pair<int, double>& b);
}  // namespace level_k_utils
}  // namespace planning
}  // namespace neodrive