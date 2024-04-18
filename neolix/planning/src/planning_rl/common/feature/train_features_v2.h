#pragma once
#include <vector>
#include <map>
#include "container/container_manager.h"
#include "common/frame/traffic_light.h"
#include "common/feature/obs_representation_trt.h"
#include "common/feature/train_features.h"
#include "common/utils/obs_utils.h"
#include "common/utils/time_logger.h"
#include "neolix_log.h"
#include "utils/planning_rl_macros.h"

namespace neodrive {
namespace planning_rl {

using ContainerMessageShrPtr = std::shared_ptr<ContainerMessage>;

// struct EgoInfo {
//  public:
//   double x;
//   double y;
//   double heading;
//   double speed;
//   double lat_speed;
//   double lon_speed;
//   double acc;
//   double lon_acc;
//   double lat_acc;
//   double curvature;
//   double step_time;
//   double length;
//   double width;

//   EgoInfo(EgoFrame frame_ego, double start_point_curvature,
//           double frame_step_time) {
//     x = frame_ego.centroid[0];
//     y = frame_ego.centroid[1];
//     heading = frame_ego.yaw;
//     lon_speed = frame_ego.velocity[0];
//     lat_speed = frame_ego.velocity[1];
//     speed = sqrt(pow(lon_speed, 2) + pow(lat_speed, 2));
//     lon_acc = frame_ego.linearAcceleration[0];
//     lat_acc = frame_ego.linearAcceleration[1];
//     acc = sqrt(pow(lon_acc, 2) + pow(lat_acc, 2));
//     curvature = start_point_curvature;
//     step_time = frame_step_time;
//     length = frame_ego.extent[0];
//     width = frame_ego.extent[1];
//   }
//   EgoInfo(EgoFrame frame_ego) {
//     x = frame_ego.centroid[0];
//     y = frame_ego.centroid[1];
//     heading = frame_ego.yaw;
//     lon_speed = frame_ego.velocity[0];
//     lat_speed = frame_ego.velocity[1];
//     speed = sqrt(pow(lon_speed, 2) + pow(lat_speed, 2));
//     lon_acc = frame_ego.linearAcceleration[0];
//     lat_acc = frame_ego.linearAcceleration[1];
//     acc = sqrt(pow(lon_acc, 2) + pow(lat_acc, 2));
//     curvature = 0.0;
//     step_time = 0.0;
//     length = frame_ego.extent[0];
//     width = frame_ego.extent[1];
//   }
// };

class FeaturePreparationV2 {
 public:
  FeaturePreparationV2() {
  }

  void GetData(const std::vector<EgoFrame> &egoframes,
               const std::vector<std::vector<AgentFrame>> &agentsframes,
               const std::vector<std::vector<double>> &refline_input,
               const std::vector<double> &step_times,
               const std::vector<double> &refline_curvatures,
               const std::vector<double> &thetas);

  void GetData(const ContainerMessageShrPtr &container_msg,
                const std::vector<EgoFrame> &egoframes,
                const std::vector<std::vector<AgentFrame>> &agentsframes,
                const std::vector<std::vector<double>> &refline_input,
                const std::vector<double> &step_times,
                const std::vector<double> &refline_curvatures,
                const std::vector<double> &thetas);

  std::vector<std::vector<double>> GetAgentFeatureOriginal(std::vector<AgentFrame> &agents_frame);
  std::tuple<std::vector<std::vector<std::vector<double>>>, std::vector<std::vector<double>>> AlignAgents(std::vector<std::map<size_t, std::vector<double>>> agents_window);

  std::vector<double> GetEgoFeature(const EgoFrame &frame_ego);

  std::vector<std::vector<double>> GetReflineFeatureStrong(const std::vector<ReferencePoint> &refline_point100);

  void ConstructTypeMap();

  std::vector<std::vector<std::vector<double>>> GetObs(const int &frame_index, const EgoInfo &ego);
  std::vector<std::vector<std::vector<double>>> GetObsBeforeSafety(const EgoInfo &ego);
  std::vector<std::vector<std::vector<double>>> GetObsBeforeOriginalFrenetStrong(std::vector<ReferencePoint> nearest_100m_refpoint);
  std::vector<std::vector<std::vector<double>>> GetObsUrbanDriver(const int &frame_index);
  

  void GetAllData();

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(EgoFrame, ego_data);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<AgentFrame>, agents_data);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, step_time);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<EgoFrame>, ego_lists);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::vector<AgentFrame>>,
                                       agents_lists);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<ReferencePoint>,
                                       refline_points);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<double>, step_time_list);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<double>, refline_curvatures);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<double>, thetas);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, frame_num);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(int, result_limit);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::vector<double>>,
                                       agents_input);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::vector<double>>,
                                       ego_input);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::vector<double>>,
                                       refline_input);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::vector<double>>,
                                       ego_window);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(
      std::vector<std::vector<std::vector<double>>>, agents_window);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(
      std::vector<std::vector<std::vector<double>>>, refline_window);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::vector<double>>,
                                       reference_line_points_optj);
//   DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::map<size_t, std::vector<std::vector<double>>>>,
//                                        agents_window_dict);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<std::vector<double>>,
                                       agents_window_availability);

//   std::vector<std::map<size_t, std::vector<std::vector<double>>>> agents_window_dict() {
//     return agents_window_dict_;
//   }
//   std::vector<std::map<size_t, std::vector<std::vector<double>>>> agents_window_availability() {
//     return agents_window_availability_;
//   }

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(EgoInfo, ego_info);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(
      std::vector<std::vector<std::vector<double>>>, target_before);

 private:
  ContainerMessageShrPtr container_msg_;
  EgoFrame ego_data_;
  std::vector<AgentFrame> agents_data_;
  double step_time_;

  std::vector<EgoFrame> ego_lists_;
  std::vector<std::vector<AgentFrame>> agents_lists_;
  std::vector<ReferencePoint> refline_points_;
  std::vector<TraLight> traffic_lights_;
  std::vector<double> step_time_list_;
  std::vector<double> refline_curvatures_;
  std::vector<double> thetas_;
  int frame_num_;
  const int result_limit_ = 64;

  std::vector<std::vector<double>> agents_input_;
  std::vector<std::vector<double>> ego_input_;
  std::vector<std::vector<double>> refline_input_;

  std::vector<std::vector<double>> ego_window_;
  std::vector<std::vector<std::vector<double>>> agents_window_;
  std::vector<std::vector<std::vector<double>>> refline_window_;
  std::vector<std::vector<TraLight>> traffic_light_window_;

  std::vector<std::map<size_t, std::vector<double>>> agents_window_dict_;
  std::vector<std::vector<double>> agents_window_availability_;
  std::map<std::string, int> type_map_;

  std::vector<std::vector<double>> reference_line_points_optj_;

  EgoInfo ego_info_;

  utils::TimeLogger time_log_{"features_pl"};

  std::vector<std::vector<std::vector<double>>> target_before_;
};

int Testa();

int Tester();

}  // namespace planning_rl
}  // namespace neodrive