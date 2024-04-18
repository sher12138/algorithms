#pragma once
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include "common/model_trt/feature_net/featurenet_evaluator.h"
#include "common/utils/cout_sequence.h"
#include "common/utils/obs_utils.h"
#include "common/utils/time_logger.h"
#include "config/planning_rl_config.h"
#include "neolix_log.h"

namespace neodrive {
namespace planning_rl {

struct AgentData {
 public:
  std::vector<double> center_point;
  double heading;
  std::vector<double> speed;
  std::string agent_type;
  double agent_length;
  double agent_width;

  AgentData(std::vector<double> cp, double h, std::vector<double> s,
            std::string a_type, double l, double w) {
    center_point.assign(cp.begin(), cp.end());
    heading = h;
    speed.assign(s.begin(), s.end());
    agent_type = a_type;
    agent_length = l;
    agent_width = w;
  }
};

struct AgentDataFrenet {
 public:
  double lon_distance;
  double lat_distance;
  double lon_speed;
  double lat_speed;
  std::string agent_type;
  double agent_length;
  double agent_width;

  AgentDataFrenet() {}
  AgentDataFrenet(double lon_dis, double lat_dis, double lon_spe,
                  double lat_spe, std::string a_type, double a_length,
                  double a_width) {
    lon_distance = lon_dis;
    lat_distance = lat_dis;
    lon_speed = lon_spe;
    lat_speed = lat_spe;
    agent_type = a_type;
    agent_length = a_length;
    agent_width = a_width;
  }
};

struct ReferLineFrenet {
 public:
  double lon_dis;
  double lat_dis;
  double heading;
  double curvature;
};

struct LineData {
 public:
  std::vector<double> vec_x;
  std::vector<double> vec_y;

  LineData(std::vector<double> x_, std::vector<double> y_) {
    vec_x = x_;
    vec_y = y_;
  }
};

struct EgoFrame {
 public:
  std::vector<double> centroid{0, 0, 0};
  double yaw = 0.1;
  std::vector<double> velocity{1, 1};
  std::vector<double> linearAcceleration{1.5, 1.5};
  std::vector<double> extent{2, 2};
  EgoFrame() {}
  EgoFrame(const std::vector<double> &centroid, const double &yaw,
           const std::vector<double> &velocity,
           const std::vector<double> &linearAcceleration,
           const std::vector<double> &extent)
      : centroid(centroid),
        yaw(yaw),
        velocity(velocity),
        linearAcceleration(linearAcceleration),
        extent(extent) {}
};

struct AgentFrame {
 public:
  std::vector<double> centroid{0, 0, 0};
  double yaw = 0.1;
  std::vector<double> velocity{1, 1};
  std::vector<double> extent{1.5, 1.5};
  std::string type = "VEHICLE";
  size_t track_id = 0;
  AgentFrame() {}
  AgentFrame(const std::vector<double> &centroid, const double &yaw,
             const std::vector<double> &velocity,
             const std::vector<double> &extent, const std::string &type)
      : centroid(centroid),
        yaw(yaw),
        velocity(velocity),
        extent(extent),
        type(type){}
  AgentFrame(const std::vector<double> &centroid, const double &yaw,
             const std::vector<double> &velocity,
             const std::vector<double> &extent, const std::string &type, const size_t track_id)
      : centroid(centroid),
        yaw(yaw),
        velocity(velocity),
        extent(extent),
        type(type),
        track_id(track_id) {}
};

// class Point2D
// {
// public:
//     double x;
//     double y;
//     Point2D(double _x, double _y) {
//         x = _x;
//         y = _y;
//     }
// };

struct EgoSafeParas {
 public:
  std::vector<std::vector<std::vector<double>>> ego_safe_paras;
  std::vector<std::vector<double>> ego_agents_lon_dis_tensor;
  std::vector<std::vector<double>> ego_frenet_lon_speed;
  std::vector<std::vector<double>> ego_lat_dis;
  std::vector<std::vector<double>> ego_lon_dis;
  std::vector<std::vector<double>> ego_agents_lat_dis_tensor;
  std::vector<std::vector<double>> ego_frenet_lat_speed;
  std::vector<std::vector<double>> agents_frenet_lat_dis;
  std::vector<std::vector<double>> position_pts_in_world;
  std::vector<std::vector<double>> position_origins_in_world;
  int agents_data_len;
  AgentDataFrenet ego_data_frenet;
  EgoSafeParas(const AgentDataFrenet &ego_frenet, int agents_len) {
    ego_data_frenet = ego_frenet;
    agents_data_len = agents_len;
    ego_safe_paras = std::vector<std::vector<std::vector<double>>>(
        1, std::vector<std::vector<double>>(agents_data_len,
                                            std::vector<double>(16, 0.0)));
    ego_frenet_lon_speed = std::vector<std::vector<double>>(
        1, std::vector<double>(agents_data_len, 0.0));
    ego_frenet_lat_speed = std::vector<std::vector<double>>(
        1, std::vector<double>(agents_data_len, 0.0));
    ego_lat_dis = std::vector<std::vector<double>>(
        1, std::vector<double>(agents_data_len, 0.0));
    ego_lon_dis = std::vector<std::vector<double>>(
        1, std::vector<double>(agents_data_len, 0.0));
    ego_agents_lon_dis_tensor = std::vector<std::vector<double>>(
        1, std::vector<double>(agents_data_len, 0.0));
  }

  void set_ego_safe_paras();

  std::vector<std::vector<double>> UpdateLonDis(
      const std::vector<std::pair<int, double>> &ego_agents_dis,
      const std::vector<int> &agents_nearest_indexes);
  void UpdatePosWorld(const std::vector<AgentFrame> &agents_frame,
                      const std::vector<std::vector<double>> &refline_points,
                      const std::vector<int> &agents_nearest_indexes);
  std::vector<std::vector<double>> UpdateLatDis();
};

struct AgentSafeParas {
 public:
  std::vector<std::vector<std::vector<double>>> agents_safe_paras;
  std::vector<std::vector<double>> speed_origins_in_world;
  std::vector<std::vector<double>> speed_pts_in_world;
  std::vector<double> agents_frenet_lon_speed;
  std::vector<double> agents_frenet_lat_speed;
  std::vector<AgentFrame> agents_frame;
  std::vector<std::vector<std::vector<double>>> agents_data_tensor;
  int agents_data_len;

  AgentSafeParas(const std::vector<AgentFrame> &agents_data, int agents_len) {
    agents_frame = agents_data;
    agents_data_len = agents_len;
    agents_safe_paras = std::vector<std::vector<std::vector<double>>>(
        1, std::vector<std::vector<double>>(agents_data_len,
                                            std::vector<double>(16, 0.0)));
    agents_data_tensor = std::vector<std::vector<std::vector<double>>>(
        1, std::vector<std::vector<double>>(agents_data_len,
                                            std::vector<double>(3, 0.0)));
  }
  void UpdateSpeed(const std::vector<std::vector<double>> &refline_points,
                   const std::vector<int> &agents_nearest_indexes);
  void set_agent_safe_paras();
};

double GetEgoAgentDisFromCentralLine(const int &ego_index,
                                     const int &agent_index,
                                     const LineData &central_line);
double GetHeadingFromPoints(const std::pair<double, double> &point_1,
                            const std::pair<double, double> &point_2);
double GetCentralLineHeadingFromIndex(const int &agent_index,
                                      const LineData &central_line);
int FindLaneClosestPoint(const AgentData &agent_data,
                         const LineData &central_line);
std::vector<int> FindAgentsReferLineClosestPoint(
    const std::vector<std::vector<double>> &agents_points,
    const std::vector<std::vector<double>> &refline_points);
std::tuple<AgentDataFrenet, std::vector<double>> ConvertToFrenetCoordinateEgo(
    const AgentData &ego_data, const LineData &central_line);

std::vector<std::vector<double>> GenAgentsPoints(
    const std::vector<AgentFrame> &agents_frame);
std::vector<std::pair<int, double>> GetEgoAgentsDis(
    const int &ego_nereast_index,
    const std::vector<int> &agents_nearest_indexes,
    const LineData &central_line,
    const std::vector<std::vector<double>> &refline_points);

std::vector<std::vector<double>> GetAgentRepresentationTrt(
    const EgoFrame &ego_frame, std::vector<AgentFrame> &agents_frame,
    const std::vector<std::vector<double>> &refline_points);

// struct ObstacleSafePara {
//     // ego和agents计算安全距离时0-15对应的参数含义
//     double lon_speed;
//     double lon_acc;
//     double lat_speed;
//     double length;
//     double width;
//     double max_acc_lon;
//     double max_acc_lat;
//     double max_brake_lon;
//     double max_brake_lat;
//     double min_brake_lon;
//     double min_brake_lat;
//     double max_jerk;
//     double min_jerk;
//     double react_time;
//     double minimum_lon_dis;
//     double minimum_lat_dis;
// };

}  // namespace planning_rl
}  // namespace neodrive