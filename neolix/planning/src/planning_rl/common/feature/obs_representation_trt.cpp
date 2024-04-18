#include "common/feature/obs_representation_trt.h"

namespace neodrive {
namespace planning_rl {

const double pi = acos(-1.0);

double GetEgoAgentDisFromCentralLine(const int &ego_index,
                                     const int &agent_index,
                                     const LineData &central_line) {
  double dis = 0;
  int max_index = std::max(ego_index, agent_index);
  int min_index = std::min(ego_index, agent_index);
  if (max_index == min_index) {
    return dis;
  } else {
    for (int i = min_index; i < max_index; i++) {
      double diff_x = central_line.vec_x[i + 1] - central_line.vec_x[i];
      double diff_y = central_line.vec_y[i + 1] - central_line.vec_y[i];
      dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) + dis;
    }
  }
  if (ego_index > agent_index) {
    dis = -dis;
  }
  return dis;
}

double GetHeadingFromPoints(const std::pair<double, double> &point_1,
                            const std::pair<double, double> &point_2) {
  double diff_x = point_2.first - point_1.first;
  double diff_y = point_2.second - point_1.second;
  return atan2(diff_y, diff_x);
}

double GetCentralLineHeadingFromIndex(const int &agent_index,
                                      const LineData &central_line) {
  int central_line_len = central_line.vec_x.size();
  std::pair<double, double> point_1;
  std::pair<double, double> point_2;
  if (agent_index == (central_line_len - 1)) {
    point_1 = std::make_pair(central_line.vec_x[agent_index - 1],
                             central_line.vec_y[agent_index - 1]);
    point_2 = std::make_pair(central_line.vec_x[agent_index],
                             central_line.vec_y[agent_index]);
  } else {
    point_1 = std::make_pair(central_line.vec_x[agent_index],
                             central_line.vec_y[agent_index]);
    point_2 = std::make_pair(central_line.vec_x[agent_index + 1],
                             central_line.vec_y[agent_index + 1]);
  }
  return GetHeadingFromPoints(point_1, point_2);
}

int FindLaneClosestPoint(const AgentData &agent_data,
                         const LineData &central_line) {
  int nearest_index = 0;
  double min_dis = std::numeric_limits<double>::max();
  for (int i = 0; i < int(central_line.vec_x.size()); i++) {
    double diff_x = agent_data.center_point[0] - central_line.vec_x[i];
    double diff_y = agent_data.center_point[1] - central_line.vec_y[i];
    double dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
    if (dis <= min_dis) {
      nearest_index = i;
      min_dis = dis;
    }
  }
  return nearest_index;
}

std::vector<int> FindAgentsReferLineClosestPoint(
    const std::vector<std::vector<double>> &agents_points,
    const std::vector<std::vector<double>> &refline_points) {
  utils::TimeLogger time_log_{"find_agents_ref_close_point_decc"};
  time_log_.ResetStartTime();
  time_log_.RegisterTimeAndPrint("find_agents_ref_close_point_decc_start");
  int refer_line_len = refline_points.size();
  int agents_len = agents_points.size();
  std::vector<int> argmin_dis;
  for (int j = 0; j < agents_len; j++) {
    int min_index = 0;
    double min_dis = std::numeric_limits<double>::max();
    for (int i = 0; i < refer_line_len; i++) {
      double diff_x = agents_points[j][0] - refline_points[i][0];
      double diff_y = agents_points[j][1] - refline_points[i][1];
      double dis_xy = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
      if (dis_xy < min_dis) {
        min_dis = dis_xy;
        min_index = i;
      }
    }
    argmin_dis.emplace_back(min_index);
  }
  time_log_.RegisterTimeAndPrint("compute_argmin");
  return argmin_dis;
}

std::tuple<AgentDataFrenet, std::vector<double>> ConvertToFrenetCoordinateEgo(
    const AgentData &ego_data, const LineData &central_line) {
  utils::TimeLogger time_log_{"cvtfce"};
  time_log_.ResetStartTime();
  int ego_nereast_index = FindLaneClosestPoint(ego_data, central_line);
  // time_log_.RegisterTimeAndPrint("cvtfce1");
  double ego_central_line_heading =
      GetCentralLineHeadingFromIndex(ego_nereast_index, central_line);
  // time_log_.RegisterTimeAndPrint("cvtfce2");
  ego_central_line_heading = NormalizeAngle(ego_central_line_heading);
  // time_log_.RegisterTimeAndPrint("cvtfce3");
  std::vector<double> origin_in_world = {central_line.vec_x[ego_nereast_index],
                                         central_line.vec_y[ego_nereast_index],
                                         ego_central_line_heading};
  std::vector<double> pt_in_world = ego_data.center_point;
  std::vector<double> lon_line_dis =
      ConvertToRelativeCoordinate2D(pt_in_world, origin_in_world);
  // time_log_.RegisterTimeAndPrint("cvtfce4");
  double ego_frenet_lon_dis = lon_line_dis[0];
  double ego_central_line_dis = lon_line_dis[1];
  std::vector<double> origin_speed_in_world = {0.0, 0.0,
                                               ego_central_line_heading};
  std::vector<double> pt_speed_in_world = {ego_data.speed[0],
                                           ego_data.speed[1]};
  std::vector<double> lon_line_speed =
      ConvertToRelativeCoordinate2D(pt_speed_in_world, origin_speed_in_world);
  // time_log_.RegisterTimeAndPrint("cvtfce5");
  double ego_lon_speed = lon_line_speed[0];
  double ego_lat_speed = lon_line_speed[1];
  AgentDataFrenet ego_data_frenet = AgentDataFrenet(
      ego_frenet_lon_dis, ego_central_line_dis, ego_lon_speed, ego_lat_speed,
      ego_data.agent_type, ego_data.agent_length, ego_data.agent_width);
  time_log_.RegisterTimeAndPrint("cvtfce6");
  return std::make_tuple(ego_data_frenet, origin_in_world);
}

std::vector<std::vector<double>> GenAgentsPoints(
    const std::vector<AgentFrame> &agents_frame) {
  int agents_data_len = agents_frame.size();
  if (agents_data_len < 1) {
    agents_data_len = 1;
  }
  std::vector<std::vector<double>> agents_points(agents_data_len,
                                                 std::vector<double>(4, 0.0));
  for (int i = 0; i < agents_data_len; i++) {
    agents_points[i][0] = agents_frame[i].centroid[0];
    agents_points[i][1] = agents_frame[i].centroid[1];
    agents_points[i][2] = agents_frame[i].velocity[0];
    agents_points[i][3] = agents_frame[i].velocity[1];
  }
  return agents_points;
}

std::vector<std::pair<int, double>> GetEgoAgentsDis(
    const int &ego_nereast_index,
    const std::vector<int> &agents_nearest_indexes,
    const LineData &central_line,
    const std::vector<std::vector<double>> &refline_points) {
  // int index_min = std::distance(agents_nearest_indexes.begin(),
  // std::min_element(agents_nearest_indexes.begin(),
  // agents_nearest_indexes.end())); int index_max =
  // std::distance(agents_nearest_indexes.begin(),
  // std::max_element(agents_nearest_indexes.begin(),
  // agents_nearest_indexes.end()));
  int index_min = *std::min_element(agents_nearest_indexes.begin(),
                                    agents_nearest_indexes.end());
  int index_max = *std::max_element(agents_nearest_indexes.begin(),
                                    agents_nearest_indexes.end());
  std::vector<std::pair<int, double>> ego_agents_dis;
  double ego_agents_min_dis =
      GetEgoAgentDisFromCentralLine(ego_nereast_index, index_min, central_line);
  double cum_dis = 0.0;
  int refer_line_len = refline_points.size();
  for (int i = index_min; i < (index_max + 1); i++) {
    if (i == (refer_line_len - 1)) {
      double temp_increment_dis = 0.0;
      cum_dis = cum_dis + temp_increment_dis;
      double temp_ego_agent_dis = ego_agents_min_dis + cum_dis;
      ego_agents_dis.emplace_back(std::make_pair(i, temp_ego_agent_dis));
    } else {
      double cur_diff_x = refline_points[i + 1][0] - refline_points[i][0];
      double cur_diff_y = refline_points[i + 1][1] - refline_points[i][1];
      double temp_increment_dis = sqrt(pow(cur_diff_x, 2) + pow(cur_diff_y, 2));
      cum_dis = cum_dis + temp_increment_dis;
      double temp_ego_agent_dis = ego_agents_min_dis + cum_dis;
      ego_agents_dis.emplace_back(std::make_pair(i, temp_ego_agent_dis));
    }
  }
  return ego_agents_dis;
}

void EgoSafeParas::set_ego_safe_paras() {
  auto &my_config =
      config::PlanningRLConfig::Instance()->config_ego_attribuate();
  for (int i = 0; i < agents_data_len; i++) {
    ego_frenet_lon_speed[0][i] = ego_data_frenet.lon_speed;
    ego_frenet_lat_speed[0][i] = ego_data_frenet.lat_speed;
  }
  for (int i = 0; i < agents_data_len; i++) {
    // 横纵向速度在feature-net里面赋值，这里给0.0
    // ego_safe_paras[0][i][0] = ego_data_frenet.lon_speed;
    // ego_safe_paras[0][i][2] = ego_data_frenet.lat_speed;
    ego_safe_paras[0][i][0] = 0.0;
    ego_safe_paras[0][i][2] = 0.0;
    ego_safe_paras[0][i][3] = ego_data_frenet.agent_length;
    ego_safe_paras[0][i][4] = ego_data_frenet.agent_width;
    ego_safe_paras[0][i][1] = my_config.ego.ego_acc;
    ego_safe_paras[0][i][5] = my_config.ego.ego_max_accel_lon;
    ego_safe_paras[0][i][6] = my_config.ego.ego_max_accel_lat;
    ego_safe_paras[0][i][7] = my_config.ego.ego_max_brake_lon;
    ego_safe_paras[0][i][8] = my_config.ego.ego_max_brake_lat;
    ego_safe_paras[0][i][9] = my_config.ego.ego_min_brake_lon;
    ego_safe_paras[0][i][10] = my_config.ego.ego_min_brake_lat;
    ego_safe_paras[0][i][11] = my_config.ego.ego_j_max;
    ego_safe_paras[0][i][12] = my_config.ego.ego_j_min;
    ego_safe_paras[0][i][13] = my_config.ego.ego_react_time;
    ego_safe_paras[0][i][14] = my_config.ego.min_lon_distance;
    ego_safe_paras[0][i][15] = my_config.ego.min_lat_distance;
  }
}

std::vector<std::vector<double>> EgoSafeParas::UpdateLonDis(
    const std::vector<std::pair<int, double>> &ego_agents_dis,
    const std::vector<int> &agents_nearest_indexes) {
  for (int i = 0; i < agents_data_len; i++) {
    auto key = agents_nearest_indexes[i];
    // 原本用映射，因为映射有连续性，这里0<->min_index
    key = key - ego_agents_dis[0].first;
    ego_agents_lon_dis_tensor[0][i] = ego_agents_dis[key].second;
    ego_lon_dis[0][i] = ego_data_frenet.lon_distance;
  }
  return ego_agents_lon_dis_tensor;
}

void EgoSafeParas::UpdatePosWorld(
    const std::vector<AgentFrame> &agents_frame,
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<int> &agents_nearest_indexes) {
  position_origins_in_world = std::vector<std::vector<double>>(
      agents_data_len, std::vector<double>(3, 0.0));
  position_pts_in_world = std::vector<std::vector<double>>(
      agents_data_len, std::vector<double>(2, 0.0));
  for (int i = 0; i < agents_data_len; i++) {
    auto key = agents_nearest_indexes[i];
    position_origins_in_world[i][0] = refline_points[key][0];
    position_origins_in_world[i][1] = refline_points[key][1];
    position_origins_in_world[i][2] = refline_points[key][2];
    position_pts_in_world[i][0] = agents_frame[i].centroid[0];
    position_pts_in_world[i][1] = agents_frame[i].centroid[1];
  }
}

std::vector<std::vector<double>> EgoSafeParas::UpdateLatDis() {
  ego_lat_dis = std::vector<std::vector<double>>(
      1, std::vector<double>(agents_data_len, 0.0));
  ego_agents_lat_dis_tensor = std::vector<std::vector<double>>(
      1, std::vector<double>(agents_data_len, 0.0));
  for (int i = 0; i < agents_data_len; i++) {
    ego_lat_dis[0][i] = ego_data_frenet.lat_distance;
    auto agent_frenet_dis = ConvertToRelativeCoordinate2D(
        position_pts_in_world[i], position_origins_in_world[i]);
    double ego_agents_lat_d = abs(ego_lat_dis[0][i] - agent_frenet_dis[1]);
    ego_agents_lat_dis_tensor[0][i] = ego_agents_lat_d;
  }
  return ego_agents_lat_dis_tensor;
}

void AgentSafeParas::UpdateSpeed(
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<int> &agents_nearest_indexes) {
  speed_origins_in_world = std::vector<std::vector<double>>(
      agents_data_len, std::vector<double>(3, 0.0));
  speed_pts_in_world = std::vector<std::vector<double>>(
      agents_data_len, std::vector<double>(2, 0.0));
  agents_frenet_lon_speed = std::vector<double>(agents_data_len, 0.0);
  agents_frenet_lat_speed = std::vector<double>(agents_data_len, 0.0);
  for (int i = 0; i < agents_data_len; i++) {
    auto key = agents_nearest_indexes[i];
    speed_origins_in_world[i][0] = 0.0;
    speed_origins_in_world[i][1] = 0.0;
    speed_origins_in_world[i][2] = refline_points[key][2];
    speed_pts_in_world[i][0] = agents_frame[i].velocity[0];
    speed_pts_in_world[i][1] = agents_frame[i].velocity[1];
    auto agent_frenet_speed = ConvertToRelativeCoordinate2D(
        speed_pts_in_world[i], speed_origins_in_world[i]);
    agents_frenet_lon_speed[i] = agent_frenet_speed[0];
    agents_frenet_lat_speed[i] = agent_frenet_speed[1];
  }
}

void AgentSafeParas::set_agent_safe_paras() {
  auto &my_config =
      config::PlanningRLConfig::Instance()->config_ego_attribuate();
  for (int i = 0; i < agents_data_len; i++) {
    // 横纵向速度在feature-net里面赋值，这里给0.0
    // agents_safe_paras[0][i][0] = agents_frenet_lon_speed[i];
    // agents_safe_paras[0][i][2] = agents_frenet_lat_speed[i];
    agents_safe_paras[0][i][0] = 0.0;
    agents_safe_paras[0][i][2] = 0.0;
    agents_safe_paras[0][i][3] = agents_frame[i].extent[0];
    agents_safe_paras[0][i][4] = agents_frame[i].extent[1];
    agents_data_tensor[0][i][0] = agents_frame[i].extent[0];
    agents_data_tensor[0][i][1] = agents_frame[i].extent[1];
    agents_data_tensor[0][i][2] = agents_frame[i].yaw;

    std::string a_type = agents_frame[i].type;
    if (a_type == "PEDESTRIAN") {
      agents_safe_paras[0][i][1] = my_config.agent_pedestrian.agent_acc;
      agents_safe_paras[0][i][5] =
          my_config.agent_pedestrian.agent_max_accel_lon;
      agents_safe_paras[0][i][6] =
          my_config.agent_pedestrian.agent_max_accel_lat;
      agents_safe_paras[0][i][7] =
          my_config.agent_pedestrian.agent_max_brake_lon;
      agents_safe_paras[0][i][8] =
          my_config.agent_pedestrian.agent_max_brake_lat;
      agents_safe_paras[0][i][9] =
          my_config.agent_pedestrian.agent_min_brake_lon;
      agents_safe_paras[0][i][10] =
          my_config.agent_pedestrian.agent_min_brake_lat;
      agents_safe_paras[0][i][11] = my_config.agent_pedestrian.agent_j_max;
      agents_safe_paras[0][i][12] = my_config.agent_pedestrian.agent_j_min;
      agents_safe_paras[0][i][13] = my_config.agent_pedestrian.agent_react_time;
      agents_safe_paras[0][i][14] = my_config.agent_pedestrian.min_lon_distance;
      agents_safe_paras[0][i][15] = my_config.agent_pedestrian.min_lat_distance;
    } else if (a_type == "BICYCLE") {
      agents_safe_paras[0][i][1] = my_config.agent_bicycle.agent_acc;
      agents_safe_paras[0][i][5] = my_config.agent_bicycle.agent_max_accel_lon;
      agents_safe_paras[0][i][6] = my_config.agent_bicycle.agent_max_accel_lat;
      agents_safe_paras[0][i][7] = my_config.agent_bicycle.agent_max_brake_lon;
      agents_safe_paras[0][i][8] = my_config.agent_bicycle.agent_max_brake_lat;
      agents_safe_paras[0][i][9] = my_config.agent_bicycle.agent_min_brake_lon;
      agents_safe_paras[0][i][10] = my_config.agent_bicycle.agent_min_brake_lat;
      agents_safe_paras[0][i][11] = my_config.agent_bicycle.agent_j_max;
      agents_safe_paras[0][i][12] = my_config.agent_bicycle.agent_j_min;
      agents_safe_paras[0][i][13] = my_config.agent_bicycle.agent_react_time;
      agents_safe_paras[0][i][14] = my_config.agent_bicycle.min_lon_distance;
      agents_safe_paras[0][i][15] = my_config.agent_bicycle.min_lat_distance;
    } else if (a_type == "VEHICLE") {
      agents_safe_paras[0][i][1] = my_config.agent_vehicle.agent_acc;
      agents_safe_paras[0][i][5] = my_config.agent_vehicle.agent_max_accel_lon;
      agents_safe_paras[0][i][6] = my_config.agent_vehicle.agent_max_accel_lat;
      agents_safe_paras[0][i][7] = my_config.agent_vehicle.agent_max_brake_lon;
      agents_safe_paras[0][i][8] = my_config.agent_vehicle.agent_max_brake_lat;
      agents_safe_paras[0][i][9] = my_config.agent_vehicle.agent_min_brake_lon;
      agents_safe_paras[0][i][10] = my_config.agent_vehicle.agent_min_brake_lat;
      agents_safe_paras[0][i][11] = my_config.agent_vehicle.agent_j_max;
      agents_safe_paras[0][i][12] = my_config.agent_vehicle.agent_j_min;
      agents_safe_paras[0][i][13] = my_config.agent_vehicle.agent_react_time;
      agents_safe_paras[0][i][14] = my_config.agent_vehicle.min_lon_distance;
      agents_safe_paras[0][i][15] = my_config.agent_vehicle.min_lat_distance;
    } else if (a_type == "UNKNOWN_UNMOVABLE") {
      agents_safe_paras[0][i][1] = my_config.agent_unmovable.agent_acc;
      agents_safe_paras[0][i][5] =
          my_config.agent_unmovable.agent_max_accel_lon;
      agents_safe_paras[0][i][6] =
          my_config.agent_unmovable.agent_max_accel_lat;
      agents_safe_paras[0][i][7] =
          my_config.agent_unmovable.agent_max_brake_lon;
      agents_safe_paras[0][i][8] =
          my_config.agent_unmovable.agent_max_brake_lat;
      agents_safe_paras[0][i][9] =
          my_config.agent_unmovable.agent_min_brake_lon;
      agents_safe_paras[0][i][10] =
          my_config.agent_unmovable.agent_min_brake_lat;
      agents_safe_paras[0][i][11] = my_config.agent_unmovable.agent_j_max;
      agents_safe_paras[0][i][12] = my_config.agent_unmovable.agent_j_min;
      agents_safe_paras[0][i][13] = my_config.agent_unmovable.agent_react_time;
      agents_safe_paras[0][i][14] = my_config.agent_unmovable.min_lon_distance;
      agents_safe_paras[0][i][15] = my_config.agent_unmovable.min_lat_distance;
    } else {
      agents_safe_paras[0][i][1] = my_config.agent_vehicle.agent_acc;
      agents_safe_paras[0][i][5] = my_config.agent_vehicle.agent_max_accel_lon;
      agents_safe_paras[0][i][6] = my_config.agent_vehicle.agent_max_accel_lat;
      agents_safe_paras[0][i][7] = my_config.agent_vehicle.agent_max_brake_lon;
      agents_safe_paras[0][i][8] = my_config.agent_vehicle.agent_max_brake_lat;
      agents_safe_paras[0][i][9] = my_config.agent_vehicle.agent_min_brake_lon;
      agents_safe_paras[0][i][10] = my_config.agent_vehicle.agent_min_brake_lat;
      agents_safe_paras[0][i][11] = my_config.agent_vehicle.agent_j_max;
      agents_safe_paras[0][i][12] = my_config.agent_vehicle.agent_j_min;
      agents_safe_paras[0][i][13] = my_config.agent_vehicle.agent_react_time;
      agents_safe_paras[0][i][14] = my_config.agent_vehicle.min_lon_distance;
      agents_safe_paras[0][i][15] = my_config.agent_vehicle.min_lat_distance;
    }
  }
}

std::vector<std::vector<double>> GetAgentRepresentationTrt(
    const EgoFrame &ego_frame, std::vector<AgentFrame> &agents_frame,
    const std::vector<std::vector<double>> &refline_points) {
  utils::TimeLogger time_log_{"obs_rep_tensorrt"};
  std::vector<double> ego_center_point(ego_frame.centroid.begin(),
                                       ego_frame.centroid.begin() + 2);
  double ego_heading = ego_frame.yaw;
  std::vector<double> ego_speed(ego_frame.velocity.begin(),
                                ego_frame.velocity.begin() + 2);
  std::string agent_type = "VEHICLE";
  double ego_length = ego_frame.extent[0];
  double ego_width = ego_frame.extent[1];
  AgentData ego_data = AgentData(ego_center_point, ego_heading, ego_speed,
                                 agent_type, ego_length, ego_width);
  int refline_points_len = refline_points.size();
  std::vector<double> central_line_x, central_line_y;
  for (int i = 0; i < refline_points_len; i++) {
    central_line_x.emplace_back(refline_points[i][0]);
    central_line_y.emplace_back(refline_points[i][1]);
  }
  LineData central_line = LineData(central_line_x, central_line_y);
  AgentDataFrenet ego_data_frenet;
  std::vector<double> refline_nearpoint;
  std::tie(ego_data_frenet, refline_nearpoint) =
      ConvertToFrenetCoordinateEgo(ego_data, central_line);
  int ego_nereast_index = FindLaneClosestPoint(ego_data, central_line);

  int agents_data_len = agents_frame.size();
  // 如果超过100，则截断。目前随机截断
  if (agents_data_len < 100) {
    auto frame_agent =
        AgentFrame(std::vector<double>{ego_frame.centroid[0] + 70,
                                       ego_frame.centroid[1] + 20, 0.0},
                   -2.9152791834789893, std::vector<double>{0.0, 0.0},
                   std::vector<double>{2.1735122203826904, 0.8976821899414062,
                                       1.732364296913147},
                   "VEHICLE");
    for (int i = agents_data_len; i < 100; i++) {
      agents_frame.emplace_back(frame_agent);
    }
  }
  if (agents_data_len > 100) {
    random_shuffle(agents_frame.begin(), agents_frame.end());
    agents_frame.resize(100);
  }
  // 经过补充和裁剪，强制将长度控制为100
  agents_data_len = 100;
  // agents_data_len = agents_frame.size();

  std::vector<std::vector<double>> agents_points =
      GenAgentsPoints(agents_frame);
  time_log_.RegisterTimeAndPrint("GenAgentsPoints");
  // 保证agents_nearest_indexes是1维
  std::vector<int> agents_nearest_indexes =
      FindAgentsReferLineClosestPoint(agents_points, refline_points);
  time_log_.RegisterTimeAndPrint("FindAgentsReferLineClosestPoint");
  // ego agent dis
  std::vector<std::pair<int, double>> ego_agents_dis = GetEgoAgentsDis(
      ego_nereast_index, agents_nearest_indexes, central_line, refline_points);
  auto &my_config =
      config::PlanningRLConfig::Instance()->config_ego_attribuate();
  time_log_.RegisterTimeAndPrint("GetEgoAgentsDis");
  EgoSafeParas esp = EgoSafeParas(ego_data_frenet, agents_data_len);
  esp.set_ego_safe_paras();
  esp.UpdateLonDis(ego_agents_dis, agents_nearest_indexes);
  esp.UpdatePosWorld(agents_frame, refline_points, agents_nearest_indexes);
  esp.UpdateLatDis();
  time_log_.RegisterTimeAndPrint("EgoSafeParas");

  AgentSafeParas asp = AgentSafeParas(agents_frame, agents_data_len);
  asp.UpdateSpeed(refline_points, agents_nearest_indexes);
  asp.set_agent_safe_paras();
  time_log_.RegisterTimeAndPrint("AgentSafeParas");

  float inputs0[1][100];
  float inputs1[1][100];
  float inputs2[1][100];
  float inputs3[100][3];
  float inputs4[100][2];
  float inputs5[100][3];
  float inputs6[100][2];
  float inputs7[1][100];
  float inputs8[1][100];
  float inputs9[1][100][16];
  float inputs10[1][100][16];
  float inputs11[1];
  float inputs12[1][100][3];

  for (int i = 0; i < agents_data_len; i++) {
    inputs0[0][i] = esp.ego_agents_lon_dis_tensor[0][i];
    inputs1[0][i] = esp.ego_lat_dis[0][i];
    inputs2[0][i] = esp.ego_lon_dis[0][i];
    inputs7[0][i] = esp.ego_frenet_lon_speed[0][i];
    inputs8[0][i] = esp.ego_frenet_lat_speed[0][i];
  }
  for (int i = 0; i < agents_data_len; i++) {
    for (int j = 0; j < 3; j++) {
      inputs3[i][j] = esp.position_origins_in_world[i][j];
      inputs5[i][j] = asp.speed_origins_in_world[i][j];
    }
  }
  for (int i = 0; i < agents_data_len; i++) {
    for (int j = 0; j < 2; j++) {
      inputs4[i][j] = esp.position_pts_in_world[i][j];
      inputs6[i][j] = asp.speed_pts_in_world[i][j];
    }
  }
  for (int i = 0; i < agents_data_len; i++) {
    for (int j = 0; j < 16; j++) {
      inputs9[0][i][j] = esp.ego_safe_paras[0][i][j];
      inputs10[0][i][j] = asp.agents_safe_paras[0][i][j];
    }
  }
  inputs11[0] = my_config.ego.min_lon_distance;
  for (int i = 0; i < agents_data_len; i++) {
    for (int j = 0; j < 3; j++) {
      inputs12[0][i][j] = asp.agents_data_tensor[0][i][j];
    }
  }
  time_log_.RegisterTimeAndPrint("prepare feature inputs");

  auto *featurenet_evaluator =
      neodrive::featurenet::FeatureNet_Evaluator::Instance();
  // auto featurenet_evaluator =
  // std::make_shared<neodrive::featurenet::FeatureNet_Evaluator>()::Instance()->engine();
  // std::cout << "Now start to use featurenet evaluator" << std::endl;
  // std::cout << "featurenet_input:" << std::endl;
  // std::cout << "inputs0" << std::endl;
  // std::cout << esp.ego_agents_lon_dis_tensor << std::endl;
  // std::cout << "inputs1" << std::endl;
  // std::cout << esp.ego_lat_dis << std::endl;
  // std::cout << "inputs2" << std::endl;
  // std::cout << esp.ego_lon_dis << std::endl;
  // std::cout << "inputs3" << std::endl;
  // std::cout << esp.position_origins_in_world << std::endl;
  // std::cout << "inputs4" << std::endl;
  // std::cout << esp.position_pts_in_world << std::endl;
  // std::cout << "inputs5" << std::endl;
  // std::cout << asp.speed_origins_in_world << std::endl;
  // std::cout << "inputs6" << std::endl;
  // std::cout << asp.speed_pts_in_world << std::endl;
  // std::cout << "inputs7" << std::endl;
  // std::cout << esp.ego_frenet_lon_speed << std::endl;
  // std::cout << "inputs8" << std::endl;
  // std::cout << esp.ego_frenet_lat_speed << std::endl;
  // std::cout << "inputs9" << std::endl;
  // std::cout << esp.ego_safe_paras << std::endl;
  // std::cout << "inputs10" << std::endl;
  // std::cout << asp.agents_safe_paras << std::endl;
  // std::cout << "inputs11" << std::endl;
  // std::cout << my_config.ego.min_lon_distance << std::endl;
  // std::cout << "inputs12" << std::endl;
  // std::cout << asp.agents_data_tensor << std::endl;

  auto output = featurenet_evaluator->Evaluate(
      inputs0, inputs1, inputs2, inputs3, inputs4, inputs5, inputs6, inputs7,
      inputs8, inputs9, inputs10, inputs11, inputs12);
  // std::cout << "featurenet_output: " << std::endl;
  // std::cout << output.size() << std::endl;
  // std::cout << output[0].size() << std::endl;
  // std::cout << output[0] << std::endl;
  // std::cout << output[1].size() << std::endl;
  // std::cout << output[1] << std::endl;
  // std::cout << output[2].size() << std::endl;
  // std::cout << output[2] << std::endl;
  // std::cout << output[3].size() << std::endl;
  // std::cout << output[3] << std::endl;
  // std::cout << output << std::endl;
  time_log_.RegisterTimeAndPrint("model feature evaluator");
  return output;
}

}  // namespace planning_rl
}  // namespace neodrive