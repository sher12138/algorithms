#include "data_reader.h"
#include "common/feature/train_features.h"
#include "common/utils/obs_utils.h"

namespace neodrive {
namespace planning_rl {

int DataJsonReader(Json::Value &root) {
  std::ifstream in(
      "/home/caros/planning/unit_test/planning_rl_test/config/demo.json",
      std::ios::binary);
  Json::Reader reader;
  if (!in.is_open()) {
    LOG_WARN("file not open");
    return 1;
  } else {
    LOG_DEBUG("file open");
  }
  if (reader.parse(in, root)) {
    LOG_DEBUG("file parse success");
  } else {
    LOG_WARN("file parse fail");
    return 1;
  }
  in.close();
  return 0;
}

int UnitTestDataJsonReader(Json::Value &root) {
  std::ifstream in(
      "/home/caros/planning/unit_test/planning_rl_test/config/test_data.json",
      std::ios::binary);
  Json::Reader reader;
  if (!in.is_open()) {
    LOG_WARN("file not open");
    return 1;
  } else {
    LOG_DEBUG("file open");
  }
  if (reader.parse(in, root)) {
    LOG_DEBUG("file parse success");
  } else {
    LOG_WARN("file parse fail");
    return 1;
  }
  in.close();
  return 0;
}

int ModelConfReader(Json::Value &root) {
  std::ifstream in(
      "/home/caros/planning/src/planning_rl/conf/"
      "model_config.json",
      std::ios::binary);
  Json::Reader reader;
  if (!in.is_open()) {
    LOG_WARN("model config not open");
    return 1;
  } else {
    LOG_DEBUG("model config open");
  }
  if (reader.parse(in, root)) {
    LOG_DEBUG("model config parse success");
  } else {
    LOG_WARN("model config parse fail");
    return 1;
  }
  in.close();
  return 0;
}

int PostPredictConfReader(Json::Value &root) {
  std::ifstream in(
      "/home/caros/planning/src/planning_rl/conf/"
      "post_predict.json",
      std::ios::binary);
  Json::Reader reader;
  if (!in.is_open()) {
    LOG_WARN("post predict config not open");
    return 1;
  } else {
    LOG_DEBUG("post predict config open");
  }
  if (reader.parse(in, root)) {
    LOG_DEBUG("post predict config parse success");
  } else {
    LOG_WARN("post predict config parse fail");
    return 1;
  }
  in.close();
  return 0;
}

FeaturePreparation JsonDataParser(Json::Value &root) {
  // int frame_num = root.size();
  // if (frame_num < 5) {
  //     return 1;
  // }
  std::cout << "start_json_data_parse" << std::endl;
  // std::cout << root << std::endl;
  std::vector<EgoFrame> ego_lists;
  std::vector<std::vector<AgentFrame>> agents_lists;
  std::vector<std::vector<double>> refline_points{
      {1, 2, 3, 1, 1}, {3, 4, 5, 1, 1},   {5, 6, 7, 1, 1},
      {7, 8, 9, 1, 1}, {9, 10, 11, 1, 1}, {11, 12, 13, 1, 1}};
  std::vector<double> step_time_list;
  std::vector<double> refline_curvatures;
  std::vector<double> thetas;
  for (int i = 0; i < 5; i++) {
    auto frame_data = root[i];
    auto agents_data = frame_data["agents"]["agent"];
    auto ego_data = frame_data["ego"];
    ego_lists.emplace_back(TransEgoFrame(ego_data));
    int agent_num = agents_data.size();
    std::vector<AgentFrame> agentsframe;
    for (int j = 0; j < agent_num; j++) {
      agentsframe.emplace_back(TransAgentFrame(agents_data[j]));
    }
    agents_lists.emplace_back(agentsframe);
    double step_time = frame_data["step_time"].asDouble();
    step_time_list.emplace_back(step_time);
  }
  std::tie(refline_curvatures, thetas) =
      CalCurvatureFromOriRefline(refline_points);
  FeaturePreparation fp = FeaturePreparation();
  fp.GetData(ego_lists, agents_lists, refline_points, step_time_list,
             refline_curvatures, thetas);
  return fp;
}

EgoFrame TransEgoFrame(const Json::Value &ego_data) {
  EgoFrame ret = EgoFrame();
  for (int i = 0; i < 3; i++) {
    ret.centroid[0] = ego_data["position"]["x"].asDouble();
    ret.centroid[1] = ego_data["position"]["y"].asDouble();
    ret.centroid[2] = ego_data["position"]["heading"].asDouble();
  }
  ret.yaw = ego_data["position"]["heading"].asDouble();
  for (int i = 0; i < 2; i++) {
    ret.velocity[0] = ego_data["velocity"]["x"].asDouble();
    ret.velocity[1] = ego_data["velocity"]["y"].asDouble();
  }
  for (int i = 0; i < 2; i++) {
    ret.linearAcceleration[0] = ego_data["linear_acceleration"]["x"].asDouble();
    ret.linearAcceleration[1] = ego_data["linear_acceleration"]["y"].asDouble();
  }
  for (int i = 0; i < 2; i++) {
    ret.extent[0] = 2.74;
    ret.extent[1] = 1.06;
  }
  return ret;
}

AgentFrame TransAgentFrame(const Json::Value &agent_data) {
  AgentFrame ret = AgentFrame();
  for (int i = 0; i < 3; i++) {
    ret.centroid[0] = agent_data["position"]["x"].asDouble();
    ret.centroid[1] = agent_data["position"]["y"].asDouble();
    ret.centroid[2] = agent_data["position"]["heading"].asDouble();
  }
  ret.yaw = agent_data["position"]["heading"].asDouble();
  for (int i = 0; i < 2; i++) {
    ret.velocity[0] = agent_data["velocity"]["x"].asDouble();
    ret.velocity[1] = agent_data["velocity"]["y"].asDouble();
  }
  for (int i = 0; i < 2; i++) {
    ret.extent[0] = agent_data["extent"]["length"].asDouble();
    ret.extent[1] = agent_data["extent"]["width"].asDouble();
  }
  // type undefine
  return ret;
}

std::tuple<std::vector<double>, std::vector<double>> CalCurvatureFromOriRefline(
    const std::vector<std::vector<double>> &refline_points) {
  std::vector<double> reference_line_points_x, reference_line_points_y;
  int ref_length = refline_points.size();
  for (int i = 0; i < ref_length; i++) {
    reference_line_points_x.emplace_back(refline_points[i][0]);
    reference_line_points_y.emplace_back(refline_points[i][1]);
  }
  std::vector<double> refline_curvatures, thetas;
  return CalCurvature(reference_line_points_x, reference_line_points_y);
}

FeaturePreparation UnitJsonDataParser(Json::Value &root) {
  // int frame_num = root.size();
  // if (frame_num < 5) {
  //     return 1;
  // }
  std::vector<EgoFrame> ego_lists;
  std::vector<std::vector<AgentFrame>> agents_lists;
  std::vector<std::vector<double>> refline_points;
  std::vector<double> step_time_list;
  std::vector<double> refline_curvatures;
  std::vector<double> thetas;
  auto agents_data = root["agents"];
  auto ego_data = root["ego"];
  auto step_time_data = root["step_time"];
  auto refline_data = root["refline"];
  for (int i = 0; i < 5; i++) {
    auto agents_data_frame = agents_data[i];
    int agent_num = agents_data_frame.size();
    std::vector<AgentFrame> agentsframe;
    for (int j = 0; j < agent_num; j++) {
      AgentFrame agentsframeagent = AgentFrame(
          std::vector<double>{agents_data_frame[j]["centroid"][0].asDouble(),
                              agents_data_frame[j]["centroid"][1].asDouble(),
                              0.0},
          agents_data_frame[j]["yaw"].asDouble(),
          std::vector<double>{agents_data_frame[j]["velocity"][0].asDouble(),
                              agents_data_frame[j]["velocity"][1].asDouble(),
                              0.0},
          std::vector<double>{agents_data_frame[j]["extent"][0].asDouble(),
                              agents_data_frame[j]["extent"][1].asDouble(),
                              agents_data_frame[j]["extent"][2].asDouble()},
          agents_data_frame[j]["type"].asString());
      agentsframe.emplace_back(agentsframeagent);
    }
    agents_lists.emplace_back(agentsframe);
    auto ego_data_frame = ego_data[i];
    EgoFrame egoframe = EgoFrame(
        std::vector<double>{ego_data_frame["centroid"][0].asDouble(),
                            ego_data_frame["centroid"][1].asDouble(),
                            ego_data_frame["centroid"][2].asDouble()},
        ego_data_frame["yaw"].asDouble(),
        std::vector<double>{ego_data_frame["velocity"][0].asDouble(),
                            ego_data_frame["velocity"][1].asDouble(),
                            ego_data_frame["velocity"][2].asDouble()},
        std::vector<double>{ego_data_frame["linearAcceleration"][0].asDouble(),
                            ego_data_frame["linearAcceleration"][1].asDouble(),
                            ego_data_frame["linearAcceleration"][2].asDouble()},
        std::vector<double>{ego_data_frame["extent"][0].asDouble(),
                            ego_data_frame["extent"][1].asDouble(),
                            ego_data_frame["extent"][2].asDouble()});
    ego_lists.emplace_back(egoframe);
    step_time_list.emplace_back(step_time_data[i].asDouble());
  }
  int refline_point_num = refline_data.size();
  for (int i = 0; i < refline_point_num; i++) {
    std::vector<double> ref_point{
        refline_data[i][0].asDouble(), refline_data[i][1].asDouble(),
        refline_data[i][2].asDouble(), refline_data[i][3].asDouble(),
        refline_data[i][4].asDouble()};
    refline_points.emplace_back(ref_point);
  }
  std::tie(refline_curvatures, thetas) =
      CalCurvatureFromOriRefline(refline_points);
  FeaturePreparation fp = FeaturePreparation();
  fp.GetData(ego_lists, agents_lists, refline_points, step_time_list,
             refline_curvatures, thetas);
  return fp;
}

}  // namespace planning_rl
}  // namespace neodrive