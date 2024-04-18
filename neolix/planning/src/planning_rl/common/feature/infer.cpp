#include "infer.h"

namespace neodrive {
namespace planning_rl {

void Infer() {
  Json::Value root;
  DataJsonReader(root);
  FeaturePreparation fp = JsonDataParser(root);
  // auto obs_before = fp.get_all_data_frame(4);
  // 提前终止
  int refline_points_num = fp.refline_points().size();
  if (refline_points_num <= 1) {
    return;
  }
  fp.GetAllData();
  auto target_before = fp.target_before();
  LOG_DEBUG("target:{}", target_before[0][0][0]);
  auto output = ModelPredictTrt(target_before);
  std::cout << "model_output:" << std::endl;
  std::cout << output << std::endl;
  LOG_INFO("model predict done");
  // auto vec_output = Tensor2Vec(output);
  LOG_DEBUG("target_before:{}", target_before[0][0][0]);
  LOG_DEBUG("vec_output:{}", output[0]);
  LOG_DEBUG("ego_info:{}", fp.ego_info().x);
  auto ppr = PostPredict(target_before, output, fp.ego_info());
  std::vector<TrajectoryPoint2D> pred_traj_opt =
      OptimalTrajectoryRefpoint(ppr, CvtVecref2Point(fp.refline_points()));
  WriteTrajJson(pred_traj_opt);
}

void UnitInfer() {
  // std::cout << "infer_start:" << std::endl;
  Json::Value root;
  UnitTestDataJsonReader(root);
  // std::cout << "data_reader:" << std::endl;
  FeaturePreparation fp = UnitJsonDataParser(root);
  // std::cout << "get_data:" << std::endl;
  fp.GetAllData();
  auto target_before = fp.target_before();
  // std::cout << "imitationnet_input:" << std::endl;
  // std::cout << target_before << std::endl;
  auto output = ModelPredictTrt(target_before);
  // std::cout << "imitationnet_output:" << std::endl;
  // std::cout << output << std::endl;
  auto ppr = PostPredict(target_before, output, fp.ego_info());
  // std::cout << "initation_output_prediction:" << std::endl;
  // for (auto x: ppr) {
  //   std::cout << x.x << std::endl;
  //   std::cout << x.y << std::endl;
  //   std::cout << x.heading << std::endl;
  //   std::cout << x.speed << std::endl;
  //   std::cout << x.acc << std::endl;
  //   std::cout << x.curvature << std::endl;
  // }
  std::vector<TrajectoryPoint2D> pred_traj_opt =
      OptimalTrajectoryRefpoint(ppr, CvtVecref2Point(fp.refline_points()));
  WriteTrajJson(pred_traj_opt);
}

std::vector<double> ModelPredictTrt(
    const std::vector<std::vector<std::vector<double>>> &target_before) {
  utils::TimeLogger time_log_{"ModelPredictTrt"};
  time_log_.ResetStartTime();
  float inputs1[1 * 5 * 1815];
  // 5 * 165 * 11 -> 1 * 5 * 1815
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 165; j++) {
      for (int k = 0; k < 11; k++) {
        inputs1[i * 1815 + j * 11 + k] = target_before[i][j][k];
      }
    }
  }
  auto *imitation_evaluator =
      neodrive::imitation::ImitationNet_Evaluator::Instance();
  // auto* imitation_evaluator = new
  // neodrive::imitation::ImitationNet_Evaluator(); auto imitation_evaluator =
  // std::make_shared<neodrive::imitation::ImitationNet_Evaluator>()::Instance()->engine();
  auto output = imitation_evaluator->Evaluate(inputs1);
  time_log_.RegisterTimeAndPrint("forward");
  return output[0];
}

int WriteTrajJson(const std::vector<TrajectoryPoint2D> &pred_traj_opt) {
  Json::Value root;
  // Json::Value traj;
  // root["trajectory"] = traj;
  int point_num = pred_traj_opt.size();
  for (int i = 0; i < point_num; i++) {
    Json::Value point;
    point["x"] = pred_traj_opt[i].x;
    point["y"] = pred_traj_opt[i].y;
    point["theta"] = pred_traj_opt[i].theta;
    point["kappa"] = pred_traj_opt[i].kappa;
    point["t"] = pred_traj_opt[i].t;
    point["v"] = pred_traj_opt[i].v;
    root.append(point);
  }
  std::ofstream os;
  Json::StyledWriter sw;
  // std::cout << "traj_json" << std::endl;
  os.open("traj.json", std::ios::out | std::ios::app);
  if (!os.is_open()) {
    LOG_WARN("write file not open");
  }
  os << sw.write(root);
  os.close();
  return 0;
}

}  // namespace planning_rl
}  // namespace neodrive