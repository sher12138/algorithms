#include <vector>

#include "common/feature/infer.h"
#include "common/feature/train_features.h"
#include "common/math/math_utils.h"
#include "common/math/smooth.h"
#include "cyber.h"
#include "gtest/gtest.h"
#include "neolix_log.h"
#include "time/time.h"
namespace neodrive {
namespace planning_rl {

TEST(SmoothTest, test_init) { Smooth::Instance(); }

TEST(SmoothTest, test_linear_interpolate) {
  PlanningRLMap::MapPoint start_point{0, 0, 0};
  PlanningRLMap::MapPoint end_point{1, 1, 0};
  PlanningRLMap::MapPoint2d new_points;
  Smooth::Instance().linear_interpolate(start_point, end_point, new_points,
                                        0.2);
  for (size_t i = 0; i < new_points.size(); i++) {
    std::cout << "[" << new_points[i].x << ", " << new_points[i].y << "]"
              << std::endl;
  }
}

TEST(SmoothTest, test_linear_interpolate2) {
  std::pair<double, double> start_point{0, 0};
  std::pair<double, double> end_point{1, 1};
  std::vector<std::pair<double, double>> new_points;
  Smooth::Instance().linear_interpolate(start_point, end_point, &new_points,
                                        0.2);
  for (size_t i = 0; i < new_points.size(); i++) {
    std::cout << "[" << new_points[i].first << ", " << new_points[i].second
              << "]" << std::endl;
  }
}
TEST(SmoothTest, test_line_linear_interpolate) {
  PlanningRLMap::MapPoint point_1{0, 0, 0, 0};
  PlanningRLMap::MapPoint point_2{1, 1, 0, 0};
  PlanningRLMap::MapPoint point_3{2, 2, 0, 0};
  PlanningRLMap::MapPoint2d path;
  path.push_back(point_1);
  path.push_back(point_2);
  path.push_back(point_3);
  PlanningRLMap::MapPoint2d new_points =
      Smooth::Instance().line_linear_interpolate(path);
  for (size_t i = 0; i < new_points.size(); i++) {
    std::cout << "[" << new_points[i].x << ", " << new_points[i].y << "]"
              << std::endl;
  }
}

TEST(SmoothTest, test_linear_interpolate_with_smooth_path) {
  PlanningRLMap::MapPoint start_point{0, 0, 0, 0};
  PlanningRLMap::MapPoint end_point{1, 1, 0, 0};
  PlanningRLMap::MapPoint2d new_points;
  Smooth::Instance().linear_interpolate(start_point, end_point, new_points,
                                        0.2);
  for (size_t i = 0; i < new_points.size(); i++) {
    std::cout << "[" << new_points[i].x << ", " << new_points[i].y << "]"
              << std::endl;
  }
  new_points[1] = {0, 0, 0, 0};
  PlanningRLMap::MapPoint2d final_path;
  Smooth::Instance().smooth_path(new_points, final_path, 0.5, 0.2, 0.000001);
  std::cout << "-------smooth path-----" << std::endl;
  for (size_t i = 0; i < final_path.size(); i++) {
    std::cout << "[" << final_path[i].x << ", " << final_path[i].y << "]"
              << std::endl;
  }
}

TEST(SmoothTest, test_linear_interpolate_with_smooth_path2) {
  PlanningRLMap::MapPoint start_point{0, 0, 0, 0};
  PlanningRLMap::MapPoint end_point{1, 1, 0, 0};
  PlanningRLMap::MapPoint2d new_points;
  Smooth::Instance().linear_interpolate(start_point, end_point, new_points,
                                        0.2);
  std::vector<std::vector<double>> path;
  std::vector<std::vector<double>> final_path;
  for (size_t i = 0; i < new_points.size(); i++) {
    std::cout << "[" << new_points[i].x << ", " << new_points[i].y << "]"
              << std::endl;
    std::vector<double> p{new_points[i].x, new_points[i].y};
    path.push_back(p);
  }
  path[1] = {0, 0};
  Smooth::Instance().smooth_path(path, final_path, 0.5, 0.2, 0.000001);
  std::cout << "-------smooth path-----" << std::endl;
  for (size_t i = 0; i < final_path.size(); i++) {
    std::cout << "[" << final_path[i][0] << ", " << final_path[i][1] << "]"
              << std::endl;
  }
}

TEST(CurvatureTest, test_curvature) {
  PlanningRLMap::MapPoint point_1{0, -1, 0, 0};
  PlanningRLMap::MapPoint point_2{1, 0, 0, 0};
  PlanningRLMap::MapPoint point_3{0, 1, 0, 0};
  PlanningRLMap::MapPoint2d path;
  path.push_back(point_1);
  path.push_back(point_2);
  path.push_back(point_3);
  PlanningRLMap::MapPoint2d new_points =
      Smooth::Instance().line_linear_interpolate(path);
  for (size_t i = 0; i < new_points.size(); i++) {
    std::cout << "[" << new_points[i].x << ", " << new_points[i].y << "]"
              << std::endl;
  }

  // new_points[1] = {0,0,0};
  PlanningRLMap::MapPoint2d final_path;
  Smooth::Instance().smooth_path(new_points, final_path, 0.5, 0.2, 0.000001);
  std::cout << "-------smooth path-----" << std::endl;
  for (size_t i = 0; i < final_path.size(); i++) {
    std::cout << "[" << final_path[i].x << ", " << final_path[i].y << "]"
              << std::endl;
  }
  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<std::vector<double>> curvature_and_theta;
  for (size_t i = 0; i < final_path.size(); i++) {
    x_list.push_back(final_path[i].x);
    y_list.push_back(final_path[i].y);
  }
  cal_curvature(x_list, y_list, curvature_and_theta);
  std::cout << "----------------curvature_and_theta--------------"
            << curvature_and_theta.size() << std::endl;
  for (size_t i = 0; i < curvature_and_theta[0].size(); i++) {
    std::cout << "[" << curvature_and_theta[0][i] << ", "
              << curvature_and_theta[1][i] << "]" << std::endl;
  }
  LOG_INFO("2222222222");
}

TEST(FeatureTest, test_feature) {
  // Json::Reader reader;
  // Json::Value root;

  // //从文件中读取，保证当前文件有demo.json文件
  // std::ifstream in(
  //     "/home/caros/share_workspace/reinforcement_learnning/planning_rl/"
  //     "unit_test/config/ego_attribuate.json",
  //     std::ios::binary);

  // if (!in.is_open()) {
  //   std::cout << "Error opening file\n";
  //   return;
  // }

  // if (reader.parse(in, root)) {
  //   //读取根节点信息
  //   std::cout << root["ego"] << std::endl;
  // }

  // Testa();
  // Tester();
  // Infer();
  LOG_INFO("123456789");
}

TEST(UnitFeatureTest, unit_test_feature) {
  std::cout.precision(10);
  UnitInfer();
  LOG_INFO("UnitFeatureTest");
}

// TEST(FindAgentTest, test_findagents) {
//   int agents_num = 50;
//   int point_num = 500;
//   torch::Tensor agent_points, refline_points;
//   agent_points = (torch::rand({agents_num, 4}) * 100).to(torch::kDouble);
//   refline_points = (torch::rand({point_num, 2}) * 200).to(torch::kDouble);
//   utils::TimeLogger time_log_{"test_find_agents"};
//   torch::Tensor agents_nearest_indexes1 =
//       FindAgentsReferLineClosestPoint(agent_points, refline_points);
//   time_log_.RegisterTimeAndPrint("TestFindAgentsReferLineClosestPoint");
//   torch::Tensor agents_nearest_indexes2 =
//       FindAgentsReferLineClosestPointDecc(agent_points, refline_points);
//   time_log_.RegisterTimeAndPrint("TestFindAgentsReferLineClosestPointDecc");
//   torch::Tensor agents_nearest_indexes =
//       FindAgentsReferLineClosestPointAcce(agent_points, refline_points);
//   time_log_.RegisterTimeAndPrint("TestFindAgentsReferLineClosestPointAcce");
//   std::cout << "agents_nearest" << std::endl;
//   std::cout << agents_nearest_indexes2 << std::endl;
//   std::cout << agents_nearest_indexes << std::endl;
//   std::cout << "equal" <<
//   agents_nearest_indexes.equal(agents_nearest_indexes2) <<
//   agents_nearest_indexes << agents_nearest_indexes2 << std::endl;

// }

}  // namespace planning_rl
}  // namespace neodrive