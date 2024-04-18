#include "src/planning/math/common/temporal_solver.h"

#include "common/json_config_util.h"
#include "cyber/time/time.h"
#include "gtest/gtest.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/obsmap.h"

namespace neodrive {
namespace planning {
using namespace math;
using namespace obsmap;

void LoadData(std::string name_str,
              std::shared_ptr<std::vector<TrajectoryData>> data_vec) {
  auto raw_data = neodrive::cyber::common::LoadJsonConfig(name_str).second;
  auto length = raw_data.size();
  for (auto i = 0; i < length; i++) {
    int id = raw_data[i]["id"].asInt();
    auto time_length = raw_data[i]["traj"]["timestamp"].size();
    data_vec->push_back(TrajectoryData());
    data_vec->back().id = id;
    for (auto j = 0; j < time_length; j++) {
      data_vec->back().timestamp.push_back(
          raw_data[i]["traj"]["timestamp"][j].asDouble());
      data_vec->back().position.push_back(
          {raw_data[i]["traj"]["position"][0][j].asDouble(),
           raw_data[i]["traj"]["position"][1][j].asDouble()});
      data_vec->back().heading.push_back(
          std::atan2(raw_data[i]["traj"]["direction"][1][j].asDouble(),
                     raw_data[i]["traj"]["direction"][0][j].asDouble()));
      std::vector<AD2> corner;
      for (auto k = 0; k < 4; k++) {
        corner.push_back(
            {raw_data[i]["traj"]["aabox"][k * 2][j].asDouble(),
             raw_data[i]["traj"]["aabox"][k * 2 + 1][j].asDouble()});
      }
      data_vec->back().polygon.push_back(Polygon(corner));
    }
  }
}

class TemporalCollisionDetectionTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    ss_data_ = std::make_shared<std::vector<TrajectoryData>>();
    ms_data_ = std::make_shared<std::vector<TrajectoryData>>();
    ls_data_ = std::make_shared<std::vector<TrajectoryData>>();
    LoadData(
        "/home/caros/cyberrt/data/planning/test/temporal_solver/"
        "ss_test.json",
        ss_data_);
    LoadData(
        "/home/caros/cyberrt/data/planning/test/temporal_solver/"
        "ms_test.json",
        ms_data_);
    LoadData(
        "/home/caros/cyberrt/data/planning/test/temporal_solver/"
        "ls_test.json",
        ls_data_);
  }

  static void TearDownTestSuite() {
    ss_data_.reset();
    ms_data_.reset();
    ls_data_.reset();
  }

  void SmallScaleTester() { solution.Process(*(ss_data_.get())); }
  void MediumScaleTester() { solution.Process(*(ms_data_.get())); }
  void LargeScaleTester() { solution.Process(*(ls_data_.get())); }
  void SingleFrameTester() {
    solution.PreProcess(*(ls_data_.get()));
    std::vector<int> ans_size{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0,
                              1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    auto start_t = cyber::Time::Now().ToSecond();
    constexpr int total_test_cnt = 100;
    for (int i = 0; i < total_test_cnt; i++) {
      for (int j = 0; j < solution.obsmap().Length(); j++) {
        EXPECT_EQ(
            solution.DetectCollisionAt(j, ls_data_->at(0).polygon[j]).size(),
            ans_size[j]);
      }
    }
    LOG_INFO("Running for {} cycles in {} s", total_test_cnt,
             cyber::Time::Now().ToSecond() - start_t);
  }

  static std::shared_ptr<std::vector<TrajectoryData>> ss_data_;
  static std::shared_ptr<std::vector<TrajectoryData>> ms_data_;
  static std::shared_ptr<std::vector<TrajectoryData>> ls_data_;
  obsmap::TemporalCollision solution;
};

std::shared_ptr<std::vector<TrajectoryData>>
    TemporalCollisionDetectionTest::ss_data_ = nullptr;
std::shared_ptr<std::vector<TrajectoryData>>
    TemporalCollisionDetectionTest::ms_data_ = nullptr;
std::shared_ptr<std::vector<TrajectoryData>>
    TemporalCollisionDetectionTest::ls_data_ = nullptr;

TEST_F(TemporalCollisionDetectionTest, DefaultConstructor) {
  EXPECT_GE(ss_data_->size(), 0);
  EXPECT_GE(ls_data_->size(), 0);
  EXPECT_GE(ms_data_->size(), 0);
}

TEST_F(TemporalCollisionDetectionTest, DataLoader) {
  const auto& data = *(ss_data_.get());
  for (int i = 0; i < data.size(); i++) {
    for (int j = 0; j < data[i].timestamp.size(); j++) {
      EXPECT_DOUBLE_EQ(data[i].polygon[j].aabox().cen[0],
                       data[i].position[j][0]);
      EXPECT_DOUBLE_EQ(data[i].polygon[j].aabox().cen[1],
                       data[i].position[j][1]);
    }
  }
}

TEST_F(TemporalCollisionDetectionTest, SSTest) { SmallScaleTester(); }

TEST_F(TemporalCollisionDetectionTest, MSTest) { MediumScaleTester(); }

TEST_F(TemporalCollisionDetectionTest, LSTest) { LargeScaleTester(); }

TEST_F(TemporalCollisionDetectionTest, SFTest) { SingleFrameTester(); }

}  // namespace planning
}  // namespace neodrive