#include "common/global_data.h"
#include "cyber.h"
#include "gtest/gtest.h"
#include "reader_manager/reader_manager.h"
#include "src/planning/planning_manager/planning_manager.h"

int main(int argc, char **argv) {
  INIT_NEOLOG_NAME("planning_test");
  LOG_WARN("[thread_name]planning_test");
  testing::InitGoogleTest(&argc, argv);
  neodrive::cyber::Init(argv[0]);
  auto globalData = neodrive::cyber::common::GlobalData::Instance();
  globalData->DisableSimulationMode();
  std::shared_ptr<neodrive::cyber::Node> node =
      std::move(neodrive::cyber::CreateNode("planning_test", ""));
  neodrive::common::ReaderManager::Instance()->Init(node);
  neodrive::planning::PlanningManager::Instance()->Init(node);
  return RUN_ALL_TESTS();
}