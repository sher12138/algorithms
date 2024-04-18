#include "common/global_data.h"
#include "cyber.h"
#include "gtest/gtest.h"
#include "neolix_log.h"

int main(int argc, char **argv) {
  INIT_NEOLOG_NAME("common_test");
  testing::InitGoogleTest(&argc, argv);
  neodrive::cyber::Init(argv[0]);
  auto globalData = neodrive::cyber::common::GlobalData::Instance();
  globalData->DisableSimulationMode();
  std::shared_ptr<neodrive::cyber::Node> node =
      std::move(neodrive::cyber::CreateNode("common_test", ""));
  return RUN_ALL_TESTS();
}