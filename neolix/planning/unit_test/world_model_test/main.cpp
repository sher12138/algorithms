#include "common/global_data.h"
#include "cyber.h"
#include "gtest/gtest.h"
#include "reader_manager/reader_manager.h"
#include "world_model/coordinator_transform/coordinator_transform.h"
#include "world_model/world_model.h"

int main(int argc, char **argv) {
  INIT_NEOLOG_NAME("world_model_test");
  testing::InitGoogleTest(&argc, argv);
  neodrive::cyber::Init(argv[0]);
  auto globalData = neodrive::cyber::common::GlobalData::Instance();
  globalData->DisableSimulationMode();
  std::shared_ptr<neodrive::cyber::Node> node =
      std::move(neodrive::cyber::CreateNode("world_model_test", ""));
  neodrive::common::ReaderManager::Instance()->Init(node);
  neodrive::world_model::WorldModel::Instance()->Init(node);
  return RUN_ALL_TESTS();
}