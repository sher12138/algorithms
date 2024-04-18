#include "src/planning/scenario_manager/scenario_manager.h"

#include "common/global_data.h"
#include "gtest/gtest.h"

namespace neodrive {
namespace planning {
namespace {

TEST(TestScenarioManager, Init) {
  EXPECT_TRUE(ScenarioManager::Instance()->Init());
}

}  // namespace
}  // namespace planning
}  // namespace neodrive
