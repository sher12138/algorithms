#include "planning_manager_test.h"

#include "common/data_center/data_center.h"
#include "common/global_data.h"
#include "gtest/gtest.h"
#include "time/time.h"

namespace neodrive {
namespace planning {
PlanningManagerTest planning_manager_test;

void PlanningManagerTest::TestAebMsgCallback() {}

TEST(TestPlanningManager, RunOnce) {
  planning_manager_test.TestAebMsgCallback();
}

}  // namespace planning
}  // namespace neodrive
