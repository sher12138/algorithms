#include "src/planning/planning_manager/planning_manager.h"

namespace neodrive {
namespace planning {

class PlanningManagerTest {
 public:
  PlanningManagerTest() = default;
  void TestAebMsgCallback();

 private:
  PlanningManager* planning_manager_{PlanningManager::Instance()};
};

}  // namespace planning
}  // namespace neodrive
