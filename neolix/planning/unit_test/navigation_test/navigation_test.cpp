#include "common/global_data.h"
#include "gtest/gtest.h"
#include "navigation/a_star_navigation.h"
#include "navigation/breadth_first_navigation.h"
#include "navigation/nonmotorway_navigation.h"
#include "time/time.h"

namespace neodrive {
namespace planning {

TEST(TestNavigationComponent, Init) {
  NavigationContext ctx;
  std::shared_ptr<NavigationProcessBase> navigation_ptr;
  navigation_ptr = std::make_shared<AStarNavigationProcess>(&ctx);
  EXPECT_EQ(navigation_ptr->Init(), true);

  navigation_ptr = std::make_shared<BreadthFirstNavigationProcess>(&ctx);
  EXPECT_EQ(navigation_ptr->Init(), true);

  navigation_ptr = std::make_shared<NonMotorwayNavigationProcess>(&ctx);
  EXPECT_EQ(navigation_ptr->Init(), true);
}

}  // namespace planning
}  // namespace neodrive
