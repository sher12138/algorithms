#include "src/planning/math/ilqr_solver/problem.h"

#include <memory>
#include <utility>

#include "gtest/gtest.h"
#include "src/planning/math/ilqr_solver/utils.h"

namespace neodrive {
namespace planning {
namespace ilqr {

TEST(ProblemTest, Construct) {
  Problem<1, 1> p(Problem<1, 1>::VecX::Zero());
  for (int i = 0; i < 10; ++i) {
    std::unique_ptr<CostFunction<1, 1>> c = std::make_unique<QuadCost<1, 1>>();
    std::unique_ptr<DynamicModel<1, 1>> m =
        std::make_unique<FirstOrderModel>(0.1);
    p.AddStep(std::move(c), std::move(m));
  }
  CHECK_EQ(p.num_steps(), 10);
  // Out of range when getting dynamic model.
  // EXPECT_DEATH(p.dynamic_model(11), "");
  // Terminal cost has not been set yet.
  // EXPECT_DEATH(p.terminal_cost_function(), "");
}

}  // namespace ilqr
}  // namespace planning
}  // namespace neodrive
