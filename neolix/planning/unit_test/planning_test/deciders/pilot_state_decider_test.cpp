#include "deciders/pilot_state_decider/pilot_state_decider.h"

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {
namespace {
using ChangeFlag = neodrive::global::planning::PilotChangeFlag;

TEST(TestPilotStateDecider, PilotStateDeciderInit) {
  auto pilot_state_decider = PilotStateDecider::Instance();
  EXPECT_TRUE(pilot_state_decider != nullptr);
  EXPECT_EQ(pilot_state_decider->initialized(), true);
  EXPECT_EQ(pilot_state_decider->current_state(), "ACTIVE");
  EXPECT_EQ(pilot_state_decider->current_decision(),
            ChangeFlag::T_ACTIVE_STANDBY);
}

TEST(TestPilotStateDecider, PilotStateDeciderName) {
  auto pilot_state_decider = PilotStateDecider::Instance();
  EXPECT_EQ(pilot_state_decider->Name(), "PilotStateDecider");
}

TEST(TestPilotStateDecider, PilotStateDeciderReset) {
  auto pilot_state_decider = PilotStateDecider::Instance();
  pilot_state_decider->Reset();
  EXPECT_EQ(pilot_state_decider->current_state(), "ACTIVE");
  EXPECT_EQ(pilot_state_decider->current_decision(),
            ChangeFlag::T_ACTIVE_STANDBY);
}

}  // namespace
}  // namespace planning
}  // namespace neodrive
