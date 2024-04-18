#include <string>

#include "gtest/gtest.h"
#include "scenario_manager_msgs.pb.h"
#include "state_machine/state_machine.h"

namespace {
using ChangeFlag = neodrive::global::planning::CruiseStageChangeFlag;

std::string current_state = "OFF";
neodrive::common::state_machine::StateMachine state_machine;

void testStateTransition(const int flag, const char *state) {
  auto prev_state = state_machine.GetCurrentStateString();
  state_machine.ChangeState(flag);
  current_state = state_machine.GetCurrentStateString();
  std::cout << prev_state << "->" << state_machine.GetChangeFlagStr(flag)
            << "->" << state << std::endl;
  EXPECT_EQ(current_state, state);
}

TEST(TestCruiseStateMachine, test_init) {
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_cruise_stage";
  state_machine.LoadStateMachine(state_machine_file);
  state_machine.SetInitializeState("KEEP");
}

TEST(TestCruiseStateMachine, test_CENTERING) {
  state_machine.SetInitializeState("KEEP");
  testStateTransition(ChangeFlag::T1_KEEP_CENTERING, "CENTERING");
  state_machine.SetInitializeState("CENTERING");
  testStateTransition(ChangeFlag::T2_CENTERING_KEEP, "KEEP");
}

TEST(TestCruiseStateMachine, test_BIAS_DRIVE) {
  state_machine.SetInitializeState("KEEP");
  testStateTransition(ChangeFlag::T3_KEEP_BIAS_DRIVE, "BIAS_DRIVE");
  state_machine.SetInitializeState("BIAS_DRIVE");
  testStateTransition(ChangeFlag::T4_BIAS_DRIVE_KEEP, "KEEP");
}

TEST(TestCruiseStateMachine, test_RIGHT_AVOID) {
  state_machine.SetInitializeState("KEEP");
  testStateTransition(ChangeFlag::T5_KEEP_RIGHT_AVOID, "RIGHT_AVOID");
  state_machine.SetInitializeState("RIGHT_AVOID");
  testStateTransition(ChangeFlag::T6_RIGHT_AVOID_KEEP, "KEEP");
}

TEST(TestCruiseStateMachine, test_LEFT_OVERTAKE) {
  state_machine.SetInitializeState("KEEP");
  testStateTransition(ChangeFlag::T7_KEEP_LEFT_OVERTAKE, "LEFT_OVERTAKE");
  state_machine.SetInitializeState("LEFT_OVERTAKE");
  testStateTransition(ChangeFlag::T8_LEFT_OVERTAKE_KEEP, "KEEP");
}

TEST(TestCruiseStateMachine, test_PULL_OVER) {
  state_machine.SetInitializeState("KEEP");
  testStateTransition(ChangeFlag::T9_KEEP_PULL_OVER, "PULL_OVER");
  state_machine.SetInitializeState("PULL_OVER");
  testStateTransition(ChangeFlag::T10_PULL_OVER_KEEP, "KEEP");
}

}  // namespace