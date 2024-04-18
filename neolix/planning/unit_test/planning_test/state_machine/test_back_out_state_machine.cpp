#include <string>

#include "gtest/gtest.h"
#include "scenario_manager_msgs.pb.h"
#include "state_machine/state_machine.h"

namespace{
	using ChangeFlag = neodrive::global::planning::BackOutStageChangeFlag;
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

TEST(TestBackOutStateMachine, test_init) {
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/back_out";
  state_machine.LoadStateMachine(state_machine_file);
  state_machine.SetInitializeState("PREPARE");
}

TEST(TestBackOutStateMachine, test_PREPARE) {
  state_machine.SetInitializeState("PREPARE");
  testStateTransition(ChangeFlag::T1_PREPARE_FORWARD, "FORWARD");
  state_machine.SetInitializeState("PREPARE");
  testStateTransition(ChangeFlag::T2_PREPARE_BACKWARD, "BACKWARD");
}

TEST(TestBackOutStateMachine, test_BACKWARD) {
  state_machine.SetInitializeState("BACKWARD");
  testStateTransition(ChangeFlag::T5_BACKWARD_FORWARD, "FORWARD");
  state_machine.SetInitializeState("BACKWARD");
  testStateTransition(ChangeFlag::T6_BACKWARD_EXIT, "EXIT");
	state_machine.SetInitializeState("BACKWARD");
  testStateTransition(ChangeFlag::T0_PREPARE, "PREPARE");
}

TEST(TestBackOutStateMachine, test_FORWARD) {
  state_machine.SetInitializeState("FORWARD");
  testStateTransition(ChangeFlag::T3_FORWARD_BACKWARD, "BACKWARD");
  state_machine.SetInitializeState("FORWARD");
  testStateTransition(ChangeFlag::T4_FORWARD_EXIT, "EXIT");
	state_machine.SetInitializeState("FORWARD");
  testStateTransition(ChangeFlag::T0_PREPARE, "PREPARE");
}
}