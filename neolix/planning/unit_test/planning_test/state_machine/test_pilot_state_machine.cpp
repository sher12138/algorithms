#include <string>

#include "gtest/gtest.h"
#include "pilot_state_msgs.pb.h"
#include "state_machine/state_machine.h"

namespace {
using ChangeFlag = neodrive::global::planning::PilotChangeFlag;

std::string current_state = "ACTIVE";
neodrive::common::state_machine::StateMachine state_machine;

void testStateTransition(const int flag, const char *state) {
  auto prev_state = state_machine.GetCurrentStateString();
  state_machine.ChangeState(flag);
  current_state = state_machine.GetCurrentStateString();
  std::cout << prev_state << "->" << state_machine.GetChangeFlagStr(flag)
            << "->" << state << std::endl;
  EXPECT_EQ(current_state, state);
}

TEST(TestPilotStateMachine, test_init) {
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/pilot_state";
  state_machine.LoadStateMachine(state_machine_file);
  state_machine.SetInitializeState("ACTIVE");
}

TEST(TestPilotStateMachine, test_INIT) {
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T_ACTIVE_STANDBY, "STANDBY");
}

TEST(TestPilotStateMachine, test_STANDBY) {
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T_STANDBY_ACTIVE, "ACTIVE");
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T_STANDBY_REJECT, "REJECT");
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T_STANDBY_DEGRADATION, "DEGRADATION");
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T_STANDBY_ESCALATION, "ESCALATION");
}

TEST(TestPilotStateMachine, test_REJECT) {
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T_REJECT_ACTIVE, "ACTIVE");
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T_REJECT_STANDBY, "STANDBY");
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T_REJECT_DEGRADATION, "DEGRADATION");
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T_REJECT_ESCALATION, "ESCALATION");
}

TEST(TestPilotStateMachine, test_ACTIVE) {
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T_ACTIVE_REJECT, "REJECT");
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T_ACTIVE_STANDBY, "STANDBY");
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T_ACTIVE_DEGRADATION, "DEGRADATION");
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T_ACTIVE_ESCALATION, "ESCALATION");
}

TEST(TestPilotStateMachine, test_DEGRADATION) {
  state_machine.SetInitializeState("DEGRADATION");
  testStateTransition(ChangeFlag::T_DEGRADATION_STANDBY, "STANDBY");
  state_machine.SetInitializeState("DEGRADATION");
  testStateTransition(ChangeFlag::T_DEGRADATION_REJECT, "REJECT");
  state_machine.SetInitializeState("DEGRADATION");
  testStateTransition(ChangeFlag::T_DEGRADATION_ACTIVE, "ACTIVE");
  state_machine.SetInitializeState("DEGRADATION");
  testStateTransition(ChangeFlag::T_DEGRADATION_ESCALATION, "ESCALATION");
}

TEST(TestPilotStateMachine, test_ESCALATION) {
  state_machine.SetInitializeState("ESCALATION");
  testStateTransition(ChangeFlag::T_ESCALATION_STANDBY, "STANDBY");
  state_machine.SetInitializeState("ESCALATION");
  testStateTransition(ChangeFlag::T_ESCALATION_REJECT, "REJECT");
  state_machine.SetInitializeState("ESCALATION");
  testStateTransition(ChangeFlag::T_ESCALATION_ACTIVE, "ACTIVE");
}

}  // namespace