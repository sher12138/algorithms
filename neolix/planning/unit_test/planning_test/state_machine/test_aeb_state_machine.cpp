#include <string>

#include "aeb_msgs.pb.h"
#include "gtest/gtest.h"
#include "state_machine/state_machine.h"

namespace {
using ChangeFlag = neodrive::global::planning::AebChangeFlag;

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

TEST(TestAebStateMachine, test_init) {
  std::string state_machine_file = "/home/caros/cyberrt/conf/state_machine/aeb";
  state_machine.LoadStateMachine(state_machine_file);
  state_machine.SetInitializeState("OFF");
}

TEST(TestAebStateMachine, test_INIT) {
  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T2_INIT_STANDBY, "STANDBY");
}

TEST(TestAebStateMachine, test_OFF) {
  // * -> OFF
  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T0_OFF, "OFF");
  state_machine.SetInitializeState("OFF");
  testStateTransition(ChangeFlag::T0_OFF, "OFF");
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T0_OFF, "OFF");
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T0_OFF, "OFF");
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T0_OFF, "OFF");
  state_machine.SetInitializeState("FAILURE_REVERSIBLE");
  testStateTransition(ChangeFlag::T0_OFF, "OFF");
  state_machine.SetInitializeState("FAILURE_IRREVERSIBLE");
  testStateTransition(ChangeFlag::T0_OFF, "OFF");

  state_machine.SetInitializeState("OFF");
  testStateTransition(ChangeFlag::T3_OFF_STANDBY, "STANDBY");
}

TEST(TestAebStateMachine, test_STANDBY) {
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T4_STANDBY_REJECT, "REJECT");
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T5_STANDBY_ACTIVE, "ACTIVE");
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T10_PASSIVE_FAILURE_REVERSIBLE,
                      "FAILURE_REVERSIBLE");
  state_machine.SetInitializeState("STANDBY");
  testStateTransition(ChangeFlag::T10_PASSIVE_FAILURE_IRREVERSIBLE,
                      "FAILURE_IRREVERSIBLE");
}

TEST(TestAebStateMachine, test_REJECT) {
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T8_REJECT_STANDBY, "STANDBY");
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T10_PASSIVE_FAILURE_REVERSIBLE,
                      "FAILURE_REVERSIBLE");
  state_machine.SetInitializeState("REJECT");
  testStateTransition(ChangeFlag::T10_PASSIVE_FAILURE_IRREVERSIBLE,
                      "FAILURE_IRREVERSIBLE");
}

TEST(TestAebStateMachine, test_ACTIVE) {
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T6_ACTIVE_STANDBY, "STANDBY");
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T7_ACTIVE_REJECT, "REJECT");
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T9_ACTIVE_FAILURE_REVERSIBLE,
                      "FAILURE_REVERSIBLE");
  state_machine.SetInitializeState("ACTIVE");
  testStateTransition(ChangeFlag::T9_ACTIVE_FAILURE_IRREVERSIBLE,
                      "FAILURE_IRREVERSIBLE");
}

TEST(TestAebStateMachine, test_FAILURE_REVERSIBLE) {
  state_machine.SetInitializeState("FAILURE_REVERSIBLE");
  testStateTransition(ChangeFlag::T11_FAILURE_REVERSIBLE_STANDBY, "STANDBY");
  state_machine.SetInitializeState("FAILURE_REVERSIBLE");
  testStateTransition(ChangeFlag::T12_FAILURE_REVERSIBLE_FAILURE_IRREVERSIBLE,
                      "FAILURE_IRREVERSIBLE");
}

}  // namespace