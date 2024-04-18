#include <string>

#include "gtest/gtest.h"
#include "scenario_manager_msgs.pb.h"
#include "state_machine/state_machine.h"

namespace {
using ChangeFlag = neodrive::global::planning::IntersectionChangeFlag;

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

TEST(TestIntersectionStateMachine, test_INIT) {
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/intersection_state";
  state_machine.LoadStateMachine(state_machine_file);
  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T7_INIT_STRAIGHT, "STRAIGHT");
  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T8_INIT_TURN_RIGHT, "TURN_RIGHT");
  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T9_INIT_TURN_LEFT, "TURN_LEFT");

  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T0_STRAIGHT_ABOUT_TO_TURN_RIGHT, "INIT");
  testStateTransition(ChangeFlag::T1_STRAIGHT_ABOUT_TO_TURN_LEFT, "INIT");
  testStateTransition(ChangeFlag::T2_ABOUT_TO_TURN_RIGHT_TURN_RIGHT, "INIT");
  testStateTransition(ChangeFlag::T3_ABOUT_TO_TURN_LEFT_TURN_LEFT, "INIT");
  testStateTransition(ChangeFlag::T4_TURN_RIGHT_STRAIGHT, "INIT");
  testStateTransition(ChangeFlag::T5_TURN_LEFT_STRAIGHT, "INIT");
  testStateTransition(ChangeFlag::T6_TURN_RIGHT_ABOUT_TO_TURN_LEFT, "INIT");
}

TEST(TestIntersectionStateMachine, test_STRAIGHT) {
  state_machine.SetInitializeState("STRAIGHT");

  testStateTransition(ChangeFlag::T0_STRAIGHT_ABOUT_TO_TURN_RIGHT,
                      "ABOUT_TO_TURN_RIGHT");
  state_machine.SetInitializeState("STRAIGHT");
  testStateTransition(ChangeFlag::T1_STRAIGHT_ABOUT_TO_TURN_LEFT,
                      "ABOUT_TO_TURN_LEFT");

  state_machine.SetInitializeState("STRAIGHT");
  testStateTransition(ChangeFlag::T2_ABOUT_TO_TURN_RIGHT_TURN_RIGHT,
                      "STRAIGHT");
  testStateTransition(ChangeFlag::T3_ABOUT_TO_TURN_LEFT_TURN_LEFT, "STRAIGHT");
  testStateTransition(ChangeFlag::T4_TURN_RIGHT_STRAIGHT, "STRAIGHT");
  testStateTransition(ChangeFlag::T5_TURN_LEFT_STRAIGHT, "STRAIGHT");
  testStateTransition(ChangeFlag::T6_TURN_RIGHT_ABOUT_TO_TURN_LEFT, "STRAIGHT");
  testStateTransition(ChangeFlag::T7_INIT_STRAIGHT, "STRAIGHT");
  testStateTransition(ChangeFlag::T8_INIT_TURN_RIGHT, "STRAIGHT");
  testStateTransition(ChangeFlag::T9_INIT_TURN_LEFT, "STRAIGHT");
}

TEST(TestIntersectionStateMachine, test_ABOUT_TO_TURN_RIGHT) {
  state_machine.SetInitializeState("ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T2_ABOUT_TO_TURN_RIGHT_TURN_RIGHT,
                      "TURN_RIGHT");

  state_machine.SetInitializeState("ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T0_STRAIGHT_ABOUT_TO_TURN_RIGHT,
                      "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T1_STRAIGHT_ABOUT_TO_TURN_LEFT,
                      "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T3_ABOUT_TO_TURN_LEFT_TURN_LEFT,
                      "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T4_TURN_RIGHT_STRAIGHT,
                      "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T5_TURN_LEFT_STRAIGHT, "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T6_TURN_RIGHT_ABOUT_TO_TURN_LEFT,
                      "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T7_INIT_STRAIGHT, "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T8_INIT_TURN_RIGHT, "ABOUT_TO_TURN_RIGHT");
  testStateTransition(ChangeFlag::T9_INIT_TURN_LEFT, "ABOUT_TO_TURN_RIGHT");
}

TEST(TestIntersectionStateMachine, test_TURN_RIGHT) {
  state_machine.SetInitializeState("TURN_RIGHT");
  testStateTransition(ChangeFlag::T4_TURN_RIGHT_STRAIGHT, "STRAIGHT");
  state_machine.SetInitializeState("TURN_RIGHT");
  testStateTransition(ChangeFlag::T6_TURN_RIGHT_ABOUT_TO_TURN_LEFT,
                      "ABOUT_TO_TURN_LEFT");

  state_machine.SetInitializeState("TURN_RIGHT");
  testStateTransition(ChangeFlag::T0_STRAIGHT_ABOUT_TO_TURN_RIGHT,
                      "TURN_RIGHT");
  testStateTransition(ChangeFlag::T1_STRAIGHT_ABOUT_TO_TURN_LEFT, "TURN_RIGHT");
  testStateTransition(ChangeFlag::T2_ABOUT_TO_TURN_RIGHT_TURN_RIGHT,
                      "TURN_RIGHT");
  testStateTransition(ChangeFlag::T3_ABOUT_TO_TURN_LEFT_TURN_LEFT,
                      "TURN_RIGHT");
  testStateTransition(ChangeFlag::T5_TURN_LEFT_STRAIGHT, "TURN_RIGHT");
  testStateTransition(ChangeFlag::T7_INIT_STRAIGHT, "TURN_RIGHT");
  testStateTransition(ChangeFlag::T8_INIT_TURN_RIGHT, "TURN_RIGHT");
  testStateTransition(ChangeFlag::T9_INIT_TURN_LEFT, "TURN_RIGHT");
}

TEST(TestIntersectionStateMachine, test_ABOUT_TO_TURN_LEFT) {
  state_machine.SetInitializeState("ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T3_ABOUT_TO_TURN_LEFT_TURN_LEFT, "TURN_LEFT");

  state_machine.SetInitializeState("ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T0_STRAIGHT_ABOUT_TO_TURN_RIGHT,
                      "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T1_STRAIGHT_ABOUT_TO_TURN_LEFT,
                      "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T2_ABOUT_TO_TURN_RIGHT_TURN_RIGHT,
                      "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T4_TURN_RIGHT_STRAIGHT, "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T5_TURN_LEFT_STRAIGHT, "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T6_TURN_RIGHT_ABOUT_TO_TURN_LEFT,
                      "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T7_INIT_STRAIGHT, "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T8_INIT_TURN_RIGHT, "ABOUT_TO_TURN_LEFT");
  testStateTransition(ChangeFlag::T9_INIT_TURN_LEFT, "ABOUT_TO_TURN_LEFT");
}

TEST(TestIntersectionStateMachine, test_TURN_LEFT) {
  state_machine.SetInitializeState("TURN_LEFT");
  testStateTransition(ChangeFlag::T5_TURN_LEFT_STRAIGHT, "STRAIGHT");

  state_machine.SetInitializeState("TURN_LEFT");
  testStateTransition(ChangeFlag::T0_STRAIGHT_ABOUT_TO_TURN_RIGHT, "TURN_LEFT");
  testStateTransition(ChangeFlag::T1_STRAIGHT_ABOUT_TO_TURN_LEFT, "TURN_LEFT");
  testStateTransition(ChangeFlag::T2_ABOUT_TO_TURN_RIGHT_TURN_RIGHT,
                      "TURN_LEFT");
  testStateTransition(ChangeFlag::T3_ABOUT_TO_TURN_LEFT_TURN_LEFT, "TURN_LEFT");
  testStateTransition(ChangeFlag::T4_TURN_RIGHT_STRAIGHT, "TURN_LEFT");
  testStateTransition(ChangeFlag::T6_TURN_RIGHT_ABOUT_TO_TURN_LEFT,
                      "TURN_LEFT");
  testStateTransition(ChangeFlag::T7_INIT_STRAIGHT, "TURN_LEFT");
  testStateTransition(ChangeFlag::T8_INIT_TURN_RIGHT, "TURN_LEFT");
  testStateTransition(ChangeFlag::T9_INIT_TURN_LEFT, "TURN_LEFT");
}

}  // namespace