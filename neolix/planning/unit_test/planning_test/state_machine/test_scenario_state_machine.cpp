#include <string>

#include "gtest/gtest.h"
#include "scenario_manager_msgs.pb.h"
#include "state_machine/state_machine.h"

namespace {
using ChangeFlag = neodrive::global::planning::ScenarioChangeFlag;

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

TEST(TestScenarioStateMachine, test_init) {
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_manager";
  state_machine.LoadStateMachine(state_machine_file);
  state_machine.SetInitializeState("INIT");
}

TEST(TestScenarioStateMachine, test_INIT) {
  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T_INIT_CRUISE, "CRUISE");

  state_machine.SetInitializeState("INIT");
  testStateTransition(ChangeFlag::T_INIT_MOTORWAY_CRUISE, "MOTORWAY_CRUISE");
}

TEST(TestScenarioStateMachine, test_CRUISE) {
  state_machine.SetInitializeState("CRUISE");
  testStateTransition(ChangeFlag::T_CRUISE_NARROW_RAOD, "NARROW_RAOD");
  state_machine.SetInitializeState("CRUISE");
  testStateTransition(ChangeFlag::T_CRUISE_DETOUR, "DETOUR");
  state_machine.SetInitializeState("CRUISE");
  testStateTransition(ChangeFlag::T_CRUISE_INTERSECTION, "INTERSECTION");
  state_machine.SetInitializeState("CRUISE");
  testStateTransition(ChangeFlag::T_CRUISE_BACK_OUT, "BACK_OUT");
  state_machine.SetInitializeState("CRUISE");
  testStateTransition(ChangeFlag::T_CRUISE_BARRIER_GATE, "BARRIER_GATE");
  state_machine.SetInitializeState("CRUISE");
  testStateTransition(ChangeFlag::T_CRUISE_SIDE_WAY_INTERSECTION,
                      "SIDE_WAY_INTERSECTION");
  state_machine.SetInitializeState("CRUISE");
  testStateTransition(ChangeFlag::T_CRUISE_MOTORWAY_CRUISE, "MOTORWAY_CRUISE");
}

TEST(TestScenarioStateMachine, test_NARROW_RAOD) {
  state_machine.SetInitializeState("NARROW_RAOD");
  testStateTransition(ChangeFlag::T_NARROW_RAOD_CRUISE, "CRUISE");
  state_machine.SetInitializeState("NARROW_RAOD");
  testStateTransition(ChangeFlag::T_NARROW_RAOD_DETOUR, "DETOUR");
  state_machine.SetInitializeState("NARROW_RAOD");
  testStateTransition(ChangeFlag::T_NARROW_RAOD_INTERSECTION, "INTERSECTION");
}

TEST(TestScenarioStateMachine, test_DETOUR) {
  state_machine.SetInitializeState("DETOUR");
  testStateTransition(ChangeFlag::T_DETOUR_CRUISE, "CRUISE");
  state_machine.SetInitializeState("DETOUR");
  testStateTransition(ChangeFlag::T_DETOUR_BACK_OUT, "BACK_OUT");
}

TEST(TestScenarioStateMachine, test_INTERSECTION) {
  state_machine.SetInitializeState("INTERSECTION");
  testStateTransition(ChangeFlag::T_INTERSECTION_CRUISE, "CRUISE");
  state_machine.SetInitializeState("INTERSECTION");
  testStateTransition(ChangeFlag::T_INTERSECTION_NARROW_RAOD, "NARROW_RAOD");
  state_machine.SetInitializeState("INTERSECTION");
  testStateTransition(ChangeFlag::T_INTERSECTION_DETOUR, "DETOUR");
}

TEST(TestScenarioStateMachine, test_BACK_OUT) {
  state_machine.SetInitializeState("BACK_OUT");
  testStateTransition(ChangeFlag::T_BACK_OUT_CRUISE, "CRUISE");
  state_machine.SetInitializeState("BACK_OUT");
  testStateTransition(ChangeFlag::T_BACK_OUT_DETOUR, "DETOUR");
}

TEST(TestScenarioStateMachine, test_BARRIER_GATE) {
  state_machine.SetInitializeState("BARRIER_GATE");
  testStateTransition(ChangeFlag::T_BARRIER_GATE_CRUISE, "CRUISE");
  state_machine.SetInitializeState("BARRIER_GATE");
  testStateTransition(ChangeFlag::T_BARRIER_GATE_MOTORWAY_CRUISE,
                      "MOTORWAY_CRUISE");
}

TEST(TestScenarioStateMachine, test_SIDE_WAY_INTERSECTION) {
  state_machine.SetInitializeState("SIDE_WAY_INTERSECTION");
  testStateTransition(ChangeFlag::T_SIDE_WAY_INTERSECTION_CRUISE, "CRUISE");
}

TEST(TestScenarioStateMachine, test_MOTORWAY_CRUISE) {
  state_machine.SetInitializeState("MOTORWAY_CRUISE");
  testStateTransition(ChangeFlag::T_MOTORWAY_CRUISE_CRUISE, "CRUISE");
  state_machine.SetInitializeState("MOTORWAY_CRUISE");
  testStateTransition(ChangeFlag::T_MOTORWAY_CRUISE_MOTORWAY_LANE_CHANGE,
                      "MOTORWAY_LANE_CHANGE");
  state_machine.SetInitializeState("MOTORWAY_CRUISE");
  testStateTransition(ChangeFlag::T_MOTORWAY_CRUISE_MOTORWAY_INTERSECTION,
                      "MOTORWAY_INTERSECTION");
  state_machine.SetInitializeState("MOTORWAY_CRUISE");
  testStateTransition(ChangeFlag::T_MOTORWAY_CRUISE_MOTORWAY_DETOUR,
                      "MOTORWAY_DETOUR");
  state_machine.SetInitializeState("MOTORWAY_CRUISE");
  testStateTransition(ChangeFlag::T_MOTORWAY_CRUISE_BARRIER_GATE,
                      "BARRIER_GATE");
}

TEST(TestScenarioStateMachine, test_MOTORWAY_LANE_CHANGE) {
  state_machine.SetInitializeState("MOTORWAY_LANE_CHANGE");
  testStateTransition(ChangeFlag::T_MOTORWAY_LANE_CHANGE_MOTORWAY_CRUISE,
                      "MOTORWAY_CRUISE");
}

TEST(TestScenarioStateMachine, test_MOTORWAY_INTERSECTION) {
  state_machine.SetInitializeState("MOTORWAY_INTERSECTION");
  testStateTransition(ChangeFlag::T_MOTORWAY_INTERSECTION_MOTORWAY_CRUISE,
                      "MOTORWAY_CRUISE");
}

TEST(TestScenarioStateMachine, test_MOTORWAY_DETOUR) {
  state_machine.SetInitializeState("MOTORWAY_DETOUR");
  testStateTransition(ChangeFlag::T_MOTORWAY_DETOUR_MOTORWAY_CRUISE,
                      "MOTORWAY_CRUISE");
}

}  // namespace