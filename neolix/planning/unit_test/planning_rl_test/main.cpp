#include "gtest/gtest.h"
#include "neolix_log.h"

int main(int argc, char **argv) {
  INIT_NEOLOG_NAME("planning_rl_test");
  LOG_WARN("[thread_name]planning_rl_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}