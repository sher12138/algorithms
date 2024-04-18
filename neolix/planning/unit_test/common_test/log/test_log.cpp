// for every scenario and state, check if function can get pipeline.
#include "gtest/gtest.h"
#include "neolix_log.h"

namespace neodrive {
namespace neolog {

enum class TestEnum { ENUM_1 = 0, ENUM_2 = 1 };

TEST(NeoLogTest, test_log) {
  LOG_DEBUG("test {}", 1234);
  LOG_INFO("test {}", 1234);
  LOG_WARN("test {}", 1234);
  LOG_ERROR("test {}", 1234);
  LOG_DEBUG_STREAM("test "
                   << "debug" << 1234);
  LOG_INFO_STREAM("test "
                  << "info" << 1234);
  LOG_WARN_STREAM("test "
                  << "warn" << 1234);
  LOG_ERROR_STREAM("test "
                   << "error" << 1234);
  LOG_STREAM(DEBUG) << "test debug" << 1234 << " " << 12.34;
  LOG_STREAM(INFO) << "test debug" << 1234 << " " << 12.34;
  LOG_STREAM(WARN) << "test debug" << 1234 << " " << 12.34;
  LOG_STREAM(ERROR) << "test debug" << 1234 << " " << 12.34;

  NL_DEBUG << "test debug" << 1234 << " " << 12.34;
  NL_INFO << "test debug" << 1234 << " " << 12.34;
  NL_WARN << "test debug" << 1234 << " " << 12.34;
  NL_ERROR << "test debug" << 1234 << " " << 12.34;
}

TEST(NeoLogTest, test_log_format) {
  int i = 1;
  size_t j = 2;
  float x = 10.232;
  double y = 12312.12414;
  std::string str = "string test";
  char* str2 = "string test2";

  LOG_DEBUG("i: {}, j: {}, x: {}, y: {}", i, j, x, y);
  LOG_DEBUG("i: {1}, j: {0}, x: {3}, y: {2}", j, i, y, x);
  LOG_DEBUG("x: {:2.1f}, y: {:4.2f} str: {}, str2: {}", x, y, str, str2);
  LOG_DEBUG("str address: {}", fmt::ptr(&str));
  TestEnum enum1 = TestEnum::ENUM_1;
  TestEnum enum2 = TestEnum::ENUM_2;
  LOG_DEBUG("enum {} {} ", int(enum1), int(enum2));
}

}  // namespace neolog
}  // namespace neodrive