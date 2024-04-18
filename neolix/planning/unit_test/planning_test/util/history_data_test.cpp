#include "src/planning/util/history_data.h"

#include "gtest/gtest.h"

namespace neodrive {
namespace planning {

TEST(HistoryData, Init) {
  HistoryData history_int_data;
  int int_count = 5;
  history_int_data.Init(HISTORY_DATA_TYPE::HISTORY_DATA_INT, int_count);
  EXPECT_EQ(history_int_data.type(), HISTORY_DATA_TYPE::HISTORY_DATA_INT);
  EXPECT_EQ(history_int_data.count(), int_count);
  history_int_data.Push(1);
  history_int_data.Push(2);
  history_int_data.Push(3);
  history_int_data.Push(4);
  history_int_data.Push(5);
  EXPECT_EQ(history_int_data.GetSumValue(), 15);
  EXPECT_EQ(history_int_data.GetAverage(), 3);
  EXPECT_EQ(history_int_data.GetMaxValue(), 5);
  history_int_data.Push(6);
  EXPECT_EQ(history_int_data.GetMaxValue(), 6);
  history_int_data.Push(1);
  history_int_data.Push(2);
  history_int_data.Push(3);
  history_int_data.Push(4);
  history_int_data.Push(1);
  history_int_data.Push(2);
  history_int_data.Push(3);
  history_int_data.Push(4);
  EXPECT_EQ(history_int_data.GetMaxValue(), 4);
}

}  // namespace planning
}  // namespace neodrive
