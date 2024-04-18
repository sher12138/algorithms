#include "print_util.h"

namespace neodrive {
namespace planning {

void PrintUtil::TimeStatisticTool(const std::string &s) {
  double t = cyber::Time::Now().ToSecond();
  TimeStatisticInfo info;
  if (time_statistic_.empty()) {
    info = TimeStatisticInfo{.sign = s, .abs_time = t, .rel_time = 0.};
  } else {
    info = TimeStatisticInfo{.sign = s,
                             .abs_time = t,
                             .rel_time = t - time_statistic_.back().abs_time};
  }
  time_statistic_.emplace_back(info);
}

void PrintUtil::TimeStatisticPrint(const std::string &s) {
  if (time_statistic_.empty() || time_statistic_.size() == 1) {
    LOG_INFO("{} time statistic size less than 2!", s);
    return;
  } else {
    for (int i = 0; i < time_statistic_.size(); ++i) {
      TimeStatisticInfo time_statistic_info = time_statistic_.at(i);
      LOG_INFO(
          "{} time statistic: index:{}, abs_time:{:.4f}, "
          "rel_time:{:.4f}, sign:{},",
          s, i, time_statistic_info.abs_time, time_statistic_info.rel_time,
          time_statistic_info.sign);
    }
    LOG_INFO(
        "{} time statistic all time: {:.4f}", s,
        time_statistic_.back().abs_time - time_statistic_.front().abs_time);
  }
  time_statistic_.clear();
}

}  // namespace planning
}  // namespace neodrive
