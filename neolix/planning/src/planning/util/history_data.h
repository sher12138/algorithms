#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>

namespace neodrive {
namespace planning {

enum HISTORY_DATA_TYPE { HISTORY_DATA_INT = 0, HISTORY_DATA_DOUBLE = 1 };

class HistoryData {
 public:
  HistoryData();
  ~HistoryData();

  bool Init(const int type, const int count);
  int type() { return type_; }
  int count() { return count_; }
  void Reset();
  // store histroy value
  void Push(const int val);
  void Push(const double val);

  // get average of histroy value
  double GetAverage();
  // get sum value
  double GetSumValue() const;
  // get max value
  double GetMaxValue();

  int GetNumber() const;
  // whether histroy keep the same?
  bool AllEqual(const int val) const;

 private:
  int type_;
  int count_;
  int curr_pos_;

  bool history_data_error_flag_;  // True:error
  int history_data_error_index_;  // 1:type undefined
  std::deque<double> history_double_data_{};
  std::deque<int> history_int_data_{};
  double val_sum_{0};
  bool val_average_updated_{false};
  double val_average_{0.};
  double max_val_{-10000.};
};

template <typename T1>
class HistoryLogData {
 public:
  HistoryLogData() : count_(0) {}
  ~HistoryLogData() = default;

  void Init(const int count);

  void Reset();

  void Push(const T1 val);

  double GetAverage() const;

  double GetSumValue() const;

  std::size_t GetNumber() const;

  // whether histroy keep the same?
  bool AllEqual(const T1 val) const;

  bool AllLessThan(const T1 val) const;

  bool AllLargerThan(const T1 val) const;

  double LessThanPercent(const T1 val) const;

  double LargerThanPercent(const T1 val) const;

  // get last n data function
  bool LastNLessThan(const T1 val, const int last_size) const;

  bool LastNLargerThan(const T1 val, const int last_size) const;

  double LastNLessThanPercent(const T1 val, const int last_size) const;

  double LastNLargerThanPercent(const T1 val, const int last_size) const;

  bool LastNAllEqual(const T1 val, const int last_size) const;

  double LastNGetAverage(const int last_size) const;

  double LastNGetSumValue(const int last_size) const;

  T1 GetLastData() const;

 private:
  int count_;
  std::deque<T1> history_data_{};
};

template <typename T1>
void HistoryLogData<T1>::Init(const int count) {
  history_data_.clear();
  count_ = count;
}

template <typename T1>
void HistoryLogData<T1>::Reset() {
  history_data_.clear();
}

template <typename T1>
void HistoryLogData<T1>::Push(const T1 val) {
  if (history_data_.size() >= count_) {
    history_data_.pop_front();
    history_data_.push_back(val);
  } else {
    history_data_.push_back(val);
  }
}

template <typename T1>
double HistoryLogData<T1>::GetAverage() const {
  if (history_data_.empty()) return 0.0;
  return GetSumValue() / history_data_.size();
}

template <typename T1>
double HistoryLogData<T1>::GetSumValue() const {
  return std::accumulate(history_data_.begin(), history_data_.end(), 0.0);
}

template <typename T1>
std::size_t HistoryLogData<T1>::GetNumber() const {
  return history_data_.size();
}

template <typename T1>
bool HistoryLogData<T1>::AllEqual(const T1 val) const {
  for (auto& tmp : history_data_) {
    if (std::abs(tmp - val) > 1.e-3) {
      return false;
    }
  }
  return true;
}

template <typename T1>
bool HistoryLogData<T1>::AllLessThan(const T1 val) const {
  for (auto& tmp : history_data_) {
    if (tmp - val > -1.e-3) {
      return false;
    }
  }
  return true;
}

template <typename T1>
bool HistoryLogData<T1>::AllLargerThan(const T1 val) const {
  for (auto& tmp : history_data_) {
    if (val - tmp > -1.e-3) {
      return false;
    }
  }
  return true;
}

template <typename T1>
double HistoryLogData<T1>::LessThanPercent(const T1 val) const {
  if (history_data_.empty()) return 0.0;
  std::size_t less_num{0};
  for (auto& tmp : history_data_) {
    if (val - tmp > 1.e-3) {
      ++less_num;
    }
  }

  return static_cast<double>(less_num) / history_data_.size();
}

template <typename T1>
double HistoryLogData<T1>::LargerThanPercent(const T1 val) const {
  if (history_data_.empty()) return 0.0;
  std::size_t larger_num{0};
  for (auto& tmp : history_data_) {
    if (tmp - val > 1.e-3) {
      ++larger_num;
    }
  }

  return static_cast<double>(larger_num) / history_data_.size();
}

template <typename T1>
bool HistoryLogData<T1>::LastNLessThan(const T1 val,
                                       const int last_size) const {
  int start_pos =
      std::max(0, static_cast<int>(history_data_.size()) - last_size);

  for (int i = start_pos; i < history_data_.size(); ++i) {
    if (history_data_[i] - val > -1.e-3) {
      return false;
    }
  }
  return true;
}

template <typename T1>
bool HistoryLogData<T1>::LastNLargerThan(const T1 val,
                                         const int last_size) const {
  int start_pos =
      std::max(0, static_cast<int>(history_data_.size()) - last_size);

  for (int i = start_pos; i < history_data_.size(); ++i) {
    if (val - history_data_[i] > -1.e-3) {
      return false;
    }
  }
  return true;
}

template <typename T1>
double HistoryLogData<T1>::LastNLessThanPercent(const T1 val,
                                                const int last_size) const {
  if (history_data_.empty() || last_size == 0) return 0.0;
  int start_pos =
      std::max(0, static_cast<int>(history_data_.size()) - last_size);

  std::size_t less_num{0};
  for (int i = start_pos; i < history_data_.size(); ++i) {
    if (val - history_data_[i] > 1.e-3) {
      ++less_num;
    }
  }

  return static_cast<double>(less_num) /
         std::min(static_cast<int>(history_data_.size()), last_size);
}

template <typename T1>
double HistoryLogData<T1>::LastNLargerThanPercent(const T1 val,
                                                  const int last_size) const {
  if (history_data_.empty() || last_size == 0) return 0.0;
  int start_pos =
      std::max(0, static_cast<int>(history_data_.size()) - last_size);

  std::size_t less_num{0};
  for (int i = start_pos; i < history_data_.size(); ++i) {
    if (history_data_[i] - val > 1.e-3) {
      ++less_num;
    }
  }

  return static_cast<double>(less_num) /
         std::min(static_cast<int>(history_data_.size()), last_size);
}

template <typename T1>
bool HistoryLogData<T1>::LastNAllEqual(const T1 val,
                                       const int last_size) const {
  int start_pos =
      std::max(0, static_cast<int>(history_data_.size()) - last_size);

  for (int i = start_pos; i < history_data_.size(); ++i) {
    if (std::abs(val - history_data_[i]) > 1.e-3) {
      return false;
    }
  }
  return true;
}

template <typename T1>
double HistoryLogData<T1>::LastNGetAverage(const int last_size) const {
  if (history_data_.empty()) return 0.0;
  if (last_size < history_data_.size()) {
    return static_cast<double>(LastNGetSumValue(last_size)) / last_size;
  } else {
    return static_cast<double>(LastNGetSumValue(last_size)) /
           history_data_.size();
  }
}

template <typename T1>
double HistoryLogData<T1>::LastNGetSumValue(const int last_size) const {
  if (history_data_.empty()) return 0.0;
  if (last_size < history_data_.size()) {
    int pos = history_data_.size() - last_size;
    return std::accumulate(history_data_.begin() + pos, history_data_.end(),
                           0.0);
  } else {
    return std::accumulate(history_data_.begin(), history_data_.end(), 0.0);
  }
}

template <typename T1>
T1 HistoryLogData<T1>::GetLastData() const {
  return history_data_.empty() ? T1{} : history_data_.back();
}

}  // namespace planning
}  // namespace neodrive
