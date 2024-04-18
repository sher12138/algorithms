#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace neodrive {
namespace planning {
/*
 *
 * T1 must have a val of val;
 * it
 */

template <class T1>
class HistoryDefinedData {
 public:
  bool Init(int max_size) {
    history_defined_data_.clear();
    max_size_ = max_size;
    val_sum_ = 0.;
    val_average_updated_ = false;
    return true;
  }

  void Reset() {
    history_defined_data_.clear();
    val_sum_ = 0.;
    val_average_updated_ = false;
  }

  // store histroy value
  void Push(T1 val) {
    val_average_updated_ = false;
    if (history_defined_data_.size() >= max_size_) {
      val_sum_ += val.val_;
      val_sum_ -= history_defined_data_.front().val_;
      history_defined_data_.pop_front();
      history_defined_data_.push_back(val);
    } else {
      val_sum_ += val.val_;
      history_defined_data_.push_back(val);
    }
  }

  // get average of histroy value
  double GetAverage() {
    if (val_average_updated_) {
      return val_average_;
    }
    if (history_defined_data_.size() > 0) {
      val_average_ = val_sum_ / history_defined_data_.size();
    }
    val_average_updated_ = true;
    return val_average_;
  }

  double GetDistribute() const {
    double val = 0;
    int icount = history_defined_data_.size();

    std::vector<double> proba;
    double tmp_val = 0.0;
    for (std::size_t i = 0, j = 0; i < icount; ++i, ++j) {
      if (j >= 10) {
        tmp_val = tmp_val / 10;
        proba.push_back(tmp_val);
        j = 0;
        tmp_val = 0.0;
        tmp_val += history_defined_data_[i].val_;
      }
      tmp_val += history_defined_data_[i].val_;
    }

    if (proba.empty()) {
      return val;
    }
    std::size_t i = 0;
    double total = 0.0;
    double total_val = 0.0;
    for (; i < proba.size(); ++i) {
      total += std::exp(i + 1) * proba[i];
      total_val += std::exp(i + 1);
    }
    if (fabs(total_val) < 0.01) {
      return val;
    }
    val = total / total_val;
    val = (static_cast<int>(val * 100)) * 0.01;
    return val;
  }

  // get sum value
  double GetSumValue() const { return val_sum_; }

  int GetNumber() const { return history_defined_data_.size(); }

  // whether histroy keep the same?
  bool AllEqual(int val) const {
    bool ret_val = true;
    int icount = 0;

    int data_size = history_defined_data_.size();
    icount = std::min(curr_pos_, data_size);
    for (int i = 0; i < icount; ++i) {
      if (fabs(history_defined_data_[i].val - val) > 0.001) {
        ret_val = false;
        break;
      }
    }

    return ret_val;
  }

  double LastTimeStamp() {
    if (history_defined_data_.size() < 1) {
      return 0.0;
    }
    return history_defined_data_.back().time_stamp_;
  }

 private:
  std::deque<T1> history_defined_data_;
  int max_size_{0};
  int curr_pos_{0};
  double val_sum_{0};
  bool val_average_updated_{false};
  double val_average_{0.};
};

class ChangeLaneHistoryData {
 public:
  ChangeLaneHistoryData() {}
  ~ChangeLaneHistoryData() {}

  int val_ = 0.0;
  double time_stamp_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
