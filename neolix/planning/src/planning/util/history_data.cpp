#include "history_data.h"
namespace neodrive {
namespace planning {

HistoryData::HistoryData()
    : type_(0),
      count_(0),
      curr_pos_(0),
      history_data_error_flag_(false),
      history_data_error_index_(0) {
  history_double_data_.clear();
  history_int_data_.clear();
  val_sum_ = 0.;
  val_average_updated_ = false;
  max_val_ = -10000.;
}

HistoryData::~HistoryData() {
  history_double_data_.clear();
  history_int_data_.clear();
}

bool HistoryData::Init(const int type, const int count) {
  if (count_ != 0) {
    history_double_data_.clear();
    history_int_data_.clear();
    count_ = 0;
  }
  val_sum_ = 0.;
  val_average_updated_ = false;
  max_val_ = -10000.;
  if (type == HISTORY_DATA_TYPE::HISTORY_DATA_INT ||
      type == HISTORY_DATA_TYPE::HISTORY_DATA_DOUBLE) {
    count_ = count;
    curr_pos_ = 0;
    type_ = type;
    return true;
  } else {
    history_data_error_flag_ = true;
    history_data_error_index_ = 1;
    return false;
  }
}

// reset
void HistoryData::Reset() {
  history_double_data_.clear();
  history_int_data_.clear();
  history_data_error_flag_ = false;
  history_data_error_index_ = 0;
  val_sum_ = 0.;
  val_average_updated_ = false;
  curr_pos_ = 0;
  max_val_ = -10000.;
}

// save history value
void HistoryData::Push(const int val) {
  val_average_updated_ = false;
  if (curr_pos_ >= count_) {
    auto remove_val = history_int_data_.front();
    val_sum_ += val;
    val_sum_ -= remove_val;
    history_int_data_.pop_front();
    history_int_data_.push_back(val);
    if (val >= max_val_) {
      max_val_ = val;
    } else if (std::fabs(remove_val - max_val_) < 0.0001) {
      max_val_ = -10000.;
      for (auto item_val : history_int_data_) {
        if (item_val > max_val_) {
          max_val_ = item_val;
        }
      }
    }
    curr_pos_ = count_;
  } else {
    val_sum_ += val;
    history_int_data_.push_back(val);
    if (val > max_val_) {
      max_val_ = val;
    }
    ++curr_pos_;
  }
}

// save history value
void HistoryData::Push(const double val) {
  if (curr_pos_ >= count_) {
    history_double_data_.pop_front();
    history_double_data_.push_back(val);
    curr_pos_ = count_;
  } else {
    history_double_data_.push_back(val);
    ++curr_pos_;
  }
}

int HistoryData::GetNumber() const { return curr_pos_; }

// get average of histroy value
double HistoryData::GetAverage() {
  if (val_average_updated_) {
    return val_average_;
  }
  double val = 0.0;
  int icount = 0;
  if (type_ == HISTORY_DATA_TYPE::HISTORY_DATA_INT) {
    int num_size = static_cast<int>(history_int_data_.size());
    icount = std::min(num_size, curr_pos_);
    for (int i = 0; i < icount; ++i) {
      val += history_int_data_[i];
    }
  } else if (type_ == HISTORY_DATA_TYPE::HISTORY_DATA_DOUBLE) {
    int num_size = static_cast<int>(history_int_data_.size());
    icount = std::min(curr_pos_, num_size);
    for (int i = 0; i < icount; ++i) {
      val += history_double_data_[i];
    }
  }
  if (icount > 0) {
    val_average_ = val / icount;
    val_average_updated_ = true;
  }
  return val_average_;
}

// get total number
double HistoryData::GetSumValue() const { return val_sum_; }
// get total number
double HistoryData::GetMaxValue() {
  double max_val = -100000.;
  int icount = 0;
  if (type_ == HISTORY_DATA_TYPE::HISTORY_DATA_INT) {
    int data_size = history_int_data_.size();
    icount = std::min(curr_pos_, data_size);
    for (int i = 0; i < icount; ++i) {
      if (history_int_data_[i] > max_val) {
        max_val = history_int_data_[i];
      }
    }
  } else if (type_ == HISTORY_DATA_TYPE::HISTORY_DATA_DOUBLE) {
    int tmp = history_double_data_.size();
    icount = std::min(curr_pos_, tmp);
    for (int i = 0; i < icount; ++i) {
      if (history_int_data_[i] > max_val) {
        max_val = history_int_data_[i];
      }
    }
  }
  return max_val;
}

// whether histroy keep the same?
bool HistoryData::AllEqual(const int val) const {
  bool ret_val = true;
  int icount = 0;
  if (type_ == HISTORY_DATA_TYPE::HISTORY_DATA_INT) {
    int data_size = history_int_data_.size();
    icount = std::min(curr_pos_, data_size);
    for (int i = 0; i < icount; ++i) {
      if (history_int_data_[i] != val) {
        ret_val = false;
        break;
      }
    }
  } else if (type_ == HISTORY_DATA_TYPE::HISTORY_DATA_DOUBLE) {
    int data_size = history_int_data_.size();
    icount = std::min(curr_pos_, data_size);
    for (int i = 0; i < icount; ++i) {
      if (fabs(history_double_data_[i] - val) > 0.001) {
        ret_val = false;
        break;
      }
    }
  }
  return ret_val;
}

}  // namespace planning
}  // namespace neodrive
