#include "polynomial_xd.h"

namespace neodrive {
namespace planning {

PolynomialXd::PolynomialXd(const std::size_t order) : params_(order + 1, 0.0) {}

PolynomialXd::PolynomialXd(const std::vector<double>& params)
    : params_(params){};

std::size_t PolynomialXd::order() const { return params_.size(); }

void PolynomialXd::set_params(const std::vector<double>& params) {
  params_ = params;
}

const std::vector<double>& PolynomialXd::params() const { return params_; }

void PolynomialXd::derived_from(const PolynomialXd& base) {
  if (base.order() <= 1) {
    params_.clear();
  } else {
    params_.resize(base.order() - 1);
    for (std::size_t i = 1; i < base.order(); ++i) {
      params_[i - 1] = base[i] * i;
    }
  }
}

void PolynomialXd::integrated_from(const PolynomialXd& base) {
  params_.resize(base.order() + 1);
  params_[0] = 0.0;
  for (std::size_t i = 0; i < base.order(); ++i) {
    params_[i + 1] = base[i] / (i + 1);
  }
}

void PolynomialXd::integrated_from(const PolynomialXd& base,
                                   const double intercept) {
  params_.resize(base.order() + 1);
  params_[0] = intercept;
  for (std::size_t i = 0; i < base.order(); ++i) {
    params_[i + 1] = base[i] / (i + 1);
  }
}

double PolynomialXd::operator()(const double value) const {
  double result = 0.0;
  for (auto rit = params_.rbegin(); rit != params_.rend(); ++rit) {
    result *= value;
    result += (*rit);
  }
  return result;
}

double PolynomialXd::operator[](const std::size_t index) const {
  if (index >= params_.size()) {
    return 0.0;
  } else {
    return params_[index];
  }
}

std::string PolynomialXd::to_json() const { return ""; }

}  // namespace planning
}  // namespace neodrive
