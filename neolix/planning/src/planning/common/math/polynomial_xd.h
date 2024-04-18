#pragma once

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace neodrive {
namespace planning {

class PolynomialXd {
 public:
  PolynomialXd() = default;
  explicit PolynomialXd(const std::size_t order);
  explicit PolynomialXd(const std::vector<double>& params);
  double operator()(const double value) const;
  double operator[](const std::size_t index) const;
  void set_params(const std::vector<double>& params);
  void derived_from(const PolynomialXd& base);
  void integrated_from(const PolynomialXd& base);
  void integrated_from(const PolynomialXd& base, const double intercept);
  std::size_t order() const;
  const std::vector<double>& params() const;
  virtual std::string to_json() const;

 private:
  std::vector<double> params_;
};

}  // namespace planning
}  // namespace neodrive
