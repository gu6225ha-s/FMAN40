#include "util.h"

namespace ppr {

std::vector<std::string> SplitString(const std::string &str,
                                     const std::string &delim) {
  size_t start = 0, end;
  std::vector<std::string> ret;

  while ((end = str.find(delim, start)) != std::string::npos) {
    ret.push_back(str.substr(start, end - start));
    start = end + delim.length();
  }

  ret.push_back(str.substr(start));
  return ret;
}

void PlaneEstimator::AddCorrespondence(const Eigen::Vector3d &x,
                                       const Eigen::Vector3d &y,
                                       const Eigen::Matrix3d &R,
                                       const Eigen::Vector3d &t) {
  x_.push_back(x);
  y_.push_back(y);
  Rx_.push_back(R * x);
  t_.push_back(t);
}

Eigen::Vector3d PlaneEstimator::Solve() const {
  size_t n = x_.size();
  Eigen::MatrixXd A(3 * n, 3 + n);
  Eigen::MatrixXd b(3 * n, 1);

  A.setZero();
  b.setZero();

  for (size_t i = 0; i < n; i++) {
    A.block(3 * i, 0, 3, 3) = t_[i] * x_[i].transpose();
    A.block(3 * i, 3 + i, 3, 1) = y_[i];
    b.block(3 * i, 0, 3, 1) = Rx_[i];
  }

  Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
  return x.head(3);
}

} // namespace ppr
