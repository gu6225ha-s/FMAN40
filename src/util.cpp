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
  Eigen::Index n = A_.rows() / 3;

  A_.conservativeResize(3 * (n + 1), 3 + (n + 1));
  A_.block(0, 3 + n, 3 * n, 1).setZero();
  A_.block(3 * n, 0, 3, 3) = t * x.transpose();
  A_.block(3 * n, 3, 3, n).setZero();
  A_.block(3 * n, 3 + n, 3, 1) = y;

  b_.conservativeResize(3 * (n + 1), 1);
  b_.block(3 * n, 0, 3, 1) = R * x;
}

Eigen::Vector3d PlaneEstimator::Solve() const {
  Eigen::VectorXd x = A_.colPivHouseholderQr().solve(b_);
  return x.head(3);
}

} // namespace ppr
