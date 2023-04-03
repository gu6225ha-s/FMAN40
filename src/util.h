#ifndef UTIL_H_
#define UTIL_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace ppr {

std::vector<std::string> SplitString(const std::string &str,
                                     const std::string &delim);

class PlaneEstimator {
public:
  PlaneEstimator() {}
  void AddCorrespondence(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
                         const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
  Eigen::Vector3d Solve() const;

private:
  Eigen::MatrixXd A_, b_;
};

} // namespace ppr

#endif /* UTIL_H_ */
