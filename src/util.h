#ifndef UTIL_H_
#define UTIL_H_

#include "mesh.h"
#include "polygon.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace ppr {

std::vector<std::string> SplitString(const std::string &str,
                                     const std::string &delim);

std::vector<std::pair<std::string, ppr::Polygon2d>>
ReadPolygons(const std::string &path);

void WriteGltf(const std::vector<Mesh> &meshes, const std::string &path);

class PlaneEstimator {
public:
  PlaneEstimator() {}
  void AddCorrespondence(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
                         const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
  Eigen::Vector3d Solve() const;

private:
  std::vector<Eigen::Vector3d> x_, y_, Rx_, t_;
};

} // namespace ppr

#endif /* UTIL_H_ */
