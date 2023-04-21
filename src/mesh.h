#ifndef MESH_H_
#define MESH_H_

#include <Eigen/Dense>

namespace ppr {

class Mesh {
public:
  Mesh() {}
  Mesh(const std::vector<Eigen::Vector3d> &verts,
       const std::vector<std::tuple<int, int, int>> &triangles,
       const Eigen::Vector3d &color)
      : verts_(verts), triangles_(triangles), color_(color) {}

  const std::vector<Eigen::Vector3d> &Verts() const { return verts_; }
  const std::vector<std::tuple<int, int, int>> &Triangles() const {
    return triangles_;
  }
  const Eigen::Vector3d &Color() const { return color_; }

  void CalcBbox(Eigen::Vector3d &min, Eigen::Vector3d &max) const;

private:
  std::vector<Eigen::Vector3d> verts_;
  std::vector<std::tuple<int, int, int>> triangles_;
  Eigen::Vector3d color_;
};

} // namespace ppr

#endif /* MESH_H_ */
